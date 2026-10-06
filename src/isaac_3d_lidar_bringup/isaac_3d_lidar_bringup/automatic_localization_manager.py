"""Safely gate saved-map navigation on automatic AMCL localization."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
import json
import math
import time
import uuid

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from isaac_3d_lidar_bringup.automatic_localization_quality import (
    angular_difference,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    covariance_quality,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    deterministic_global_search,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    particle_concentration,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    planar_window_span,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    quaternion_yaw,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    scan_map_metrics,
)
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    trim_time_window,
)
from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError,
    decode_motion_profile,
    decode_motion_status,
    encode_motion_request,
    FrameRole,
    GuardState,
    MotionOperation,
    MotionPolicy,
    REJECT_REASON_TEXT,
    RejectReason,
    RotationAttestation,
    SCHEMA_VERSION,
    SE2,
    Strategy,
    validate_confined_parameters,
    validate_strategy_configuration,
)
from isaac_3d_lidar_bringup.localization_hypotheses import (
    grid_snapshot,
    map_hash,
    run_search_job,
    run_validation_job,
    SearchConfig,
    SearchWorker,
    seed_pose_at_current_time,
    ValidationThresholds,
)
from isaac_3d_lidar_bringup.localization_motion_guard import (
    ProbeLink,
    profile_hash,
)
from isaac_3d_lidar_bringup.localization_observations import (
    collect_keyframes,
    make_keyframe,
    Reject,
    snapshot_scan,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (
    choose_probe,
    evaluate_localization_rotation,
    evidence_from_scan,
    footprint_geometry_hash,
    RotationGateConfig,
)


LOCALIZATION_MANAGER = '/lifecycle_manager_localization/manage_nodes'
NAVIGATION_MANAGER = '/lifecycle_manager_navigation/manage_nodes'


class State(Enum):
    """Automatic localization state."""

    WAIT_FOR_START = 'WAIT_FOR_START'
    WAIT_SENSORS = 'WAIT_SENSORS'
    START_LOCALIZATION = 'START_LOCALIZATION'
    GLOBAL_LOCALIZATION = 'GLOBAL_LOCALIZATION'
    ROTATE_AND_SCORE = 'ROTATE_AND_SCORE'
    STOP_AND_VERIFY = 'STOP_AND_VERIFY'
    SEARCH_GLOBAL_POSE = 'SEARCH_GLOBAL_POSE'
    START_NAVIGATION = 'START_NAVIGATION'
    CANDIDATE_READY = 'CANDIDATE_READY'
    READY = 'READY'
    SAFE_STOP = 'SAFE_STOP'
    WAIT_MANUAL_POSE = 'WAIT_MANUAL_POSE'
    VERIFY_MANUAL_POSE = 'VERIFY_MANUAL_POSE'
    FAULT_STOPPED = 'FAULT_STOPPED'
    # Issue #13 strategies.  None of these states commands motion.
    COLLECT_STATIC = 'COLLECT_STATIC'
    SEARCH_MULTI_VIEW = 'SEARCH_MULTI_VIEW'
    VERIFY_HYPOTHESES = 'VERIFY_HYPOTHESES'
    # segmented_rotation only: the guard, not this node, drives the robot.
    PLAN_PROBE = 'PLAN_PROBE'
    EXECUTE_PROBE = 'EXECUTE_PROBE'
    SETTLE_PROBE = 'SETTLE_PROBE'


CONFINED_STATES = frozenset((
    State.COLLECT_STATIC, State.SEARCH_MULTI_VIEW, State.VERIFY_HYPOTHESES,
    State.PLAN_PROBE, State.EXECUTE_PROBE, State.SETTLE_PROBE))

# A new viewpoint can resolve these; data and map failures it cannot.
PROBE_RESOLVABLE = frozenset((
    RejectReason.SEARCH_INCOMPLETE.value,
    RejectReason.AMBIGUOUS_LOCATION.value,
    RejectReason.UNOBSERVABLE_AXIS.value,
    RejectReason.NO_VALID_CANDIDATE.value,
))


class AutomaticLocalizationManager(Node):
    """Drive a guarded global-localization startup state machine."""

    def __init__(self):
        """Create subscriptions, services, safety state, and the timer."""
        super().__init__('automatic_localization_manager')
        self._status_instance_id = uuid.uuid4().hex
        self._declare_parameters()
        # Immutable for this run; dynamic parameter changes cannot enable Nav2.
        # The maintained launcher defaults this to true and only its explicit
        # post-acceptance automatic-activation mode overrides it at startup.
        self._validation_only = bool(self._parameter('validation_only'))
        self._stationary_only = False
        # Also immutable: no dynamic change may enable motion or relax gates.
        self._configure_strategy()

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            # Safety decisions must use the newest sensor/evidence sample.
            # A large queue lets AMCL's global particle-cloud burst delay
            # LaserScan callbacks in rclpy's single-threaded executor.
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_publisher = self.create_publisher(
            String, '/automatic_localization/status', status_qos
        )
        self._pose_seed_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self._parameter('amcl_initial_pose_topic'),
            10,
        )
        self._command_topic = self._parameter('cmd_vel_topic')
        self._command_publisher = None
        self._configure_probe_link()
        self._raw_odom_subscription = self.create_subscription(
            Odometry,
            self._parameter('fast_lio_odom_topic'),
            self._on_raw_odom,
            20,
        )
        self._odom_subscription = self.create_subscription(
            Odometry, self._parameter('odom_topic'), self._on_odom, 20
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            self._parameter('scan_topic'),
            self._on_scan,
            scan_qos,
        )
        self._safety_scan_subscription = self.create_subscription(
            LaserScan,
            self._parameter('safety_scan_topic'),
            self._on_safety_scan,
            scan_qos,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid, self._parameter('map_topic'), self._on_map, map_qos
        )
        self._amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._parameter('amcl_pose_topic'),
            self._on_amcl_pose,
            10,
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._parameter('initial_pose_topic'),
            self._on_initial_pose,
            10,
        )
        self._particle_subscription = self.create_subscription(
            ParticleCloud,
            self._parameter('particle_cloud_topic'),
            self._on_particle_cloud,
            scan_qos,
        )

        self._localization_client = self.create_client(
            ManageLifecycleNodes, LOCALIZATION_MANAGER
        )
        self._navigation_client = self.create_client(
            ManageLifecycleNodes, NAVIGATION_MANAGER
        )
        self._global_localization_client = self.create_client(
            Empty, self._parameter('global_localization_service')
        )
        self._nomotion_update_client = self.create_client(
            Empty, self._parameter('nomotion_update_service')
        )
        self._start_service = self.create_service(
            Trigger,
            '/automatic_localization/start',
            self._on_start_request,
        )
        self._prepare_service = self.create_service(
            Trigger, '/automatic_localization/prepare_stationary',
            self._on_prepare_request,
        )
        self._cancel_service = self.create_service(
            Trigger, '/automatic_localization/cancel', self._on_cancel_request,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        now = time.monotonic()
        self._state = (
            State.WAIT_SENSORS
            if self._parameter('start_armed')
            else State.WAIT_FOR_START
        )
        self._state_started = now
        self._startup_started = now
        self._future = None
        # Set once a Nav2 STARTUP/RESUME request may have taken effect.  A
        # later failure must then pause Nav2, not only report SAFE_STOP.
        self._navigation_may_be_active = False
        self._navigation_pause_state = None
        self._pause_future = None
        self._nomotion_future = None
        self._last_nomotion_request = None
        self._last_raw_odom_time = None
        self._last_odom_time = None
        self._last_scan_time = None
        self._last_safety_scan_time = None
        self._last_amcl_time = None
        self._last_score_time = None
        self._last_map_odom_time = None
        self._last_particle_time = None
        self._last_odom_pose = None
        self._last_odom_yaw = None
        self._stationary_since = None
        self._moving_since = None
        self._last_linear_speed = math.inf
        self._last_angular_speed = math.inf
        self._sensors_ready_since = None
        self._latest_map = None
        self._latest_amcl_pose = None
        self._scan_metrics = {}
        self._scan_score = 0.0
        self._valid_scan_beams = 0
        self._sampled_scan_beams = 0
        self._finite_scan_beams = 0
        self._scan_min_range = math.inf
        self._particle_concentration = 0.0
        self._particle_count = 0
        self._rotation_progress = 0.0
        self._rotation_data_stale_since = None
        self._rotation_obstacle_since = None
        self._tf_window = deque()
        self._quality_since = None
        self._candidate_invalid_since = None
        self._failure_reason = ''
        self._manual_recovery_allowed = False
        self._last_status = None
        self._source_stamps = {}
        self._timing = {}
        self._last_tf_stamp = None
        self._quality_failure = 'NO_EVIDENCE'
        self._evidence_epoch_ns = 0
        self._score_duration_sec = None
        self._scan_tf_error = ''
        self._pending_scans = deque(maxlen=3)
        self._stationary_search_scans = deque(
            maxlen=int(self._parameter('global_search_scan_count')))
        self._search_executor = ThreadPoolExecutor(max_workers=1)
        self._search_future = None
        self._search_best = None
        self._search_runner_up = None
        self._awaiting_amcl_initial_pose = False
        self._requested_initial_pose = None
        self._manual_reference_pose = None
        self._candidate_anchor_pose = None
        self._reset_confined_session()
        self._latest_grid = None
        self._map_hash = ''
        self._worker = (
            SearchWorker() if self._strategy != Strategy.LEGACY_FULL_ROTATION
            else None)

        self._timer = self.create_timer(0.1, self._tick)
        self._publish_status(force=True)

    def _declare_parameters(self):
        parameters = {
            'fast_lio_odom_topic': '/fast_lio/imu_odom',
            'odom_topic': '/odom',
            'scan_topic': '/scan_localization',
            'safety_scan_topic': '/scan',
            'map_topic': '/map',
            'amcl_pose_topic': '/amcl_pose',
            'initial_pose_topic': '/initialpose',
            'amcl_initial_pose_topic': '/amcl_initialpose',
            'particle_cloud_topic': '/particle_cloud',
            'cmd_vel_topic': '/cmd_vel_command',
            'global_localization_service':
                '/reinitialize_global_localization',
            'nomotion_update_service': '/request_nomotion_update',
            'start_armed': False,
            'validation_only': True,
            'max_future_stamp_sec': 0.05,
            'max_tf_future_sec': 0.6,
            # Must equal AMCL transform_tolerance (carbot_amcl_real.yaml).
            'map_odom_postdate_sec': 1.5,
            'sensor_freshness_sec': 0.5,
            'localization_evidence_freshness_sec': 10.0,
            'nomotion_update_interval_sec': 1.0,
            'manual_seed_xy_std_m': 0.10,
            'manual_seed_yaw_std_rad': math.radians(5.0),
            'max_manual_pose_drift_m': 0.20,
            'max_manual_pose_drift_yaw_rad': math.radians(10.0),
            'max_candidate_pose_drift_m': 0.08,
            'max_candidate_pose_drift_yaw_rad': math.radians(2.5),
            'sensor_stable_sec': 5.0,
            'startup_timeout_sec': 90.0,
            'service_timeout_sec': 20.0,
            'max_stationary_linear_speed': 0.02,
            'max_stationary_angular_speed': 0.03,
            'stationary_motion_grace_sec': 0.3,
            'max_odom_position_jump': 0.20,
            'max_odom_yaw_jump': 0.35,
            'min_scan_beams_for_motion': 10,
            'min_rotation_clearance_m': 0.55,
            'rotation_obstacle_confirmation_sec': 0.3,
            'rotation_speed_rad_s': 0.40,
            'rotation_target_rad': 2.0 * math.pi,
            'rotation_timeout_sec': 45.0,
            'rotation_data_grace_sec': 3.0,
            'zero_command_duration_sec': 0.5,
            'stopped_confirmation_sec': 1.0,
            'verification_timeout_sec': 15.0,
            'quality_hold_sec': 3.0,
            # A stale TF clears the stability window.  Allow enough time for a
            # measured scan gap plus the full TF window to be rebuilt.
            'candidate_recheck_grace_sec': 7.0,
            'max_amcl_xy_std': 0.20,
            'max_amcl_yaw_std': 0.15,
            'min_particle_concentration': 0.65,
            'particle_score_max_samples': 1000,
            'particle_cluster_xy_radius': 0.75,
            'particle_cluster_yaw_radius': 0.50,
            'min_scan_map_score': 0.65,
            'min_scan_map_coverage': 0.65,
            'min_valid_scan_beams': 30,
            'scan_match_tolerance_m': 0.15,
            'scan_score_max_beams': 120,
            'occupied_threshold': 65,
            'global_search_position_step_m': 0.20,
            'global_search_yaw_step_rad': math.radians(10.0),
            'global_search_coarse_beams': 40,
            'global_search_refine_count': 120,
            'global_search_fine_position_step_m': 0.05,
            'global_search_fine_yaw_step_rad': math.radians(2.0),
            'global_search_fine_position_radius_m': 0.20,
            'global_search_fine_yaw_radius_rad': math.radians(10.0),
            'global_search_fine_seed_count': 1,
            'global_search_final_position_step_m': 0.01,
            'global_search_final_yaw_step_rad': math.radians(0.5),
            'global_search_final_position_radius_m': 0.04,
            'global_search_final_yaw_radius_rad': math.radians(2.0),
            'global_search_seed_xy_std_m': 0.02,
            'global_search_seed_yaw_std_rad': math.radians(1.0),
            'max_post_search_translation_drift_m': 0.08,
            'max_post_search_yaw_drift_rad': math.radians(2.5),
            'global_search_scan_count': 3,
            'global_search_timeout_sec': 120.0,
            'global_search_ambiguity_distance_m': 0.75,
            'global_search_min_score_margin': 0.12,
            'global_search_max_wall_conflict_ratio': 0.25,
            'tf_window_sec': 3.0,
            'max_tf_translation_span': 0.08,
            'max_tf_yaw_span': 0.08,
            # Issue #13.  legacy_full_rotation keeps the behaviour above.
            'localization_strategy': Strategy.LEGACY_FULL_ROTATION.value,
            'motion_policy': MotionPolicy.FORBID.value,
            'motion_profile_path': '',
            'operator_rotation_clear': False,
            'train_frames_per_view': 3,
            'holdout_frames_per_view': 3,
            'max_views': 8,
            'max_probe_segments': 6,
            'probe_angles_rad': [
                math.pi / 6.0, -math.pi / 6.0, math.pi / 3.0,
                -math.pi / 3.0, math.pi / 2.0, -math.pi / 2.0],
            'max_total_probe_yaw_rad': 2.0 * math.pi,
            'probe_motion_timeout_sec': 45.0,
            'motion_request_timeout_sec': 0.30,
            'search_timeout_sec': 120.0,
            'session_timeout_sec': 240.0,
            'collect_static_timeout_sec': 10.0,
            'independent_cluster_xy_m': 0.30,
            'independent_cluster_yaw_rad': math.pi / 12.0,
            'max_refined_clusters': 8,
            # segmented_rotation (PR3).  The guard re-checks everything.
            'motion_request_topic': '/automatic_localization/motion_request',
            'motion_status_topic': '/automatic_localization/motion_status',
            'extrinsics_hash': '',
            'control_chain_hash': '',
            'footprint_xy': [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133],
            'rotation_padding_m': 0.05,
            'probe_speed_rad_s': 0.40,
            'attestation_max_translation_m': 0.05,
            'attestation_max_age_sec': 240.0,
            'probe_plan_timeout_sec': 2.0,
            'probe_settle_sec': 1.0,
            'probe_settle_timeout_sec': 5.0,
            'guard_status_freshness_sec': 0.5,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _parameter(self, name):
        return self.get_parameter(name).value

    def _configure_probe_link(self):
        """Wire the guard lease topics; segmented_rotation only."""
        self._probe = None
        self._guard_status = None
        self._latest_safety_scan = None
        self._view_id = 0
        if not self._segmented:
            return
        self._motion_request_publisher = self.create_publisher(
            String, self._parameter('motion_request_topic'), 10)
        self.create_subscription(
            String, self._parameter('motion_status_topic'),
            self._on_motion_status, 10)
        flat = list(self._parameter('footprint_xy'))
        self._footprint = tuple(zip(flat[0::2], flat[1::2]))
        self._rotation_gate = RotationGateConfig(
            padding_m=float(self._parameter('rotation_padding_m')))
        self._probe_hashes = (
            footprint_geometry_hash(self._footprint,
                                    self._rotation_gate.padding_m),
            self._parameter('extrinsics_hash'),
            self._parameter('control_chain_hash'))
        self._motion_profile = None
        path = self._parameter('motion_profile_path')
        try:
            with open(path, encoding='utf-8') as stream:
                self._motion_profile = decode_motion_profile(stream.read())
        except (OSError, ContractError) as error:
            # Every probe then reports PROFILE_INVALID; nothing moves.
            self.get_logger().error(f'motion profile rejected: {error}')

    def _on_motion_status(self, message):
        try:
            status = decode_motion_status(message.data)
        except ContractError as error:
            self.get_logger().warning(f'invalid motion status: {error}')
            return
        self._guard_status = (status, time.monotonic())

    def _fresh_guard_status(self, now):
        """Return the guard's fresh status for our session, or None."""
        if self._guard_status is None or self._probe is None:
            return None
        status, received = self._guard_status
        if (now - received > self._parameter('guard_status_freshness_sec')
                or status.session != self._probe.session):
            return None
        return status

    def _refresh_probe_lease(self):
        if self._probe is not None:
            self._motion_request_publisher.publish(String(
                data=encode_motion_request(self._probe.request())))

    def _guard_released(self, now):
        status = self._fresh_guard_status(now)
        return (status is not None and self._probe.owns(status)
                and status.state == GuardState.RELEASED
                and self.count_publishers(self._command_topic) == 0)

    def _configure_strategy(self):
        """Validate the Issue #13 strategy once; conflicts abort startup."""
        strategy, policy = validate_strategy_configuration(
            self._parameter('localization_strategy'),
            self._parameter('motion_policy'),
            self._validation_only,
            self._parameter('motion_profile_path'),
            bool(self._parameter('operator_rotation_clear')),
        )
        # segmented_rotation probes through the separate motion guard; this
        # node never creates a velocity publisher for it.
        self._segmented = strategy == Strategy.SEGMENTED_ROTATION
        names = (
            'train_frames_per_view', 'holdout_frames_per_view', 'max_views',
            'max_probe_segments', 'max_refined_clusters', 'probe_angles_rad',
            'max_total_probe_yaw_rad', 'probe_motion_timeout_sec',
            'motion_request_timeout_sec', 'sensor_freshness_sec',
            'search_timeout_sec', 'session_timeout_sec',
            'independent_cluster_xy_m', 'independent_cluster_yaw_rad',
        )
        self._confined = validate_confined_parameters(
            {name: self._parameter(name) for name in names})
        self._strategy = strategy
        self._motion_policy = policy

    def _search_config(self):
        return SearchConfig(
            occupied_threshold=int(self._parameter('occupied_threshold')),
            tolerance_cells=math.ceil(
                self._parameter('scan_match_tolerance_m')
                / self._latest_grid.info.resolution),
            coarse_step_m=self._parameter('global_search_position_step_m'),
            coarse_yaw_step_rad=self._parameter(
                'global_search_yaw_step_rad'),
            coarse_beams=int(self._parameter('global_search_coarse_beams')),
            refine_beams=int(self._parameter('scan_score_max_beams')),
            cluster_xy_m=self._confined['independent_cluster_xy_m'],
            cluster_yaw_rad=self._confined['independent_cluster_yaw_rad'],
            max_refined_clusters=self._confined['max_refined_clusters'],
        )

    def _validation_thresholds(self):
        return ValidationThresholds(
            min_score=self._parameter('min_scan_map_score'),
            min_coverage=self._parameter('min_scan_map_coverage'),
            min_known=int(self._parameter('min_valid_scan_beams')),
            max_conflict=self._parameter(
                'global_search_max_wall_conflict_ratio'),
            min_margin=self._parameter('global_search_min_score_margin'),
        )

    def _reset_confined_session(self):
        """Forget every keyframe, job and result of the previous session."""
        self._session = ''
        self._session_started = None
        self._confined_scans = deque(maxlen=10)
        self._keyframes = []
        self._next_keyframe_id = 0
        self._last_keyframe_stamp_ns = 0
        self._search_result = None
        self._validation_submitted = False
        self._reject_reason = ''

    def _cancel_worker(self):
        worker = getattr(self, '_worker', None)
        if worker is not None:
            worker.cancel()

    def _begin_confined_session(self, now):
        self._cancel_worker()
        self._reset_confined_session()
        self._session = uuid.uuid4().hex[:16]
        self._session_started = now
        self._reset_evidence()
        if self._segmented:
            self._view_id = 0
            self._probe = ProbeLink(self._session)    # STOP handshake
            self._refresh_probe_lease()
            self._probe_hypothesis_counts = []
            self._probe_unknown_cells = None
            self._attestation = None
            if (self._parameter('operator_rotation_clear')
                    and self._last_odom_pose is not None):
                self._attestation = RotationAttestation(
                    self._session, SE2(*self._last_odom_pose), now,
                    float(self._parameter('attestation_max_translation_m')),
                    float(self._parameter('attestation_max_age_sec')))
        self._search_best = None
        self._search_runner_up = None
        self._transition(State.COLLECT_STATIC)

    def _reject(self, reason, detail=''):
        """Stop a confined session with an explicit contract reason."""
        reason = RejectReason(reason)
        self._cancel_worker()
        self._reject_reason = reason.value
        self._quality_failure = reason.value
        text, manual = REJECT_REASON_TEXT[reason]
        suffix = f' ({detail})' if detail else ''
        self._fail(f'{reason.value}: {text}{suffix}', manual_recovery=manual)

    def _on_cancel_request(self, _request, response):
        # Idempotent: the response only acknowledges the request.  Stopping
        # is reported through the status topic, never claimed here.
        response.success = True
        if self._state in (State.WAIT_FOR_START, State.READY,
                           State.SAFE_STOP, State.WAIT_MANUAL_POSE,
                           State.FAULT_STOPPED):
            response.message = f'nothing to cancel in {self._state.value}'
            return response
        if self._state == State.START_NAVIGATION:
            # The Nav2 lifecycle STARTUP request may already be in flight and
            # cannot be withdrawn; reporting SAFE_STOP here would hide an
            # activating controller.  Refuse; it finishes in READY or fails
            # on its own timeout.
            response.success = False
            response.message = (
                'cannot cancel during START_NAVIGATION; Nav2 activation may '
                'already be in flight')
            return response
        self._reject(RejectReason.CANCELED)
        response.message = 'cancel accepted; watch status for the stop'
        return response

    def _accept_source(self, key, message, max_age):
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        clock_now = self.get_clock().now().nanoseconds
        age = (clock_now - stamp) / 1e9
        self._timing[key] = {'source_stamp_ns': stamp, 'source_age_sec': age}
        previous = self._source_stamps.get(key, 0)
        if (stamp <= previous or stamp <= self._evidence_epoch_ns
                or age < -self._parameter('max_future_stamp_sec')
                or age > max_age):
            return False
        self._source_stamps[key] = stamp
        return True

    def _on_raw_odom(self, message):
        if self._accept_source('raw_odom', message,
                               self._parameter('sensor_freshness_sec')):
            self._last_raw_odom_time = time.monotonic()

    def _on_odom(self, message):
        if not self._accept_source('odom', message,
                                   self._parameter('sensor_freshness_sec')):
            return
        now = time.monotonic()
        pose = message.pose.pose
        yaw = quaternion_yaw(pose.orientation)
        current_pose = (pose.position.x, pose.position.y, yaw)
        if self._last_odom_pose is not None:
            distance = math.hypot(
                current_pose[0] - self._last_odom_pose[0],
                current_pose[1] - self._last_odom_pose[1],
            )
            yaw_step = abs(angular_difference(
                current_pose[2], self._last_odom_pose[2]
            ))
            if (
                distance > self._parameter('max_odom_position_jump')
                or yaw_step > self._parameter('max_odom_yaw_jump')
            ):
                self._sensors_ready_since = None
                if self._state == State.ROTATE_AND_SCORE:
                    self._fail('odometry jumped during localization')
                elif self._state in CONFINED_STATES:
                    self._reject(RejectReason.ODOM_JUMP,
                                 f'{distance:.3f} m / {yaw_step:.3f} rad')

        if (
            self._state == State.ROTATE_AND_SCORE
            and self._last_odom_yaw is not None
        ):
            delta = angular_difference(yaw, self._last_odom_yaw)
            direction = math.copysign(
                1.0, self._parameter('rotation_speed_rad_s')
            )
            self._rotation_progress += max(0.0, direction * delta)
        self._last_odom_yaw = yaw
        self._last_odom_pose = current_pose
        self._last_odom_time = now

        twist = message.twist.twist
        linear = math.hypot(twist.linear.x, twist.linear.y)
        angular = abs(twist.angular.z)
        self._last_linear_speed = linear
        self._last_angular_speed = angular
        if (
            linear <= self._parameter('max_stationary_linear_speed')
            and angular <= self._parameter('max_stationary_angular_speed')
        ):
            self._moving_since = None
            if self._stationary_since is None:
                self._stationary_since = now
        else:
            if self._moving_since is None:
                self._moving_since = now
            elif now - self._moving_since >= self._parameter(
                    'stationary_motion_grace_sec'):
                self._stationary_since = None

    def _on_scan(self, message):
        if not self._accept_source('scan', message,
                                   self._parameter('sensor_freshness_sec')):
            return
        received = time.monotonic()
        self._last_scan_time = received
        self._pending_scans.append((message, received))
        if self._state == State.SEARCH_GLOBAL_POSE:
            self._stationary_search_scans.append(message)
        elif self._state in (State.COLLECT_STATIC, State.VERIFY_HYPOTHESES):
            self._confined_scans.append((message, received))
        self._score_pending_scans()

    def _on_safety_scan(self, message):
        if not self._accept_source(
                'safety_scan', message,
                self._parameter('sensor_freshness_sec')):
            return
        received = time.monotonic()
        self._last_safety_scan_time = received
        self._latest_safety_scan = message
        finite_ranges = [
            value for value in message.ranges
            if (
                math.isfinite(value)
                and message.range_min <= value <= message.range_max
            )
        ]
        self._finite_scan_beams = len(finite_ranges)
        self._scan_min_range = (
            min(finite_ranges) if finite_ranges else math.inf
        )

    def _score_pending_scans(self):
        """Score the newest scan whose exact-time transform is available."""
        if self._latest_map is None:
            return False
        transform = None
        selected = None
        last_error = None
        for message, received in reversed(self._pending_scans):
            try:
                transform = self._tf_buffer.lookup_transform(
                    'map', message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                ).transform
                selected = (message, received)
                break
            except TransformException as error:
                last_error = error
        if selected is None:
            if last_error is not None:
                self._scan_tf_error = str(last_error)
            return False

        message, received = selected
        started = time.monotonic()
        self._scan_tf_error = ''
        tolerance_cells = math.ceil(
            self._parameter('scan_match_tolerance_m')
            / self._latest_map.info.resolution
        )
        self._scan_metrics = scan_map_metrics(
            self._latest_map, message, transform,
            self._parameter('occupied_threshold'), tolerance_cells,
            self._parameter('scan_score_max_beams'),
        )
        self._scan_score = self._scan_metrics['score']
        self._valid_scan_beams = self._scan_metrics['known']
        self._sampled_scan_beams = self._scan_metrics['sampled']
        self._last_score_time = received
        self._score_duration_sec = time.monotonic() - started
        self._pending_scans.clear()
        return True

    def _on_map(self, message):
        self._latest_map = message
        if getattr(self, '_strategy', None) in (
                None, Strategy.LEGACY_FULL_ROTATION):
            return
        grid = grid_snapshot(message)
        digest = map_hash(grid)
        if self._map_hash and digest != self._map_hash \
                and self._state in CONFINED_STATES:
            self._reject(RejectReason.MAP_CHANGED)
        self._latest_grid = grid
        self._map_hash = digest

    def _on_amcl_pose(self, message):
        if not self._accept_source('amcl', message, self._parameter(
                'localization_evidence_freshness_sec')):
            return
        if self._awaiting_amcl_initial_pose:
            requested = self._requested_initial_pose
            pose = message.pose.pose
            distance = math.hypot(
                pose.position.x - requested[0],
                pose.position.y - requested[1],
            )
            yaw_error = abs(angular_difference(
                quaternion_yaw(pose.orientation), requested[2]
            ))
            if distance > 1.0 or yaw_error > math.radians(45.0):
                self._quality_failure = 'AMCL_INITIAL_POSE_NOT_ACKNOWLEDGED'
                return
            self._awaiting_amcl_initial_pose = False
            self.get_logger().info(
                'AMCL acknowledged the new initial pose; starting verification'
            )
        self._latest_amcl_pose = message
        self._last_amcl_time = time.monotonic()

    def _on_particle_cloud(self, message):
        if self._awaiting_amcl_initial_pose:
            return
        if not self._accept_source('particles', message, self._parameter(
                'localization_evidence_freshness_sec')):
            return
        self._particle_count = len(message.particles)
        if self._latest_amcl_pose is None:
            return
        maximum = max(1, int(self._parameter('particle_score_max_samples')))
        stride = max(1, math.ceil(len(message.particles) / maximum))
        self._particle_concentration = particle_concentration(
            message.particles[::stride],
            self._latest_amcl_pose.pose.pose,
            self._parameter('particle_cluster_xy_radius'),
            self._parameter('particle_cluster_yaw_radius'),
        )
        self._last_particle_time = time.monotonic()

    def _on_initial_pose(self, message):
        if self._state not in (State.WAIT_MANUAL_POSE, State.CANDIDATE_READY):
            return
        self._reset_evidence()
        pose = message.pose.pose
        self._requested_initial_pose = (
            pose.position.x,
            pose.position.y,
            quaternion_yaw(pose.orientation),
        )
        self._awaiting_amcl_initial_pose = True
        # RViz stamps /initialpose with its current time.  At 10 Hz that can be
        # slightly newer than the latest odom->base transform, making AMCL
        # reject the estimate.  Forward it on a private topic with zero time so
        # tf2 deliberately uses the latest complete transform instead.
        forwarded = PoseWithCovarianceStamped()
        forwarded.header.frame_id = message.header.frame_id or 'map'
        odom_stamp_ns = self._source_stamps.get('odom', 0)
        if odom_stamp_ns > 0:
            forwarded.header.stamp.sec = odom_stamp_ns // 1_000_000_000
            forwarded.header.stamp.nanosec = odom_stamp_ns % 1_000_000_000
        forwarded.pose = message.pose
        manual_xy_std = self._parameter('manual_seed_xy_std_m')
        manual_yaw_std = self._parameter('manual_seed_yaw_std_rad')
        forwarded.pose.covariance[0] = manual_xy_std ** 2
        forwarded.pose.covariance[7] = manual_xy_std ** 2
        forwarded.pose.covariance[35] = manual_yaw_std ** 2
        self._manual_reference_pose = self._requested_initial_pose
        self._pose_seed_publisher.publish(forwarded)
        self._transition(State.VERIFY_MANUAL_POSE)
        self.get_logger().info(
            'Manual 2D Pose Estimate forwarded to AMCL; waiting for acknowledgement'
        )

    def _reset_evidence(self):
        self._evidence_epoch_ns = self.get_clock().now().nanoseconds
        self._tf_window.clear()
        self._quality_since = None
        self._latest_amcl_pose = None
        self._last_amcl_time = None
        self._last_particle_time = None
        self._last_score_time = None
        self._last_map_odom_time = None
        self._particle_concentration = 0.0
        self._scan_metrics = {}
        self._scan_score = 0.0
        self._manual_reference_pose = None
        self._candidate_anchor_pose = None

    def _stopped(self, now):
        return (self._recent(self._last_odom_time, now)
                and self._stationary_since is not None
                and self._moving_since is None
                and now - self._stationary_since >= self._parameter(
                    'stopped_confirmation_sec'))

    def _on_prepare_request(self, request, response):
        if self._state != State.WAIT_FOR_START:
            response.success = False
            response.message = 'stationary preparation requires WAIT_FOR_START'
            return response
        self._stationary_only = True
        return self._on_start_request(request, response)

    def _on_start_request(self, _request, response):
        if self._state != State.WAIT_FOR_START:
            response.success = False
            response.message = (
                f'automatic localization is already {self._state.value}'
            )
            return response
        now = time.monotonic()
        self._startup_started = now
        self._sensors_ready_since = None
        if self._strategy == Strategy.LEGACY_FULL_ROTATION:
            # New strategies never own a velocity publisher in this node.
            self._ensure_command_publisher()
        self._transition(State.WAIT_SENSORS)
        response.success = True
        response.message = 'automatic localization armed'
        return response

    def _recent(self, value, now):
        return (
            value is not None
            and now - value <= self._parameter('sensor_freshness_sec')
        )

    def _sensors_ready(self, now):
        if not all((
            self._recent(self._last_raw_odom_time, now),
            self._recent(self._last_odom_time, now),
            self._recent(self._last_scan_time, now),
            self._recent(self._last_safety_scan_time, now),
        )):
            return False
        if self._stationary_since is None:
            return False
        if self._finite_scan_beams < self._parameter(
                'min_scan_beams_for_motion'):
            return False
        return self._tf_buffer.can_transform(
            'odom', 'base_footprint', Time()
        )

    def _call_lifecycle(self, client):
        if client is self._navigation_client and self._validation_only:
            raise RuntimeError('validation_only forbids navigation activation')
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        if client is self._navigation_client:
            # After a confirmed pause the Nav2 nodes are configured but
            # inactive; STARTUP would try to configure them a second time.
            if self._navigation_pause_state == 'confirmed':
                request.command = ManageLifecycleNodes.Request.RESUME
            self._navigation_may_be_active = True
            self._navigation_pause_state = None
        self._future = client.call_async(request)

    def _request_navigation_pause(self):
        """Pause Nav2 after a failure that may follow a STARTUP request."""
        # The lifecycle manager serves requests one at a time, so a PAUSE
        # sent while STARTUP is still running takes effect after it.
        if not self._navigation_client.service_is_ready():
            self._navigation_pause_state = 'service_unavailable'
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        self._pause_future = self._navigation_client.call_async(request)
        self._navigation_pause_state = 'requested'
        self.get_logger().error(
            'requested Nav2 lifecycle pause after failed activation')

    def _tick_navigation_pause(self):
        if self._navigation_pause_state == 'service_unavailable':
            self._request_navigation_pause()
            return
        if (self._navigation_pause_state != 'requested'
                or not self._pause_future.done()):
            return
        try:
            response = self._pause_future.result()
            paused = bool(getattr(response, 'success', response is not None))
        except Exception as error:
            self.get_logger().error(f'Nav2 pause request failed: {error}')
            paused = False
        self._pause_future = None
        if paused:
            self._navigation_pause_state = 'confirmed'
            self._navigation_may_be_active = False
        else:
            # Reported, not retried: the lifecycle manager refuses PAUSE
            # when STARTUP itself failed and the nodes never became active.
            self._navigation_pause_state = 'failed'
            self.get_logger().error('Nav2 lifecycle pause was not confirmed')

    def _future_succeeded(self):
        if self._future is None or not self._future.done():
            return None
        try:
            response = self._future.result()
        except Exception as error:
            self.get_logger().error(f'service call failed: {error}')
            self._future = None
            return False
        self._future = None
        if hasattr(response, 'success'):
            return bool(response.success)
        return response is not None

    def _transition(self, state):
        self._state = state
        self._state_started = time.monotonic()
        if state == State.CANDIDATE_READY:
            self._candidate_invalid_since = None
            if self._latest_amcl_pose is not None:
                pose = self._latest_amcl_pose.pose.pose
                self._candidate_anchor_pose = (
                    pose.position.x,
                    pose.position.y,
                    quaternion_yaw(pose.orientation),
                )
        self._future = None
        self._publish_status(force=True)
        self.get_logger().info(f'automatic localization: {state.value}')

    def _ensure_command_publisher(self):
        if self._command_publisher is None:
            self._command_publisher = self.create_publisher(
                Twist, self._command_topic, 10
            )

    def _fail(self, reason, manual_recovery=False):
        if self._state in (State.SAFE_STOP, State.WAIT_MANUAL_POSE):
            return
        self._failure_reason = reason
        self._manual_recovery_allowed = manual_recovery
        self._cancel_worker()
        if self._strategy == Strategy.LEGACY_FULL_ROTATION:
            self._ensure_command_publisher()
        if (self._probe is not None
                and self._probe.operation != MotionOperation.RELEASE):
            self._probe.stop()
            self._refresh_probe_lease()      # do not wait for the next tick
        self.get_logger().error(reason)
        if self._navigation_may_be_active:
            self._request_navigation_pause()
        self._transition(State.SAFE_STOP)

    def _publish_zero(self):
        if self._command_publisher is not None:
            self._command_publisher.publish(Twist())

    def _publish_rotation(self):
        if self._command_publisher is None:
            return
        command = Twist()
        command.angular.z = self._parameter('rotation_speed_rad_s')
        self._command_publisher.publish(command)

    def _record_tf(self, now):
        try:
            stamped = self._tf_buffer.lookup_transform(
                'map', 'odom', Time()
            )
            transform = stamped.transform
        except TransformException:
            return
        stamp = Time.from_msg(stamped.header.stamp).nanoseconds
        # AMCL stamps map->odom transform_tolerance ahead of its scan.  Judge
        # freshness and clock skew on the scan time it was computed from.
        source = stamp - round(
            self._parameter('map_odom_postdate_sec') * 1e9)
        age = (self.get_clock().now().nanoseconds - source) / 1e9
        self._timing['map_odom'] = {
            'source_stamp_ns': source, 'source_age_sec': age}
        if (stamp <= (self._last_tf_stamp or 0)
                or age > self._parameter('sensor_freshness_sec')
                or age < -self._parameter('max_tf_future_sec')):
            return
        self._last_tf_stamp = stamp
        # AMCL may publish a new map->odom only when a forced no-motion update
        # completes (roughly once per second here).  Preserve those distinct
        # samples across the configured localization-evidence window so a
        # multi-second stability span can form.  _quality_passes() separately
        # requires the newest TF sample to meet the tighter sensor-freshness
        # limit at the instant a candidate is accepted.
        if not self._localization_evidence_recent(
                self._last_map_odom_time, now):
            self._tf_window.clear()
        sample = (
            transform.translation.x,
            transform.translation.y,
            quaternion_yaw(transform.rotation),
        )
        self._tf_window.append((now, sample))
        self._last_map_odom_time = now
        window = self._parameter('tf_window_sec')
        # Retain the one sample that brackets the requested window.  Dropping
        # every sample older than the boundary makes the later `>= window`
        # coverage check impossible with a discretely sampled timer.
        trim_time_window(self._tf_window, now, window)

    def _request_nomotion_update(self, now):
        if (
            self._nomotion_future is not None
            and self._nomotion_future.done()
        ):
            self._nomotion_future = None
        if self._nomotion_future is not None:
            return
        if not self._nomotion_update_client.service_is_ready():
            return
        if (
            self._last_nomotion_request is not None
            and now - self._last_nomotion_request
            < self._parameter('nomotion_update_interval_sec')
        ):
            return
        self._nomotion_future = self._nomotion_update_client.call_async(
            Empty.Request()
        )
        self._last_nomotion_request = now

    def _localization_evidence_recent(self, value, now):
        return (
            value is not None
            and now - value <= self._parameter(
                'localization_evidence_freshness_sec'
            )
        )

    def _reject_quality(self, reason):
        self._quality_failure = reason
        return False

    def _quality_passes(self, now):
        if not self._stopped(now):
            return self._reject_quality('NOT_STATIONARY')
        if self._latest_amcl_pose is None:
            return self._reject_quality('NO_AMCL_POSE')
        if not self._localization_evidence_recent(
                self._last_amcl_time, now):
            return self._reject_quality('STALE_AMCL')
        pose = self._latest_amcl_pose.pose.pose
        if self._manual_reference_pose is not None:
            manual_translation_drift = math.hypot(
                pose.position.x - self._manual_reference_pose[0],
                pose.position.y - self._manual_reference_pose[1],
            )
            manual_yaw_drift = abs(angular_difference(
                quaternion_yaw(pose.orientation),
                self._manual_reference_pose[2],
            ))
            if (
                manual_translation_drift > self._parameter(
                    'max_manual_pose_drift_m')
                or manual_yaw_drift > self._parameter(
                    'max_manual_pose_drift_yaw_rad')
            ):
                return self._reject_quality('MANUAL_POSE_DRIFT')
        if self._candidate_anchor_pose is not None:
            candidate_translation_drift = math.hypot(
                pose.position.x - self._candidate_anchor_pose[0],
                pose.position.y - self._candidate_anchor_pose[1],
            )
            candidate_yaw_drift = abs(angular_difference(
                quaternion_yaw(pose.orientation),
                self._candidate_anchor_pose[2],
            ))
            if (
                candidate_translation_drift > self._parameter(
                    'max_candidate_pose_drift_m')
                or candidate_yaw_drift > self._parameter(
                    'max_candidate_pose_drift_yaw_rad')
            ):
                return self._reject_quality('CANDIDATE_POSE_DRIFT')
        if self._state == State.STOP_AND_VERIFY and self._search_best:
            translation_drift = math.hypot(
                pose.position.x - self._search_best['x'],
                pose.position.y - self._search_best['y'],
            )
            yaw_drift = abs(angular_difference(
                quaternion_yaw(pose.orientation),
                self._search_best['yaw'],
            ))
            if (
                translation_drift > self._parameter(
                    'max_post_search_translation_drift_m')
                or yaw_drift > self._parameter(
                    'max_post_search_yaw_drift_rad')
            ):
                return self._reject_quality('POST_SEARCH_POSE_DRIFT')
        if not self._recent(self._last_scan_time, now):
            return self._reject_quality('STALE_SCAN')
        if not self._recent(self._last_score_time, now):
            return self._reject_quality('SCAN_TF_UNAVAILABLE_OR_STALE_SCORE')
        if not self._recent(self._last_map_odom_time, now):
            return self._reject_quality('STALE_TF')
        if not self._localization_evidence_recent(
                self._last_particle_time, now):
            return self._reject_quality('STALE_PARTICLES')
        if not covariance_quality(
            self._latest_amcl_pose,
            self._parameter('max_amcl_xy_std'),
            self._parameter('max_amcl_yaw_std'),
        ):
            return self._reject_quality('COVARIANCE_TOO_LARGE')
        if self._scan_score < self._parameter('min_scan_map_score'):
            return self._reject_quality('SCAN_MISMATCH')
        if self._valid_scan_beams < self._parameter('min_valid_scan_beams'):
            return self._reject_quality('INSUFFICIENT_KNOWN_BEAMS')
        if self._sampled_scan_beams <= 0:
            return self._reject_quality('NO_SAMPLED_BEAMS')
        if (
            self._valid_scan_beams / self._sampled_scan_beams
            < self._parameter('min_scan_map_coverage')
        ):
            return self._reject_quality('INSUFFICIENT_COVERAGE')
        if self._particle_concentration < self._parameter(
                'min_particle_concentration'):
            return self._reject_quality('PARTICLES_DISPERSED')
        if not self._tf_window:
            return self._reject_quality('NO_TF_WINDOW')
        if now - self._tf_window[0][0] < self._parameter('tf_window_sec'):
            return self._reject_quality('TF_WINDOW_TOO_SHORT')
        samples = [entry[1] for entry in self._tf_window]
        x_span, y_span, yaw_span = planar_window_span(samples)
        translation_span = math.hypot(x_span, y_span)
        stable = (
            translation_span
            <= self._parameter('max_tf_translation_span')
            and yaw_span <= self._parameter('max_tf_yaw_span')
        )

        self._quality_failure = '' if stable else 'TF_UNSTABLE'
        return stable

    def _candidate_quality_passes(self, now):
        """
        Continuously validate an already qualified stationary candidate.

        Entry to CANDIDATE_READY already required fresh AMCL, particles and a
        full stable TF window.  AMCL does not promise to republish those while
        stopped.  Rechecking therefore uses each new scan scored with its
        exact-time transform, plus odometry stop and pose-drift gates.
        """
        if not self._stopped(now):
            return self._reject_quality('NOT_STATIONARY')
        if not self._recent(self._last_safety_scan_time, now):
            return self._reject_quality('STALE_SAFETY_SCAN')
        if not self._recent(self._last_scan_time, now):
            return self._reject_quality('STALE_SCAN')
        if not self._recent(self._last_score_time, now):
            return self._reject_quality('SCAN_TF_UNAVAILABLE_OR_STALE_SCORE')
        if self._latest_amcl_pose is not None and self._candidate_anchor_pose:
            pose = self._latest_amcl_pose.pose.pose
            translation_drift = math.hypot(
                pose.position.x - self._candidate_anchor_pose[0],
                pose.position.y - self._candidate_anchor_pose[1],
            )
            yaw_drift = abs(angular_difference(
                quaternion_yaw(pose.orientation),
                self._candidate_anchor_pose[2],
            ))
            if (
                translation_drift > self._parameter(
                    'max_candidate_pose_drift_m')
                or yaw_drift > self._parameter(
                    'max_candidate_pose_drift_yaw_rad')
            ):
                return self._reject_quality('CANDIDATE_POSE_DRIFT')
        if self._scan_score < self._parameter('min_scan_map_score'):
            return self._reject_quality('SCAN_MISMATCH')
        if self._valid_scan_beams < self._parameter('min_valid_scan_beams'):
            return self._reject_quality('INSUFFICIENT_KNOWN_BEAMS')
        if self._sampled_scan_beams <= 0:
            return self._reject_quality('NO_SAMPLED_BEAMS')
        if (
            self._valid_scan_beams / self._sampled_scan_beams
            < self._parameter('min_scan_map_coverage')
        ):
            return self._reject_quality('INSUFFICIENT_COVERAGE')
        self._quality_failure = ''
        return True

    def _verify_quality(self, now):
        self._request_nomotion_update(now)
        self._record_tf(now)
        if self._quality_passes(now):
            if self._quality_since is None:
                self._quality_since = now
            if now - self._quality_since >= self._parameter(
                    'quality_hold_sec'):
                self._transition(
                    State.CANDIDATE_READY if self._validation_only
                    else State.START_NAVIGATION
                )
        else:
            self._quality_since = None

    def _start_global_search(self):
        scans = list(self._stationary_search_scans)
        tolerance_cells = math.ceil(
            self._parameter('scan_match_tolerance_m')
            / self._latest_map.info.resolution
        )
        self._search_future = self._search_executor.submit(
            deterministic_global_search,
            self._latest_map,
            scans,
            self._parameter('occupied_threshold'),
            tolerance_cells,
            self._parameter('global_search_position_step_m'),
            self._parameter('global_search_yaw_step_rad'),
            self._parameter('global_search_coarse_beams'),
            self._parameter('scan_score_max_beams'),
            self._parameter('global_search_refine_count'),
            self._parameter('global_search_fine_position_step_m'),
            self._parameter('global_search_fine_yaw_step_rad'),
            self._parameter('global_search_fine_position_radius_m'),
            self._parameter('global_search_fine_yaw_radius_rad'),
            self._parameter('global_search_fine_seed_count'),
            self._parameter('global_search_final_position_step_m'),
            self._parameter('global_search_final_yaw_step_rad'),
            self._parameter('global_search_final_position_radius_m'),
            self._parameter('global_search_final_yaw_radius_rad'),
        )

    def _accept_global_search(self):
        try:
            candidates = self._search_future.result()
        except Exception as error:
            self.get_logger().error(f'global pose search failed: {error}')
            return False, 'GLOBAL_SEARCH_FAILED'
        if not candidates:
            return False, 'NO_GLOBAL_CANDIDATE'
        best = candidates[0]
        self._search_best = best
        separation = self._parameter('global_search_ambiguity_distance_m')
        runner_up = next((
            candidate for candidate in candidates[1:]
            if math.hypot(
                candidate['x'] - best['x'],
                candidate['y'] - best['y']) >= separation
        ), None)
        self._search_runner_up = runner_up
        if (
            best['score'] < self._parameter('min_scan_map_score')
            or best['coverage'] < self._parameter('min_scan_map_coverage')
            or best['known'] < self._parameter('min_valid_scan_beams')
            or best['wall_conflict_ratio'] > self._parameter(
                'global_search_max_wall_conflict_ratio')
        ):
            return False, 'GLOBAL_CANDIDATE_MISMATCH'
        if (
            runner_up is not None
            and best['score'] - runner_up['score']
            < self._parameter('global_search_min_score_margin')
        ):
            return False, 'AMBIGUOUS_LOCATION'

        self._publish_global_seed(best['x'], best['y'], best['yaw'])
        return True, ''

    def _publish_global_seed(self, x, y, yaw):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        seed_xy_std = self._parameter('global_search_seed_xy_std_m')
        seed_yaw_std = self._parameter('global_search_seed_yaw_std_rad')
        message.pose.covariance[0] = seed_xy_std ** 2
        message.pose.covariance[7] = seed_xy_std ** 2
        message.pose.covariance[35] = seed_yaw_std ** 2
        self._reset_evidence()
        self._requested_initial_pose = (x, y, yaw)
        self._manual_reference_pose = None
        self._awaiting_amcl_initial_pose = True
        self._pose_seed_publisher.publish(message)

    def _tick(self):
        now = time.monotonic()
        # LaserScan and odom->base_footprint arrive at similar rates.  Retry a
        # short bounded scan queue so exact-time scoring waits for the matching
        # TF rather than falling back to the newest transform or discarding the
        # scan permanently.
        if self._pending_scans:
            self._score_pending_scans()
        self._refresh_probe_lease()
        state_age = now - self._state_started
        if self._state == State.WAIT_FOR_START:
            pass

        elif self._state == State.WAIT_SENSORS:
            if now - self._startup_started > self._parameter(
                    'startup_timeout_sec'):
                self._fail('timed out waiting for stable FAST-LIO and scan')
            elif self._sensors_ready(now):
                if self._sensors_ready_since is None:
                    self._sensors_ready_since = now
                elif now - self._sensors_ready_since >= self._parameter(
                        'sensor_stable_sec'):
                    self._transition(State.START_LOCALIZATION)
            else:
                self._sensors_ready_since = None

        elif self._state == State.START_LOCALIZATION:
            if not self._localization_client.service_is_ready():
                if state_age > self._parameter('service_timeout_sec'):
                    self._fail('localization lifecycle service unavailable')
            elif self._future is None:
                self._call_lifecycle(self._localization_client)
            else:
                result = self._future_succeeded()
                if (
                    result is None
                    and state_age > self._parameter('service_timeout_sec')
                ):
                    self._fail('localization lifecycle startup timed out')
                elif result is False:
                    self._fail('localization lifecycle startup failed')
                elif result is True:
                    if self._stationary_only:
                        self._transition(State.WAIT_MANUAL_POSE)
                    elif self._strategy != Strategy.LEGACY_FULL_ROTATION:
                        # Both new strategies start stationary; only
                        # segmented_rotation may later probe via the guard.
                        self._begin_confined_session(now)
                    else:
                        # The explainable map-wide search below now owns global
                        # pose discovery.  Do not also spread AMCL particles
                        # across the map: that creates a misleading provisional
                        # RViz pose and consumes the scan-projection CPU budget.
                        self._reset_evidence()
                        self._rotation_progress = 0.0
                        self._last_odom_yaw = None
                        self._rotation_data_stale_since = None
                        self._rotation_obstacle_since = None
                        self._transition(State.ROTATE_AND_SCORE)

        elif self._state == State.GLOBAL_LOCALIZATION:
            ready = (
                self._latest_map is not None
                and self._global_localization_client.service_is_ready()
            )
            if not ready:
                if state_age > self._parameter('service_timeout_sec'):
                    self._fail(
                        'map or AMCL global-localization service unavailable',
                        manual_recovery=self._latest_map is not None,
                    )
            elif self._future is None:
                self._future = self._global_localization_client.call_async(
                    Empty.Request()
                )
            else:
                result = self._future_succeeded()
                if (
                    result is None
                    and state_age > self._parameter('service_timeout_sec')
                ):
                    self._fail(
                        'AMCL global localization request timed out',
                        manual_recovery=True,
                    )
                elif result is False:
                    self._fail(
                        'AMCL global localization request failed',
                        manual_recovery=True,
                    )
                elif result is True:
                    self._rotation_progress = 0.0
                    self._last_odom_yaw = None
                    self._rotation_data_stale_since = None
                    self._rotation_obstacle_since = None
                    self._transition(State.ROTATE_AND_SCORE)

        elif self._state == State.ROTATE_AND_SCORE:
            data_fresh = all((
                self._recent(self._last_odom_time, now),
                self._recent(self._last_scan_time, now),
                self._recent(self._last_safety_scan_time, now),
            ))
            if not data_fresh:
                self._publish_zero()
                if self._rotation_data_stale_since is None:
                    self._rotation_data_stale_since = now
                stale_duration = now - self._rotation_data_stale_since
                if stale_duration > self._parameter(
                        'rotation_data_grace_sec'):
                    odom_age = (
                        math.inf if self._last_odom_time is None
                        else now - self._last_odom_time
                    )
                    scan_age = (
                        math.inf if self._last_scan_time is None
                        else now - self._last_scan_time
                    )
                    safety_scan_age = (
                        math.inf if self._last_safety_scan_time is None
                        else now - self._last_safety_scan_time
                    )
                    self._fail(
                        'odometry, localization scan, or safety scan became '
                        'stale while rotating: '
                        f'odom_age={odom_age:.3f}s '
                        f'scan_age={scan_age:.3f}s '
                        f'safety_scan_age={safety_scan_age:.3f}s'
                    )
            else:
                self._rotation_data_stale_since = None

            if self._state != State.ROTATE_AND_SCORE:
                self._publish_zero()
                self._publish_status()
                return

            # Once the requested full turn is complete, no further motion is
            # needed.  Stop and retain the evidence even if the final safety
            # scan contains a close return; the stationary search cannot move
            # the robot toward that obstacle.
            if (
                data_fresh
                and self._rotation_progress >= self._parameter(
                    'rotation_target_rad')
            ):
                self._publish_zero()
                self._tf_window.clear()
                self._quality_since = None
                self._stationary_search_scans.clear()
                self._search_future = None
                self._search_best = None
                self._search_runner_up = None
                self._transition(State.SEARCH_GLOBAL_POSE)
                return

            obstacle_close = (
                data_fresh
                and self._scan_min_range < self._parameter(
                    'min_rotation_clearance_m')
            )
            if obstacle_close:
                self._publish_zero()
                if self._rotation_obstacle_since is None:
                    self._rotation_obstacle_since = now
                obstacle_duration = now - self._rotation_obstacle_since
                if obstacle_duration > self._parameter(
                        'rotation_obstacle_confirmation_sec'):
                    self._fail(
                        'obstacle remained too close for automatic rotation: '
                        f'minimum_scan_range={self._scan_min_range:.3f}m'
                    )
            else:
                self._rotation_obstacle_since = None

            if self._state != State.ROTATE_AND_SCORE:
                self._publish_zero()
                self._publish_status()
                return

            if (
                data_fresh
                and not obstacle_close
                and state_age > self._parameter('rotation_timeout_sec')
            ):
                self._fail('automatic localization rotation timed out')
            elif data_fresh and not obstacle_close:
                self._publish_rotation()

        elif self._state == State.SEARCH_GLOBAL_POSE:
            self._publish_zero()
            if (
                self._search_future is None
                and self._stopped(now)
                and state_age >= self._parameter('zero_command_duration_sec')
                and len(self._stationary_search_scans) >= self._parameter(
                    'global_search_scan_count')
            ):
                self._start_global_search()
            elif self._search_future is not None and self._search_future.done():
                accepted, reason = self._accept_global_search()
                if accepted:
                    self._transition(State.STOP_AND_VERIFY)
                else:
                    self._quality_failure = reason
                    self._fail(
                        'deterministic global pose search rejected',
                        manual_recovery=True,
                    )
            elif state_age > self._parameter('global_search_timeout_sec'):
                self._quality_failure = 'GLOBAL_SEARCH_TIMEOUT'
                self._fail(
                    'deterministic global pose search timed out',
                    manual_recovery=True,
                )

        elif self._state in CONFINED_STATES:
            self._publish_zero()
            self._tick_confined(now, state_age)

        elif self._state == State.STOP_AND_VERIFY:
            self._publish_zero()
            stopped = (
                self._stopped(now)
                and state_age >= self._parameter(
                    'zero_command_duration_sec')
            )
            if stopped:
                self._verify_quality(now)
            else:
                self._quality_since = None
                self._quality_failure = 'NOT_STATIONARY'
            if (
                self._state == State.STOP_AND_VERIFY
                and state_age > self._parameter('verification_timeout_sec')
            ):
                self._fail(
                    'automatic localization confidence is too low',
                    manual_recovery=True,
                )

        elif self._state == State.VERIFY_MANUAL_POSE:
            self._publish_zero()
            self._verify_quality(now)
            if (
                self._state == State.VERIFY_MANUAL_POSE
                and state_age > self._parameter('verification_timeout_sec')
            ):
                self._failure_reason = (
                    'AMCL did not acknowledge the new manual pose'
                    if self._awaiting_amcl_initial_pose else
                    'manual pose did not pass localization confidence checks'
                )
                self._transition(State.WAIT_MANUAL_POSE)

        elif self._state == State.CANDIDATE_READY:
            self._publish_zero()
            # Entry already required fresh AMCL/particle evidence.  Repeated
            # no-motion updates make AMCL resample the same stationary scene
            # indefinitely and can walk map->odom away from the independently
            # measured geometric optimum.  Natural odometry motion will make
            # AMCL update again after navigation is enabled; while this
            # validation-only candidate is stopped, live exact-time scan
            # scoring is the continuing observation check.
            self._record_tf(now)
            if self._candidate_quality_passes(now):
                self._candidate_invalid_since = None
            elif self._candidate_invalid_since is None:
                self._candidate_invalid_since = now
            elif now - self._candidate_invalid_since > self._parameter(
                    'candidate_recheck_grace_sec'):
                failure = self._quality_failure
                self._fail(
                    'candidate localization evidence became invalid: '
                    f'{failure}',
                    manual_recovery=True,
                )

        elif self._state == State.START_NAVIGATION:
            if self._validation_only:
                self._publish_zero()
                self._transition(State.CANDIDATE_READY)
                return
            self._publish_zero()
            if state_age < self._parameter('zero_command_duration_sec'):
                pass
            elif self._command_publisher is not None:
                self.destroy_publisher(self._command_publisher)
                self._command_publisher = None
            elif self._probe is not None and not self._guard_released(now):
                # Nav2 never activates while the guard can still publish.
                if self._probe.operation != MotionOperation.RELEASE:
                    self._probe.release()
                    self._refresh_probe_lease()
                if state_age > self._parameter('service_timeout_sec'):
                    self._fail('motion guard did not release control')
            elif not self._navigation_client.service_is_ready():
                if state_age > self._parameter('service_timeout_sec'):
                    self._fail('navigation lifecycle service unavailable')
            elif self._future is None:
                self._call_lifecycle(self._navigation_client)
            else:
                result = self._future_succeeded()
                if (
                    result is None
                    and state_age > self._parameter('service_timeout_sec')
                ):
                    self._fail('navigation lifecycle startup timed out')
                elif result is False:
                    self._fail('navigation lifecycle startup failed')
                elif result is True:
                    self._failure_reason = ''
                    self._transition(State.READY)

        elif self._state == State.SAFE_STOP:
            self._publish_zero()
            self._tick_navigation_pause()
            stopped_long_enough = (
                state_age >= self._parameter('zero_command_duration_sec')
                and self._stopped(now)
            )
            if stopped_long_enough:
                if self._manual_recovery_allowed:
                    self._transition(State.WAIT_MANUAL_POSE)
                else:
                    self._transition(State.FAULT_STOPPED)

        elif self._state == State.WAIT_MANUAL_POSE:
            self._publish_zero()
            self._tick_navigation_pause()

        elif self._state == State.FAULT_STOPPED:
            self._publish_zero()
            self._tick_navigation_pause()

        self._publish_status()

    def _tf_se2(self, target, source, stamp_ns):
        """Planar transform at exactly ``stamp_ns``; LookupError if absent."""
        try:
            transform = self._tf_buffer.lookup_transform(
                target, source, Time(nanoseconds=stamp_ns)).transform
        except TransformException as error:
            raise LookupError(str(error)) from error
        return SE2(transform.translation.x, transform.translation.y,
                   quaternion_yaw(transform.rotation))

    def _take_keyframes(self, role, now):
        """
        Turn buffered stopped scans into keyframes of one role.

        Scans must be newer than every frame already used, so HOLDOUT frames
        are always captured after the TRAIN frames.  A scan whose source-time
        TF has not arrived waits for at most the sensor freshness limit.
        """
        freshness = self._parameter('sensor_freshness_sec')
        pending = deque(maxlen=self._confined_scans.maxlen)
        for message, received in self._confined_scans:
            scan = snapshot_scan(message)
            if scan.stamp_ns <= self._last_keyframe_stamp_ns:
                continue
            frame = make_keyframe(
                scan, self._tf_se2, self._session, self._view_id, role,
                self._next_keyframe_id, received)
            if isinstance(frame, Reject):
                if now - received <= freshness:
                    pending.append((message, received))
                continue
            self._keyframes.append(frame)
            self._next_keyframe_id += 1
            self._last_keyframe_stamp_ns = scan.stamp_ns
        self._confined_scans = pending

    def _frames(self, role, now):
        return collect_keyframes(
            self._keyframes, role, self._confined['max_views'],
            self._parameter('session_timeout_sec'), now, self._session)

    def _role_complete(self, role, now):
        wanted = self._confined[
            'train_frames_per_view' if role == FrameRole.TRAIN
            else 'holdout_frames_per_view']
        frames = self._frames(role, now)
        if isinstance(frames, Reject):
            if frames.reason == RejectReason.ODOM_JUMP:
                self._reject(frames.reason, frames.detail)
            return None
        # Each new view must contribute its own TRAIN frames.
        current = [frame for frame in frames
                   if role != FrameRole.TRAIN
                   or frame.view_id == self._view_id]
        return frames if len(current) >= wanted else None

    def _gather(self, role, now):
        """Collect frames only while verifiably stopped."""
        if not self._stopped(now):
            # Frames must come from one stationary view; discard partials.
            self._keyframes = [
                frame for frame in self._keyframes if frame.role != role]
            self._confined_scans.clear()
            return None
        self._take_keyframes(role, now)
        return self._role_complete(role, now)

    def _tick_confined(self, now, state_age):
        if now - self._session_started > self._parameter(
                'session_timeout_sec'):
            self._reject(RejectReason.SEARCH_INCOMPLETE, 'session timeout')
            return
        if self._latest_grid is None:
            if state_age > self._parameter('collect_static_timeout_sec'):
                self._reject(RejectReason.SENSOR_STALE, 'no map')
            return

        if self._state == State.COLLECT_STATIC:
            train = self._gather(FrameRole.TRAIN, now)
            if self._state != State.COLLECT_STATIC:
                return
            if train is None:
                if state_age > self._parameter('collect_static_timeout_sec'):
                    self._reject(RejectReason.SENSOR_STALE,
                                 'no qualified stationary TRAIN frames')
                return
            self._worker.submit(
                (self._session, self._map_hash), run_search_job,
                self._latest_grid, train, self._search_config(),
                self._parameter('search_timeout_sec'))
            self._transition(State.SEARCH_MULTI_VIEW)

        elif self._state == State.SEARCH_MULTI_VIEW:
            outcome = self._worker.poll()
            if outcome is None:
                # The worker owns its own deadline; this is the backstop.
                if state_age > self._parameter('search_timeout_sec') + 5.0:
                    self._reject(RejectReason.SEARCH_INCOMPLETE,
                                 'search worker deadline')
                return
            token, status, result = outcome
            if token != (self._session, self._map_hash):
                self._reject(RejectReason.MAP_CHANGED, 'stale search result')
            elif status != 'ok':
                self._reject(RejectReason.SEARCH_INCOMPLETE, str(result))
            elif not result.complete:
                self._search_result = result
                self._reject_or_probe(result.reason, now)
            else:
                self._search_result = result
                self._validation_submitted = False
                self._transition(State.VERIFY_HYPOTHESES)

        elif self._state == State.VERIFY_HYPOTHESES:
            if not self._validation_submitted:
                holdout = self._gather(FrameRole.HOLDOUT, now)
                if self._state != State.VERIFY_HYPOTHESES:
                    return
                if holdout is None:
                    if state_age > self._parameter(
                            'collect_static_timeout_sec'):
                        self._reject(RejectReason.SENSOR_STALE,
                                     'no qualified HOLDOUT frames')
                    return
                self._worker.submit(
                    (self._session, self._map_hash), run_validation_job,
                    self._latest_grid, self._search_result,
                    self._frames(FrameRole.TRAIN, now), holdout,
                    self._search_config(), self._validation_thresholds(),
                    self._parameter('verification_timeout_sec'))
                self._validation_submitted = True
                return
            outcome = self._worker.poll()
            if outcome is None:
                if state_age > self._parameter(
                        'collect_static_timeout_sec') + self._parameter(
                        'verification_timeout_sec') + 5.0:
                    self._reject(RejectReason.SEARCH_INCOMPLETE,
                                 'validation worker deadline')
                return
            token, status, decision = outcome
            if token != (self._session, self._map_hash):
                self._reject(RejectReason.MAP_CHANGED,
                             'stale validation result')
            elif status != 'ok':
                self._reject(RejectReason.SEARCH_INCOMPLETE, str(decision))
            elif not decision.accepted:
                self._reject_or_probe(decision.reason, now)
            else:
                self._accept_confined(decision.winner, now)

        elif self._state == State.PLAN_PROBE:
            self._plan_probe(now, state_age)

        elif self._state == State.EXECUTE_PROBE:
            status = self._fresh_guard_status(now)
            if status is None:
                self._reject(RejectReason.CONTROL_CONFLICT,
                             'motion guard status missing')
            elif not self._probe.owns(status):
                if state_age > self._parameter('guard_status_freshness_sec'):
                    self._reject(RejectReason.CONTROL_CONFLICT,
                                 'motion guard ignored the probe request')
            elif status.reason and status.state != GuardState.ROTATING:
                self._reject(status.reason, 'refused by motion guard')
            elif status.state == GuardState.ROTATING:
                self._probe_moved = True
            elif self._probe_moved:
                self._probe.stop()
                self._refresh_probe_lease()
                self._transition(State.SETTLE_PROBE)
            elif state_age > self._parameter('probe_motion_timeout_sec'):
                self._reject(RejectReason.MOTION_BUDGET_EXHAUSTED,
                             'probe never started')

        elif self._state == State.SETTLE_PROBE:
            status = self._fresh_guard_status(now)
            settled = (status is not None and self._probe.owns(status)
                       and status.stopped and self._stopped(now))
            if settled and state_age >= self._parameter('probe_settle_sec'):
                self._start_next_view()
            elif state_age > self._parameter('probe_settle_timeout_sec'):
                self._fail('probe rotation did not settle',
                           manual_recovery=False)

    def _reject_or_probe(self, reason, now):
        """Ambiguity in segmented_rotation plans a probe; else reject."""
        if not self._segmented or reason not in PROBE_RESOLVABLE:
            self._reject(reason)
            return
        counts = self._probe_hypothesis_counts
        result = self._search_result
        counts.append(None if result is None else len(result.hypotheses))
        # Two probes in a row without fewer hypotheses: stop exploring.
        if len(counts) >= 3 and None not in counts[-3:] and (
                counts[-1] >= counts[-2] >= counts[-3]):
            self._reject(RejectReason.AMBIGUOUS_LOCATION,
                         'probes did not reduce the hypotheses')
            return
        self._probe_reason = reason
        self._transition(State.PLAN_PROBE)

    def _plan_probe(self, now, state_age):
        """Read-only choice of the next probe; the guard re-checks it."""
        if self._motion_policy != MotionPolicy.GUARDED:
            self._reject(RejectReason.PROFILE_INVALID, 'motion_policy=forbid')
            return
        if state_age > self._parameter('probe_plan_timeout_sec'):
            self._reject(RejectReason.SENSOR_STALE, 'probe planning timed out')
            return
        if self._fresh_guard_status(now) is None:
            if state_age > self._parameter('guard_status_freshness_sec'):
                self._reject(RejectReason.CONTROL_CONFLICT,
                             'motion guard is not responding')
            return
        scan = self._latest_safety_scan
        if scan is None or self._last_odom_pose is None:
            return
        try:
            sensor = self._tf_se2('odom', scan.header.frame_id,
                                  Time.from_msg(scan.header.stamp).nanoseconds)
            base = self._tf_se2('odom', 'base_footprint',
                                Time.from_msg(scan.header.stamp).nanoseconds)
            evidence = evidence_from_scan(
                scan, sensor, (base.x, base.y), frame_id='odom',
                deadline=time.monotonic() + 0.5)
            decisions = [
                evaluate_localization_rotation(
                    evidence, self._footprint, base, angle,
                    profile=self._motion_profile, hashes=self._probe_hashes,
                    config=self._rotation_gate,
                    attestation=self._attestation, session=self._session,
                    now_mono=now).decision
                for angle in self._confined['probe_angles_rad']]
        except (LookupError, ContractError):
            return                     # retried until the plan timeout
        self._probe_unknown_cells = min(d.unknown_cells for d in decisions)
        views = self._frames(FrameRole.TRAIN, now)
        headings = sorted({frame.T_odom_base.yaw for frame in views}
                          if not isinstance(views, Reject) else set())
        choice, reason = choose_probe(decisions, headings, base.yaw)
        if choice is None:
            self._reject(reason, 'no admissible probe rotation')
            return
        self._probe.rotate(choice.delta_yaw,
                           float(self._parameter('probe_speed_rad_s')),
                           profile_hash(self._motion_profile))
        self._refresh_probe_lease()
        self._probe_moved = False
        self._transition(State.EXECUTE_PROBE)

    def _start_next_view(self):
        """After a settled probe: drop used HOLDOUT frames, collect anew."""
        self._view_id += 1
        self._keyframes = [frame for frame in self._keyframes
                           if frame.role == FrameRole.TRAIN]
        self._confined_scans.clear()
        self._search_result = None
        self._validation_submitted = False
        self._transition(State.COLLECT_STATIC)

    def _accept_confined(self, winner, now):
        """Seed AMCL with the winner moved to the current odometry pose."""
        if not self._stopped(now) or self._last_odom_pose is None:
            self._reject(RejectReason.SENSOR_STALE,
                         'not stopped with fresh odometry at seed time')
            return
        reference = self._frames(FrameRole.TRAIN, now)
        if isinstance(reference, Reject) or not reference:
            self._reject(reference.reason if isinstance(reference, Reject)
                         else RejectReason.SENSOR_STALE)
            return
        seed = seed_pose_at_current_time(
            SE2(winner.x, winner.y, winner.yaw), reference[0].T_odom_base,
            SE2(*self._last_odom_pose))
        self._search_best = {
            'x': seed.x, 'y': seed.y, 'yaw': seed.yaw,
            'score': winner.score, 'coverage': winner.coverage,
            'wall_conflict_ratio': winner.conflict,
            'cluster_id': winner.cluster_id,
            'support_bounds': list(winner.support_bounds),
        }
        self._publish_global_seed(seed.x, seed.y, seed.yaw)
        if self._probe is not None:
            # No probe follows an accepted candidate: hand control back.
            self._probe.release()
            self._refresh_probe_lease()
        self._transition(State.STOP_AND_VERIFY)

    def _confined_status(self):
        """Issue #13 status fields; existing fields are left unchanged."""
        result = self._search_result
        reason = self._reject_reason
        text, manual = (REJECT_REASON_TEXT[RejectReason(reason)]
                        if reason else ('', None))
        legacy = self._strategy == Strategy.LEGACY_FULL_ROTATION
        guard = getattr(self, '_guard_status', None)
        guard_state = None if guard is None else guard[0].state.value
        guard_travel = 0.0 if guard is None else guard[0].abs_travel_rad
        return {
            'schema_version': SCHEMA_VERSION,
            'strategy': self._strategy.value,
            'motion_policy': self._motion_policy.value,
            'session': self._session,
            'search_complete': None if result is None else result.complete,
            'hypothesis_count': (
                None if result is None else len(result.hypotheses)),
            'ambiguity_reason': reason,
            'reject_reason_text': text,
            'manual_pose_allowed': manual,
            'motion_guard_state': guard_state,
            'unknown_sweep_cells': getattr(
                self, '_probe_unknown_cells', None),
            'total_abs_yaw': (
                round(self._rotation_progress, 3) if legacy
                else round(guard_travel, 3)),
            'map_hash': self._map_hash,
        }

    def _publish_status(self, force=False):
        now = time.monotonic()
        amcl_xy_std = None
        amcl_yaw_std = None
        if self._latest_amcl_pose is not None:
            covariance = self._latest_amcl_pose.pose.covariance
            xy_variance = max(covariance[0], covariance[7])
            if math.isfinite(xy_variance) and xy_variance >= 0.0:
                amcl_xy_std = round(math.sqrt(xy_variance), 3)
            if math.isfinite(covariance[35]) and covariance[35] >= 0.0:
                amcl_yaw_std = round(math.sqrt(covariance[35]), 3)

        post_search_translation_drift = None
        post_search_yaw_drift = None
        if self._latest_amcl_pose is not None and self._search_best:
            pose = self._latest_amcl_pose.pose.pose
            post_search_translation_drift = math.hypot(
                pose.position.x - self._search_best['x'],
                pose.position.y - self._search_best['y'],
            )
            post_search_yaw_drift = abs(angular_difference(
                quaternion_yaw(pose.orientation), self._search_best['yaw']))

        tf_translation_span = None
        tf_yaw_span = None
        tf_window_age = None
        if self._tf_window:
            samples = [entry[1] for entry in self._tf_window]
            x_span, y_span, yaw_span = planar_window_span(samples)
            tf_translation_span = round(math.hypot(x_span, y_span), 3)
            tf_yaw_span = round(yaw_span, 3)
            tf_window_age = round(now - self._tf_window[0][0], 3)

        def age(value):
            if value is None:
                return None
            return round(now - value, 3)

        data = {
            'instance_id': self._status_instance_id,
            'source_stamp_ns': self.get_clock().now().nanoseconds,
            'evidence_epoch_ns': self._evidence_epoch_ns,
            'state': self._state.value,
            'ready': self._state == State.READY,
            'validation_only': self._validation_only,
            'candidate_ready': self._state == State.CANDIDATE_READY,
            'awaiting_amcl_initial_pose': self._awaiting_amcl_initial_pose,
            'navigation_activated': self._state == State.READY,
            'navigation_pause': self._navigation_pause_state,
            'timing': self._timing,
            'scan_tf_error': self._scan_tf_error,
            'score_duration_sec': self._score_duration_sec,
            'candidate_pose': ({
                'x': self._latest_amcl_pose.pose.pose.position.x,
                'y': self._latest_amcl_pose.pose.pose.position.y,
                'yaw': quaternion_yaw(
                    self._latest_amcl_pose.pose.pose.orientation),
            } if self._latest_amcl_pose is not None else None),
            'manual_reference_pose': self._manual_reference_pose,
            'candidate_anchor_pose': self._candidate_anchor_pose,
            'failure_reason': self._failure_reason,
            'quality_failure': self._quality_failure,
            'amcl_xy_std': amcl_xy_std,
            'amcl_yaw_std': amcl_yaw_std,
            'amcl_age_sec': age(self._last_amcl_time),
            'rotation_progress_rad': round(self._rotation_progress, 3),
            'global_search_best': self._search_best,
            'global_search_runner_up': self._search_runner_up,
            'post_search_translation_drift_m': post_search_translation_drift,
            'post_search_yaw_drift_rad': post_search_yaw_drift,
            'scan_map_score': round(self._scan_score, 3),
            'scan_metrics': self._scan_metrics,
            'scan_age_sec': age(self._last_scan_time),
            'safety_scan_age_sec': age(self._last_safety_scan_time),
            'scan_score_age_sec': age(self._last_score_time),
            'valid_scan_beams': self._valid_scan_beams,
            'sampled_scan_beams': self._sampled_scan_beams,
            'scan_map_coverage': round(
                self._valid_scan_beams / self._sampled_scan_beams, 3
            ) if self._sampled_scan_beams else 0.0,
            'finite_scan_beams': self._finite_scan_beams,
            'minimum_scan_range_m': (
                None if not math.isfinite(self._scan_min_range)
                else round(self._scan_min_range, 3)
            ),
            'particle_concentration': round(
                self._particle_concentration, 3
            ),
            'particle_count': self._particle_count,
            'particle_age_sec': age(self._last_particle_time),
            'tf_age_sec': age(self._last_map_odom_time),
            'tf_translation_span': tf_translation_span,
            'tf_yaw_span': tf_yaw_span,
            'tf_window_age_sec': tf_window_age,
            'quality_hold_age_sec': age(self._quality_since),
            'candidate_invalid_age_sec': age(self._candidate_invalid_since),
            'stationary_age_sec': age(self._stationary_since),
            'moving_age_sec': age(self._moving_since),
            'rotation_obstacle_age_sec': age(
                self._rotation_obstacle_since
            ),
            'odom_linear_speed_mps': (
                round(self._last_linear_speed, 4)
                if math.isfinite(self._last_linear_speed) else None
            ),
            'odom_angular_speed_rps': (
                round(self._last_angular_speed, 4)
                if math.isfinite(self._last_angular_speed) else None
            ),
            'manual_pose_required':
                self._state == State.WAIT_MANUAL_POSE,
        }
        data.update(self._confined_status())
        encoded = json.dumps(data, separators=(',', ':'), sort_keys=True)
        if force or encoded != self._last_status:
            message = String()
            message.data = encoded
            self._status_publisher.publish(message)
            self._last_status = encoded


def main(args=None):
    """Run the automatic localization manager until shutdown."""
    rclpy.init(args=args)
    node = AutomaticLocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        node._cancel_worker()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
