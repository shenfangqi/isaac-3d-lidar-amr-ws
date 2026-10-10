"""Safely gate saved-map navigation on automatic AMCL localization."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
import hashlib
import json
import math
import time
import uuid

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
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
    Hypothesis,
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
    diagnostic_snapshot,
    grid_snapshot,
    map_hash,
    PosePrior,
    QualityDecision,
    run_recheck_job,
    run_search_job,
    run_validation_job,
    SearchConfig,
    SearchOutput,
    SearchWorker,
    seed_pose_at_current_time,
    ValidationThresholds,
)
from isaac_3d_lidar_bringup.localization_motion_guard import (
    PoseSpeed,
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
from isaac_3d_lidar_bringup.localization_route_planner import (
    check_route,
    ClearanceField,
    FORWARD,
    MotionModel,
    PlannerConfig,
    PoseBounds,
    ROTATE,
    ROUTE_FOUND,
    run_route_plan_job,
    static_map_from_grid,
)
from isaac_3d_lidar_bringup.localization_saved_pose import (
    load_pose,
    save_pose,
    SavedPose,
)
from isaac_3d_lidar_bringup.localization_surface_check import (
    load_ply_vertices,
    planar,
    pointcloud2_xyz,
    SurfaceCheckConfig,
    transform,
)
from isaac_3d_lidar_bringup.localization_surface_model import load_surface_model
from isaac_3d_lidar_bringup.localization_surface_validation import (
    run_surface_decision_job,
    SurfaceConflictConfig,
)
from isaac_3d_lidar_bringup.localization_translation_contracts import (
    decode_linear_profile,
    decode_translation_status,
    encode_route_evidence,
    encode_translation_request,
    RouteEvidence,
    TranslationLink,
)
from isaac_3d_lidar_bringup.localization_translation_guard import LinearProbeGuard


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
    # Phase 3 translation (translation_policy guarded, ACCEPTED linear
    # profile only); the guard drives, this node sends map evidence.
    PLAN_ROUTE = 'PLAN_ROUTE'
    EXECUTE_TRANSLATION = 'EXECUTE_TRANSLATION'
    SETTLE_TRANSLATION = 'SETTLE_TRANSLATION'


CONFINED_STATES = frozenset((
    State.COLLECT_STATIC, State.SEARCH_MULTI_VIEW, State.VERIFY_HYPOTHESES,
    State.PLAN_PROBE, State.EXECUTE_PROBE, State.SETTLE_PROBE,
    State.PLAN_ROUTE, State.EXECUTE_TRANSLATION, State.SETTLE_TRANSLATION))

# Linear guard faults that are not contract reasons themselves.
LINEAR_FAULT_REASONS = {
    'UNKNOWN_OR_OUTSIDE_IN_SWEEP': RejectReason.UNKNOWN_SWEEP,
    'SWEEP_STALE': RejectReason.SENSOR_STALE,
    'SWEEP_POSE_MISMATCH': RejectReason.SENSOR_STALE,
    'SWEEP_TOO_SHORT': RejectReason.SENSOR_STALE,
    'LEASE_EXPIRED': RejectReason.CANCELED,
    'NO_PROGRESS': RejectReason.MOTION_BUDGET_EXHAUSTED,
    'PATH_DEVIATION': RejectReason.ODOM_JUMP,
    'LINEAR_PROFILE_INVALID': RejectReason.PROFILE_INVALID,
}

# A new viewpoint can resolve these; data and map failures it cannot.
PROBE_RESOLVABLE = frozenset((
    RejectReason.SEARCH_INCOMPLETE.value,
    RejectReason.AMBIGUOUS_LOCATION.value,
    RejectReason.UNOBSERVABLE_AXIS.value,
    RejectReason.NO_VALID_CANDIDATE.value,
))
# 2D verdicts of a complete search the 3D decision may revisit.  2026-10-10
# labelled captures: the true pose failed the 2D conflict gate (0.275 > 0.25,
# NO_VALID_CANDIDATE) or the 2D corridor check (UNOBSERVABLE_AXIS after a 3D
# prior).  The 3D decision replaces the 2D margin and corridor checks with
# independent 3D evidence; the leader keeps a 2D sanity check and AMCL still
# verifies after seeding.  A budget-limited search is handled separately.
SURFACE_RECHECK_REASONS = frozenset((
    RejectReason.AMBIGUOUS_LOCATION.value,
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
        self._configure_surface_check()

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
        self._cloud_subscription = None
        if self._surface_policy != 'off':
            self._cloud_subscription = self.create_subscription(
                PointCloud2, self._parameter('surface_cloud_topic'),
                self._on_cloud, scan_qos)

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
        self._last_twist_linear_speed = math.inf
        self._pose_speed = PoseSpeed(
            self._parameter('stationary_speed_window_sec'))
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
            'publish_localization_diagnostics': False,
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
            # Translation speed comes from odometry positions over this
            # window; FAST-LIO's twist can stay biased after a long turn.
            'stationary_speed_window_sec': 0.5,
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
            # Real-robot test of the probe path: probe once even when the
            # stationary result already passed.  Validation-only, guarded
            # segmented_rotation only.
            'force_probe_once': False,
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
            # 2026-10-09: four probes plus one repeated map-wide search used
            # 233 s of the initial 240 s; the guard uses the same values.
            'session_timeout_sec': 360.0,
            'collect_static_timeout_sec': 10.0,
            'independent_cluster_xy_m': 0.30,
            'independent_cluster_yaw_rad': math.pi / 12.0,
            'max_refined_clusters': 8,
            'max_extra_refined_clusters': 16,
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
            'attestation_max_age_sec': 360.0,
            # Saved-pose prior (opt-in per launch with robot_not_moved).  The
            # pose is saved while READY; '' disables saving and the prior.
            'saved_pose_path': '',
            'robot_not_moved': False,
            'saved_pose_period_sec': 5.0,
            'saved_pose_max_age_sec': 14.0 * 86400.0,
            'saved_pose_xy_tolerance_m': 0.25,
            'saved_pose_yaw_tolerance_rad': math.radians(10.0),
            # Static 3D decision among the 2D candidates against the map mesh:
            # off, record (evidence only) or decide (an independent 3D
            # decision plus a 2D sanity check may accept; AMCL still verifies).
            'surface_recheck_policy': 'off',
            'surface_mesh_path': '',
            'surface_cloud_topic': '/fast_lio/cloud_registered_body',
            # Refined candidates plus the unrefined seeds of a budget-limited
            # search (2026-10-10 capture_02 replay: 26-66); more is refused.
            'surface_max_candidates': 80,
            'surface_cloud_window_sec': 5.0,
            # Score against points on the mesh triangles, not its vertices
            # (stage A); 0 = vertices only, the pre-2026-10-11 behaviour.
            'surface_spacing_m': 0.02,
            # Map-derived cache; '' = next to the mesh.
            'surface_cache_dir': '',
            'surface_max_points_per_cloud': 6000,
            'surface_min_composite_gap': 0.15,
            'surface_min_leader_composite': 0.70,
            # Gap of refined candidates at this inlier tolerance; the leader's
            # absolute fit keeps the 0.10 m tolerance (localization_surface_validation).
            'surface_rank_tolerance_m': 0.05,
            # Gross-contradiction cap on the 3D leader's 2D conflict ratio;
            # labelled true poses reached 0.275-0.292 (gate 0.25).
            'surface_max_2d_conflict': 0.35,
            # Workstation 39 candidates ~5 s; the Jetson is ~5.7x slower.
            'surface_decision_timeout_sec': 120.0,
            # Phase 3 translation probes: forbid unless guarded AND an
            # ACCEPTED linear profile matches this robot.
            'translation_policy': 'forbid',
            'linear_profile_path': '',
            'translation_request_topic':
                '/automatic_localization/translation_request',
            'route_evidence_topic': '/automatic_localization/route_evidence',
            'translation_status_topic':
                '/automatic_localization/translation_status',
            'translation_speed_mps': 0.05,
            'route_plan_timeout_sec': 60.0,
            'route_max_depth': 2,
            'route_min_difference': 0.15,
            'route_candidate_window': 0.25,
            'route_max_candidates': 24,
            'translation_motion_timeout_sec': 30.0,
            'translation_settle_sec': 1.0,
            'translation_settle_timeout_sec': 8.0,
            'translation_status_freshness_sec': 0.5,
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
        self._guard_release_confirmed = False
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
        self._configure_translation()

    def _configure_translation(self):
        """Enable translation only for an ACCEPTED, matching linear profile."""
        self._translation = None
        self._translation_enabled = False
        self._translation_status = None
        self._linear_profile = None
        self._linear_stop_extension = math.inf
        self._route_fields = None
        policy = str(self._parameter('translation_policy'))
        if policy not in ('forbid', 'guarded'):
            raise ContractError('translation_policy must be forbid or guarded')
        self._translation_reason = 'translation_policy=forbid'
        if policy == 'forbid':
            return
        if self._motion_policy != MotionPolicy.GUARDED:
            raise ContractError('translation_policy guarded needs motion_policy guarded')
        profile = None
        try:
            with open(self._parameter('linear_profile_path'), encoding='utf-8') as stream:
                profile = decode_linear_profile(stream.read())
        except (OSError, ContractError) as error:
            self._translation_reason = f'linear profile rejected: {error}'
        speed = float(self._parameter('translation_speed_mps'))
        if profile is not None:
            probe = LinearProbeGuard(profile, expected_hashes=self._probe_hashes)
            if not probe.permitted:
                self._translation_reason = (
                    'linear profile is not ACCEPTED for this geometry, '
                    'extrinsics and control chain')
            elif not 0 < speed <= profile.max_speed_mps:
                self._translation_reason = 'translation_speed_mps exceeds the profile'
            else:
                self._linear_profile = profile
                self._linear_stop_extension = probe.stop_extension_m
                self._translation_enabled = True
                self._translation_reason = ''
        self._translation_request_publisher = self.create_publisher(
            String, self._parameter('translation_request_topic'), 10)
        self._route_evidence_publisher = self.create_publisher(
            String, self._parameter('route_evidence_topic'), 10)
        self.create_subscription(
            String, self._parameter('translation_status_topic'),
            self._on_translation_status, 10)
        if not self._translation_enabled:
            self.get_logger().error(f'translation disabled: {self._translation_reason}')

    def _on_translation_status(self, message):
        try:
            status = decode_translation_status(message.data)
        except ContractError as error:
            self.get_logger().warning(f'invalid translation status: {error}')
            return
        self._translation_status = (status, time.monotonic())

    def _fresh_translation_status(self, now):
        """Fresh translation status for our latest request, or None."""
        if self._translation is None or self._translation_status is None:
            return None
        status, received = self._translation_status
        if (now - received > self._parameter('translation_status_freshness_sec')
                or not self._translation.owns(status)):
            return None
        return status

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
        if getattr(self, '_translation', None) is not None:
            self._translation_request_publisher.publish(String(
                data=encode_translation_request(self._translation.request())))

    def _guard_released(self, now):
        status = self._fresh_guard_status(now)
        linear = self._fresh_translation_status(now)
        released = (status is not None and self._probe.owns(status)
                    and status.state == GuardState.RELEASED
                    and (getattr(self, '_translation', None) is None
                         or (linear is not None and linear.state == 'RELEASED'))
                    and self.count_publishers(self._command_topic) == 0)
        if released:
            self._guard_release_confirmed = True
        return released

    def _configure_surface_check(self):
        """Validate the 3D re-check policy and load the mesh once."""
        policy = str(self._parameter('surface_recheck_policy'))
        if policy not in ('off', 'record', 'decide'):
            raise ContractError('surface_recheck_policy must be off, record or decide')
        self._surface_vertices = None
        self._surface_config = SurfaceCheckConfig(
            min_composite_gap=float(self._parameter('surface_min_composite_gap')),
            min_leader_composite=float(self._parameter('surface_min_leader_composite')))
        self._surface_rank_config = SurfaceCheckConfig(
            tolerance_m=float(self._parameter('surface_rank_tolerance_m')),
            min_composite_gap=float(self._parameter('surface_min_composite_gap')),
            min_leader_composite=0.01)
        if self._strategy == Strategy.LEGACY_FULL_ROTATION:
            if policy == 'decide':
                raise ContractError(
                    'the 3D re-check needs stationary_only or segmented_rotation')
            policy = 'off'             # record is evidence only; nothing to record
        if policy != 'off':
            path = self._parameter('surface_mesh_path')
            spacing = float(self._parameter('surface_spacing_m'))
            try:
                if spacing > 0:
                    model = load_surface_model(
                        path, spacing, self._parameter('surface_cache_dir') or None)
                    self._surface_vertices = model.points
                    self.get_logger().info(
                        f'3D surface: {model.report["samples"]} samples at '
                        f'{spacing:.3f} m from {model.report["faces"]} faces '
                        f'(mesh {model.mesh_sha256[:12]}, cache {model.report["cache"]})')
                else:
                    self._surface_vertices = load_ply_vertices(path)
            except (OSError, ValueError) as error:
                if policy == 'decide':
                    raise ContractError(f'3D re-check mesh unusable: {error}') from error
                self.get_logger().warn(
                    f'3D re-check disabled, mesh unusable ({path}): {error}')
                policy = 'off'
        self._surface_policy = policy

    def _on_cloud(self, message):
        """Keep deskewed clouds taken while stopped during localization."""
        if self._state not in (State.COLLECT_STATIC, State.SEARCH_MULTI_VIEW,
                               State.VERIFY_HYPOTHESES):
            return
        now = time.monotonic()
        if self._stationary_since is None or self._moving_since is not None:
            # Motion: start over.
            self._surface_clouds.clear()
            self._surface_cloud_counts['cleared_on_motion'] += 1
            return
        if not self._stopped(now):
            # Stillness not (yet) confirmed, e.g. a late /odom under load
            # (2026-10-10 on the robot: up to 1.7 s).  Skip only this cloud;
            # each kept cloud is placed with the odometry at its own stamp.
            self._surface_cloud_counts['skipped_unconfirmed'] += 1
            return
        stamp = message.header.stamp
        stamp_ns = stamp.sec * 10**9 + stamp.nanosec
        try:
            xyz = pointcloud2_xyz(
                message, int(self._parameter('surface_max_points_per_cloud')))
        except ContractError:
            return
        self._surface_clouds.append((stamp_ns, message.header.frame_id, xyz, now))
        self._surface_cloud_counts['kept'] += 1

    def _surface_points(self, train, now):
        """All stationary cloud points in the reference keyframe's base frame."""
        parts = [part[1] for part in self._surface_cloud_parts(train, now)]
        return np.vstack(parts) if parts else np.zeros((0, 3))

    def _surface_point_sets(self, train, now):
        """
        Two temporally disjoint point sets: earlier and later clouds.

        The 3D decision ranks the candidates on each independently.
        """
        parts = sorted(self._surface_cloud_parts(train, now), key=lambda item: item[0])
        half = len(parts) // 2
        sets = [[part[1] for part in chunk]
                for chunk in (parts[:half], parts[half:])]
        return tuple(np.vstack(chunk) if chunk else np.zeros((0, 3)) for chunk in sets)

    def _surface_cloud_parts(self, train, now):
        """
        ``[(stamp_ns, points, sensor_origin)]`` of stationary clouds.

        Points and origin are in the reference base frame.  Each cloud uses
        the full 3D base<-sensor transform and the planar odometry at its own
        source stamp; clouds without both are skipped.
        """
        reference = train[0].T_odom_base.inverse()
        window = float(self._parameter('surface_cloud_window_sec'))
        parts = []
        for stamp_ns, frame, xyz, received in self._surface_clouds:
            if now - received > window or not len(xyz):
                continue
            try:
                edge = self._tf_buffer.lookup_transform(
                    'base_footprint', frame, Time(nanoseconds=stamp_ns)).transform
                odom_base = self._tf_se2('odom', 'base_footprint', stamp_ns)
            except (LookupError, TransformException):
                continue
            t, q = edge.translation, edge.rotation
            relative = reference.compose(odom_base)
            matrix = (planar(relative.x, relative.y, relative.yaw)
                      @ transform((t.x, t.y, getattr(t, 'z', 0.0)),
                                  (q.x, q.y, q.z, q.w)))
            homogeneous = np.c_[xyz, np.ones(len(xyz))]
            parts.append((stamp_ns, (matrix @ homogeneous.T).T[:, :3], matrix[:3, 3]))
        return parts

    def _surface_sensor_origin(self, train, now):
        """Median sensor position of the stationary clouds, or None."""
        origins = [part[2] for part in self._surface_cloud_parts(train, now)
                   if len(part) > 2]
        return tuple(float(v) for v in np.median(origins, axis=0)) if origins else None

    def _start_surface_check(self, decision, train, holdout, now):
        """Submit the 3D decision for a refused 2D verdict; False if skipped."""
        result = self._search_result
        if self._surface_policy == 'off' or result is None:
            return False
        if result.complete:
            if (decision.reason not in SURFACE_RECHECK_REASONS
                    or getattr(decision, 'prior_conflict', False)):
                return False
            by_id = {h.cluster_id: h for h in result.hypotheses}
            ranked = [by_id[cluster] for cluster, _score in decision.holdout
                      if cluster in by_id]
            seeds = ()
        elif (decision.reason == RejectReason.SEARCH_INCOMPLETE.value
              and result.unrefined and result.hypotheses):
            # Stopped only by the refinement budget: the unrefined
            # competitors' coarse seeds join the refined candidates, so the
            # 3D decision covers what a complete search would have.
            by_id = {h.cluster_id: h for h in result.hypotheses}
            ranked = list(result.hypotheses)
            seeds = result.unrefined
        else:
            return False
        if (len(ranked) + len(seeds) > int(self._parameter('surface_max_candidates'))
                or {h.cluster_id for h in ranked} != set(by_id)):
            self._surface_status = {
                'policy': self._surface_policy, 'resolved': False, 'used': False,
                'reason': 'CANDIDATE_SET_INCOMPLETE',
                'candidate_count': len(by_id) + len(seeds),
                'ranked_count': len(ranked),
                'search_complete': result.complete}
            return False
        first, second = self._surface_point_sets(train, now)
        minimum = self._surface_config.min_points
        if len(ranked) < 2 or min(len(first), len(second)) < minimum:
            self._surface_status = {
                'policy': self._surface_policy, 'resolved': False, 'used': False,
                'reason': 'TOO_FEW_CANDIDATES' if len(ranked) < 2 else 'TOO_FEW_POINTS',
                'points': [int(len(first)), int(len(second))],
                'clouds': dict(self._surface_cloud_counts),
                'search_complete': result.complete}
            return False
        self._surface_poses = tuple((h.x, h.y, h.yaw) for h in ranked) + tuple(seeds)
        self._surface_hypotheses = tuple(ranked)
        self._surface_original = decision
        self._surface_inputs = (train, holdout, self._validation_inputs[2])
        self._worker.submit(
            (self._session, self._map_hash), run_surface_decision_job,
            self._surface_vertices, first, second,
            tuple((h.x, h.y, h.yaw) for h in ranked),
            self._latest_grid, tuple(holdout), train[0].T_odom_base,
            self._search_config(), self._validation_thresholds(),
            float(self._parameter('surface_max_2d_conflict')),
            self._surface_rank_config, self._surface_config,
            float(self._parameter('surface_decision_timeout_sec')), tuple(seeds),
            self._surface_sensor_origin(train, now))
        self._surface_stage = 'deciding'
        return True

    def _on_surface_decision(self, result, now):
        """Record the 3D decision; in decide mode an accepted one seeds AMCL."""
        validation = result.validation
        leader = result.pose or (
            validation.poses[validation.leader]
            if validation is not None and 0 <= validation.leader < len(validation.poses)
            else None)
        gaps = [None if part is None else part.composite_gap
                for part in ((validation.train, validation.holdout)
                             if validation is not None else (None, None))]
        _train, _holdout, gates = self._surface_inputs
        # Earlier views (a translation in this session) are not part of the
        # 3D decision; it is evidence only then.
        use = self._surface_policy == 'decide' and result.accepted and not gates
        self._surface_status = {
            'policy': self._surface_policy, 'resolved': result.accepted,
            'reason': result.reason,
            'composite_gaps': gaps,
            'composite': (list(validation.train.composite)
                          if validation is not None and validation.train is not None
                          else []),
            'support_extents': (list(validation.support_extents)
                                if validation is not None else []),
            'leader_pose_at_reference': (
                None if leader is None else
                {'x': round(leader[0], 3), 'y': round(leader[1], 3),
                 'yaw': round(leader[2], 4)}),
            'metrics_2d': dict(result.metrics_2d),
            # Per window, the leader's see-through ratio and the number of
            # candidates it excluded (None without a sensor origin).
            'see_through': (
                [{'leader': (ratios[validation.leader]
                             if 0 <= validation.leader < len(ratios)
                             and math.isfinite(ratios[validation.leader]) else None),
                  'excluded': sum(r > SurfaceConflictConfig().max_ratio for r in ratios)}
                 for ratios in validation.see_through]
                if validation is not None and validation.see_through else None),
            'candidate_count': len(self._surface_poses),
            'clouds': dict(self._surface_cloud_counts),
            'search_complete': self._search_result.complete,
            'used': use}
        self.get_logger().info(
            f'3D decision ({self._surface_policy}): accepted={result.accepted} '
            f'reason={result.reason or "-"} gaps={gaps} '
            f'leader={self._surface_status["leader_pose_at_reference"]}'
            f'{" -> used" if use else ""}')
        if not use:
            self._finish_validation(self._surface_original, now)
            return
        extents = result.validation.support_extents
        angles = (0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)
        origin = validation.origins[result.leader]
        hypotheses = self._surface_hypotheses
        if origin < len(hypotheses):
            cluster_id, per_view = hypotheses[origin].cluster_id, hypotheses[origin].per_view
        else:                               # an unrefined competitor's seed
            cluster_id = max(h.cluster_id for h in hypotheses) + 1 + origin - len(hypotheses)
            per_view = ()
        winner = Hypothesis(
            x=leader[0], y=leader[1], yaw=leader[2],
            score=result.metrics_2d['score'], coverage=result.metrics_2d['coverage'],
            conflict=result.metrics_2d['conflict'], cluster_id=cluster_id,
            per_view=per_view,
            support_bounds=(max(abs(math.cos(a)) * e for a, e in zip(angles, extents)),
                            max(abs(math.sin(a)) * e for a, e in zip(angles, extents)),
                            extents[4]))
        decision = QualityDecision(
            True, '', winner=winner,
            runner_up_score=self._surface_original.runner_up_score,
            holdout=self._surface_original.holdout)
        self._diagnostic_decision = decision
        self._finish_validation(decision, now)

    def _finish_validation(self, decision, now):
        """Act on a final validation decision (accept, probe or reject)."""
        if getattr(decision, 'prior_conflict', False):
            # Rotating cannot fix this: the attestation or the
            # localization is wrong.  Hand over to the operator.
            self._reject(RejectReason.AMBIGUOUS_LOCATION,
                         'clear winner contradicts the saved pose '
                         '(robot_not_moved)')
        elif not decision.accepted:
            self._reject_or_probe(decision.reason, now)
        elif getattr(self, '_force_probe_pending', False):
            # Test mode: the result passed, but probe once anyway; the
            # next accepted result after that probe is used normally.
            self._force_probe_pending = False
            self._probe_hypothesis_counts.append(
                len(self._search_result.hypotheses))
            self._probe_reason = 'FORCED_PROBE_TEST'
            self.get_logger().info(
                'force_probe_once: stationary result passed; probing '
                'once anyway (validation test, Nav2 stays inactive)')
            self._transition(State.PLAN_PROBE)
        else:
            if getattr(decision, 'prior_used', False):
                self._saved_prior_used = True
                self.get_logger().info(
                    'saved pose resolved near-equal candidates '
                    '(robot_not_moved)')
            self._accept_confined(decision.winner, now)

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
        self._force_probe_once = bool(self._parameter('force_probe_once'))
        if self._force_probe_once and not (
                self._segmented and policy == MotionPolicy.GUARDED
                and self._validation_only):
            raise ContractError(
                'force_probe_once is a test of the probe path: it needs '
                'segmented_rotation, motion_policy guarded and '
                'validation_only (Nav2 never activates)')
        names = (
            'train_frames_per_view', 'holdout_frames_per_view', 'max_views',
            'max_probe_segments', 'max_refined_clusters',
            'max_extra_refined_clusters', 'probe_angles_rad',
            'max_total_probe_yaw_rad', 'probe_motion_timeout_sec',
            'motion_request_timeout_sec', 'sensor_freshness_sec',
            'search_timeout_sec', 'session_timeout_sec',
            'independent_cluster_xy_m', 'independent_cluster_yaw_rad',
        )
        self._confined = validate_confined_parameters(
            {name: self._parameter(name) for name in names})
        self._strategy = strategy
        self._motion_policy = policy
        self._robot_not_moved = bool(self._parameter('robot_not_moved'))
        if self._robot_not_moved and (
                strategy == Strategy.LEGACY_FULL_ROTATION
                or not self._parameter('saved_pose_path')):
            raise ContractError(
                'robot_not_moved needs stationary_only or segmented_rotation '
                'and a saved_pose_path')

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
            max_extra_refined_clusters=self._confined[
                'max_extra_refined_clusters'],
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
        self._search_reference = None
        self._search_is_recheck = False
        self._recheck_basis = None
        self._coarse_cache = None
        self._validation_submitted = False
        self._diagnostic_train = ()
        self._diagnostic_holdout = ()
        self._diagnostic_decision = None
        self._validation_inputs = ((), (), ())
        self._surface_clouds = deque(maxlen=80)
        self._surface_cloud_counts = dict.fromkeys(
            ('kept', 'skipped_unconfirmed', 'cleared_on_motion'), 0)
        self._surface_stage = None
        self._surface_original = None
        self._surface_inputs = None
        self._surface_hypotheses = ()
        self._surface_poses = ()
        self._surface_status = None
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
            self._translation = (TranslationLink(self._session)
                                 if self._translation_enabled else None)
            self._translation_status = None
            self._translated = False
            self._route_rotation = None
            self._route_status = None
            self._guard_release_confirmed = False
            self._force_probe_pending = self._force_probe_once
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
        self._load_saved_prior()
        self._search_best = None
        self._search_runner_up = None
        self._transition(State.COLLECT_STATIC)

    def _load_saved_prior(self):
        """Reset the saved-pose prior; it is loaded at first validation."""
        self._saved_prior = None
        self._saved_prior_reason = ''
        self._saved_prior_loaded = False
        self._saved_prior_used = False
        self._session_start_odom = self._last_odom_pose

    def _pose_prior(self, train):
        """Return the saved pose at the reference keyframe, or None."""
        if not getattr(self, '_robot_not_moved', False):
            return None
        if not self._saved_prior_loaded:
            # The map hash is known by now (search ran on the map).
            self._saved_prior_loaded = True
            pose, reason = load_pose(
                self._parameter('saved_pose_path'), self._map_hash,
                time.time(),
                float(self._parameter('saved_pose_max_age_sec')))
            if pose is not None and self._session_start_odom is None:
                pose, reason = None, 'no odometry at session start'
            self._saved_prior, self._saved_prior_reason = pose, reason
            self.get_logger().info(
                'robot_not_moved: using the saved pose as a prior' if pose
                else f'robot_not_moved: saved pose not used ({reason})')
        prior = self._saved_prior
        if prior is None or prior.map_hash != self._map_hash:
            return None
        # The robot was attested unmoved at session start; carry the saved
        # map pose along odometry to the reference keyframe.
        at_reference = SE2(prior.x, prior.y, prior.yaw).compose(
            SE2(*self._session_start_odom).inverse()).compose(
                train[0].T_odom_base)
        return PosePrior(
            at_reference,
            float(self._parameter('saved_pose_xy_tolerance_m')),
            float(self._parameter('saved_pose_yaw_tolerance_rad')))

    def _maybe_save_pose(self, now):
        """While READY, keep the latest trusted AMCL pose on disk."""
        path = self._parameter('saved_pose_path')
        if (not path or self._state != State.READY
                or self._latest_amcl_pose is None or not self._map_hash
                or not self._recent(self._last_amcl_time, now)):
            return
        last = getattr(self, '_last_pose_save', None)
        if last is not None and now - last < self._parameter(
                'saved_pose_period_sec'):
            return
        message = self._latest_amcl_pose.pose
        covariance = message.covariance
        xy_std = math.sqrt(max(covariance[0], covariance[7], 0.0))
        yaw_std = math.sqrt(max(covariance[35], 0.0))
        if (xy_std > self._parameter('max_amcl_xy_std')
                or yaw_std > self._parameter('max_amcl_yaw_std')):
            return
        self._last_pose_save = now
        pose = message.pose
        try:
            save_pose(path, SavedPose(
                self._map_hash, pose.position.x, pose.position.y,
                quaternion_yaw(pose.orientation), time.time(), xy_std,
                yaw_std))
        except (OSError, ContractError) as error:
            if not getattr(self, '_pose_save_warned', False):
                self._pose_save_warned = True
                self.get_logger().warn(f'cannot save the pose: {error}')

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
        self._last_odom_stamp_ns = (message.header.stamp.sec * 10**9
                                    + message.header.stamp.nanosec)
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
                self._pose_speed.reset()
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
        stamp = message.header.stamp
        linear = self._pose_speed.update(
            stamp.sec + stamp.nanosec * 1e-9, pose.position.x,
            pose.position.y)
        angular = abs(twist.angular.z)
        self._last_linear_speed = math.inf if linear is None else linear
        self._last_twist_linear_speed = math.hypot(
            twist.linear.x, twist.linear.y)
        self._last_angular_speed = angular
        if (
            linear is not None
            and linear <= self._parameter('max_stationary_linear_speed')
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
        if (getattr(self, '_translation', None) is not None
                and self._translation.operation != 'RELEASE'):
            self._translation.stop()
            self._refresh_probe_lease()
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
            elif (self._probe is not None
                  and not self._guard_release_confirmed
                  and not self._guard_released(now)):
                # Nav2 never activates while the guard can still publish.
                # Checked only before STARTUP: once Nav2 runs, its velocity
                # smoother is the /cmd_vel_command publisher by design.
                if self._probe.operation != MotionOperation.RELEASE:
                    self._probe.release()
                    self._refresh_probe_lease()
                if (self._translation is not None
                        and self._translation.operation != 'RELEASE'):
                    self._translation.release()
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

        self._maybe_save_pose(now)
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
        # Each new view must contribute its own frames of this role.
        current = [frame for frame in frames if frame.view_id == self._view_id]
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
            self._submit_search(train)
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
            if status == 'ok' and isinstance(result, SearchOutput):
                # Coarse scores of these views serve the next map-wide search.
                self._coarse_cache = result.coarse_cache
                result = result.result
            if token != (self._session, self._map_hash):
                self._reject(RejectReason.MAP_CHANGED, 'stale search result')
            elif status != 'ok':
                self._reject(RejectReason.SEARCH_INCOMPLETE, str(result))
            elif not result.complete:
                self._search_result = result
                if self._surface_policy != 'off' and result.unrefined and result.hypotheses:
                    # The 3D decision may still cover the unrefined
                    # competitors; it needs HOLDOUT frames first.
                    self._validation_submitted = False
                    self._transition(State.VERIFY_HYPOTHESES)
                else:
                    self._reject_or_probe(result.reason, now)
            elif self._search_is_recheck and not self._recheck_survives(
                    result):
                # Spec 5.3: every candidate refuted -> search the whole map
                # again; nothing is accepted until it completes.
                self.get_logger().info(
                    'candidate re-check refuted every hypothesis; searching '
                    'the whole map again')
                train = self._frames(FrameRole.TRAIN, now)
                if isinstance(train, Reject):
                    self._reject(train.reason, train.detail)
                    return
                self._recheck_basis = None
                self._submit_search(train)
                self._transition(State.SEARCH_MULTI_VIEW)
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
                train = self._frames(FrameRole.TRAIN, now)
                # Margin on the current view; earlier views (kept only after
                # a translation) only gate the winner per view.
                gates = tuple(f for f in holdout if f.view_id != self._view_id)
                holdout = tuple(f for f in holdout if f.view_id == self._view_id)
                self._diagnostic_train = (
                    () if isinstance(train, Reject) else tuple(train))
                self._diagnostic_holdout = tuple(holdout)
                self._validation_inputs = (train, holdout, gates)
                self._surface_stage = None
                self._worker.submit(
                    (self._session, self._map_hash), run_validation_job,
                    self._latest_grid, self._search_result, train, holdout,
                    self._search_config(), self._validation_thresholds(),
                    self._parameter('verification_timeout_sec'),
                    None if isinstance(train, Reject)
                    else self._pose_prior(train),
                    gates)
                self._validation_submitted = True
                return
            outcome = self._worker.poll()
            if outcome is None:
                allowance = (float(self._parameter('surface_decision_timeout_sec'))
                             if self._surface_stage else 0.0)
                if state_age > self._parameter(
                        'collect_static_timeout_sec') + self._parameter(
                        'verification_timeout_sec') + allowance + 5.0:
                    self._reject(RejectReason.SEARCH_INCOMPLETE,
                                 'validation worker deadline')
                return
            token, status, payload = outcome
            stage, self._surface_stage = self._surface_stage, None
            if token != (self._session, self._map_hash):
                self._reject(RejectReason.MAP_CHANGED,
                             'stale validation result')
            elif status != 'ok' and stage is not None:
                # A failed 3D job leaves the 2D verdict.
                self.get_logger().warn(f'3D re-check failed: {payload}')
                self._surface_status = {'policy': self._surface_policy,
                                        'resolved': False, 'reason': 'JOB_FAILED'}
                self._finish_validation(self._surface_original, now)
            elif status != 'ok':
                self._reject(RejectReason.SEARCH_INCOMPLETE, str(payload))
            elif stage == 'deciding':
                self._on_surface_decision(payload, now)
            else:
                self._diagnostic_decision = payload
                train, holdout, gates = self._validation_inputs
                if (isinstance(train, Reject)
                        or not self._start_surface_check(payload, train, holdout, now)):
                    self._finish_validation(payload, now)

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

        elif self._state == State.PLAN_ROUTE:
            self._tick_plan_route(now, state_age)

        elif self._state == State.EXECUTE_TRANSLATION:
            self._tick_execute_translation(now, state_age)

        elif self._state == State.SETTLE_TRANSLATION:
            status = self._fresh_translation_status(now)
            settled = (status is not None and status.state == 'STOPPED'
                       and self._stopped(now))
            if settled and state_age >= self._parameter('translation_settle_sec'):
                self._translated = True
                self._start_next_view()
            elif state_age > self._parameter('translation_settle_timeout_sec'):
                self._fail('translation did not settle', manual_recovery=False)

    def _reject_or_probe(self, reason, now):
        """Ambiguity in segmented_rotation plans a probe; else reject."""
        if not self._segmented or reason not in PROBE_RESOLVABLE:
            self._reject(reason)
            return
        counts = self._probe_hypothesis_counts
        result = self._search_result
        # An incomplete search refines a capped number of clusters, so its
        # count says nothing about progress (2026-10-09: 8, 8, 8 stopped the
        # probes after two segments).  Only complete searches count.
        counts.append(None if result is None or not result.complete
                      else len(result.hypotheses))
        # Two probes in a row without fewer hypotheses: stop exploring.
        if len(counts) >= 3 and None not in counts[-3:] and (
                counts[-1] >= counts[-2] >= counts[-3]):
            self._reject(RejectReason.AMBIGUOUS_LOCATION,
                         'probes did not reduce the hypotheses')
            return
        self._probe_reason = reason
        if (getattr(self, '_translation_enabled', False)
                and reason == RejectReason.AMBIGUOUS_LOCATION.value
                and result is not None and result.complete):
            self.get_logger().info(f'{reason}: planning a common safe route')
            self._route_submitted = False
            self._transition(State.PLAN_ROUTE)
            return
        self.get_logger().info(
            f'{reason}: planning a probe rotation (segmented_rotation)')
        self._transition(State.PLAN_PROBE)

    def _route_model(self):
        """Motion envelope from the ACCEPTED linear profile."""
        profile = self._linear_profile
        return MotionModel(
            footprint=self._footprint, padding_m=self._rotation_gate.padding_m,
            forward_stop_extension_m=self._linear_stop_extension,
            forward_lateral_error_m=profile.lateral_error_m, calibrated=True)

    def _route_candidates(self):
        """
        Plausible candidates at the current odometry pose, best first.

        Every candidate within ``route_candidate_window`` of the best
        HOLDOUT score is kept for safety. Exceeding the processing budget
        refuses planning instead of dropping candidates. Also returns the indices
        of those within the validation margin (+0.05) of the leader: only
        they can keep the verdict ambiguous, so the route separates them.
        """
        result = self._search_result
        if result is None or not result.hypotheses or self._last_odom_pose is None:
            return (), ()
        by_id = {h.cluster_id: h for h in result.hypotheses}
        decision = getattr(self, '_diagnostic_decision', None)
        ranked = ([(by_id[c], score) for c, score in decision.holdout if c in by_id]
                  if decision is not None and decision.holdout
                  else [(h, h.score) for h in sorted(result.hypotheses,
                                                     key=lambda h: -h.score)])
        best = ranked[0][1]
        window = float(self._parameter('route_candidate_window'))
        kept = [(h, score) for h, score in ranked if score >= best - window]
        if (len(kept) > int(self._parameter('route_max_candidates'))
                or {h.cluster_id for h, _ in ranked} != set(by_id)):
            self._route_status = {'status': 'CANDIDATE_SET_INCOMPLETE',
                                  'candidate_count': len(kept),
                                  'ranked_count': len(ranked)}
            return (), ()
        contend = self._validation_thresholds().min_margin + 0.05
        focus = tuple(i for i, (_h, score) in enumerate(kept) if score >= best - contend)
        current = SE2(*self._last_odom_pose)
        return tuple(
            seed_pose_at_current_time(SE2(h.x, h.y, h.yaw), self._search_reference,
                                      current)
            for h, _score in kept), focus

    def _route_layers(self, static_map):
        """Observation layers: the navigation map, plus mesh bands if loaded."""
        occupied = static_map.data == 100
        layers = {'nav_map': (occupied, occupied)}
        vertices = getattr(self, '_surface_vertices', None)
        if vertices is not None:
            for lo, hi in ((0.35, 0.6), (0.6, 1.0), (1.0, 1.5), (1.5, 2.0)):
                band = vertices[(vertices[:, 2] >= lo) & (vertices[:, 2] < hi)]
                i, j = static_map.cells(band[:, 0], band[:, 1])
                inside = ((i >= 0) & (i < static_map.width)
                          & (j >= 0) & (j < static_map.height))
                grid = np.zeros(occupied.shape, bool)
                grid[j[inside], i[inside]] = True
                layers[f'mesh_{lo:.2f}-{hi:.2f}'] = (grid, grid)
        return layers

    def _tick_plan_route(self, now, state_age):
        """Plan a commonly safe route in the worker; fall back to rotation."""
        if not getattr(self, '_route_submitted', False):
            self._route_status = None
            poses, focus = self._route_candidates()
            if len(poses) < 2:
                if not self._route_status:
                    self._route_status = {'status': 'TOO_FEW_CANDIDATES'}
                self._transition(State.PLAN_PROBE)
                return
            static_map = static_map_from_grid(self._latest_grid, self._map_hash)
            config = PlannerConfig(
                rotations=(math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2),
                forwards=tuple(d for d in (0.2, 0.4, 0.6) if d > self._linear_stop_extension),
                max_depth=int(self._parameter('route_max_depth')),
                min_difference=float(self._parameter('route_min_difference')),
                time_budget_s=max(1.0, float(self._parameter('route_plan_timeout_sec')) - 5.0))
            bounds = PoseBounds()
            self._route_poses = poses
            self._route_bounds = bounds
            self._worker.submit((self._session, self._map_hash), run_route_plan_job,
                                static_map, self._route_layers(static_map),
                                tuple(((p.x, p.y, p.yaw), bounds) for p in poses),
                                self._route_model(), config,
                                focus if len(focus) >= 2 else None)
            self._route_submitted = True
            return
        outcome = self._worker.poll()
        if outcome is None:
            if state_age > float(self._parameter('route_plan_timeout_sec')) + 5.0:
                self._cancel_worker()
                self._route_status = {'status': 'PLAN_TIMEOUT'}
                self._transition(State.PLAN_PROBE)
            return
        token, status, plan = outcome
        if token != (self._session, self._map_hash) or status != 'ok':
            self._route_status = {'status': 'PLAN_FAILED', 'detail': str(plan)}
            self._transition(State.PLAN_PROBE)
            return
        self._route_status = {
            'status': plan.status, 'objective': round(plan.objective, 3),
            'route': [[kind, round(value, 4)] for kind, value in plan.route],
            'candidates': len(self._route_poses)}
        self.get_logger().info(f'route plan: {self._route_status}')
        if plan.status != ROUTE_FOUND or not plan.route:
            self._transition(State.PLAN_PROBE)      # existing rotation probing
            return
        kind, value = plan.route[0]
        if kind == ROTATE:
            self._route_rotation = value
            self._transition(State.PLAN_PROBE)
            return
        self._translation_start = SE2(*self._last_odom_pose)
        self._translation_target = value
        self._translation_moved = False
        self._translation.move(value, float(self._parameter('translation_speed_mps')),
                               self._linear_profile.digest)
        self._refresh_probe_lease()
        self._transition(State.EXECUTE_TRANSLATION)

    def _route_field_pair(self):
        """Clearance fields of the current map, built once per map hash."""
        cached = getattr(self, '_route_fields', None)
        if cached is None or cached[0] != self._map_hash:
            static_map = static_map_from_grid(self._latest_grid, self._map_hash)
            cached = (self._map_hash, (
                ClearanceField(static_map),
                ClearanceField(static_map, blocked=static_map.data == 100,
                               outside_blocked=False)))
            self._route_fields = cached
        return cached[1]

    def _publish_route_evidence(self):
        """Re-check the remaining path under every candidate; send evidence."""
        stamp = getattr(self, '_last_odom_stamp_ns', 0)
        if self._last_odom_pose is None or stamp <= 0:
            return
        odom = SE2(*self._last_odom_pose)
        delta = self._translation_start.inverse().compose(odom)
        remaining = max(0.0, self._translation_target - delta.x)
        clear, reason = True, ''
        if remaining > 1e-3:
            fields, model = self._route_field_pair(), self._route_model()
            for start in self._route_poses:
                pose = start.compose(delta)
                check = check_route(fields, model, (pose.x, pose.y, pose.yaw),
                                    self._route_bounds, ((FORWARD, remaining),))
                if not check.safe:
                    clear, reason = False, check.reason
                    break
        digest = hashlib.sha256(json.dumps(
            [[round(p.x, 4), round(p.y, 4), round(p.yaw, 4)] for p in self._route_poses]
        ).encode()).hexdigest()
        self._route_evidence_publisher.publish(String(data=encode_route_evidence(
            RouteEvidence(self._session, self._translation.sequence, stamp,
                          odom.x, odom.y, odom.yaw, remaining,
                          self._linear_stop_extension, clear, reason,
                          self._map_hash, digest))))

    def _tick_execute_translation(self, now, state_age):
        """Feed map evidence and follow the guard's translation status."""
        self._publish_route_evidence()
        status = self._fresh_translation_status(now)
        if status is None:
            if state_age > self._parameter('translation_status_freshness_sec') * 3:
                self._reject(RejectReason.CONTROL_CONFLICT,
                             'translation guard status missing')
            return
        if status.state == 'FAULT':
            reason = LINEAR_FAULT_REASONS.get(status.reason)
            if reason is None:
                try:
                    reason = RejectReason(status.reason)
                except ValueError:
                    reason = RejectReason.CONTROL_CONFLICT
            self._reject(reason, f'translation refused: {status.reason}')
        elif status.state == 'MOVING':
            self._translation_moved = True
        elif self._translation_moved and status.state in ('STOPPING', 'STOPPED'):
            self._translation.stop()
            self._refresh_probe_lease()
            self._transition(State.SETTLE_TRANSLATION)
        elif state_age > self._parameter('translation_motion_timeout_sec'):
            self._reject(RejectReason.MOTION_BUDGET_EXHAUSTED,
                         'translation never completed')

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
            route_rotation = getattr(self, '_route_rotation', None)
            angles = ((route_rotation,) if route_rotation is not None
                      else self._confined['probe_angles_rad'])
            decisions = [
                evaluate_localization_rotation(
                    evidence, self._footprint, base, angle,
                    profile=self._motion_profile, hashes=self._probe_hashes,
                    config=self._rotation_gate,
                    attestation=self._attestation, session=self._session,
                    now_mono=now).decision
                for angle in angles]
        except (LookupError, ContractError):
            return                     # retried until the plan timeout
        self._probe_unknown_cells = min(d.unknown_cells for d in decisions)
        views = self._frames(FrameRole.TRAIN, now)
        headings = sorted({frame.T_odom_base.yaw for frame in views}
                          if not isinstance(views, Reject) else set())
        choice, reason = choose_probe(decisions, headings, base.yaw)
        self._route_rotation = None
        if choice is None:
            self._reject(reason, 'no admissible probe rotation')
            return
        self._probe.rotate(choice.delta_yaw,
                           float(self._parameter('probe_speed_rad_s')),
                           profile_hash(self._motion_profile))
        self._refresh_probe_lease()
        self._probe_moved = False
        self._transition(State.EXECUTE_PROBE)

    def _submit_search(self, train):
        """
        Search the whole map once, then re-check its candidates.

        After a probe, a complete earlier search is re-checked with every
        TRAIN view instead of repeating the map-wide pass, whose cost grows
        with each view (2026-10-08: 42 s, 84 s, then past the session).
        """
        self._diagnostic_train = tuple(train)
        self._diagnostic_holdout = ()
        self._diagnostic_decision = None
        basis = self._recheck_basis
        self._search_is_recheck = basis is not None
        if basis is None:
            self._worker.submit(
                (self._session, self._map_hash), run_search_job,
                self._latest_grid, train, self._search_config(),
                self._parameter('search_timeout_sec'), self._coarse_cache)
        else:
            previous, previous_reference = basis
            self.get_logger().info(
                f're-checking {len(previous.hypotheses)} hypotheses with '
                f'view {self._view_id}')
            self._worker.submit(
                (self._session, self._map_hash), run_recheck_job,
                self._latest_grid, previous, previous_reference, train,
                self._search_config(), self._parameter('search_timeout_sec'))
        self._search_reference = train[0].T_odom_base

    def _recheck_survives(self, result):
        """Whether any re-checked hypothesis still meets the score gate."""
        floor = self._validation_thresholds().min_score
        return any(h.score >= floor for h in result.hypotheses)

    def _start_next_view(self):
        """After a settled probe: drop used HOLDOUT frames, collect anew."""
        result = self._search_result
        # Only a complete search keeps every alternative; otherwise the next
        # view needs a new map-wide search.
        self._recheck_basis = (
            (result, self._search_reference)
            if result is not None and result.complete and result.hypotheses
            else None)
        self._view_id += 1
        self._surface_clouds.clear()
        self._surface_cloud_counts = dict.fromkeys(self._surface_cloud_counts, 0)
        self._surface_stage = None
        self._diagnostic_train = ()
        self._diagnostic_holdout = ()
        self._diagnostic_decision = None
        # After a translation every view keeps its independent HOLDOUT, so
        # the winner must pass the gates at each place (per-view gates).
        keep = ((FrameRole.TRAIN, FrameRole.HOLDOUT)
                if getattr(self, '_translated', False) else (FrameRole.TRAIN,))
        self._keyframes = [frame for frame in self._keyframes
                           if frame.role in keep]
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
            if getattr(self, '_translation', None) is not None:
                self._translation.release()
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
        data = {
            'schema_version': SCHEMA_VERSION,
            'strategy': self._strategy.value,
            'motion_policy': self._motion_policy.value,
            'session': self._session,
            'search_complete': None if result is None else result.complete,
            'search_recheck': bool(getattr(self, '_search_is_recheck', False)),
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
            'forced_probe_test': getattr(self, '_force_probe_once', False),
            'robot_not_moved': getattr(self, '_robot_not_moved', False),
            'surface_recheck': getattr(self, '_surface_status', None),
            'translation': {
                'enabled': getattr(self, '_translation_enabled', False),
                'reason': getattr(self, '_translation_reason', ''),
                'translated': getattr(self, '_translated', False),
                'plan': getattr(self, '_route_status', None)},
            'saved_pose_prior': (
                'used' if getattr(self, '_saved_prior_used', False)
                else 'loaded' if getattr(self, '_saved_prior', None)
                else getattr(self, '_saved_prior_reason', '') or None),
        }
        if self._parameter('publish_localization_diagnostics'):
            data['localization_diagnostics'] = diagnostic_snapshot(
                result, self._diagnostic_train, self._diagnostic_holdout,
                self._diagnostic_decision)
        return data

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
            'odom_twist_linear_speed_mps': (
                round(self._last_twist_linear_speed, 4)
                if math.isfinite(self._last_twist_linear_speed) else None
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
