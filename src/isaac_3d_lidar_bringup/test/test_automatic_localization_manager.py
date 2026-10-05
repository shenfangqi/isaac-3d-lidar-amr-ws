"""
Execute production state-machine methods with deterministic ROS test doubles.

No ROS graph, network, velocity publisher or physical robot is used.
"""
import ast
from collections import deque
from enum import Enum
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time

import pytest

from isaac_3d_lidar_bringup import localization_contracts
from isaac_3d_lidar_bringup import localization_hypotheses
from isaac_3d_lidar_bringup import localization_observations
from isaac_3d_lidar_bringup.automatic_localization_quality import (
    angular_difference, quaternion_yaw, trim_time_window,
)


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.sec * 10**9 + stamp.nanosec)


class FakeManageLifecycleNodes:
    class Request:
        STARTUP = 1

        def __init__(self):
            self.command = None


@pytest.fixture
def manager():
    path = (Path(__file__).resolve().parents[1]
            / 'isaac_3d_lidar_bringup/automatic_localization_manager.py')
    tree = ast.parse(path.read_text())
    # Keep the real class bodies and module constants; substitute only ROS
    # import boundaries.  Pure Issue #13 modules are used unchanged.
    tree.body = [item for item in tree.body
                 if isinstance(item, (ast.ClassDef, ast.Assign))]
    scope = dict(vars(localization_contracts))
    scope.update(vars(localization_observations))
    scope.update(vars(localization_hypotheses))
    scope.update(Enum=Enum, Node=object, time=time, math=math, Time=FakeTime,
                 deque=deque, uuid=__import__('uuid'),
                 ManageLifecycleNodes=FakeManageLifecycleNodes,
                 TransformException=LookupError, quaternion_yaw=quaternion_yaw,
                 angular_difference=angular_difference,
                 trim_time_window=trim_time_window,
                 PoseWithCovarianceStamped=lambda: NS(
                     header=NS(frame_id='', stamp=NS(sec=0, nanosec=0)),
                     pose=None),
                 scan_map_metrics=lambda *args: {
                     'score': 0.8, 'known': 100, 'sampled': 110})
    exec(compile(tree, str(path), 'exec'), scope)
    node = object.__new__(scope['AutomaticLocalizationManager'])
    node.State = scope['State']
    defaults = {}
    node.declare_parameter = defaults.setdefault
    node._declare_parameters()
    node._parameter = defaults.__getitem__
    node.params = defaults
    node._validation_only = True
    node._configure_strategy()
    node._worker = None
    node._reset_confined_session()
    node._latest_grid = None
    node._map_hash = ''
    node._stationary_only = False
    node._rotation_progress = 0.0
    node._state = node.State.STOP_AND_VERIFY
    node._state_started = time.monotonic()
    node._quality_since = 1.
    node._quality_failure = ''
    node._tf_window = deque()
    node._last_map_odom_time = None
    node._last_safety_scan_time = None
    node._last_tf_stamp = None
    node._timing = {}
    node._source_stamps = {}
    node._evidence_epoch_ns = 0
    node._pending_scans = deque(maxlen=3)
    node._latest_map = None
    node._latest_amcl_pose = None
    node._scan_tf_error = ''
    node._awaiting_amcl_initial_pose = False
    node._requested_initial_pose = None
    node._manual_reference_pose = None
    node._candidate_anchor_pose = None
    node._pose_seed_publisher = NS(publish=lambda message: None)
    node.get_clock = lambda: NS(now=lambda: FakeTime(10 * 10**9))
    node._publish_status = lambda **kwargs: None
    node.get_logger = lambda: NS(info=lambda msg: None)
    node._navigation_client = NS(call_async=lambda req: pytest.fail('Nav2 activated'))
    return node


def test_all_quality_checks_pass_but_navigation_stays_inactive(manager):
    manager._request_nomotion_update = lambda now: None
    manager._record_tf = lambda now: None
    manager._quality_passes = lambda now: True
    manager._verify_quality(10.)
    assert manager._state == manager.State.CANDIDATE_READY
    with pytest.raises(RuntimeError, match='forbids'):
        manager._call_lifecycle(manager._navigation_client)


def test_start_navigation_cannot_bypass_validation_mode(manager):
    manager._state = manager.State.START_NAVIGATION
    commands = []
    manager._publish_zero = lambda: commands.append('zero')
    manager._tick()
    assert manager._state == manager.State.CANDIDATE_READY
    assert commands == ['zero']


def test_explicit_activation_mode_calls_navigation_lifecycle(manager):
    manager._validation_only = False
    manager._state = manager.State.START_NAVIGATION
    manager._state_started = time.monotonic() - 1.0
    manager._command_publisher = None
    manager._future = None
    manager._publish_zero = lambda: None
    calls = []
    manager._navigation_client = NS(
        service_is_ready=lambda: True,
        call_async=lambda request: calls.append(request) or object(),
    )

    manager._tick()

    assert len(calls) == 1


def test_cached_tf_cannot_extend_quality_window(manager):
    stamped = NS(header=NS(stamp=NS(sec=10, nanosec=0)), transform=NS(
        translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    manager._tf_buffer = NS(lookup_transform=lambda *args: stamped)
    manager._record_tf(1.)
    manager._record_tf(5.)
    assert len(manager._tf_window) == 1
    assert manager._last_map_odom_time == 1.
    assert not manager._recent(manager._last_map_odom_time, 5.)


def test_sparse_new_tf_samples_can_form_stability_window(manager):
    clock = {'sec': 9}

    def lookup(*args):
        clock['sec'] += 1
        return NS(
            header=NS(stamp=NS(sec=clock['sec'], nanosec=0)),
            transform=NS(
                translation=NS(x=0., y=0.),
                rotation=NS(x=0., y=0., z=0., w=1.)))

    manager.get_clock = lambda: NS(
        now=lambda: FakeTime(clock['sec'] * 10**9))
    manager._tf_buffer = NS(lookup_transform=lookup)

    manager._record_tf(1.)
    manager._record_tf(2.)

    assert len(manager._tf_window) == 2
    assert manager._last_map_odom_time == 2.


def test_old_future_duplicate_and_out_of_order_messages_rejected(manager):
    def accept(sec, nanosec=0):
        return manager._accept_source('scan', NS(header=NS(
            stamp=NS(sec=sec, nanosec=nanosec))), .5)
    assert not accept(8)
    assert not accept(11)
    assert accept(9, 800000000)
    assert not accept(9, 800000000)
    assert not accept(9, 700000000)
    assert accept(10)


def test_manual_pose_invalidates_previous_solution(manager):
    manager._state = manager.State.WAIT_MANUAL_POSE
    manager._latest_amcl_pose = object()
    manager._last_amcl_time = manager._last_particle_time = 9.
    manager._last_score_time = 9.
    manager._particle_concentration = 1.
    manager._source_stamps['odom'] = 9_876_543_210
    forwarded = []
    manager._pose_seed_publisher = NS(publish=forwarded.append)
    pose = NS(
        position=NS(x=3.0, y=2.0),
        orientation=NS(x=0., y=0., z=0., w=1.),
    )
    manager._on_initial_pose(NS(
        header=NS(frame_id='map', stamp=NS(sec=11, nanosec=123)),
        pose=NS(pose=pose, covariance=[0.] * 36),
    ))
    assert manager._state == manager.State.VERIFY_MANUAL_POSE
    assert manager._awaiting_amcl_initial_pose
    assert manager._requested_initial_pose == (3.0, 2.0, 0.0)
    assert forwarded[0].header.stamp.sec == 9
    assert forwarded[0].header.stamp.nanosec == 876_543_210
    assert forwarded[0].pose.covariance[0] == pytest.approx(.1 ** 2)
    assert forwarded[0].pose.covariance[35] == pytest.approx(
        math.radians(5.) ** 2)
    assert manager._latest_amcl_pose is None
    assert manager._last_particle_time is None
    assert manager._last_score_time is None
    assert manager._quality_since is None
    assert manager._evidence_epoch_ns == 10 * 10**9


def test_old_amcl_pose_cannot_acknowledge_new_manual_estimate(manager):
    manager._awaiting_amcl_initial_pose = True
    manager._requested_initial_pose = (5.0, 4.0, 1.0)
    manager._accept_source = lambda *args: True
    old = NS(pose=NS(pose=NS(
        position=NS(x=1.0, y=1.0),
        orientation=NS(x=0., y=0., z=0., w=1.),
    )))
    manager._on_amcl_pose(old)
    assert manager._awaiting_amcl_initial_pose
    assert manager._latest_amcl_pose is None
    assert manager._quality_failure == 'AMCL_INITIAL_POSE_NOT_ACKNOWLEDGED'


def test_near_amcl_pose_acknowledges_new_manual_estimate(manager):
    manager._awaiting_amcl_initial_pose = True
    manager._requested_initial_pose = (5.0, 4.0, 1.0)
    manager._accept_source = lambda *args: True
    accepted = NS(pose=NS(pose=NS(
        position=NS(x=5.1, y=3.9),
        orientation=NS(x=0., y=0., z=math.sin(.95 / 2),
                       w=math.cos(.95 / 2)),
    )))
    manager._on_amcl_pose(accepted)
    assert not manager._awaiting_amcl_initial_pose
    assert manager._latest_amcl_pose is accepted


def test_motion_grace_is_never_quality_evidence(manager):
    manager._last_odom_time = 10.
    manager._stationary_since = 1.
    manager._moving_since = 9.95
    assert not manager._stopped(10.)
    manager._moving_since = None
    assert manager._stopped(10.)
    assert not manager._stopped(11.)


def test_scan_queries_source_time_and_invalidates_score_on_tf_failure(manager):
    manager._latest_map = object()
    manager._last_score_time = 8.
    requested = []

    def lookup(target, source, stamp):
        requested.append((target, source, stamp.nanoseconds))
        raise LookupError('no transform at scan time')
    manager._tf_buffer = NS(lookup_transform=lookup)
    scan = NS(header=NS(frame_id='base_footprint',
                        stamp=NS(sec=9, nanosec=900000000)),
              ranges=[1.], range_min=.1, range_max=20.)
    manager._on_scan(scan)
    assert requested == [('map', 'base_footprint', 9900000000)]
    assert manager._last_score_time == 8.
    assert manager._scan_tf_error
    assert len(manager._pending_scans) == 1


def test_localization_scan_cannot_weaken_rotation_clearance(manager):
    manager._finite_scan_beams = 0
    manager._scan_min_range = math.inf
    localization_scan = NS(
        header=NS(frame_id='base_footprint', stamp=NS(sec=10, nanosec=0)),
        ranges=[2.0], range_min=.1, range_max=20.)
    manager._latest_map = None
    manager._on_scan(localization_scan)
    assert manager._finite_scan_beams == 0
    assert math.isinf(manager._scan_min_range)

    safety_scan = NS(
        header=NS(frame_id='base_footprint', stamp=NS(sec=10, nanosec=0)),
        ranges=[.51, 2.0], range_min=.1, range_max=20.)
    manager._on_safety_scan(safety_scan)
    assert manager._finite_scan_beams == 2
    assert manager._scan_min_range == pytest.approx(.51)


def test_scan_is_retried_after_its_exact_time_transform_arrives(manager):
    manager._latest_map = NS(info=NS(resolution=0.05))
    manager._last_score_time = None
    ready = False
    requested = []

    def lookup(target, source, stamp):
        requested.append((target, source, stamp.nanoseconds))
        if not ready:
            raise LookupError('transform is one frame late')
        return NS(transform=object())

    manager._tf_buffer = NS(lookup_transform=lookup)
    manager._scan_metrics = {}
    manager._parameter = lambda name: {
        'sensor_freshness_sec': .5,
        'max_future_stamp_sec': .05,
        'scan_match_tolerance_m': .15,
        'occupied_threshold': 65,
        'scan_score_max_beams': 120,
    }[name]
    scan = NS(header=NS(frame_id='base_footprint',
                        stamp=NS(sec=9, nanosec=900000000)),
              ranges=[1.], range_min=.1, range_max=20.)
    manager._on_scan(scan)
    assert manager._last_score_time is None
    assert len(manager._pending_scans) == 1

    ready = True
    assert manager._score_pending_scans()
    assert manager._last_score_time is not None
    assert not manager._pending_scans
    assert requested == [
        ('map', 'base_footprint', 9900000000),
        ('map', 'base_footprint', 9900000000),
    ]


def test_stop_window_is_reset_during_motion_grace(manager):
    manager._publish_zero = lambda: None
    manager._stopped = lambda now: False
    manager._tick()
    assert manager._quality_since is None
    assert manager._quality_failure == 'NOT_STATIONARY'


def test_post_search_amcl_drift_is_rejected_before_acceptance(manager):
    manager._stopped = lambda now: True
    manager._localization_evidence_recent = lambda value, now: True
    manager._last_amcl_time = 10.0
    manager._search_best = {'x': 1.0, 'y': 2.0, 'yaw': 0.1}
    manager._latest_amcl_pose = NS(pose=NS(pose=NS(
        position=NS(x=1.09, y=2.0),
        orientation=NS(x=0.0, y=0.0, z=math.sin(.1 / 2),
                       w=math.cos(.1 / 2)))))

    assert not manager._quality_passes(10.0)
    assert manager._quality_failure == 'POST_SEARCH_POSE_DRIFT'


def test_manual_pose_drift_is_rejected_before_acceptance(manager):
    manager._stopped = lambda now: True
    manager._localization_evidence_recent = lambda value, now: True
    manager._last_amcl_time = 10.0
    manager._manual_reference_pose = (1.0, 2.0, 0.1)
    manager._latest_amcl_pose = NS(pose=NS(pose=NS(
        position=NS(x=1.21, y=2.0),
        orientation=NS(x=0.0, y=0.0, z=math.sin(.1 / 2),
                       w=math.cos(.1 / 2)))))

    assert not manager._quality_passes(10.0)
    assert manager._quality_failure == 'MANUAL_POSE_DRIFT'


def test_candidate_drift_is_rejected_during_recheck(manager):
    manager._stopped = lambda now: True
    manager._localization_evidence_recent = lambda value, now: True
    manager._last_amcl_time = 10.0
    manager._candidate_anchor_pose = (1.0, 2.0, 0.1)
    manager._latest_amcl_pose = NS(pose=NS(pose=NS(
        position=NS(x=1.09, y=2.0),
        orientation=NS(x=0.0, y=0.0, z=math.sin(.1 / 2),
                       w=math.cos(.1 / 2)))))

    assert not manager._quality_passes(10.0)
    assert manager._quality_failure == 'CANDIDATE_POSE_DRIFT'


def test_automatic_mode_skips_amcl_global_particle_spread(manager):
    manager._state = manager.State.START_LOCALIZATION
    manager._state_started = time.monotonic()
    manager._stationary_only = False
    manager._localization_client = NS(service_is_ready=lambda: True)
    manager._future = object()
    manager._future_succeeded = lambda: True
    manager._reset_evidence = lambda: None
    manager._rotation_progress = 99.0
    manager._last_odom_yaw = 1.0
    manager._rotation_data_stale_since = 1.0
    manager._rotation_obstacle_since = 1.0

    manager._tick()

    assert manager._state == manager.State.ROTATE_AND_SCORE
    assert manager._rotation_progress == 0.0
    assert manager._last_odom_yaw is None


def test_completed_rotation_stops_before_close_obstacle_fault(manager):
    manager._state = manager.State.ROTATE_AND_SCORE
    manager._state_started = time.monotonic() - 5.0
    manager._last_odom_time = time.monotonic()
    manager._last_scan_time = time.monotonic()
    manager._last_safety_scan_time = time.monotonic()
    manager._rotation_progress = 2.0 * math.pi
    manager._scan_min_range = 0.50
    manager._rotation_data_stale_since = None
    manager._rotation_obstacle_since = time.monotonic() - 2.0
    manager._tf_window = deque([object()])
    manager._stationary_search_scans = deque([object()])
    manager._search_future = object()
    manager._search_best = object()
    manager._search_runner_up = object()
    manager._quality_since = 1.0
    commands = []
    manager._publish_zero = lambda: commands.append('zero')

    manager._tick()

    assert manager._state == manager.State.SEARCH_GLOBAL_POSE
    assert commands == ['zero']
    assert not manager._tf_window
    assert not manager._stationary_search_scans


def test_candidate_ready_continues_rechecking_and_revokes_stale_pose(manager):
    manager._state = manager.State.CANDIDATE_READY
    manager._state_started = time.monotonic() - 10.0
    manager._candidate_invalid_since = time.monotonic() - 8.0
    manager._quality_failure = 'STALE_AMCL'
    manager._publish_zero = lambda: None
    manager._request_nomotion_update = lambda now: pytest.fail(
        'candidate must not force repeated AMCL no-motion updates')
    manager._record_tf = lambda now: None
    manager._candidate_quality_passes = lambda now: False
    failures = []
    manager._fail = lambda reason, manual_recovery=False: failures.append(
        (reason, manual_recovery))

    manager._tick()

    assert failures == [(
        'candidate localization evidence became invalid: STALE_AMCL', True)]


def test_candidate_ready_recovers_from_transient_data_gap(manager):
    manager._state = manager.State.CANDIDATE_READY
    manager._candidate_invalid_since = time.monotonic() - 1.0
    manager._publish_zero = lambda: None
    manager._request_nomotion_update = lambda now: pytest.fail(
        'candidate must not force repeated AMCL no-motion updates')
    manager._record_tf = lambda now: None
    manager._candidate_quality_passes = lambda now: True

    manager._tick()

    assert manager._candidate_invalid_since is None


def test_qualified_candidate_uses_live_scan_when_amcl_is_quiet(manager):
    manager._stopped = lambda now: True
    manager._last_safety_scan_time = 10.0
    manager._last_scan_time = 10.0
    manager._last_score_time = 10.0
    manager._last_amcl_time = 0.0
    manager._last_particle_time = None
    manager._last_map_odom_time = None
    manager._scan_score = 0.75
    manager._valid_scan_beams = 105
    manager._sampled_scan_beams = 120
    manager._candidate_anchor_pose = (1.0, 2.0, 0.1)
    manager._latest_amcl_pose = NS(pose=NS(pose=NS(
        position=NS(x=1.02, y=2.01),
        orientation=NS(x=0., y=0., z=math.sin(.11 / 2),
                       w=math.cos(.11 / 2)))))

    assert manager._candidate_quality_passes(10.0)
    assert manager._quality_failure == ''


# --- Issue #13 PR1: stationary_only strategy -------------------------------

class FakeWorker:
    """Synchronous stand-in recording every job; results are scripted."""

    def __init__(self, results):
        self.results = list(results)
        self.jobs = []
        self.canceled = 0
        self.token = None

    @property
    def busy(self):
        return self.token is not None

    def submit(self, token, function, *arguments):
        assert not self.busy
        self.token = token
        self.jobs.append((function.__name__, arguments))

    def poll(self):
        if self.token is None:
            return None
        token, self.token = self.token, None
        return (token, 'ok', self.results.pop(0))

    def cancel(self):
        self.canceled += 1
        self.token = None


def _quaternion_ns(yaw):
    return NS(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


@pytest.fixture
def stationary(manager):
    manager.params['localization_strategy'] = 'stationary_only'
    manager._configure_strategy()
    clock = {'ns': 20 * 10**9}
    manager.get_clock = lambda: NS(now=lambda: FakeTime(clock['ns']))
    manager.clock = clock
    manager.create_publisher = lambda *args: pytest.fail(
        'stationary_only created a velocity publisher')
    manager._command_publisher = None
    manager._command_topic = '/cmd_vel_command'
    manager._future = None
    manager._startup_started = time.monotonic()
    manager._sensors_ready_since = None
    manager._stopped = lambda now: True
    manager._last_odom_pose = (0.5, 0.2, 0.3)
    manager.get_logger = lambda: NS(info=lambda msg: None,
                                    error=lambda msg: None)
    manager._latest_grid = localization_hypotheses.SimpleNamespace(
        info=NS(resolution=0.05))
    manager._map_hash = 'ab' * 32
    manager.tf_requests = []

    def lookup(target, source, stamp):
        manager.tf_requests.append((target, source, stamp.nanoseconds))
        pose = (0.5, 0.2, 0.3) if target == 'odom' else (0.0, 0.0, 0.0)
        return NS(transform=NS(translation=NS(x=pose[0], y=pose[1]),
                               rotation=_quaternion_ns(pose[2])))

    manager._tf_buffer = NS(lookup_transform=lookup)
    seeds = []
    manager._publish_global_seed = lambda x, y, yaw: seeds.append(
        (x, y, yaw))
    manager.seeds = seeds
    return manager


def _scan(clock, manager):
    clock['ns'] += 100_000_000
    return NS(header=NS(frame_id='base_footprint', stamp=NS(
        sec=clock['ns'] // 10**9, nanosec=clock['ns'] % 10**9)),
        angle_min=-math.pi, angle_increment=0.1, range_min=0.5,
        range_max=20.0, ranges=[1.0] * 63)


def _feed(manager, count):
    for _ in range(count):
        manager._on_scan(_scan(manager.clock, manager))


def _search_result(complete=True, reason=''):
    winner = localization_contracts.Hypothesis(
        1.0, 2.0, 0.5, 0.9, 0.9, 0.0, 0, (0.9,), ())
    return localization_contracts.SearchResult(
        'x' * 16, 'ab' * 32, complete, (winner,), 10, 0.1, reason)


def _start_to_collect(manager):
    response = NS(success=None, message='')
    manager._state = manager.State.WAIT_FOR_START
    manager._on_start_request(None, response)
    assert response.success
    manager._state = manager.State.START_LOCALIZATION
    manager._state_started = time.monotonic()
    manager._localization_client = NS(service_is_ready=lambda: True)
    manager._future = object()
    manager._future_succeeded = lambda: True
    manager._tick()
    assert manager._state == manager.State.COLLECT_STATIC


def test_stationary_never_commands_motion(stationary):
    # L01: the whole stationary flow, including failure, owns no publisher.
    winner = localization_contracts.Hypothesis(
        1.0, 2.0, 0.5, 0.9, 0.9, 0.0, 0, (0.9,), (0.05, 0.05, 0.02))
    decision = localization_hypotheses.QualityDecision(True, '', winner)
    stationary._worker = FakeWorker([_search_result(), decision])
    _start_to_collect(stationary)
    _feed(stationary, 3)
    stationary._tick()
    assert stationary._state == stationary.State.SEARCH_MULTI_VIEW
    name, arguments = stationary._worker.jobs[0]
    assert name == 'run_search_job'
    train = arguments[1]
    assert [frame.role.value for frame in train] == ['TRAIN'] * 3
    # Every keyframe used TF at its own source stamp.
    stamps = {frame.stamp_ns for frame in train}
    assert {request[2] for request in stationary.tf_requests} >= stamps

    stationary._tick()
    assert stationary._state == stationary.State.VERIFY_HYPOTHESES
    stationary._tick()
    assert len(stationary._worker.jobs) == 1  # waiting for new HOLDOUT
    _feed(stationary, 3)
    stationary._tick()
    name, arguments = stationary._worker.jobs[1]
    assert name == 'run_validation_job'
    holdout = arguments[3]
    assert min(f.stamp_ns for f in holdout) > max(f.stamp_ns for f in train)
    assert not {f.id for f in holdout} & {f.id for f in train}

    stationary._last_odom_pose = (0.5, 0.2, 0.4)
    stationary._tick()
    assert stationary._state == stationary.State.STOP_AND_VERIFY
    # L03 at the manager boundary: the seed follows the current odometry.
    (x, y, yaw), = stationary.seeds
    assert yaw == pytest.approx(0.6)
    assert stationary._command_publisher is None

    stationary._fail('forced failure')
    assert stationary._command_publisher is None


def test_incomplete_search_is_rejected_without_seed(stationary):
    stationary._worker = FakeWorker([
        _search_result(False, 'SEARCH_INCOMPLETE')])
    _start_to_collect(stationary)
    _feed(stationary, 3)
    stationary._tick()
    stationary._tick()
    assert stationary._state == stationary.State.SAFE_STOP
    assert stationary._quality_failure == 'SEARCH_INCOMPLETE'
    assert stationary._manual_recovery_allowed
    assert not stationary.seeds


def test_ambiguous_validation_falls_back_to_manual_pose(stationary):
    decision = localization_hypotheses.QualityDecision(
        False, 'AMBIGUOUS_LOCATION')
    stationary._worker = FakeWorker([_search_result(), decision])
    _start_to_collect(stationary)
    _feed(stationary, 3)
    stationary._tick()
    stationary._tick()
    _feed(stationary, 3)
    stationary._tick()
    stationary._tick()
    assert stationary._state == stationary.State.SAFE_STOP
    assert stationary._confined_status()['ambiguity_reason'] == (
        'AMBIGUOUS_LOCATION')
    assert stationary._confined_status()['manual_pose_allowed'] is True
    assert not stationary.seeds


def test_moving_robot_discards_partial_frames(stationary):
    stationary._worker = FakeWorker([])
    _start_to_collect(stationary)
    _feed(stationary, 2)
    stationary._tick()
    assert len(stationary._keyframes) == 2
    stationary._stopped = lambda now: False
    stationary._tick()
    assert stationary._keyframes == []
    assert stationary._worker.jobs == []


def test_cancel_terminates_search_and_drops_result(stationary):
    stationary._worker = FakeWorker([_search_result()])
    _start_to_collect(stationary)
    _feed(stationary, 3)
    stationary._tick()
    response = NS(success=None, message='')
    stationary._on_cancel_request(None, response)
    assert response.success
    assert stationary._worker.canceled >= 1
    assert stationary._state == stationary.State.SAFE_STOP
    assert stationary._quality_failure == 'CANCELED'
    stationary._on_cancel_request(None, response)  # idempotent
    assert stationary._state == stationary.State.SAFE_STOP


def test_stale_token_result_is_not_used(stationary):
    stationary._worker = FakeWorker([_search_result()])
    _start_to_collect(stationary)
    _feed(stationary, 3)
    stationary._tick()
    stationary._worker.token = ('old-session', 'ab' * 32)
    stationary._tick()
    assert stationary._state == stationary.State.SAFE_STOP
    assert stationary._quality_failure == 'MAP_CHANGED'
    assert stationary._search_result is None


def test_odometry_jump_rejects_confined_session(stationary):
    stationary._worker = FakeWorker([])
    _start_to_collect(stationary)
    stationary._last_odom_pose = (0.0, 0.0, 0.0)
    stationary._last_odom_yaw = 0.0
    stationary._last_odom_time = None
    stationary._stationary_since = None
    stationary._moving_since = None
    stationary._accept_source = lambda *args: True
    stationary._on_odom(NS(
        header=NS(stamp=NS(sec=30, nanosec=0)),
        pose=NS(pose=NS(position=NS(x=1.0, y=0.0),
                        orientation=_quaternion_ns(0.0))),
        twist=NS(twist=NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0)))))
    assert stationary._state == stationary.State.SAFE_STOP
    assert stationary._quality_failure == 'ODOM_JUMP'


def test_strategy_conflicts_abort_startup(manager):
    manager.params['localization_strategy'] = 'segmented_rotation'
    with pytest.raises(localization_contracts.ContractError):
        manager._configure_strategy()
    manager.params['localization_strategy'] = 'stationary_only'
    manager.params['motion_policy'] = 'guarded'
    manager.params['motion_profile_path'] = '/tmp/profile.json'
    with pytest.raises(localization_contracts.ContractError):
        manager._configure_strategy()


def test_legacy_status_keeps_existing_fields(manager):
    status = manager._confined_status()
    assert status['strategy'] == 'legacy_full_rotation'
    assert status['motion_policy'] == 'forbid'
    assert set(localization_contracts.STATUS_EXTENSION_FIELDS) <= set(status)
