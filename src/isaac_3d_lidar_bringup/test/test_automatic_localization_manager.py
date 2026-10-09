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
from isaac_3d_lidar_bringup import localization_motion_guard
from isaac_3d_lidar_bringup import localization_observations
from isaac_3d_lidar_bringup import localization_rotation_policy
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
        STARTUP = 0
        PAUSE = 1
        RESUME = 2

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
    scope.update(vars(localization_motion_guard))
    scope.update(vars(localization_rotation_policy))
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
    node._configure_probe_link()
    node._worker = None
    node._reset_confined_session()
    node._pose_speed = scope['PoseSpeed'](
        defaults['stationary_speed_window_sec'])
    node._last_twist_linear_speed = math.inf
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
    node._future = None
    node._navigation_may_be_active = False
    node._navigation_pause_state = None
    node._pause_future = None
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


class FakeFuture:
    def __init__(self, response=None, done=True):
        self._response = response
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._response


def _activating(manager, startup_future, ready=True):
    """Put the manager in START_NAVIGATION with Nav2 calls recorded."""
    manager._validation_only = False
    manager._state = manager.State.START_NAVIGATION
    manager._state_started = time.monotonic() - 1.0
    manager._command_publisher = None
    manager._manual_recovery_allowed = False
    manager._publish_zero = lambda: None
    # Legacy SAFE_STOP recreates its zero-velocity publisher.
    manager._ensure_command_publisher = lambda: None
    manager._stopped = lambda now: True
    manager.get_logger = lambda: NS(info=lambda msg: None,
                                    error=lambda msg: None)
    calls = []

    def call(request):
        calls.append(request.command)
        return startup_future if len(calls) == 1 else manager._next_future

    manager._navigation_client = NS(service_is_ready=lambda: ready,
                                    call_async=call)
    manager._next_future = FakeFuture(NS(success=True))
    return calls


@pytest.mark.parametrize('startup, expire', [
    (FakeFuture(done=False), True),           # STARTUP timed out
    (FakeFuture(NS(success=False)), False),   # STARTUP reported failure
])
def test_failed_nav2_activation_pauses_navigation(manager, startup, expire):
    # Review of PR #16: SAFE_STOP was reported while the in-flight STARTUP
    # could still activate Nav2.  The failure must now pause Nav2.
    calls = _activating(manager, startup)
    manager._tick()                       # sends STARTUP
    assert calls == [FakeManageLifecycleNodes.Request.STARTUP]
    if expire:
        manager._state_started -= manager.params['service_timeout_sec']
    manager._tick()                       # timeout or failure -> SAFE_STOP

    assert manager._state == manager.State.SAFE_STOP
    assert calls[-1] == FakeManageLifecycleNodes.Request.PAUSE
    assert manager._navigation_pause_state == 'requested'

    manager._tick()                       # PAUSE acknowledged
    assert manager._navigation_pause_state == 'confirmed'
    assert manager._navigation_may_be_active is False


def test_failure_before_startup_request_does_not_pause(manager):
    calls = _activating(manager, FakeFuture(done=False), ready=False)
    manager._state_started -= manager.params['service_timeout_sec'] + 1.0
    manager._tick()                       # service never ready -> fail

    assert manager._state == manager.State.SAFE_STOP
    assert calls == []
    assert manager._navigation_pause_state is None


def test_unconfirmed_pause_is_reported_not_hidden(manager):
    calls = _activating(manager, FakeFuture(NS(success=False)))
    manager._next_future = FakeFuture(NS(success=False))
    manager._tick()
    manager._tick()
    manager._tick()

    assert calls[-1] == FakeManageLifecycleNodes.Request.PAUSE
    assert manager._navigation_pause_state == 'failed'
    assert manager._navigation_may_be_active is True


def test_pause_waits_for_lifecycle_service_then_is_sent(manager):
    ready = {'value': True}
    calls = _activating(manager, FakeFuture(NS(success=False)))
    client = manager._navigation_client
    manager._navigation_client = NS(
        service_is_ready=lambda: ready['value'],
        call_async=client.call_async)
    manager._tick()                       # STARTUP sent
    ready['value'] = False                # lifecycle manager disappears
    manager._state_started -= manager.params['service_timeout_sec']
    manager._tick()                       # 'service unavailable' failure
    assert manager._state == manager.State.SAFE_STOP
    assert manager._navigation_pause_state == 'service_unavailable'
    assert calls == [FakeManageLifecycleNodes.Request.STARTUP]

    ready['value'] = True
    manager._tick()
    assert calls[-1] == FakeManageLifecycleNodes.Request.PAUSE
    assert manager._navigation_pause_state == 'requested'


def test_activation_after_confirmed_pause_uses_resume(manager):
    calls = _activating(manager, FakeFuture(NS(success=False)))
    manager._tick()
    manager._tick()
    manager._tick()
    assert manager._navigation_pause_state == 'confirmed'

    manager._future = None
    manager._call_lifecycle(manager._navigation_client)

    assert calls[-1] == FakeManageLifecycleNodes.Request.RESUME
    assert manager._navigation_may_be_active is True
    assert manager._navigation_pause_state is None


def _postdated(manager, sec):
    """Header stamp of an AMCL map->odom computed from a scan at ``sec``."""
    return sec + manager.params['map_odom_postdate_sec']


def test_cached_tf_cannot_extend_quality_window(manager):
    stamp = _postdated(manager, 10)
    stamped = NS(
        header=NS(stamp=NS(sec=int(stamp), nanosec=round(stamp % 1 * 1e9))),
        transform=NS(translation=NS(x=0., y=0.),
                     rotation=NS(x=0., y=0., z=0., w=1.)))
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
        stamp = _postdated(manager, clock['sec'])
        return NS(
            header=NS(stamp=NS(sec=int(stamp),
                               nanosec=round(stamp % 1 * 1e9))),
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


@pytest.mark.parametrize('scan_age, accepted', [
    (0.07, True),    # live AMCL republish, post-dated by transform_tolerance
    (0.60, False),   # computed from a scan older than sensor freshness
    (-0.70, False),  # beyond the clock-skew limit even after post-dating
])
def test_map_odom_freshness_uses_amcl_scan_time(manager, scan_age, accepted):
    # 2026-10-05: AMCL transform_tolerance=1.5 put every sample 1.4 s in
    # the future, so the raw-stamp check rejected all of them (STALE_TF).
    stamp = _postdated(manager, 10.0 - scan_age)
    manager._tf_buffer = NS(lookup_transform=lambda *args: NS(
        header=NS(stamp=NS(sec=int(stamp), nanosec=round(stamp % 1 * 1e9))),
        transform=NS(translation=NS(x=0., y=0.),
                     rotation=NS(x=0., y=0., z=0., w=1.))))

    manager._record_tf(1.)

    assert (manager._last_map_odom_time == 1.) is accepted
    assert manager._timing['map_odom']['source_age_sec'] == pytest.approx(
        scan_age, abs=1e-6)


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


def _feed_odom(manager, count, x_step, twist_x, start_sec=30):
    manager._accept_source = lambda *args: True
    manager._last_odom_pose = manager._last_odom_yaw = None
    manager._stationary_since = manager._moving_since = None
    for index in range(count):
        ns = index * 100_000_000
        manager._on_odom(NS(
            header=NS(stamp=NS(sec=start_sec + ns // 10**9,
                               nanosec=ns % 10**9)),
            pose=NS(pose=NS(position=NS(x=index * x_step, y=0.0),
                            orientation=_quaternion_ns(0.0))),
            twist=NS(twist=NS(linear=NS(x=twist_x, y=0.0),
                              angular=NS(z=0.0)))))


def test_biased_twist_at_rest_is_stationary(manager):
    # 2026-10-08 real robot: FAST-LIO's twist stayed at ~0.027 m/s after a
    # long probe turn while the position did not move.
    _feed_odom(manager, 12, 0.0, 0.027)
    assert manager._stationary_since is not None
    assert manager._moving_since is None
    assert manager._last_twist_linear_speed == pytest.approx(0.027)


def test_translation_with_zero_twist_is_moving(manager):
    _feed_odom(manager, 12, 0.005, 0.0)        # 0.05 m/s from positions
    assert manager._stationary_since is None
    assert manager._last_linear_speed == pytest.approx(0.05)


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


def test_cancel_is_refused_while_nav2_activation_may_be_in_flight(
        stationary):
    stationary._transition(stationary.State.START_NAVIGATION)
    response = NS(success=None, message='')
    stationary._on_cancel_request(None, response)
    assert response.success is False
    assert stationary._state == stationary.State.START_NAVIGATION


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


@pytest.mark.parametrize('validation_only', [True, False])
@pytest.mark.parametrize('policy', ['forbid', 'guarded'])
@pytest.mark.parametrize('strategy', [
    'legacy_full_rotation', 'stationary_only', 'segmented_rotation'])
def test_strategy_matrix(manager, strategy, policy, validation_only):
    # C09: guarded is valid only for segmented_rotation (with a profile);
    # forbid is valid everywhere; validation_only is orthogonal.
    manager.params['localization_strategy'] = strategy
    manager.params['motion_policy'] = policy
    manager.params['motion_profile_path'] = '/tmp/profile.json'
    manager._validation_only = validation_only
    if policy == 'guarded' and strategy != 'segmented_rotation':
        with pytest.raises(localization_contracts.ContractError):
            manager._configure_strategy()
        return
    manager._configure_strategy()
    assert manager._segmented is (strategy == 'segmented_rotation')
    assert manager._motion_policy.value == policy


def test_guarded_needs_profile_and_attestation_needs_guarded(manager):
    manager.params['localization_strategy'] = 'segmented_rotation'
    manager.params['motion_policy'] = 'guarded'
    manager.params['motion_profile_path'] = ''
    with pytest.raises(localization_contracts.ContractError):
        manager._configure_strategy()
    manager.params['motion_policy'] = 'forbid'
    manager.params['operator_rotation_clear'] = True
    with pytest.raises(localization_contracts.ContractError):
        manager._configure_strategy()


def test_legacy_status_keeps_existing_fields(manager):
    status = manager._confined_status()
    assert status['strategy'] == 'legacy_full_rotation'
    assert status['motion_policy'] == 'forbid'
    assert set(localization_contracts.STATUS_EXTENSION_FIELDS) <= set(status)


# --- segmented_rotation: probing through the motion guard (PR3) -------------

EXTRINSICS, CONTROL = 'a' * 64, 'b' * 64


def _accepted_profile():
    footprint = ((0.155, 0.133), (0.155, -0.133),
                 (-0.130, -0.133), (-0.130, 0.133))
    return localization_contracts.MotionProfile(
        1, localization_rotation_policy.footprint_geometry_hash(
            footprint, 0.05), EXTRINSICS, CONTROL, ('bag-1',),
        0.05, 0.03, 0.1, True, 'ACCEPTED')


def _room_scan(manager, range_min=0.5):
    stamp = manager.clock['ns']
    ranges = []
    for index in range(360):
        angle = -math.pi + index * 2 * math.pi / 360
        ranges.append(1.2 / max(abs(math.cos(angle)), abs(math.sin(angle))))
    return NS(header=NS(frame_id='base_footprint', stamp=NS(
        sec=stamp // 10**9, nanosec=stamp % 10**9)),
        angle_min=-math.pi, angle_increment=2 * math.pi / 360,
        range_min=range_min, range_max=8.0, ranges=ranges)


@pytest.fixture
def segmented(stationary, tmp_path):
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String

    manager = stationary
    # The fixture executes the manager without its imports; add the ROS
    # message types the segmented path uses.
    scope = type(manager)._configure_probe_link.__globals__
    scope.update(String=String, Twist=Twist)
    profile = tmp_path / 'profile.json'
    profile.write_text(localization_contracts.encode_motion_profile(
        _accepted_profile()))
    manager.params.update({
        'localization_strategy': 'segmented_rotation',
        'motion_policy': 'guarded',
        'motion_profile_path': str(profile),
        'extrinsics_hash': EXTRINSICS,
        'control_chain_hash': CONTROL,
        'operator_rotation_clear': True,
    })
    manager._configure_strategy()
    manager.requests = []

    def create_publisher(kind, topic, qos):
        assert kind is not scope['Twist'] and 'cmd_vel' not in topic, (
            'segmented_rotation manager created a velocity publisher')
        return NS(publish=lambda msg: manager.requests.append(
            localization_contracts.decode_motion_request(msg.data)))

    manager.create_publisher = create_publisher
    manager.create_subscription = lambda *args, **kwargs: None
    manager.publishers_on_cmd = 0
    manager.count_publishers = lambda topic: manager.publishers_on_cmd
    manager._configure_probe_link()
    return manager


def _guard(manager, state, sequence=None, reason='', stopped=None,
           travel=0.0):
    status = localization_contracts.MotionStatus(
        1, manager._probe.session,
        manager._probe.sequence if sequence is None else sequence,
        localization_contracts.GuardState(state), reason, 0.0, travel,
        state != 'ROTATING' if stopped is None else stopped, ())
    manager._guard_status = (status, time.monotonic())


def _ambiguous_session(manager):
    manager._begin_confined_session(time.monotonic())
    manager._search_result = _search_result()
    manager._latest_safety_scan = _room_scan(manager)
    manager._reject_or_probe('AMBIGUOUS_LOCATION', time.monotonic())
    assert manager._state == manager.State.PLAN_PROBE


def test_ambiguity_probes_through_the_guard_then_releases(segmented):
    manager = segmented
    rejected = localization_hypotheses.QualityDecision(
        False, 'AMBIGUOUS_LOCATION')
    winner = localization_contracts.Hypothesis(
        1.0, 2.0, 0.5, 0.9, 0.9, 0.0, 0, (0.9,), (0.05, 0.05, 0.02))
    accepted = localization_hypotheses.QualityDecision(True, '', winner)
    manager._worker = FakeWorker([_search_result(), rejected,
                                  _search_result(), accepted])
    _start_to_collect(manager)
    assert manager.requests[-1].operation.value == 'STOP'   # handshake
    _feed(manager, 3)
    manager._tick()
    manager._tick()
    _feed(manager, 3)
    manager._tick()
    manager._latest_safety_scan = _room_scan(manager)
    _guard(manager, 'STOPPED')
    manager._tick()                       # validation -> ambiguous
    assert manager._state == manager.State.PLAN_PROBE

    _guard(manager, 'STOPPED')
    manager._tick()                       # read-only plan -> ROTATE
    assert manager._state == manager.State.EXECUTE_PROBE
    rotate = manager.requests[-1]
    assert rotate.operation.value == 'ROTATE'
    assert rotate.profile_hash == localization_motion_guard.profile_hash(
        _accepted_profile())
    assert manager._confined_status()['unknown_sweep_cells'] == 0

    _guard(manager, 'ROTATING', travel=0.3)
    manager._tick()
    _guard(manager, 'STOPPED')
    manager._tick()                       # segment done -> STOP, settle
    assert manager._state == manager.State.SETTLE_PROBE
    assert manager.requests[-1].operation.value == 'STOP'
    _guard(manager, 'STOPPED', stopped=True)
    manager._state_started -= manager.params['probe_settle_sec']
    manager._tick()
    assert manager._state == manager.State.COLLECT_STATIC
    assert manager._view_id == 1
    assert all(f.role.value == 'TRAIN' for f in manager._keyframes)

    _feed(manager, 3)
    manager._tick()                       # second view re-checks
    name, arguments = manager._worker.jobs[-1]
    assert name == 'run_recheck_job'
    assert arguments[1].complete
    assert {frame.view_id for frame in arguments[3]} == {0, 1}
    assert manager._confined_status()['search_recheck'] is True

    manager._tick()
    _feed(manager, 3)
    manager._tick()
    manager._tick()                       # accepted -> RELEASE the guard
    assert manager._state == manager.State.STOP_AND_VERIFY
    assert manager.requests[-1].operation.value == 'RELEASE'
    assert manager._command_publisher is None


def test_handoff_no_two_publishers(segmented):
    # C07: Nav2 STARTUP only after the guard reports RELEASED and no
    # publisher remains on /cmd_vel_command.
    manager = segmented
    manager._begin_confined_session(time.monotonic())
    manager._validation_only = False
    manager._probe.release()
    manager._state = manager.State.START_NAVIGATION
    manager._state_started = time.monotonic() - 1.0
    manager._publish_zero = lambda: None
    calls = []
    manager._navigation_client = NS(
        service_is_ready=lambda: True,
        call_async=lambda request: calls.append(request) or object())

    _guard(manager, 'STOPPED')
    manager._tick()
    assert calls == []
    _guard(manager, 'RELEASED')
    manager.publishers_on_cmd = 1             # guard publisher still alive
    manager._tick()
    assert calls == []
    manager.publishers_on_cmd = 0
    manager._tick()
    assert len(calls) == 1


def test_nav2_publisher_after_startup_does_not_block_activation(segmented):
    # 2026-10-06 real robot: after STARTUP the velocity smoother became the
    # /cmd_vel_command publisher; re-checking "no publisher" every tick hid
    # the STARTUP result and timed out, pausing a healthy Nav2.
    manager = segmented
    manager._begin_confined_session(time.monotonic())
    manager._validation_only = False
    manager._probe.release()
    manager._state = manager.State.START_NAVIGATION
    manager._state_started = time.monotonic() - 1.0
    manager._publish_zero = lambda: None
    startup = {'done': False}
    manager._navigation_client = NS(
        service_is_ready=lambda: True,
        call_async=lambda request: NS(
            done=lambda: startup['done'],
            result=lambda: NS(success=True)))
    _guard(manager, 'RELEASED')
    manager._tick()                       # released, no publisher: STARTUP
    assert manager._future is not None

    manager.publishers_on_cmd = 1         # Nav2 velocity smoother
    startup['done'] = True
    _guard(manager, 'RELEASED')
    manager._tick()
    assert manager._state == manager.State.READY


def test_guard_that_never_releases_fails_instead_of_activating(segmented):
    manager = segmented
    manager._begin_confined_session(time.monotonic())
    manager._validation_only = False
    manager._state = manager.State.START_NAVIGATION
    manager._state_started = (time.monotonic()
                              - manager.params['service_timeout_sec'] - 1)
    manager._publish_zero = lambda: None
    manager._navigation_client = NS(
        service_is_ready=lambda: True,
        call_async=lambda request: pytest.fail('Nav2 activated'))
    _guard(manager, 'STOPPED')
    manager._tick()
    assert manager._state == manager.State.SAFE_STOP


def test_search_worker_stall_only_leases_stop(segmented):
    # C02: a stuck search never produces a ROTATE; the lease keeps STOP.
    manager = segmented

    class Stuck(FakeWorker):
        def poll(self):
            return None

    manager._worker = Stuck([])
    _start_to_collect(manager)
    _feed(manager, 3)
    manager._tick()
    assert manager._state == manager.State.SEARCH_MULTI_VIEW
    for _ in range(20):
        manager._tick()
    assert {r.operation.value for r in manager.requests} == {'STOP'}


def test_refuted_recheck_searches_the_whole_map_again(segmented):
    # Spec 5.3: when the re-check refutes every candidate, a new map-wide
    # search must complete before anything can be accepted.
    manager = segmented
    refuted = localization_contracts.SearchResult(
        'x' * 16, 'ab' * 32, True, (localization_contracts.Hypothesis(
            1.0, 2.0, 0.5, 0.3, 0.9, 0.0, 0, (0.3,), ()),), 10, 0.1, '')
    manager._worker = FakeWorker([refuted, _search_result()])
    _start_to_collect(manager)
    manager._recheck_basis = (_search_result(),
                              localization_contracts.SE2(0.0, 0.0, 0.0))
    _feed(manager, 3)
    manager._tick()
    manager._tick()                       # refuted -> map-wide search
    assert manager._state == manager.State.SEARCH_MULTI_VIEW
    assert [job[0] for job in manager._worker.jobs] == [
        'run_recheck_job', 'run_search_job']
    assert manager._recheck_basis is None
    manager._tick()
    assert manager._state == manager.State.VERIFY_HYPOTHESES


def test_map_wide_search_keeps_and_passes_the_coarse_cache(segmented):
    manager = segmented
    cache = localization_hypotheses.CoarseCache(('key',), {0: (0, ())})
    incomplete = _search_result(False, 'SEARCH_INCOMPLETE')
    manager._worker = FakeWorker([
        localization_hypotheses.SearchOutput(incomplete, cache)])
    _start_to_collect(manager)
    _feed(manager, 3)
    manager._tick()
    name, arguments = manager._worker.jobs[-1]
    assert name == 'run_search_job' and arguments[-1] is None
    manager._latest_safety_scan = _room_scan(manager)
    manager._tick()                       # incomplete -> probe planning
    assert manager._coarse_cache is cache
    assert manager._search_result is incomplete
    manager._search_reference = localization_contracts.SE2(0.0, 0.0, 0.0)
    manager._start_next_view()
    assert manager._recheck_basis is None
    _feed(manager, 3)
    manager._tick()                       # next map-wide search
    name, arguments = manager._worker.jobs[-1]
    assert name == 'run_search_job' and arguments[-1] is cache


@pytest.mark.parametrize('complete', [True, False])
def test_only_a_complete_search_is_rechecked(segmented, complete):
    # An incomplete search may have dropped alternatives.
    manager = segmented
    manager._begin_confined_session(time.monotonic())
    manager._search_result = _search_result(
        complete, '' if complete else 'SEARCH_INCOMPLETE')
    manager._search_reference = localization_contracts.SE2(0.1, 0.0, 0.2)
    manager._start_next_view()
    assert (manager._recheck_basis is not None) == complete
    assert manager._search_result is None


@pytest.mark.parametrize('setup, reason', [
    ('guard_refuses', 'UNKNOWN_SWEEP'),
    ('guard_silent', 'CONTROL_CONFLICT'),
])
def test_probe_failures_reject_and_stop(segmented, setup, reason):
    manager = segmented
    _ambiguous_session(manager)
    _guard(manager, 'STOPPED')
    manager._tick()
    assert manager._state == manager.State.EXECUTE_PROBE
    if setup == 'guard_refuses':
        _guard(manager, 'STOPPED', reason='UNKNOWN_SWEEP')
    else:
        manager._guard_status = None
    manager._tick()
    assert manager._state == manager.State.SAFE_STOP
    assert manager._reject_reason == reason
    assert manager.requests[-1].operation.value == 'STOP'


def test_blind_zone_without_attestation_never_requests_rotation(segmented):
    manager = segmented
    manager.params['operator_rotation_clear'] = False
    _ambiguous_session(manager)
    _guard(manager, 'STOPPED')
    manager._tick()
    assert manager._reject_reason == 'UNKNOWN_SWEEP'
    assert all(r.operation.value != 'ROTATE' for r in manager.requests)


def test_forbid_policy_rejects_probe_planning(segmented):
    manager = segmented
    manager.params['motion_policy'] = 'forbid'
    manager.params['operator_rotation_clear'] = False
    manager._configure_strategy()
    _ambiguous_session(manager)
    manager._tick()
    assert manager._reject_reason == 'PROFILE_INVALID'
    assert all(r.operation.value != 'ROTATE' for r in manager.requests)


def test_forced_probe_runs_once_even_after_a_passing_result(segmented):
    # Real-robot test mode: the stationary result passes, the manager still
    # probes once through the guard, then accepts the next passing result.
    manager = segmented
    manager.params['force_probe_once'] = True
    manager._configure_strategy()
    winner = localization_contracts.Hypothesis(
        1.0, 2.0, 0.5, 0.9, 0.9, 0.0, 0, (0.9,), (0.05, 0.05, 0.02))
    accepted = localization_hypotheses.QualityDecision(True, '', winner)
    manager._worker = FakeWorker([_search_result(), accepted,
                                  _search_result(), accepted])
    _start_to_collect(manager)
    _feed(manager, 3)
    manager._tick()
    manager._tick()
    _feed(manager, 3)
    manager._tick()
    manager._latest_safety_scan = _room_scan(manager)
    _guard(manager, 'STOPPED')
    manager._tick()                       # passed, but probe anyway
    assert manager._state == manager.State.PLAN_PROBE
    assert manager._confined_status()['forced_probe_test'] is True
    assert manager.seeds == []

    _guard(manager, 'STOPPED')
    manager._tick()
    assert manager.requests[-1].operation.value == 'ROTATE'
    _guard(manager, 'ROTATING', travel=0.3)
    manager._tick()
    _guard(manager, 'STOPPED')
    manager._tick()
    _guard(manager, 'STOPPED', stopped=True)
    manager._state_started -= manager.params['probe_settle_sec']
    manager._tick()
    assert manager._state == manager.State.COLLECT_STATIC

    _feed(manager, 3)
    manager._tick()
    manager._tick()
    _feed(manager, 3)
    manager._tick()
    manager._tick()                       # second pass is accepted
    assert manager._state == manager.State.STOP_AND_VERIFY
    assert manager.requests[-1].operation.value == 'RELEASE'
    assert len(manager.seeds) == 1


@pytest.mark.parametrize('strategy, policy, validation_only', [
    ('segmented_rotation', 'guarded', False),
    ('segmented_rotation', 'forbid', True),
    ('stationary_only', 'forbid', True),
])
def test_force_probe_once_is_validation_only_guarded_test(
        manager, strategy, policy, validation_only):
    manager.params.update({'localization_strategy': strategy,
                           'motion_policy': policy,
                           'motion_profile_path': '/tmp/profile.json',
                           'force_probe_once': True})
    manager._validation_only = validation_only
    with pytest.raises(localization_contracts.ContractError):
        manager._configure_strategy()


def test_guard_and_manager_share_session_limits(manager):
    # The guard does not read the manager's YAML.  If its session budget or
    # attestation lifetime were shorter, late probes of a longer session
    # would be refused by the guard (2026-10-09: 240 -> 360 s).
    package = Path(__file__).resolve().parents[1]
    guard = ast.parse((package / 'isaac_3d_lidar_bringup'
                       / 'localization_motion_guard_node.py').read_text())
    declared = {
        call.args[0].value: call.args[1].value
        for call in ast.walk(guard)
        if isinstance(call, ast.Call) and getattr(call.func, 'id', '') == (
            'declare') and len(call.args) == 2
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[1], ast.Constant)}
    config = (package / 'config/nav2/carbot_auto_localization_real.yaml'
              ).read_text()
    for name in ('session_timeout_sec', 'attestation_max_age_sec'):
        assert declared[name] == manager.params[name], name
    assert f"session_timeout_sec: {manager.params['session_timeout_sec']}" in (
        config)
    assert localization_rotation_policy.ProbeBudgetLimits().max_session_s == (
        manager.params['session_timeout_sec'])
