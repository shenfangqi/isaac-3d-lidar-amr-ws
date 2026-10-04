"""Non-actuating protocol, cancellation and visibility regressions."""

from dataclasses import replace
import math
import os
import json
import time
from types import SimpleNamespace

import pytest

from carbot_nav_recovery.failure_evidence import (
    ControllerFailureEvidence, FailureCause, failure_gate,
)
from carbot_nav_recovery.runtime_context import RecoveryContext
from carbot_nav_recovery.sensor_visibility import scan_visibility
from carbot_nav_recovery.swept_footprint import (
    CostmapSnapshot, ObservedFreeSpaceSnapshot, Pose2D,
)
from carbot_nav_recovery.trace_retreat import TraceContext


def test_failure_requires_same_context_request_and_actual_collision():
    context = TraceContext('goal', 'epoch', 'map')
    evidence = ControllerFailureEvidence(
        context, 'request', 'FollowPath', FailureCause.COLLISION_PREDICTED,
        10.0, 20.0, True)
    def check(item, request='request'):
        return failure_gate(item, context, request,
                            now_ros_sec=10.1, now_monotonic_sec=20.1)
    assert check(evidence) == 'OK'
    assert check(evidence, 'old') == 'FAILURE_REQUEST_MISMATCH'
    assert check(replace(evidence, cause=FailureCause.NO_PROGRESS)) != 'OK'
    assert check(replace(evidence, source_stamp_sec=9)) != 'OK'
    assert check(replace(evidence, collision_checked=False)) != 'OK'
    assert check(replace(evidence, context=TraceContext('other', 'epoch', 'map'))) != 'OK'


def scan():
    return SimpleNamespace(
        angle_min=-math.pi, angle_increment=math.pi/180,
        range_min=0.5, range_max=5.0, ranges=[3.0]*361,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))


def grid(x, y, resolution=0.1):
    return CostmapSnapshot(1, 1, resolution, x, y, (0,), 'map', 10, 20)


def test_visibility_blind_zone_interior_invalid_beam_and_obstacle():
    beam = scan()
    pose = Pose2D(0, 0, 0)
    assert scan_visibility(beam, pose, grid(1, -0.05), 20).observed_free == (True,)
    assert scan_visibility(beam, pose, grid(0.4, 0), 20).observed_free == (False,)
    beam.ranges[180] = float('nan')
    assert scan_visibility(beam, pose, grid(1, -0.05), 20).observed_free == (False,)
    beam.ranges[180] = 0.75
    assert scan_visibility(beam, pose, grid(1, -0.05), 20).observed_free == (False,)


def test_runtime_no_progress_is_not_motion_authorization():
    state = RecoveryContext()
    state.set_goal('goal')
    for i in range(36):
        state.observe(Pose2D(0, 0, 0), 10+i*0.1, 20+i*0.1,
                      localization_ready=True, feedback_fresh=True,
                      odometry_fresh=True, tf_fresh=True, command_fresh=True,
                      linear_command=0.1, angular_command=0,
                      linear_speed=0, angular_speed=0)
    assert state.reason == 'NO_PROGRESS_OBSERVED'
    assert not state.report()['motion_eligible']
    budget = state.budget
    state.invalidate('LOCALIZATION_CHANGED')
    assert not state.history.samples
    assert state.budget is budget
    assert budget.invalid_reason
    state.set_goal(None)
    assert state.context is None


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='isolated ROS domain required')
def test_coordinator_missing_acceptance_and_exception_never_output_motion():
    import rclpy
    from carbot_recovery_interfaces.srv import RecoveryStep
    from carbot_nav_recovery.recovery_coordinator import RecoveryCoordinator
    rclpy.init()
    node = RecoveryCoordinator()
    try:
        assert tuple(value for point in node._footprint for value in point) \
            == pytest.approx((0.165, 0.143, 0.165, -0.143,
                              -0.140, -0.143, -0.140, 0.143))
        request = RecoveryStep.Request()
        response = node._step(request, RecoveryStep.Response())
        assert response.reason == 'PHYSICAL_ACCEPTANCE_REQUIRED'
        assert response.command.linear.x == response.command.angular.z == 0
        node._profile = {}  # Fault injection; no execution or publisher enabled.
        def fail(_):
            raise RuntimeError('injected')
        node._start = fail
        response = RecoveryStep.Response()
        response.command.linear.x = 0.1
        response = node._step(request, response)
        assert response.status == RecoveryStep.Response.FAILED
        assert response.command.linear.x == response.command.angular.z == 0
        topics = [name for name, _ in node.get_publisher_names_and_types_by_node(
            node.get_name(), node.get_namespace())]
        assert not any('cmd_vel' in name for name in topics)
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='isolated ROS domain required')
def test_acceptance_profile_requires_real_absolute_evidence_directory(tmp_path):
    from carbot_nav_recovery.recovery_coordinator import load_acceptance_profile

    profile_path = tmp_path / 'acceptance.json'
    profile = {
        'physical_acceptance_complete': True,
        'evidence_directory': str(tmp_path),
        'braking_distance_m': 0.02,
        'braking_yaw_rad': 0.08,
        'position_margin_m': 0.02,
        'max_odom_gap_sec': 0.2,
    }
    profile_path.write_text(json.dumps(profile))
    assert load_acceptance_profile(profile_path) == profile

    profile['evidence_directory'] = 'relative/path'
    profile_path.write_text(json.dumps(profile))
    with pytest.raises(ValueError):
        load_acceptance_profile(profile_path)

    profile['evidence_directory'] = str(tmp_path / 'missing')
    profile_path.write_text(json.dumps(profile))
    with pytest.raises(ValueError):
        load_acceptance_profile(profile_path)

    profile['evidence_directory'] = str(tmp_path)
    profile['braking_distance_m'] = True
    profile_path.write_text(json.dumps(profile))
    with pytest.raises(ValueError):
        load_acceptance_profile(profile_path)


@pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID') != '73',
                    reason='isolated ROS domain required')
def test_rotation_selection_is_sliced_across_control_heartbeats():
    import rclpy
    from carbot_nav_recovery.recovery_coordinator import RecoveryCoordinator
    from carbot_nav_recovery.swept_footprint import Pose2D

    rclpy.init()
    node = RecoveryCoordinator()
    try:
        node._profile = {
            'braking_yaw_rad': 0.08,
            'position_margin_m': 0.0,
        }
        checked = []
        planned = []
        node._clear = lambda path: checked.append(path)
        node._request_plan = lambda session: planned.append(tuple(session['candidates']))
        session = {
            'angles': [15, -15],
            'candidates': [],
            'translations': None,
            'translation_check': None,
            'retreats_evaluated': False,
            'visibility': ObservedFreeSpaceSnapshot(
                80, 80, 0.05, -2.0, -2.0, (True,) * 6400,
                'map', node.get_clock().now().nanoseconds * 1.0e-9,
                time.monotonic()),
        }
        pose = Pose2D(0.0, 0.0, 0.0)

        node._select(session, pose)
        assert len(checked) == 1
        assert session['angles'] == [-15]
        assert not planned

        node._select(session, pose)
        assert len(checked) == 2
        assert session['angles'] == []
        assert not planned

        node._select(session, pose)
        assert len(planned) == 1
        assert [candidate[0] for candidate in planned[0]] == ['ROTATE', 'ROTATE']
    finally:
        node.destroy_node()
        rclpy.shutdown()
