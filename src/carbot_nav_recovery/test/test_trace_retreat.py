"""Reverse-trace safety scenarios, independent of hardware and ROS."""

from dataclasses import replace
import math
import time

import pytest

from carbot_nav_recovery.swept_footprint import (
    CostmapSnapshot, ObservedFreeSpaceSnapshot, Pose2D, SnapshotFreshness,
)
from carbot_nav_recovery.trace_retreat import (
    PoseHistory, RetreatLimits, TraceContext, evaluate_trace_retreat,
)


CONTEXT = TraceContext('original-user-goal', 'localization-epoch-1', 'map')
BODY = ((0.015, 0.01), (0.015, -0.01), (-0.015, -0.01), (-0.015, 0.01))


def fixture(curved=False):
    history = PoseHistory()
    for i in range(11):
        yaw = i * 0.05 if curved else 0.0
        pose = (Pose2D(0.2 * math.sin(yaw), 0.2 * (1 - math.cos(yaw)), yaw)
                if curved else Pose2D(i * 0.01, 0.0, 0.0))
        assert history.append(pose, 10 + i * 0.1, CONTEXT,
                              localization_valid=True)
    grid = CostmapSnapshot(200, 200, 0.005, -0.5, -0.5,
                           (0,) * 40000, 'map', 11.0, 20.0)
    observed = ObservedFreeSpaceSnapshot(
        200, 200, 0.005, -0.5, -0.5, (True,) * 40000,
        'map', 11.0, 20.0)
    return dict(
        history=history, context=CONTEXT,
        current_pose=history.samples[-1].pose,
        local_map=grid, static_map=replace(grid, stamp_sec=1.0),
        observations=observed, footprint=BODY,
        freshness=SnapshotFreshness(11.05, 20.05, 0.25, 0.25, True, True),
        limits=RetreatLimits(stopping_distance_m=0.02),
        distance_remaining_m=0.20, time_remaining_sec=30.0,
    )


def obstacle(grid, x, y, value=254):
    costs = list(grid.costs)
    mx = math.floor((x - grid.origin_x) / grid.resolution)
    my = math.floor((y - grid.origin_y) / grid.resolution)
    costs[my * grid.width + mx] = value
    return replace(grid, costs=tuple(costs))


@pytest.mark.parametrize('curved', [False, True])
def test_retrace_keeps_original_goal_and_all_intermediate_poses(curved):
    inputs = fixture(curved)
    candidates, reason = evaluate_trace_retreat(**inputs)
    assert reason == 'CANDIDATE_AVAILABLE'
    selected = next(candidate for candidate in candidates
                    if candidate.geometry_clear)
    assert selected.context.goal_id == 'original-user-goal'
    assert selected.strategy == 'RETRACE_MEASURED_PATH'
    assert selected.path[0] == inputs['current_pose']
    expected = tuple(sample.pose for sample in
                     reversed(inputs['history'].samples))
    assert selected.path[1:] == expected[:len(selected.path) - 1]
    assert 0.03 <= selected.distance_m <= 0.20
    assert selected.minimum_duration_sec <= 30


@pytest.mark.parametrize('source', ['local_map', 'static_map'])
def test_new_obstacle_blocks_previously_clear_trace(source):
    inputs = fixture()
    inputs[source] = obstacle(inputs[source], 0.075, 0.0)
    candidates, reason = evaluate_trace_retreat(**inputs)
    assert reason == 'NO_SAFE_TRACE_RETREAT'
    assert all(not item.geometry_clear for item in candidates)
    assert candidates[0].check.reason == 'LETHAL_OBSTACLE'


def test_static_free_space_cannot_replace_rear_observation():
    inputs = fixture()
    inputs['observations'] = None
    assert evaluate_trace_retreat(**inputs)[1] == 'NO_OBSERVED_REAR_CLEARANCE'
    inputs = fixture()
    inputs['observations'] = replace(inputs['observations'],
                                     observed_free=(False,) * 40000)
    candidates, _ = evaluate_trace_retreat(**inputs)
    assert all(item.check.reason == 'UNKNOWN_SPACE' for item in candidates)
    assert all(item.obstacle_source == 'recent_sensor_visibility'
               for item in candidates)


def test_stale_visibility_and_frame_mismatch_are_rejected():
    inputs = fixture()
    inputs['observations'] = replace(inputs['observations'], stamp_sec=9.0)
    candidates, _ = evaluate_trace_retreat(**inputs)
    assert all(item.check.reason == 'STALE_SENSOR' for item in candidates)
    inputs['observations'] = replace(inputs['observations'], frame_id='odom')
    assert evaluate_trace_retreat(**inputs)[1] == 'FRAME_MISMATCH'


def test_refuge_requires_current_rotation_space():
    inputs = fixture()
    inputs['footprint'] = ((0.06, 0.01), (0.06, -0.01),
                           (-0.06, -0.01), (-0.06, 0.01))
    grid = inputs['local_map']
    for x in range(-10, 30):
        grid = obstacle(grid, x * 0.005, 0.042)
    inputs['local_map'] = grid
    candidates, _ = evaluate_trace_retreat(**inputs)
    assert all(item.check.reason == 'REFUGE_NOT_ROTATABLE'
               for item in candidates)


def test_braking_extension_must_also_be_observed():
    inputs = fixture()
    # Only permit a single endpoint; its nominal trace is clear, but the
    # stopping extension reaches an obstacle outside that nominal path.
    inputs['distance_remaining_m'] = 0.061
    inputs['local_map'] = obstacle(inputs['local_map'], 0.037, 0.0)
    candidates, _ = evaluate_trace_retreat(**inputs)
    assert candidates
    assert any(item.check.reason == 'LETHAL_OBSTACLE' for item in candidates)


def test_goal_change_lift_and_pose_jump_discard_old_trace():
    history = fixture()['history']
    assert history.append(Pose2D(0.1, 0, 0), 11.1,
                          TraceContext('new-goal', 'epoch-1', 'map'),
                          localization_valid=True)
    assert len(history.samples) == 1
    history.invalidate('LIFTED')
    assert not history.samples and history.context is None
    history = fixture()['history']
    assert not history.append(Pose2D(1, 0, 0), 11.1, CONTEXT,
                              localization_valid=True)
    assert history.last_reset_reason == 'POSE_JUMP'
    assert len(history.samples) == 1


@pytest.mark.parametrize('stamp', [10.9, 12.0])
def test_clock_reversal_or_recording_gap_never_gets_bridged(stamp):
    history = fixture()['history']
    assert not history.append(Pose2D(0.1, 0, 0), stamp, CONTEXT,
                              localization_valid=True)
    assert history.last_reset_reason == 'TRACE_TIME_GAP'
    assert len(history.samples) == 1


def test_stale_or_disconnected_start_is_rejected():
    inputs = fixture()
    inputs['current_pose'] = Pose2D(0.15, 0, 0)
    assert evaluate_trace_retreat(**inputs)[1] == 'TRACE_START_MISMATCH'
    inputs = fixture()
    inputs['freshness'] = replace(inputs['freshness'], now_ros_sec=12.0)
    inputs['local_map'] = replace(inputs['local_map'], stamp_sec=12.0)
    assert evaluate_trace_retreat(**inputs)[1] == 'STALE_TRACE'


def test_budget_and_deadline_reject_further_retreat():
    inputs = fixture()
    inputs['distance_remaining_m'] = 0.0
    assert evaluate_trace_retreat(**inputs)[1] == 'BUDGET_EXHAUSTED'
    inputs = fixture()
    inputs['time_remaining_sec'] = 0.1
    assert not evaluate_trace_retreat(**inputs)[0]
    inputs = fixture()
    inputs['deadline_monotonic'] = time.monotonic() - 1.0
    candidates, _ = evaluate_trace_retreat(**inputs)
    assert candidates[0].check.reason == 'COMPUTE_BUDGET_EXCEEDED'


def test_wrapped_heading_is_unwrapped_for_reverse_sweep():
    history = PoseHistory()
    for i, yaw in enumerate([3.12, -3.12, -3.08]):
        assert history.append(Pose2D(0, 0, yaw), 10 + i * 0.1, CONTEXT,
                              localization_valid=True)
    assert history.samples[-1].pose.yaw > math.pi
    assert history.samples[-1].pose.yaw - history.samples[0].pose.yaw < 0.1
