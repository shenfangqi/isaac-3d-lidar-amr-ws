"""Tests for conservative, non-actuating swept footprint evaluation."""

import math

from carbot_nav_recovery.swept_footprint import (
    CostmapSnapshot,
    ObservedFreeSpaceSnapshot,
    Pose2D,
    SnapshotFreshness,
    check_snapshot_freshness,
    check_snapshot_data_freshness,
    check_swept_path,
    check_observed_free_path,
    evaluate_rotation_candidates,
    rank_rotation_candidates,
)


def make_map(resolution=0.05, width=80, height=80, costs=None):
    if costs is None:
        costs = [0] * (width * height)
    return CostmapSnapshot(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=-2.0,
        origin_y=-2.0,
        costs=tuple(costs),
        frame_id='map',
        stamp_sec=10.0,
        received_monotonic_sec=20.0,
    )


def cell_index(snapshot, x, y):
    mx = int(math.floor((x - snapshot.origin_x) / snapshot.resolution))
    my = int(math.floor((y - snapshot.origin_y) / snapshot.resolution))
    return my * snapshot.width + mx


def test_free_rotation_sweep_is_safe():
    snapshot = make_map()
    result = check_swept_path(
        snapshot,
        ((0.30, 0.15), (0.30, -0.15), (-0.30, -0.15), (-0.30, 0.15)),
        (Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, math.pi / 2)),
        safety_margin=0.02,
    )
    assert result.safe
    assert result.samples > 1


def test_detects_collision_during_sweep_when_endpoints_are_clear():
    snapshot = make_map(resolution=0.02, width=200, height=200)
    costs = [0] * (snapshot.width * snapshot.height)
    obstacle = cell_index(snapshot, 0.35, 0.21)
    costs[obstacle] = 254
    snapshot = make_map(resolution=0.02, width=200, height=200,
                        costs=costs)
    footprint = ((0.40, 0.10), (0.40, -0.10),
                 (-0.40, -0.10), (-0.40, 0.10))

    start = check_swept_path(snapshot, footprint, (Pose2D(0, 0, 0),))
    end = check_swept_path(
        snapshot, footprint, (Pose2D(0, 0, math.pi / 2),))
    sweep = check_swept_path(
        snapshot, footprint,
        (Pose2D(0, 0, 0), Pose2D(0, 0, math.pi / 2)))

    assert start.safe and end.safe
    assert not sweep.safe
    assert sweep.reason == 'LETHAL_OBSTACLE'


def test_rejects_unknown_and_lethal_but_allows_soft_inflation_costs():
    footprint = ((0.15, 0.10), (0.15, -0.10),
                 (-0.15, -0.10), (-0.15, 0.10))
    center_cell = cell_index(make_map(), 0.0, 0.0)
    soft = [0] * (80 * 80)
    soft[center_cell] = 80
    soft_result = check_swept_path(
        make_map(costs=soft), footprint, (Pose2D(0, 0, 0),))
    assert soft_result.safe
    assert soft_result.max_cost == 80
    assert soft_result.inflation_cost_sum > 0

    for cost, reason in ((253, 'INSCRIBED_CENTER'),
                         (254, 'LETHAL_OBSTACLE'),
                         (255, 'UNKNOWN_SPACE')):
        blocked = [0] * (80 * 80)
        blocked[center_cell] = cost
        result = check_swept_path(
            make_map(costs=blocked), footprint, (Pose2D(0, 0, 0),))
        assert not result.safe
        assert result.reason == reason
        assert result.blocked_cell is not None


def test_rejects_map_boundary_and_bad_geometry():
    snapshot = make_map()
    footprint = ((0.3, 0.1), (0.3, -0.1), (-0.3, -0.1), (-0.3, 0.1))
    result = check_swept_path(
        snapshot, footprint, (Pose2D(1.85, 0.0, 0.0),))
    assert not result.safe
    assert result.reason == 'MAP_BOUNDARY'
    assert check_swept_path(snapshot, ((0.0, 0.0),),
                            (Pose2D(0.0, 0.0, 0.0),)).reason == (
                                'INVALID_GEOMETRY')


def test_data_freshness_checks_both_source_and_receive_ages():
    snapshot = make_map()
    fresh = SnapshotFreshness(10.1, 20.1, 0.25, 0.25, True, True)
    assert check_snapshot_freshness(snapshot, fresh).safe
    source_old = SnapshotFreshness(11.0, 20.1, 0.25, 0.25, True, True)
    assert check_snapshot_freshness(snapshot, source_old).reason == (
        'STALE_SENSOR')
    receive_old = SnapshotFreshness(10.1, 21.0, 0.25, 0.25, True, True)
    assert check_snapshot_freshness(snapshot, receive_old).reason == (
        'STALE_SENSOR')
    invalid_tf = SnapshotFreshness(10.1, 20.1, 0.25, 0.25, True, False)
    assert check_snapshot_freshness(snapshot, invalid_tf).reason == (
        'TF_INVALID')
    invalid_localization = SnapshotFreshness(
        10.1, 20.1, 0.25, 0.25, False, True)
    assert check_snapshot_freshness(snapshot, invalid_localization).reason == (
        'LOCALIZATION_INVALID')
    assert check_snapshot_data_freshness(
        snapshot, invalid_localization).safe


def test_candidates_include_reasons_and_require_safe_departure_path():
    snapshot = make_map()
    footprint = ((0.30, 0.13), (0.30, -0.13),
                 (-0.30, -0.13), (-0.30, 0.13))
    candidates = evaluate_rotation_candidates(
        snapshot, footprint, Pose2D(0, 0, 0), (math.pi / 12, -math.pi / 12),
        safety_margin=0.01,
        departure_path_factory=lambda pose: (
            pose, Pose2D(pose.x + 0.10 * math.cos(pose.yaw),
                         pose.y + 0.10 * math.sin(pose.yaw), pose.yaw)),
    )
    assert len(candidates) == 2
    assert all(candidate.sweep.reason == 'OK' for candidate in candidates)
    assert all(candidate.departure.reason == 'OK' for candidate in candidates)
    assert all(candidate.safe for candidate in rank_rotation_candidates(
        candidates))


def test_candidate_is_rejected_without_departure_path():
    candidates = evaluate_rotation_candidates(
        make_map(), ((0.2, 0.1), (0.2, -0.1), (-0.2, -0.1), (-0.2, 0.1)),
        Pose2D(0, 0, 0), (0.2,), safety_margin=0.0)
    assert candidates[0].sweep.safe
    assert candidates[0].departure.reason == 'NO_DEPARTURE_PATH'
    assert not candidates[0].safe


def test_short_translation_requires_recent_near_field_visibility():
    footprint = ((0.20, 0.10), (0.20, -0.10),
                 (-0.20, -0.10), (-0.20, 0.10))
    observed = [False] * (80 * 80)
    # A short-range scan may leave most of this swept footprint unobserved.
    observed[cell_index(make_map(), 0.0, 0.0)] = True
    evidence = ObservedFreeSpaceSnapshot(
        80, 80, 0.05, -2.0, -2.0, tuple(observed), 'base_footprint',
        10.0, 20.0)
    freshness = SnapshotFreshness(10.1, 20.1, 0.25, 0.25, True, True)
    path = (Pose2D(0.0, 0.0, 0.0), Pose2D(-0.05, 0.0, 0.0))

    result = check_observed_free_path(
        evidence, footprint, path, freshness, safety_margin=0.0)
    assert not result.safe
    assert result.reason == 'UNKNOWN_SPACE'

    visible = ObservedFreeSpaceSnapshot(
        80, 80, 0.05, -2.0, -2.0, (True,) * (80 * 80), 'base_footprint',
        10.0, 20.0)
    allowed = check_observed_free_path(
        visible, footprint, path, freshness, safety_margin=0.0)
    assert allowed.safe

    stale = SnapshotFreshness(10.1, 21.0, 0.25, 0.25, True, True)
    assert check_observed_free_path(
        visible, footprint, path, stale).reason == 'STALE_SENSOR'
