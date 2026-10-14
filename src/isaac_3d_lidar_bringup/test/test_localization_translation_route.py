"""Issue #53: the translation guard's own static-map route check."""

import math
from types import SimpleNamespace as NS

import pytest

from isaac_3d_lidar_bringup.localization_contracts import SE2
from isaac_3d_lidar_bringup.localization_hypotheses import map_hash
from isaac_3d_lidar_bringup.localization_translation_contracts import RoutePlan
from isaac_3d_lidar_bringup.localization_translation_route import (
    guard_model, RouteChecker)

pytest.importorskip('scipy.ndimage')
FOOTPRINT = ((0.155, 0.133), (0.155, -0.133), (-0.130, -0.133), (-0.130, 0.133))


def _grid(walls=()):
    """6 x 6 m free map at 0.05 m with a closed border and ``walls`` boxes."""
    size, resolution = 120, 0.05
    data = []
    for row in range(size):
        for col in range(size):
            x, y = (col + 0.5) * resolution, (row + 0.5) * resolution
            edge = row in (0, size - 1) or col in (0, size - 1)
            inside = any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in walls)
            data.append(100 if edge or inside else 0)
    return NS(info=NS(width=size, height=size, resolution=resolution,
                      origin=NS(position=NS(x=0.0, y=0.0),
                                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))),
              data=data)


def _plan(grid, candidates, start=(10.0, 20.0, 1.0)):
    return RoutePlan('session0000000001', 2, 10**9, *start, tuple(candidates),
                     0.05, math.radians(3.0), math.radians(4.0), 0.02, map_hash(grid))


def _odom(pose, stamp=2 * 10**9):
    return NS(stamp_ns=stamp, x=pose.x, y=pose.y, yaw=pose.yaw)


def _check(grid, plan, guard_start, odom, remaining=0.4):
    model = guard_model(FOOTPRINT, 0.05, 0.012, 0.18, plan)
    return RouteChecker(grid, map_hash(grid)).preview(
        plan, model, guard_start, odom, remaining, 0.18)


def test_a_clear_path_names_the_guards_own_odometry_sample():
    grid = _grid()
    plan = _plan(grid, [(2.0, 3.0, 0.0), (4.0, 3.0, math.pi)])
    start = plan.start
    here = start.compose(SE2(0.1, 0.0, 0.0))
    preview = _check(grid, plan, start, _odom(here, 7 * 10**9))
    assert preview.geometry_clear and preview.reason == ''
    assert preview.snapshot_stamp_ns == 7 * 10**9
    assert preview.start == here and preview.distance_m == pytest.approx(0.42)
    assert preview.stop_extension_m == pytest.approx(0.18)


def test_any_candidate_blocked_refuses():
    # A wall 0.4 m ahead of the second candidate only (it faces -x at 4.0).
    grid = _grid(walls=[(3.45, 2.5, 3.55, 3.5)])
    plan = _plan(grid, [(2.0, 3.0, 0.0), (4.0, 3.0, math.pi)])
    preview = _check(grid, plan, plan.start, _odom(plan.start))
    assert not preview.geometry_clear and preview.reason == 'OBSTACLE_IN_SWEEP'
    alone = _plan(grid, [(2.0, 3.0, 0.0)])
    assert _check(grid, alone, alone.start, _odom(alone.start)).geometry_clear


def test_candidates_follow_the_guards_odometry_since_the_start():
    # 0.2 m driven: the first candidate is now 0.35 m from a wall at x 2.6,
    # less than the remaining 0.2 m plus the stop extension and padding.
    grid = _grid(walls=[(2.6, 2.5, 2.7, 3.5)])
    plan = _plan(grid, [(2.0, 3.0, 0.0)])
    assert _check(grid, plan, plan.start, _odom(plan.start), remaining=0.0).geometry_clear
    driven = plan.start.compose(SE2(0.2, 0.0, 0.0))
    assert not _check(grid, plan, plan.start, _odom(driven), remaining=0.2).geometry_clear
    # Nothing left to drive: nothing to check.
    assert _check(grid, plan, plan.start, _odom(driven), remaining=0.0).geometry_clear


def test_another_start_or_map_refuses():
    grid = _grid()
    plan = _plan(grid, [(2.0, 3.0, 0.0)])
    moved = plan.start.compose(SE2(0.03, 0.0, 0.0))
    assert _check(grid, plan, moved, _odom(moved)).reason == 'ROUTE_PLAN_START_MISMATCH'
    turned = SE2(plan.start_x, plan.start_y, plan.start_yaw + 0.03)
    assert _check(grid, plan, turned, _odom(turned)).reason == 'ROUTE_PLAN_START_MISMATCH'
    jitter = plan.start.compose(SE2(0.001, 0.0, 0.0))
    assert _check(grid, plan, jitter, _odom(jitter)).geometry_clear
    other = _grid(walls=[(0.5, 0.5, 0.6, 0.6)])
    assert _check(other, plan, plan.start, _odom(plan.start)).reason == (
        'ROUTE_PLAN_MAP_MISMATCH')
