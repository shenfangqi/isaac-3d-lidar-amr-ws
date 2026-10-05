"""
Issue #13 PR1: multi-view search, clustering and holdout validation.

Maps and scans are synthesized from analytic wall segments in world
coordinates, independent of the grid conversion under test.
"""

from array import array
import math
import time
from types import SimpleNamespace as NS

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError,
    Hypothesis,
    Keyframe,
    RejectReason,
    SE2,
    SearchResult,
)
from isaac_3d_lidar_bringup.localization_hypotheses import (
    cell_center_to_world,
    cluster_hypotheses,
    map_hash,
    scan_pose_in_map,
    search_multiview,
    SearchConfig,
    SearchWorker,
    seed_pose_at_current_time,
    validate_hypotheses,
    ValidationThresholds,
    world_to_cell,
)
from isaac_3d_lidar_bringup.localization_observations import (
    collect_keyframes,
    make_keyframe,
    Reject,
    ScanSnapshot,
)


IDENTITY = SE2(0.0, 0.0, 0.0)
FAST = SearchConfig(coarse_step_m=0.20, coarse_yaw_step_rad=math.radians(15),
                    coarse_beams=36, refine_beams=90, refine_evaluations=60,
                    tolerance_cells=3)


def _rectangle(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)),
            ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def _grid(walls, size_x, size_y, resolution=0.05, origin=(0.0, 0.0, 0.0)):
    """Rasterize world segments; cells outside the walls stay free."""
    ox, oy, oyaw = origin
    width, height = round(size_x / resolution), round(size_y / resolution)
    data = array('b', [0] * (width * height))
    for (ax, ay), (bx, by) in walls:
        steps = max(2, int(math.hypot(bx - ax, by - ay) / resolution * 4))
        for i in range(steps + 1):
            wx = ax + (bx - ax) * i / steps
            wy = ay + (by - ay) * i / steps
            dx, dy = wx - ox, wy - oy
            local_x = math.cos(oyaw) * dx + math.sin(oyaw) * dy
            local_y = -math.sin(oyaw) * dx + math.cos(oyaw) * dy
            cx = int(math.floor(local_x / resolution))
            cy = int(math.floor(local_y / resolution))
            if 0 <= cx < width and 0 <= cy < height:
                data[cy * width + cx] = 100
    return NS(frame_id='map', data=data, info=NS(
        width=width, height=height, resolution=resolution,
        origin=NS(position=NS(x=ox, y=oy),
                  orientation=NS(x=0.0, y=0.0, z=math.sin(oyaw / 2),
                                 w=math.cos(oyaw / 2)))))


def _raycast(walls, pose, beams=180, range_max=8.0, stamp_ns=1):
    """Analytic scan in the sensor frame located at ``pose``."""
    ranges = []
    increment = 2.0 * math.pi / beams
    for index in range(beams):
        angle = pose.yaw - math.pi + index * increment
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for (ax, ay), (bx, by) in walls:
            ex, ey = bx - ax, by - ay
            denominator = dx * ey - dy * ex
            if abs(denominator) < 1e-12:
                continue
            t = ((ax - pose.x) * ey - (ay - pose.y) * ex) / denominator
            s = ((ax - pose.x) * dy - (ay - pose.y) * dx) / denominator
            if t > 0.0 and 0.0 <= s <= 1.0:
                best = min(best, t)
        ranges.append(best if best <= range_max else math.inf)
    return ScanSnapshot('base_footprint', stamp_ns, -math.pi, increment,
                        0.1, range_max, tuple(ranges))


def _frames(walls, true_pose, role, count=3, session='s1', start_id=0,
            view_id=0, odom=IDENTITY, range_max=8.0):
    return tuple(
        Keyframe(start_id + i, session, 1_000 + start_id + i, view_id,
                 _raycast(walls, true_pose, range_max=range_max,
                          stamp_ns=1_000 + start_id + i),
                 odom, IDENTITY, 0.0, role)
        for i in range(count))


def _polygon(points):
    return [(points[i], points[(i + 1) % len(points)])
            for i in range(len(points))]


# L-shaped room with a stub: no rotational or mirror symmetry.
ROOM = (_polygon([(0.2, 0.2), (3.2, 0.2), (3.2, 1.2), (2.0, 1.2),
                  (2.0, 2.2), (0.2, 2.2)])
        + [((0.2, 1.5), (0.8, 1.5))])


def test_multiview_transform():
    # L02: one world point seen from two views 90 deg apart, with an
    # offset scan extrinsic, must land on the same map coordinates.
    hypothesis = SE2(1.0, 2.0, 0.3)
    base_scan = SE2(0.10, -0.05, 0.2)
    point = (2.5, 3.0)
    reference = SE2(0.4, -0.2, 0.1)
    endpoints = []
    for odom in (reference, SE2(0.4, -0.2, 0.1 + math.pi / 2)):
        frame = NS(T_odom_base=odom, T_base_scan=base_scan)
        sensor = scan_pose_in_map(hypothesis, reference, frame)
        # Express the world point in this sensor frame, then map it back.
        local = sensor.inverse().compose(SE2(point[0], point[1], 0.0))
        endpoints.append(sensor.compose(SE2(local.x, local.y, 0.0)))
        # Wrong model: every frame shares H directly.
        shared = hypothesis.compose(base_scan)
        endpoints.append(shared.compose(SE2(local.x, local.y, 0.0)))
    correct = endpoints[0], endpoints[2]
    assert math.hypot(correct[0].x - correct[1].x,
                      correct[0].y - correct[1].y) < 1e-6
    assert math.hypot(correct[0].x - point[0], correct[0].y - point[1]) < 1e-6
    wrong = endpoints[1], endpoints[3]
    assert math.hypot(wrong[0].x - wrong[1].x, wrong[0].y - wrong[1].y) > 0.1


def test_seed_uses_current_pose():
    # L03: after a 30 deg turn the seed yaw follows the current odometry.
    winner = SE2(1.0, 1.0, 0.2)
    reference = SE2(0.5, 0.5, -0.4)
    current = SE2(0.5, 0.5, -0.4 + math.radians(30))
    seed = seed_pose_at_current_time(winner, reference, current)
    assert seed.yaw == pytest.approx(0.2 + math.radians(30))
    assert (seed.x, seed.y) == (pytest.approx(1.0), pytest.approx(1.0))


def test_cluster_keeps_opposite_heading_separate():
    clusters = cluster_hypotheses(
        [(0.9, 1.0, 1.0, 0.0), (0.9, 1.0, 1.0, math.pi),
         (0.8, 1.05, 1.0, 0.05), (0.7, 1.25, 1.0, 0.0),
         (0.6, 1.5, 1.0, 0.0)],
        0.30, math.pi / 12)
    seeds = [(c['seed'][1], round(abs(c['seed'][3]), 3)) for c in clusters]
    assert (1.0, 0.0) in seeds and (1.0, round(math.pi, 3)) in seeds
    # Seed-based membership: 1.25 joins, 1.5 cannot chain into the cluster.
    assert len(clusters) == 3


def test_yaw_ambiguity_same_position():
    # L04: a bare rectangle seen from its centre matches at 0 and 180 deg.
    walls = _rectangle(0.2, 0.2, 3.2, 2.2)
    grid = _grid(walls, 3.4, 2.4)
    true_pose = SE2(1.7, 1.2, 0.0)
    train = _frames(walls, true_pose, 'TRAIN')
    holdout = _frames(walls, true_pose, 'HOLDOUT', start_id=10)
    result = search_multiview(grid, train, FAST)
    assert result.complete
    near_center = [h for h in result.hypotheses
                   if math.hypot(h.x - 1.7, h.y - 1.2) < 0.15]
    yaws = {round(abs(h.yaw) / math.pi) for h in near_center}
    assert yaws == {0, 1}
    decision = validate_hypotheses(grid, result, train, holdout, FAST,
                                   ValidationThresholds())
    assert not decision.accepted
    assert decision.reason == RejectReason.AMBIGUOUS_LOCATION.value
    assert decision.winner is None


def test_distinct_room_is_accepted_with_bounded_support():
    grid = _grid(ROOM, 3.4, 2.4)
    true_pose = SE2(1.2, 1.0, 0.4)
    train = _frames(ROOM, true_pose, 'TRAIN')
    holdout = _frames(ROOM, true_pose, 'HOLDOUT', start_id=10)
    result = search_multiview(grid, train, FAST)
    decision = validate_hypotheses(grid, result, train, holdout, FAST,
                                   ValidationThresholds())
    assert decision.accepted, decision
    winner = decision.winner
    assert math.hypot(winner.x - 1.2, winner.y - 1.0) < 0.08
    assert abs(math.atan2(math.sin(winner.yaw - 0.4),
                          math.cos(winner.yaw - 0.4))) < math.radians(3)
    assert len(winner.support_bounds) == 3


def test_corridor_unobservable():
    # L05: parallel walls with flat longitudinal score never become READY.
    walls = [((0.1, 0.5), (5.9, 0.5)), ((0.1, 1.3), (5.9, 1.3))]
    grid = _grid(walls, 6.0, 1.8)
    true_pose = SE2(3.0, 0.9, 0.0)
    train = _frames(walls, true_pose, 'TRAIN', range_max=1.0)
    holdout = _frames(walls, true_pose, 'HOLDOUT', start_id=10,
                      range_max=1.0)
    lone = SearchResult('s1', map_hash(grid), True, (Hypothesis(
        3.0, 0.9, 0.0, 0.9, 0.9, 0.0, 0, (0.9,), ()),), 1, 0.1, '')
    decision = validate_hypotheses(grid, lone, train, holdout, FAST,
                                   ValidationThresholds())
    assert decision.reason == RejectReason.UNOBSERVABLE_AXIS.value
    full = search_multiview(grid, train, FAST)
    assert not validate_hypotheses(grid, full, train, holdout, FAST,
                                   ValidationThresholds()).accepted


def test_runner_up_not_pruned():
    # L06: an equal alternative is refined and causes rejection; with no
    # budget to refine it the search is incomplete, never accepted.
    walls = _rectangle(0.2, 0.2, 3.2, 2.2)
    grid = _grid(walls, 3.4, 2.4)
    true_pose = SE2(1.7, 1.2, 0.0)
    train = _frames(walls, true_pose, 'TRAIN')
    holdout = _frames(walls, true_pose, 'HOLDOUT', start_id=10)
    tight = SearchConfig(**{**FAST.__dict__, 'max_refined_clusters': 1})
    result = search_multiview(grid, train, tight)
    assert not result.complete
    assert result.reason == RejectReason.SEARCH_INCOMPLETE.value
    assert not validate_hypotheses(grid, result, train, holdout, tight,
                                   ValidationThresholds()).accepted


def test_search_deadline_and_cancel_are_incomplete():
    grid = _grid(ROOM, 3.4, 2.4)
    train = _frames(ROOM, SE2(1.2, 1.0, 0.4), 'TRAIN')
    expired = search_multiview(grid, train, FAST, deadline=0.0)
    assert not expired.complete and expired.reason == 'SEARCH_INCOMPLETE'
    canceled = search_multiview(grid, train, FAST,
                                cancel_token=lambda: True)
    assert not canceled.complete and canceled.reason == 'CANCELED'


def test_holdout_independent():
    # L07: TRAIN matches but HOLDOUT observes a different place.
    grid = _grid(ROOM, 3.4, 2.4)
    true_pose = SE2(1.2, 1.0, 0.4)
    train = _frames(ROOM, true_pose, 'TRAIN')
    elsewhere = _rectangle(0.0, 0.0, 1.0, 6.0)
    holdout = _frames(elsewhere, SE2(0.5, 3.0, 0.0), 'HOLDOUT', start_id=10)
    result = search_multiview(grid, train, FAST)
    decision = validate_hypotheses(grid, result, train, holdout, FAST,
                                   ValidationThresholds())
    assert not decision.accepted
    assert decision.reason == RejectReason.NO_VALID_CANDIDATE.value
    relabeled = tuple(
        Keyframe(f.id, f.session, f.stamp_ns, f.view_id, f.scan,
                 f.T_odom_base, f.T_base_scan, f.receipt_mono, 'HOLDOUT')
        for f in train)
    with pytest.raises(ContractError):
        validate_hypotheses(grid, result, train, relabeled, FAST,
                            ValidationThresholds())


def test_map_change_invalidates_result():
    grid = _grid(ROOM, 3.4, 2.4)
    true_pose = SE2(1.2, 1.0, 0.4)
    train = _frames(ROOM, true_pose, 'TRAIN')
    holdout = _frames(ROOM, true_pose, 'HOLDOUT', start_id=10)
    result = search_multiview(grid, train, FAST)
    grid.data[0] = 100
    decision = validate_hypotheses(grid, result, train, holdout, FAST,
                                   ValidationThresholds())
    assert decision.reason == RejectReason.MAP_CHANGED.value


def test_rotated_map_origin():
    # L08: origin yaw pi/2; conversions and the search honour it.
    origin = (3.0, -0.5, math.pi / 2)
    grid = _grid([], 2.0, 3.0, origin=origin)
    x, y = cell_center_to_world(grid, 0, 0)
    assert (x, y) == (pytest.approx(3.0 - 0.025), pytest.approx(-0.475))
    assert world_to_cell(grid, x, y) == (0, 0)
    assert world_to_cell(grid, 2.49, 0.21) == (14, 10)

    # The L-shaped room expressed in a grid whose x axis points along +y.
    rotated = _grid(ROOM, 2.4, 3.4, origin=(3.4, 0.0, math.pi / 2))
    true_pose = SE2(1.2, 1.0, -0.3)
    train = _frames(ROOM, true_pose, 'TRAIN')
    holdout = _frames(ROOM, true_pose, 'HOLDOUT', start_id=10)
    result = search_multiview(rotated, train, FAST)
    decision = validate_hypotheses(rotated, result, train, holdout, FAST,
                                   ValidationThresholds())
    assert decision.accepted, decision
    assert math.hypot(decision.winner.x - 1.2, decision.winner.y - 1.0) < 0.08


def test_keyframe_requires_source_time_transforms():
    scan = _raycast(ROOM, SE2(1.0, 1.0, 0.0), stamp_ns=5)

    def missing(target, source, stamp_ns):
        raise LookupError(f'{target}<-{source} at {stamp_ns}')

    rejected = make_keyframe(scan, missing, 's1', 0, 'TRAIN', 0, 1.0)
    assert isinstance(rejected, Reject)
    assert rejected.reason == RejectReason.TF_AT_SOURCE_MISSING
    requested = []

    def exact(target, source, stamp_ns):
        requested.append(stamp_ns)
        return IDENTITY

    frame = make_keyframe(scan, exact, 's1', 0, 'TRAIN', 0, 1.0)
    assert frame.stamp_ns == 5 and requested == [5, 5]


def test_collect_rejects_moved_view_and_other_sessions():
    scan = _raycast(ROOM, SE2(1.0, 1.0, 0.0))
    frames = [
        Keyframe(0, 's1', 1, 0, scan, IDENTITY, IDENTITY, 9.0, 'TRAIN'),
        Keyframe(1, 's1', 2, 0, scan, SE2(0.2, 0.0, 0.0), IDENTITY, 9.0,
                 'TRAIN'),
        Keyframe(2, 's0', 3, 0, scan, IDENTITY, IDENTITY, 9.0, 'TRAIN'),
    ]
    moved = collect_keyframes(frames, 'TRAIN', 8, 5.0, 10.0, 's1')
    assert isinstance(moved, Reject) and moved.reason.value == 'ODOM_JUMP'
    stale = collect_keyframes(frames[:1], 'TRAIN', 8, 0.5, 10.0, 's1')
    assert isinstance(stale, Reject)
    assert collect_keyframes(frames[:1], 'HOLDOUT', 8, 5.0, 10.0,
                             's1').reason.value == 'SENSOR_STALE'


def test_collect_thins_views_by_yaw_coverage():
    scan = _raycast(ROOM, SE2(1.0, 1.0, 0.0))
    frames = [Keyframe(i, 's1', i + 1, i, scan, SE2(0.0, 0.0, -3.0 + i * 0.5),
                       IDENTITY, 9.0, 'TRAIN') for i in range(12)]
    selected = collect_keyframes(frames, 'TRAIN', 4, 5.0, 10.0, 's1')
    assert len({frame.view_id for frame in selected}) == 4


def test_worker_cancel_reaps_process():
    worker = SearchWorker()
    worker.submit(('s1', 'h'), time.sleep, 60)
    assert worker.busy and worker.poll() is None
    with pytest.raises(RuntimeError):
        worker.submit(('s1', 'h'), time.sleep, 60)
    worker.cancel()
    assert not worker.busy and worker.poll() is None
    worker.cancel()


def test_worker_returns_token_and_result():
    worker = SearchWorker()
    worker.submit(('s1', 'abc'), map_hash, _grid(ROOM, 3.4, 2.4))
    deadline = time.monotonic() + 30.0
    outcome = None
    while outcome is None and time.monotonic() < deadline:
        outcome = worker.poll()
        time.sleep(0.05)
    assert outcome[0] == ('s1', 'abc') and outcome[1] == 'ok'
    assert len(outcome[2]) == 64
