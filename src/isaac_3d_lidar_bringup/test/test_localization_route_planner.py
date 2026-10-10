"""Phase 1: candidate-wide safe route planning on analytic static maps."""

import math

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_route_planner import (
    check_route, ClearanceField, DISTINGUISHABLE_NOW, FORWARD,
    INDISTINGUISHABLE_IN_MODEL, MotionModel, NO_COMMON_SAFE_ACTION,
    NOT_FOUND_WITHIN_BUDGET, ObservationModel, plan_route, PlannerConfig,
    PoseBounds, ROTATE, ROUTE_FOUND, StaticMap,
)

pytest.importorskip('scipy.ndimage')

RES = 0.05
TIGHT = PoseBounds(0.0, 0.0)
MODEL = MotionModel()


def _map(width_m, height_m, walls=(), unknown=()):
    """Free rectangle with an occupied border; extra boxes in metres."""
    w, h = round(width_m / RES), round(height_m / RES)
    data = np.zeros((h, w), np.int16)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    for value, boxes in ((100, walls), (-1, unknown)):
        for x0, y0, x1, y1 in boxes:
            data[round(y0 / RES):round(y1 / RES), round(x0 / RES):round(x1 / RES)] = value
    return StaticMap(data, RES, 0.0, 0.0)


def _fields(static_map):
    return (ClearanceField(static_map),
            ClearanceField(static_map, blocked=static_map.data == 100,
                           outside_blocked=False))


def _observation(static_map, extra=None, max_range_m=6.0):
    occupied = static_map.data == 100
    layers = {'map': (occupied, occupied)}
    if extra is not None:
        layers['upper'] = (extra, extra)
    return ObservationModel(static_map, layers, beams=90, max_range_m=max_range_m)


def test_clearance_is_a_lower_bound_of_the_true_distance():
    static_map = _map(3.0, 2.0, walls=[(1.4, 0.6, 1.6, 1.4)], unknown=[(0.4, 0.4, 0.6, 0.6)])
    field = ClearanceField(static_map)
    rng = np.random.default_rng(1)
    xs, ys = rng.uniform(0, 3, 400), rng.uniform(0, 2, 400)
    blocked_j, blocked_i = np.nonzero(static_map.data != 0)
    for x, y, c in zip(xs, ys, field.clearance(xs, ys)):
        # Exact distance from the point to the union of blocked cell squares.
        dx = np.maximum(0, np.maximum(blocked_i * RES - x, x - (blocked_i + 1) * RES))
        dy = np.maximum(0, np.maximum(blocked_j * RES - y, y - (blocked_j + 1) * RES))
        assert c <= np.min(np.hypot(dx, dy)) + 1e-9
    assert field.clearance(np.array([-1.0]), np.array([1.0]))[0] < 0   # outside


def test_same_body_action_is_safe_for_one_heading_and_not_the_opposite():
    static_map = _map(4.0, 2.0)
    fields = _fields(static_map)
    route = ((FORWARD, 0.6),)
    facing_open = check_route(fields, MODEL, (1.2, 1.0, 0.0), TIGHT, route)
    # Front 0.9 - 0.155 - 0.6 - 0.12 = 0.025 m from the wall's inner face.
    facing_wall = check_route(fields, MODEL, (0.9, 1.0, math.pi), TIGHT, route)
    assert facing_open.safe
    assert not facing_wall.safe and facing_wall.reason == 'OBSTACLE_IN_SWEEP'


def test_candidate_uncertainty_and_stop_extension_are_part_of_the_sweep():
    static_map = _map(4.0, 2.0, walls=[(2.30, 0.0, 2.40, 2.0)])
    fields = _fields(static_map)
    route = ((FORWARD, 0.4),)
    # Front at 1.4 + 0.155 + 0.4 = 1.955; 0.35 m to the wall is enough...
    assert check_route(fields, MODEL, (1.4, 1.0, 0.0), TIGHT, route).safe
    # ...but not with the stopping extension of a faster, uncalibrated stop.
    long_stop = MotionModel(forward_stop_extension_m=0.30)
    assert not check_route(fields, long_stop, (1.4, 1.0, 0.0), TIGHT, route).safe
    # A wide pose bound reaches the side walls of a narrow corridor.
    corridor = _map(4.0, 0.75)
    fields = _fields(corridor)
    assert check_route(fields, MODEL, (1.0, 0.375, 0.0), TIGHT, route).safe
    assert not check_route(fields, MODEL, (1.0, 0.375, 0.0),
                           PoseBounds(0.10, math.radians(8)), route).safe


def test_unknown_space_and_map_edge_are_never_traversable():
    static_map = _map(4.0, 2.0, unknown=[(2.0, 0.0, 2.2, 2.0)])
    fields = _fields(static_map)
    check = check_route(fields, MODEL, (1.4, 1.0, 0.0), TIGHT, ((FORWARD, 0.6),))
    assert not check.safe and check.reason == 'UNKNOWN_OR_OUTSIDE_IN_SWEEP'
    open_map = StaticMap(np.zeros((40, 40), np.int16), RES, 0.0, 0.0)
    edge = check_route(_fields(open_map), MODEL, (1.6, 1.0, 0.0), TIGHT,
                       ((FORWARD, 0.4),))
    assert not edge.safe and edge.reason == 'UNKNOWN_OR_OUTSIDE_IN_SWEEP'


def test_mid_path_obstacle_and_rotation_overshoot_are_caught():
    static_map = _map(4.0, 2.0, walls=[(1.85, 0.95, 1.95, 1.05)])
    fields = _fields(static_map)
    # Target 1.0 m ahead is clear but a post sits half-way.
    assert not check_route(fields, MODEL, (1.0, 1.0, 0.0), TIGHT,
                           ((FORWARD, 0.6),)).safe
    # A turn of 45 deg clears a post at 90 deg, the overshoot does not.
    box = _map(2.0, 2.0, walls=[(1.0, 1.30, 1.05, 1.35)])
    fields = _fields(box)
    assert not check_route(fields, MODEL, (1.0, 1.0, 0.0), TIGHT,
                           ((ROTATE, math.pi / 2),)).safe


def test_rotation_from_the_centre_of_a_room_is_safe_and_returns_stops():
    static_map = _map(3.0, 3.0)
    check = check_route(_fields(static_map), MODEL, (1.5, 1.5, 0.0), TIGHT,
                        ((ROTATE, math.pi / 2), (FORWARD, 0.4)))
    assert check.safe
    x, y, yaw = check.stops[-1]
    assert (x, y) == (pytest.approx(1.5), pytest.approx(1.9))
    assert yaw == pytest.approx(math.pi / 2)


def _twin_room():
    """Room symmetric under a half turn; an 'upper' marker breaks it."""
    static_map = _map(6.0, 3.0)
    upper = static_map.data == 100
    upper = upper.copy()
    upper[round(0.3 / RES):round(2.7 / RES), round(5.3 / RES):round(5.4 / RES)] = True
    return static_map, upper


def test_fully_symmetric_candidates_are_reported_indistinguishable():
    static_map, _ = _twin_room()
    candidates = [((1.5, 1.5, 0.0), TIGHT), ((4.5, 1.5, math.pi), TIGHT)]
    result = plan_route(_fields(static_map), MODEL, _observation(static_map),
                        candidates, PlannerConfig(max_depth=2, time_budget_s=60))
    assert result.status == INDISTINGUISHABLE_IN_MODEL
    assert result.objective < 0.05


def test_a_layer_that_sees_the_marker_distinguishes_now():
    static_map, upper = _twin_room()
    candidates = [((1.5, 1.5, 0.0), TIGHT), ((4.5, 1.5, math.pi), TIGHT)]
    result = plan_route(_fields(static_map), MODEL, _observation(static_map, upper),
                        candidates, PlannerConfig(layers=('upper',), min_difference=0.05))
    assert result.status == DISTINGUISHABLE_NOW and result.route == ()


def test_multi_step_route_is_needed_when_the_markers_are_out_of_range():
    # A 6 x 2 m corridor is symmetric under a half turn about (3, 1).  Two
    # posts in the upper layer break that, but neither is within 1.2 m of
    # its candidate now; turning round and driving brings each into view,
    # and they sit on opposite sides, so either truth is told apart.
    static_map = _map(6.0, 2.0)
    upper = (static_map.data == 100).copy()
    for x0, y0 in ((1.0, 0.45), (4.95, 0.45)):
        upper[round(y0 / RES):round((y0 + 0.1) / RES),
              round(x0 / RES):round((x0 + 0.1) / RES)] = True
    observation = _observation(static_map, upper, max_range_m=1.2)
    candidates = [((2.2, 1.0, 0.0), TIGHT), ((3.8, 1.0, math.pi), TIGHT)]
    config = PlannerConfig(max_depth=3, min_difference=0.02, layers=('upper',),
                           time_budget_s=120)
    result = plan_route(_fields(static_map), MODEL, observation, candidates, config)
    assert result.initial_objective < 0.02
    assert result.status == ROUTE_FOUND
    assert any(kind == FORWARD for kind, _ in result.route)


def test_absence_of_a_feature_alone_does_not_count_as_separation():
    # Only one candidate would see the post: if the other is true, its scan
    # fits both poses equally, as in the production hit-share matcher.
    static_map = _map(6.0, 2.0)
    upper = (static_map.data == 100).copy()
    upper[round(0.45 / RES):round(0.55 / RES), round(4.95 / RES):round(5.05 / RES)] = True
    observation = _observation(static_map, upper, max_range_m=1.2)
    a, b = (2.2, 1.0, 0.0), (4.4, 1.0, 0.0)
    assert observation.separability('upper', a, b) == 0.0


def test_no_common_safe_action_when_the_twin_is_boxed_in():
    # The second candidate sits in a 0.7 x 0.6 m pocket: no turn, no drive.
    static_map = _map(6.0, 3.0, walls=[(4.15, 1.15, 4.2, 1.85), (4.9, 1.15, 4.95, 1.85),
                                       (4.15, 1.15, 4.95, 1.2), (4.15, 1.8, 4.95, 1.85)])
    candidates = [((1.5, 1.5, 0.0), TIGHT), ((4.55, 1.5, 0.0), TIGHT)]
    result = plan_route(_fields(static_map), MODEL, _observation(static_map),
                        candidates, PlannerConfig(max_depth=1, min_difference=1.1))
    assert result.status == NO_COMMON_SAFE_ACTION
    assert result.unsafe_reasons


def test_budget_exhaustion_is_not_reported_as_indistinguishable():
    static_map, _ = _twin_room()
    candidates = [((1.5, 1.5, 0.0), TIGHT), ((4.5, 1.5, math.pi), TIGHT)]
    result = plan_route(_fields(static_map), MODEL, _observation(static_map),
                        candidates, PlannerConfig(time_budget_s=0.0))
    assert result.status == NOT_FOUND_WITHIN_BUDGET


def test_no_candidate_is_dropped_and_single_candidates_are_refused():
    static_map, upper = _twin_room()
    observation = _observation(static_map, upper)
    with pytest.raises(ContractError):
        plan_route(_fields(static_map), MODEL, observation, [((1.5, 1.5, 0.0), TIGHT)])
    three = [((1.5, 1.5, 0.0), TIGHT), ((4.5, 1.5, math.pi), TIGHT),
             ((1.5, 1.5, math.pi), TIGHT)]
    result = plan_route(_fields(static_map), MODEL, observation, three)
    assert set(result.pair_differences) == {(0, 1), (0, 2), (1, 2)}
