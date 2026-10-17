"""Issue #1: keep-off grid for detours around novel obstacles."""

import math

import numpy as np

from isaac_3d_lidar_bringup.novel_obstacle_keepoff import dilate
from isaac_3d_lidar_bringup.novel_obstacle_keepoff import GhostBeams
from isaac_3d_lidar_bringup.novel_obstacle_keepoff import disk_offsets
from isaac_3d_lidar_bringup.novel_obstacle_keepoff import KeepoffGrid
from isaac_3d_lidar_bringup.novel_obstacle_keepoff import TurnGate


INF = float('inf')


def _grid(**overrides):
    # 6 m x 6 m at 0.05 m; the map origin is at (-3, -3).
    parameters = dict(assumed_depth_m=0.4, keepoff_radius_m=0.5,
                      robot_clear_radius_m=0.3, confirm_hits=3)
    parameters.update(overrides)
    return KeepoffGrid(120, 120, 0.05, -3.0, -3.0, **parameters)


def _value(data, grid, x, y):
    column = int(math.floor((x - grid.origin_x) / grid.resolution))
    row = int(math.floor((y - grid.origin_y) / grid.resolution))
    return data[row, column]


def _mark_from_approach(grid, distance=1.0, half_width=0.3, scans=3):
    """Mark a face at x = distance from viewpoints approaching it."""
    for index in range(scans):
        x = -0.6 + 0.6 * index / max(scans - 1, 1)
        angles, ranges = _face_scan(distance - x, half_width)
        grid.mark((x, 0.0), 0.0, angles, ranges)


def _face_scan(distance=1.0, half_width=0.3):
    """Beams from the origin hitting a wall segment at x = distance."""
    angles = np.linspace(-math.pi, math.pi, 361)[:-1]
    ranges = np.full(angles.shape, INF)
    for index, angle in enumerate(angles):
        if abs(angle) < math.pi / 2:
            y = distance * math.tan(angle)
            if abs(y) <= half_width:
                ranges[index] = distance / math.cos(angle)
    return angles, ranges


def test_disk_dilation_matches_radius():
    mask = np.zeros((21, 21), dtype=bool)
    mask[10, 10] = True
    grown = dilate(mask, disk_offsets(3))
    assert grown[10, 13] and grown[13, 10] and grown[12, 12]
    assert not grown[10, 14] and not grown[13, 13]
    assert grown.sum() == len(disk_offsets(3))


def test_novel_face_needs_confirmation_then_keeps_off_half_a_metre():
    grid = _grid()
    _mark_from_approach(grid, scans=2)
    assert not grid.evidence().any()

    _mark_from_approach(grid, scans=3)
    data = grid.render()
    assert _value(data, grid, 1.02, 0.0) == 100
    # The unseen depth behind the face is assumed solid.
    assert _value(data, grid, 1.35, 0.0) == 100
    # Keep-off: 0.5 m in front of the face and beside its end.
    assert _value(data, grid, 0.55, 0.0) == 100
    assert _value(data, grid, 1.02, 0.75) == 100
    assert _value(data, grid, 0.40, 0.0) == -1
    assert _value(data, grid, 1.02, 0.90) == -1


def test_raw_beams_clear_evidence_they_pass_through():
    grid = _grid()
    angles, ranges = _face_scan()
    _mark_from_approach(grid)
    assert grid.evidence().any()

    # The obstacle was removed: raw beams now reach a wall at 2.5 m.
    clear = np.where(np.isfinite(ranges), 2.5, INF)
    grid.clear((0.0, 0.0), 0.0, angles, clear)
    assert not grid.evidence().any()


def test_inf_raw_beams_inside_the_blind_zone_clear_nothing():
    grid = _grid()
    angles, ranges = _face_scan(distance=1.0)
    _mark_from_approach(grid)
    before = grid.evidence().copy()

    # The robot drove to 0.3 m from the face: the MID-360 returns inf there.
    grid.clear((0.7, 0.0), 0.0, angles, np.full(angles.shape, INF))
    assert (grid.evidence() == before).all()


def test_clearing_stops_short_of_the_surface_it_hits():
    grid = _grid()
    angles, ranges = _face_scan()
    _mark_from_approach(grid)
    grid.clear((0.0, 0.0), 0.0, angles, ranges)
    data = grid.render()
    assert _value(data, grid, 1.02, 0.0) == 100


def test_robot_disk_is_free_but_never_the_obstacle_itself():
    grid = _grid()
    angles, ranges = _face_scan()
    _mark_from_approach(grid)
    robot = (0.68, 0.0)
    data = grid.render(robot_xy=robot)
    assert _value(data, grid, 0.68, 0.0) == -1
    assert _value(data, grid, 0.68, 0.25) == -1
    assert _value(data, grid, 1.02, 0.0) == 100
    # Outside the robot disk the keep-off ring is unchanged.
    assert _value(data, grid, 0.68, 0.45) == 100


def test_out_of_map_returns_are_ignored():
    grid = _grid()
    angles = np.array([0.0])
    for _ in range(3):
        grid.mark((0.0, 0.0), 0.0, angles, np.array([9.0]))
    assert not grid.evidence().any()


def test_saved_map_values_survive_outside_the_keep_off():
    grid = _grid()
    angles, ranges = _face_scan()
    _mark_from_approach(grid)
    base = np.zeros((grid.height, grid.width), dtype=np.int8)
    base[:, :10] = -1
    base[100, 100] = 100
    data = grid.render(base=base)
    assert (data[:, :10] == -1).all()
    assert data[100, 100] == 100
    assert _value(data, grid, 1.02, 0.0) == 100
    assert _value(data, grid, 0.40, 0.0) == 0


def test_scans_wait_until_the_robot_has_stopped_turning():
    gate = TurnGate(max_yaw_rate=0.1, settle_s=0.6)
    assert gate.accepts(0.0)
    gate.observe(1.0, 0.05)
    assert gate.accepts(1.0)
    gate.observe(2.0, -0.5)
    assert not gate.accepts(2.0)
    gate.observe(2.1, 0.0)
    assert not gate.accepts(2.5)
    assert gate.accepts(2.6)


def test_returns_that_move_with_the_robot_are_never_confirmed():
    grid = _grid()
    angles = np.array([0.0])
    # A ghost 1 m ahead of a robot driving forward at 1 cm per scan, and a
    # stationary robot seeing the same return 20 times.
    for step in range(40):
        grid.mark((-1.0 + 0.01 * step, 0.0), 0.0, angles, np.array([1.0]))
    for _ in range(20):
        grid.mark((1.0, 1.0), 0.0, angles, np.array([1.0]))
    assert not grid.evidence().any()


def test_a_static_return_seen_while_approaching_is_confirmed():
    grid = _grid()
    angles = np.array([0.0])
    for step in range(30):
        x = -1.0 + 0.02 * step
        grid.mark((x, 0.0), 0.0, angles, np.array([1.5 - x]))
    assert grid.evidence().any()


def test_ghost_beams_learn_returns_that_move_with_the_robot():
    ghosts = GhostBeams(3)
    for step in range(120):
        x = 0.01 * step
        # Beam 0: ghost at a fixed 1.3 m.  Beam 1: a wall ahead at x = 3.
        # Beam 2: no return.
        ghosts.update((x, 0.0), [1.3, 3.0 - x, INF])
    assert ghosts.armed
    dropped = ghosts.drop([1.32, 2.4, INF])
    assert dropped[0] == INF
    assert dropped[1] == 2.4
    # A different range on the ghost beam is still a real return.
    assert ghosts.drop([0.8, 2.4, INF])[0] == 0.8


def test_passing_abeam_of_an_object_is_not_learned_as_a_ghost():
    ghosts = GhostBeams(1)
    # A post 1 m to the side of a straight pass, seen by the abeam beam.
    for step in range(200):
        x = -1.0 + 0.01 * step
        ghosts.update((x, 0.0), [math.hypot(x, 1.0)])
    assert math.isnan(ghosts.ghost_range[0])


def test_far_novel_returns_are_not_marked():
    grid = _grid()
    angles = np.array([0.0])
    for step in range(30):
        x = -2.9 + 0.02 * step
        grid.mark((x, 0.0), 0.0, angles, np.array([5.8 - x]))
    assert not grid.evidence().any()


def test_a_stopped_robot_learns_nothing_and_is_not_armed():
    ghosts = GhostBeams(1)
    for _ in range(50):
        ghosts.update((1.0, 1.0), [1.3])
    assert not ghosts.armed
    assert ghosts.drop([1.3])[0] == 1.3


def test_obstacle_next_to_a_stopped_robot_is_confirmed_once_armed():
    grid = _grid(stationary_confirm_hits=8)
    angles = np.array([0.0])
    grid.allow_stationary = True
    # Creeping past at 0.4 cm per scan does not count as standing still.
    for step in range(10):
        grid.mark((1.0 + 0.004 * step, 1.0), 0.0, angles,
                  np.array([0.6 - 0.004 * step]))
    assert not grid.evidence().any()
    for _ in range(8):
        grid.mark((1.0, 1.0), 0.0, angles, np.array([0.6]), stationary=True)
    assert grid.evidence().any()


def test_ghost_learning_ignores_returns_beyond_the_marking_range():
    ghosts = GhostBeams(1, max_range_m=4.0)
    for step in range(200):
        ghosts.update((0.01 * step, 0.0), [6.0])
    assert math.isnan(ghosts.ghost_range[0])


def test_reset_evidence_forgets_counts():
    grid = _grid()
    _mark_from_approach(grid)
    assert grid.evidence().any()
    grid.reset_evidence()
    assert not grid.evidence().any()
