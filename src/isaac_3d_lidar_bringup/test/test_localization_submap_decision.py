import math
import time

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_submap import SubmapFrame
from isaac_3d_lidar_bringup.localization_submap_decision import (
    path_length, Stop, SubmapDecisionConfig, submap_windows)
from isaac_3d_lidar_bringup.localization_surface_check import planar
from isaac_3d_lidar_bringup.localization_surface_validation import (
    see_through_ratio, SurfaceConflictConfig, surface_occupancy, validate_surface_evidence,
    WindowOrigins)
from test_localization_submap import _visible, _wall_x, _wall_y, SENSOR


def _twin_room():
    """
    Build a room that is the same after a half turn, but for one doorway.

    It is symmetric about the origin, except that one corner nook opens
    into a corridor (a doorway) and its twin is closed.
    The nook's walls hide the doorway from the middle of the room.
    Returns ``(map surface, corridor part)``.
    """
    nooks = [_wall_x(2., 1., 3.), _wall_y(1., 2., 3.6),          # true nook
             _wall_x(-2., -3., -1.), _wall_y(-1., -3.6, -2.)]    # its twin
    room = [_wall_x(4., -3., 1.2), _wall_x(4., 2.8, 3.), _wall_x(-4., -3., 3.),
            _wall_y(3., -4., 4.), _wall_y(-3., -4., 4.)]
    corridor = np.vstack([_wall_y(1.2, 4., 8.), _wall_y(2.8, 4., 8.),
                          _wall_x(8., 1.2, 2.8)])
    return np.vstack(room + nooks), corridor


def _stop(scene, map_pose, odom_pose, rng, clouds=8):
    """Frames at one stop: the scene seen from ``map_pose``, stamped with ``odom_pose``."""
    origin = (planar(*map_pose) @ SENSOR)[:3, 3]
    seen = _visible(scene, origin, bin_rad=.04)             # walls 1 m away
    sensor_T_map = np.linalg.inv(planar(*map_pose) @ SENSOR)
    windows = []
    for _ in range(2):
        frames = []
        for index in range(clouds):
            world = seen[rng.permutation(len(seen))[:3000]]
            points = (sensor_T_map @ np.c_[world, np.ones(len(world))].T).T[:, :3]
            frames.append(SubmapFrame(index, planar(*odom_pose), SENSOR, points))
        windows.append(tuple(frames))
    return Stop(*windows)


# The odometry frame differs from the map frame by an unknown transform.
MAP_FROM_ODOM = (1.3, -.7, math.radians(35))


def _odom(map_pose):
    matrix = np.linalg.inv(planar(*MAP_FROM_ODOM)) @ planar(*map_pose)
    return matrix[0, 3], matrix[1, 3], math.atan2(matrix[1, 0], matrix[0, 0])


def _decide(walls, corridor, train, holdout, origins, poses):
    return validate_surface_evidence(
        np.vstack([walls, corridor]), train, holdout, poses,
        deadline=time.monotonic() + 120, sensor_origin=origins)


def test_a_second_stop_resolves_a_twin_that_one_view_cannot():
    rng = np.random.default_rng(7)
    walls, corridor = _twin_room()
    scene = np.vstack([walls, corridor])
    middle, nook = (0., 0., 0.), (3.5, 2., math.radians(90))     # in the doorway
    twin = (-3.5, -2., math.radians(270))                       # the nook stop, half-turned
    config = SubmapDecisionConfig(max_path_m=4.5)

    one = [_stop(scene, middle, _odom(middle), rng)]
    train, holdout, origins, _ = submap_windows(one, config)
    single = _decide(walls, corridor, train, holdout, origins, [middle, (0., 0., math.pi)])
    assert not single.supported                                # the twins tie from the middle
    assert single.train.composite_gap < .02

    both = one + [_stop(scene, nook, _odom(nook), rng)]
    train, holdout, origins, path = submap_windows(both, config)
    assert path == pytest.approx(math.hypot(3.5, 2.), abs=1e-6)
    result = _decide(walls, corridor, train, holdout, origins, [nook, twin])
    assert result.supported and result.leader == 0, result.reason
    # At the twin the corridor lies behind a solid wall: seen through.
    assert min(window[1] for window in result.see_through) > .08
    # The stops land where they were seen from: in the nook stop's base
    # frame (facing +y in the map) the middle stop is 3.5 m to its left.
    assert np.allclose(origins.train[0][:2], (-2., 3.5), atol=1e-6)


def test_each_point_is_traced_from_its_own_stop():
    walls, _ = _twin_room()
    occupancy = surface_occupancy(walls, SurfaceConflictConfig())
    config = SurfaceConflictConfig()
    # A point on the far wall, seen from a stop in front of it: no crossing.
    point = np.array([[3.9, 0., 1.]])
    assert see_through_ratio(occupancy, point, np.array([[3., 0., .3]]), (0., 0., 0.),
                             config) == 0.
    # The same kind of point traced from the room's far side crosses the
    # nook wall.
    assert see_through_ratio(occupancy, np.array([[3.9, 2., 1.]]),
                             np.array([[-3.5, 2., .3]]), (0., 0., 0.), config) == 1.
    # One origin for all points still works (the stationary decision).
    assert see_through_ratio(occupancy, point, np.array([3., 0., .3]), (0., 0., 0.),
                             config) == 0.


def test_the_path_and_the_stops_are_bounded():
    rng = np.random.default_rng(8)
    walls, corridor = _twin_room()
    scene = np.vstack([walls, corridor])
    far = [_stop(scene, (0., 0., 0.), (0., 0., 0.), rng, 1),
           _stop(scene, (3., 0., 0.), (3., 0., 0.), rng, 1),
           _stop(scene, (3., 2., 0.), (3., 2., 0.), rng, 1)]
    assert path_length(far) == pytest.approx(5.)
    with pytest.raises(ContractError):
        submap_windows(far)
    with pytest.raises(ContractError):
        submap_windows(far[:1] * 5, SubmapDecisionConfig(max_path_m=10.))
    with pytest.raises(ContractError):
        submap_windows([Stop((), far[0].second)])
    with pytest.raises(ContractError):
        SubmapDecisionConfig(max_stops=0)
    assert isinstance(submap_windows(far[:1])[2], WindowOrigins)
