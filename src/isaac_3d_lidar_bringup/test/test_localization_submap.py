import math

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_submap import (
    build_submap, compare_submap, CompareConfig, components, SubmapConfig, SubmapFrame)
from isaac_3d_lidar_bringup.localization_surface_check import planar, surface_tree

SENSOR = np.eye(4)
SENSOR[2, 3] = .3                                  # sensor 0.3 m above the base


def _wall_x(x, y0, y1, z0=0., z1=3., step=.03):
    y, z = np.meshgrid(np.arange(y0, y1, step), np.arange(z0, z1, step))
    return np.c_[np.full(y.size, x), y.ravel(), z.ravel()]


def _wall_y(y, x0, x1, z0=0., z1=3., step=.03):
    x, z = np.meshgrid(np.arange(x0, x1, step), np.arange(z0, z1, step))
    return np.c_[x.ravel(), np.full(x.size, y), z.ravel()]


def _room():
    return np.vstack([_wall_x(4., -3., 3.), _wall_x(-4., -3., 3.),
                      _wall_y(3., -4., 4.), _wall_y(-3., -4., 4.)])


def _box(centre, size, step=.03):
    """Surface points of an axis-aligned box (all six faces)."""
    low = np.asarray(centre) - np.asarray(size) / 2
    high = low + size
    axes = [np.arange(low[i], high[i] + 1e-9, step) for i in range(3)]
    grid = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    on_face = ((np.isclose(grid, low, atol=step / 2) | np.isclose(grid, high, atol=step / 2))
               .any(1))
    return grid[on_face]


def _visible(scene, origin, voxel=.05):
    """Scene points the sensor at ``origin`` sees first along its rays (coarse z-buffer)."""
    delta = scene - origin
    distance = np.linalg.norm(delta, axis=1)
    azimuth = np.round(np.arctan2(delta[:, 1], delta[:, 0]) / .01).astype(int)
    elevation = np.round(np.arcsin(delta[:, 2] / distance) / .01).astype(int)
    order = np.lexsort((distance, elevation, azimuth))
    key = np.c_[azimuth, elevation][order]
    first = np.r_[True, (np.diff(key, axis=0) != 0).any(1)]
    return scene[order[first]]


def _frames(scene_at, poses, rng):
    """Frames of the scenes ``scene_at(i)`` seen from base poses ``(x, y, yaw)``."""
    frames = []
    for index, pose in enumerate(poses):
        odom_T_base = planar(*pose)
        origin = (odom_T_base @ SENSOR)[:3, 3]
        world = _visible(scene_at(index), origin)
        world = world[rng.permutation(len(world))[:4000]]
        sensor_T_world = np.linalg.inv(odom_T_base @ SENSOR)
        points = (sensor_T_world @ np.c_[world, np.ones(len(world))].T).T[:, :3]
        frames.append(SubmapFrame(index, odom_T_base, SENSOR, points))
    return frames


def test_frames_from_different_poses_land_in_one_frame():
    rng = np.random.default_rng(1)
    room = _room()
    poses = [(0., 0., 0.), (.5, .2, math.radians(30)), (1., -.3, math.radians(-60))]
    submap = build_submap(_frames(lambda i: room, poses, rng), planar(*poses[1]),
                          SubmapConfig(min_frames=1))
    # Back in the odometry frame every voxel sits on a wall.
    world = (planar(*poses[1]) @ np.c_[submap.points, np.ones(len(submap.points))].T).T[:, :3]
    distance = surface_tree(room).query(world)[0]
    assert np.percentile(distance, 99) < .05
    assert submap.frame_count == 3
    assert np.allclose(submap.origins[0], [-.5 * math.cos(-math.radians(30)) - .2 *
                                           math.sin(-math.radians(30)), 0, .3], atol=.6)


def test_a_passer_by_is_dropped_and_the_walls_kept():
    rng = np.random.default_rng(2)
    room = _room()
    person = _box((2., .5, .8), (.3, .3, 1.6))

    def scene(index):                               # in view for 3 of 24 frames
        return np.vstack([room, person]) if index in (4, 5, 6) else room

    submap = build_submap(_frames(scene, [(0., 0., 0.)] * 24, rng), np.eye(4),
                          SubmapConfig(carve_every=1))
    kept_person = (np.abs(submap.points - (2., .5, .8)) < (.2, .2, .85)).all(1).sum()
    dropped_person = (np.abs(submap.dropped - (2., .5, .8)) < (.2, .2, .85)).all(1).sum()
    assert kept_person == 0 and dropped_person > 20
    assert (np.abs(submap.points[:, 0] - 4.) < .05).sum() > 500     # the far wall stays


def test_added_and_removed_structure_are_found_at_the_true_pose():
    rng = np.random.default_rng(3)
    room = _room()
    pillar = _box((2.5, -1., .75), (.3, .3, 1.5))   # mapped, gone today
    crate = _box((-2., 1.5, .3), (.6, .6, .6))       # new today
    pose = (.4, -.2, math.radians(20))
    frames = _frames(lambda i: np.vstack([room, crate]), [pose] * 12, rng)
    submap = build_submap(frames, planar(*pose), SubmapConfig(carve_every=2))
    result = compare_submap(submap, pose, np.vstack([room, pillar]))
    added = [c for c in result.changes if c.kind == 'added']
    removed = [c for c in result.changes if c.kind == 'removed']
    assert len(added) == 1 and np.allclose(added[0].centroid[:2], (-2., 1.5), atol=.25)
    assert removed and removed[0].points > 100
    assert all(np.allclose(c.centroid[:2], (2.5, -1.), atol=.25) for c in removed)
    assert result.added.mean() < .2
    # Without the pillar in the map nothing is removed and nothing is seen through.
    clean = compare_submap(submap, pose, room)
    assert not [c for c in clean.changes if c.kind == 'removed']
    assert clean.see_through < .01


def test_a_wrong_pose_disagrees_everywhere():
    rng = np.random.default_rng(4)
    room = _room()
    pose = (.4, -.2, math.radians(20))
    submap = build_submap(_frames(lambda i: room, [pose] * 6, rng), planar(*pose))
    right = compare_submap(submap, pose, room)
    wrong = compare_submap(submap, (1.4, .6, math.radians(50)), room)
    assert right.added.mean() < .02 and right.see_through < .01
    assert wrong.added.mean() > .3 and wrong.see_through > .1


def test_components_join_neighbours_only():
    points = np.array([[0, 0, 0], [.1, 0, 0], [.2, 0, 0], [2, 0, 0], [2.1, 0, 0.]])
    labels = components(points, .15)
    assert labels[0] == labels[1] == labels[2] != labels[3] == labels[4]
    assert components(np.zeros((1, 3)), .15).tolist() == [0]


def test_limits_are_checked():
    with pytest.raises(ContractError):
        SubmapConfig(min_range_m=20.)
    with pytest.raises(ContractError):
        SubmapConfig(max_free_share=1.5)
    with pytest.raises(ContractError):
        CompareConfig(min_removed_frames=0)
    with pytest.raises(ContractError):
        build_submap([SubmapFrame(0, np.eye(4), SENSOR, np.zeros((5, 3)))], np.eye(4))
