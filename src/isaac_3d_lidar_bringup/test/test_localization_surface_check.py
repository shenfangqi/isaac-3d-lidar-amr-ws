"""Static 3D surface re-check core: cloud parsing and leader nomination."""

import math
import struct
from types import SimpleNamespace as NS

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_surface_check import (
    check_surfaces, planar, pointcloud2_xyz, SurfaceCheckConfig,
)

pytest.importorskip('scipy.spatial')
SMALL = SurfaceCheckConfig(min_band_points=50, min_points=200)


def _cloud(points, step=16, extra_field=True):
    fields = [NS(name='x', offset=0, datatype=7), NS(name='y', offset=4, datatype=7),
              NS(name='z', offset=8, datatype=7)]
    if extra_field:
        fields.append(NS(name='intensity', offset=12, datatype=7))
    data = b''.join(struct.pack('<ffff', *p, 1.0) for p in points)
    return NS(fields=fields, is_bigendian=False, width=len(points), height=1,
              point_step=step, data=data)


def test_pointcloud2_xyz_reads_offsets_and_drops_non_finite_points():
    xyz = pointcloud2_xyz(_cloud([(1, 2, 3), (float('nan'), 0, 0), (4, 5, 6)]))
    assert xyz.tolist() == [[1, 2, 3], [4, 5, 6]]
    assert len(pointcloud2_xyz(_cloud([(i, 0, 0) for i in range(10)]), 4)) == 4


def test_pointcloud2_xyz_refuses_unexpected_layouts():
    message = _cloud([(1, 2, 3)])
    message.fields[2].datatype = 8                # float64 z
    with pytest.raises(ContractError):
        pointcloud2_xyz(message)
    message = _cloud([(1, 2, 3)])
    message.data = message.data[:8]
    with pytest.raises(ContractError):
        pointcloud2_xyz(message)


def _room(shelf=True):
    """4 x 3 m room (half-turn symmetric) plus a shelf at 1.0-1.5 m."""
    points = []
    for z in np.arange(0.0, 2.0, 0.05):
        for x in np.arange(0.0, 4.0, 0.05):
            points += [(x, 0.0, z), (x, 3.0, z)]
        for y in np.arange(0.0, 3.0, 0.05):
            points += [(0.0, y, z), (4.0, y, z)]
    if shelf:
        for z in np.arange(1.0, 1.5, 0.05):
            for x in np.arange(2.0, 4.0, 0.05):
                points.append((x, 2.6, z))
    return np.array(points)


def _seen_from(mesh, pose):
    base = (np.linalg.inv(planar(*pose)) @ np.c_[mesh, np.ones(len(mesh))].T).T[:, :3]
    return base


def test_the_true_pose_is_nominated_over_its_half_turn_twin():
    mesh = _room()
    truth = (3.0, 2.0, math.radians(30))
    twin = (1.0, 1.0, truth[2] + math.pi)
    # The shelf is about 3 % of the points, so the gap is small but real.
    config = SurfaceCheckConfig(min_band_points=50, min_points=200,
                                min_composite_gap=0.02, min_leader_composite=0.7)
    result = check_surfaces(mesh, _seen_from(mesh, truth), [twin, truth], config)
    assert result.resolved and result.leader == 1
    assert result.composite_gap > 0.02


def test_a_symmetric_scene_is_not_resolved():
    mesh = _room(shelf=False)
    truth = (3.0, 2.0, math.radians(30))
    twin = (1.0, 1.0, truth[2] + math.pi)
    result = check_surfaces(mesh, _seen_from(mesh, truth), [twin, truth], SMALL)
    assert not result.resolved and result.reason == 'LEADER_GAP_TOO_SMALL'


def test_too_few_points_and_too_few_candidates():
    mesh = _room()
    result = check_surfaces(mesh, np.zeros((10, 3)) + 1.0,
                            [(1, 1, 0), (2, 2, 0)], SMALL)
    assert not result.resolved and result.reason == 'TOO_FEW_POINTS'
    with pytest.raises(ContractError):
        check_surfaces(mesh, _seen_from(mesh, (1, 1, 0)), [(1, 1, 0)], SMALL)


def test_a_poor_absolute_fit_is_not_resolved_even_with_a_gap():
    mesh = _room()
    truth = (3.0, 2.0, math.radians(30))
    points = _seen_from(mesh, truth)
    noisy = points + np.random.default_rng(0).normal(0, 0.12, points.shape)
    config = SurfaceCheckConfig(min_band_points=50, min_points=200,
                                min_composite_gap=0.01, min_leader_composite=0.9)
    result = check_surfaces(mesh, noisy, [(1.0, 1.0, 0.0), truth], config)
    assert not result.resolved and result.reason == 'LEADER_FIT_TOO_LOW'


def test_config_is_validated():
    with pytest.raises(ContractError):
        SurfaceCheckConfig(tolerance_m=0)
    with pytest.raises(ContractError):
        SurfaceCheckConfig(min_points=0)
