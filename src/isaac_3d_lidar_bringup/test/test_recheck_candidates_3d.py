"""Offline 3D candidate re-check: geometry, mesh loading and comparison."""

import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from recheck_candidates_3d import (  # noqa: E402
    band_inliers, chain_to_base, compare, DEFAULT_BANDS, load_ply_vertices,
    planar, same_pose, transform,
)

scipy_spatial = pytest.importorskip('scipy.spatial')


def _room_with_marker():
    """4 x 3 m room (half-turn symmetric) plus a shelf only at 1.0-1.5 m."""
    points = []
    for z in np.arange(0.0, 2.0, 0.05):
        for x in np.arange(0.0, 4.0, 0.05):
            points += [(x, 0.0, z), (x, 3.0, z)]
        for y in np.arange(0.0, 3.0, 0.05):
            points += [(0.0, y, z), (4.0, y, z)]
    for z in np.arange(1.0, 1.5, 0.05):
        for x in np.arange(2.0, 4.0, 0.05):
            points.append((x, 2.6, z))          # along one wall only
    return np.array(points)


def _observed(mesh, pose):
    """Express the mesh in the base frame at ``pose`` (no visibility)."""
    world_to_base = np.linalg.inv(planar(*pose))
    base = (world_to_base @ np.c_[mesh, np.ones(len(mesh))].T).T[:, :3]
    return base[np.hypot(base[:, 0], base[:, 1]) >= 0.5]


def test_half_turn_twin_is_separated_only_where_the_marker_is():
    mesh = _room_with_marker()
    truth = (3.0, 2.0, math.radians(30))
    # Same view of the bare room, rotated by 180 deg about its centre.
    twin = (4.0 - truth[0], 3.0 - truth[1], truth[2] + math.pi)
    points = _observed(mesh, truth)
    tree = scipy_spatial.cKDTree(mesh)
    result = compare([band_inliers(points, pose, tree, DEFAULT_BANDS, 0.1)
                      for pose in (twin, truth)], min_points=50)
    assert result['order'][0] == 1                 # truth wins
    assert result['composite_gap'] > 0.0
    margins = dict(zip(DEFAULT_BANDS, result['band_margins']))
    # The shelf is ~1/8 of that band's points; only that band decides.
    assert margins[(1.0, 1.5)] > 0.05
    assert abs(margins[(0.22, 0.35)]) < 0.02       # bare walls: no evidence


def test_bands_with_too_few_points_are_excluded_for_everyone():
    result = compare([[(10, 10), (500, 100)], [(10, 0), (500, 400)]],
                     min_points=100)
    assert result['bands_used'] == [False, True]
    assert result['order'] == [1, 0]
    assert result['composite'] == [0.2, 0.8]
    assert result['band_margins'][0] is None


def test_static_chain_uses_full_3d_rotation():
    tilt = math.radians(10)
    static = {
        'base_link': ('base_footprint', transform((0, 0, 0.09), (0, 0, 0, 1))),
        'imu_link': ('base_link', transform(
            (0.1, 0, 0.1), (0, math.sin(tilt / 2), 0, math.cos(tilt / 2)))),
    }
    point = chain_to_base('imu_link', static) @ np.array([1.0, 0, 0, 1])
    assert point[0] == pytest.approx(0.1 + math.cos(tilt))
    assert point[2] == pytest.approx(0.19 - math.sin(tilt))
    with pytest.raises(ValueError):
        chain_to_base('missing', static)


def test_ascii_ply_vertices_are_read_and_binary_is_refused(tmp_path):
    ascii_ply = tmp_path / 'a.ply'
    ascii_ply.write_text(
        'ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\n'
        'property float y\nproperty float z\nproperty float nx\n'
        'element face 0\nproperty list uchar int vertex_indices\nend_header\n'
        '1 2 3 0\n4 5 6 0\n')
    assert load_ply_vertices(ascii_ply).tolist() == [[1, 2, 3], [4, 5, 6]]
    binary = tmp_path / 'b.ply'
    binary.write_text('ply\nformat binary_little_endian 1.0\nend_header\n')
    with pytest.raises(ValueError, match='ASCII'):
        load_ply_vertices(binary)


def test_evaluation_tolerance_handles_yaw_wrap():
    assert same_pose((0, 0, math.radians(179)), (0.1, 0, math.radians(-179)))
    assert not same_pose((0, 0, 0), (0, 0, math.pi))
    assert not same_pose((0, 0, 0), (0.5, 0, 0))
