"""Stage D: map-wide coarse 3D candidate generation."""

import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from isaac_3d_lidar_bringup import localization_surface_search as search
from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_surface_check import planar


def _wall(x0, y0, x1, y1, step=.04):
    length = math.hypot(x1 - x0, y1 - y0)
    s, z = np.meshgrid(np.linspace(0, 1, max(2, int(length / step))), np.arange(0, 2, step))
    return np.c_[x0 + s.ravel() * (x1 - x0), y0 + s.ravel() * (y1 - y0), z.ravel()]


def _grid(width_m=10., height_m=6., resolution=.05):
    w, h = int(width_m / resolution), int(height_m / resolution)
    data = np.zeros((h, w), np.int8)
    data[[0, -1], :] = 100
    data[:, [0, -1]] = 100
    return NS(data=data.ravel().tolist(), info=NS(
        width=w, height=h, resolution=resolution,
        origin=NS(position=NS(x=0., y=0., z=0.), orientation=NS(w=1.))))


# An L-shaped room with a pillar: one pose explains the scan.
SURFACE = np.concatenate([_wall(0, 0, 10, 0), _wall(10, 0, 10, 6), _wall(10, 6, 0, 6),
                          _wall(0, 6, 0, 0), _wall(6, 2, 6.6, 2), _wall(6.6, 2, 6.6, 3.2)])


def _scan_at(pose, points=SURFACE):
    inverse = np.linalg.inv(planar(*pose))
    local = (inverse @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    return local[np.hypot(local[:, 0], local[:, 1]) < 6]


def test_the_true_pose_leads_the_map_wide_search():
    config = search.SurfaceSearchConfig(margin=.05)
    field = search.build_surface_field(SURFACE, config)
    positions = search.free_positions(_grid(), config)
    truth = (3.1, 2.3, math.radians(33))
    result = search.search_surface(field, _scan_at(truth), positions, config)
    assert result.complete
    x, y, yaw, _ = result.candidates[0]
    assert math.hypot(x - truth[0], y - truth[1]) <= .15
    assert abs(math.atan2(math.sin(yaw - truth[2]), math.cos(yaw - truth[2]))) <= math.radians(6)
    assert result.evaluated == len(positions) * 36


def test_every_place_within_the_margin_is_kept_or_the_search_is_incomplete():
    # A long corridor seen to 1.5 m: sliding along it changes nothing.
    corridor = np.concatenate([_wall(0, 0, 10, 0), _wall(0, 2, 10, 2)])
    scan = _scan_at((5, 1, 0), corridor)
    scan = scan[np.hypot(scan[:, 0], scan[:, 1]) < 1.5]
    grid = _grid(10., 2.)
    config = search.SurfaceSearchConfig(margin=.05, max_candidates=3)
    field = search.build_surface_field(corridor, config)
    result = search.search_surface(field, scan, search.free_positions(grid, config), config)
    assert not result.complete and result.reason == 'TOO_MANY_CANDIDATES'
    assert len(result.candidates) == 3
    roomy = search.SurfaceSearchConfig(margin=.05, max_candidates=400)
    full = search.search_surface(field, scan, search.free_positions(grid, roomy), roomy)
    assert full.complete and len(full.candidates) > 10
    scores = [c[3] for c in full.candidates]
    assert scores == sorted(scores, reverse=True)
    assert min(scores) >= full.best - .05
    for i, a in enumerate(full.candidates):        # one per neighbourhood
        for b in full.candidates[:i]:
            assert (math.hypot(a[0] - b[0], a[1] - b[1]) >= roomy.nms_xy_m
                    or abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
                    >= roomy.nms_yaw_rad)


def test_free_positions_respect_clearance_and_refuse_rotated_maps():
    config = search.SurfaceSearchConfig(clearance_m=.5)
    positions = search.free_positions(_grid(), config)
    assert positions[:, 0].min() >= .5 and positions[:, 1].min() >= .5
    assert positions[:, 0].max() <= 9.5 and positions[:, 1].max() <= 5.5
    rotated = _grid()
    rotated.info.origin.orientation.w = .9
    with pytest.raises(ContractError):
        search.free_positions(rotated, config)


def test_invalid_configuration_and_empty_input():
    with pytest.raises(ContractError):
        search.SurfaceSearchConfig(margin=0)
    with pytest.raises(ContractError):
        search.build_surface_field(np.zeros((0, 3)))
    field = search.build_surface_field(SURFACE)
    assert search.search_surface(field, np.zeros((0, 3)), np.zeros((5, 2))).reason == \
        'NO_POINTS_OR_POSITIONS'
