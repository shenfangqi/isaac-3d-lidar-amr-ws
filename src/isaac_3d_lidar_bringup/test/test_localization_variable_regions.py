"""Map regions that change in use are ignored by the 3D scoring."""

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_surface_check import (
    band_inliers, check_surfaces, DEFAULT_BANDS, SurfaceCheckConfig, surface_tree)
from isaac_3d_lidar_bringup.localization_variable_regions import (
    load_variable_regions, VariableRegions)

SHA = 'ab' * 32
CURTAIN = VariableRegions(np.array([[-1., -1., .3, 1., -.3, 2.]]), ('curtain',))


def test_regions_are_bound_to_their_mesh(tmp_path):
    path = tmp_path / 'regions.yaml'
    path.write_text(f'mesh_sha256: {SHA}\nregions:\n'
                    '  - {name: curtain, min: [-1, -1, 0.3], max: [1, -0.3, 2]}\n')
    regions = load_variable_regions(path, SHA)
    assert regions.names == ('curtain',)
    assert regions.contains([[0, -.5, 1], [0, 0, 1], [0, -.5, .1]]).tolist() == [
        True, False, False]
    with pytest.raises(ContractError, match='for mesh'):
        load_variable_regions(path, 'cd' * 32)
    path.write_text(f'mesh_sha256: {SHA}\nregions:\n'
                    '  - {name: bad, min: [1, 0, 0], max: [0, 1, 1]}\n')
    with pytest.raises(ContractError, match='invalid'):
        load_variable_regions(path, SHA)
    path.write_text(f'mesh_sha256: {SHA}\nregions: []\n')
    with pytest.raises(ContractError, match='no region'):
        load_variable_regions(path, SHA)


def _wall(y, x0=-2., x1=2.):
    x, z = np.meshgrid(np.arange(x0, x1, .02), np.arange(.3, 1.9, .02))
    return np.c_[x.ravel(), np.full(x.size, y), z.ravel()]


def test_surface_and_points_in_a_region_are_not_scored():
    # Mapped: a window recess behind the curtain line (y = -0.8) and a side
    # wall.  Seen: a closed curtain (y = -0.42) and the same side wall.
    side = np.c_[np.full(4000, 1.5), np.random.default_rng(0).uniform(-.2, 2, 4000),
                 np.random.default_rng(1).uniform(.3, 1.8, 4000)]
    mapped = np.r_[_wall(-.8, -.95, .95), side]
    seen = np.r_[_wall(-.42, -.95, .95), side]
    plain = surface_tree(mapped)
    masked = surface_tree(mapped, CURTAIN)
    assert len(masked.data) < len(plain.data)
    assert not CURTAIN.contains(masked.data).any()
    pose = (0., 0., 0.)
    before = band_inliers(seen, pose, plain, DEFAULT_BANDS, .05)
    after = band_inliers(seen, pose, masked, DEFAULT_BANDS, .05)
    assert sum(n for n, _ in after) == len(side)            # curtain points dropped
    assert sum(k for _, k in after) == sum(k for _, k in before)
    config = SurfaceCheckConfig(min_points=100, min_band_points=10, min_leader_composite=.01)
    assert check_surfaces(plain, seen, (pose, pose), config).composite[0] < .5
    assert check_surfaces(masked, seen, (pose, pose), config).composite[0] > .95


def test_a_region_may_not_choose_the_leader():
    # Rooms A (y 0-4) and B (y 10-14) are alike, but B has a pillar.  The
    # robot is in A; its candidates are B and a poor pose.  Without regions
    # B's rays cross the pillar and B is excluded.  A region over the pillar
    # hides that evidence: B would lead and pass, so the decision refuses.
    import time
    from isaac_3d_lidar_bringup import localization_surface_validation as validation

    def wall(x0, y0, x1, y1):
        n = max(2, int(np.hypot(x1 - x0, y1 - y0) / .04))
        s, z = np.meshgrid(np.linspace(0, 1, n), np.arange(0., 2., .04))
        return np.c_[x0 + s.ravel() * (x1 - x0), y0 + s.ravel() * (y1 - y0), z.ravel()]

    def room(y0):
        return [wall(0, y0, 0, y0 + 4), wall(0, y0, 4, y0), wall(0, y0 + 4, 4, y0 + 4),
                wall(4, y0, 4, y0 + 4)]
    pillar = wall(2.6, 11.4, 2.6, 12.6)
    surface = np.concatenate(room(0) + room(10) + [pillar])
    rng = np.random.default_rng(3)
    seen = np.concatenate(room(0))
    seen = seen[rng.choice(len(seen), 12000, replace=False)] - (2, 2, 0)
    seen = seen[np.hypot(seen[:, 0], seen[:, 1]) > .6]
    train, held = seen[::2], seen[1::2]
    poses = [(2., 12., 0.), (2.6, 1.3, .9)]
    check = SurfaceCheckConfig(tolerance_m=.05, min_points=1000, min_band_points=100,
                               min_leader_composite=.01)
    pillar_region = VariableRegions(np.array([[2.5, 11.3, 0., 2.7, 12.7, 2.]]), ('pillar',))
    kwargs = dict(deadline=time.monotonic() + 60, check_config=check,
                  sensor_origin=(0., 0., .2))
    plain = validation.validate_surface_evidence(surface, train, held, poses, **kwargs)
    assert plain.leader != 0
    masked = validation.validate_surface_evidence(
        surface_tree(surface, pillar_region), train, held, poses, **kwargs)
    assert masked.reason == 'VARIABLE_REGIONS_CHANGE_LEADER'


def test_mask_grid_hides_regions_that_overlap_the_scan_band():
    from array import array
    from types import SimpleNamespace as NS
    from isaac_3d_lidar_bringup.localization_variable_regions import mask_grid
    grid = NS(frame_id='map', info=NS(
        width=10, height=10, resolution=.1,
        origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
        data=array('b', [100] * 100))
    low = VariableRegions(np.array([[.2, .2, .1, .4, .3, .4]]), ('bed edge',))
    high = VariableRegions(np.array([[.2, .2, 1.0, .4, .3, 2.0]]), ('curtain top',))
    masked = mask_grid(grid, low, .22, .35)
    cells = np.frombuffer(masked.data.tobytes(), np.int8).reshape(10, 10)
    assert (cells[2:3, 2:4] == -1).all() and (cells == -1).sum() == 2
    assert list(grid.data) == [100] * 100                 # the original is untouched
    assert mask_grid(grid, high, .22, .35) is grid        # above the band: unchanged
    assert mask_grid(grid, None, .22, .35) is grid
