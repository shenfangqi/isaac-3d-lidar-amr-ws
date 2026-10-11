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
