import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_surface_additions import (
    load_surface_additions, save_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import (
    planar, solid_points, surface_tree)
from isaac_3d_lidar_bringup.localization_surface_validation import (
    SurfaceConflictConfig, see_through_ratio, surface_occupancy)
from isaac_3d_lidar_bringup.localization_variable_regions import VariableRegions

SHA = 'ab' * 32


def test_additions_are_bound_to_their_mesh(tmp_path):
    path = tmp_path / 'additions.npz'
    save_surface_additions(path, [[1, 2, 0.3], [1, 2.02, 0.3]], SHA, 'bed')
    assert load_surface_additions(path, SHA).shape == (2, 3)
    with pytest.raises(ContractError):
        load_surface_additions(path, 'cd' * 32)
    save_surface_additions(path, np.zeros((0, 3)), SHA)
    with pytest.raises(ContractError):
        load_surface_additions(path, SHA)
    save_surface_additions(path, [[np.nan, 0, 0]], SHA)
    with pytest.raises(ContractError):
        load_surface_additions(path, SHA)


def _plane(x0, x1, y0, y1, z, step=.02):
    x, y = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.c_[x.ravel(), y.ravel(), np.full(x.size, z)]


def test_additions_are_fitted_but_never_solid():
    wall = np.c_[np.full(200, 3.0), np.linspace(-2, 2, 200), np.full(200, .5)]
    slats = _plane(1.0, 1.6, -.5, .5, .4)                 # missed by the mesh
    tree = surface_tree(wall, additions=slats)
    distance, _ = tree.query([[1.3, 0.0, .4]])
    assert distance[0] < .02                              # fitted
    assert len(solid_points(tree)) == len(wall)           # not solid
    regions = VariableRegions(np.array([[-5., -5., 0., 5., -1.5, 3.]]), ('r',))
    masked = surface_tree(wall, regions, slats)
    assert masked.query([[1.3, 0.0, .4]])[0][0] < .02
    assert not regions.contains(solid_points(masked)).any()
    assert len(masked.full_solid) == len(wall)


def test_a_ray_through_an_addition_is_not_see_through():
    # A ray from the origin under/through the slats to the wall behind:
    # with the slats solid the true pose would see through them.
    wall = _plane(2.95, 3.05, -1, 1, .4, step=.02)
    slats = _plane(1.0, 1.6, -.5, .5, .4)
    observed = np.c_[np.full(50, 3.0), np.linspace(-.4, .4, 50), np.full(50, .4)]
    config = SurfaceConflictConfig()
    origin = (0., 0., .4)
    as_solid = surface_occupancy(np.vstack([wall, slats]), config)
    fitted = surface_occupancy(solid_points(surface_tree(wall, additions=slats)), config)
    pose = (0., 0., 0.)
    assert see_through_ratio(as_solid, observed, origin, pose, config) > config.max_ratio
    assert see_through_ratio(fitted, observed, origin, pose, config) == 0.
    assert planar(*pose).shape == (4, 4)
