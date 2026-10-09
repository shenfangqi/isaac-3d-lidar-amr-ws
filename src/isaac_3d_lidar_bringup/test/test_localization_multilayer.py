"""Analytic twin rooms: only an upper-height structure breaks the alias."""

from dataclasses import replace

import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError, FrameRole, Hypothesis, SE2
from isaac_3d_lidar_bringup.localization_hypotheses import (
    map_hash, SearchConfig, ValidationThresholds,
)
from isaac_3d_lidar_bringup.localization_multilayer import (
    HeightLayer, LayerObservation, compare_height_layers,
)
from test_localization_hypotheses import _frames, _grid, _rectangle, IDENTITY


@pytest.fixture
def twin_rooms():
    lower = _rectangle(0.2, 0.2, 3.2, 3.2) + _rectangle(5.2, 0.2, 8.2, 3.2)
    upper = lower + [((0.2, 1.8), (1.8, 1.8))]
    truth = SE2(1., 1., 0.)
    grids = [_grid(walls, 9., 4.) for walls in (lower, upper)]
    nav_hash = map_hash(grids[0])
    layers = tuple(HeightLayer(name, lo, hi, grid, map_hash(grid), nav_hash, 'a' * 64)
                   for name, lo, hi, grid in zip(('low', 'high'), (.22, .6), (.35, 1.0), grids))
    observations = tuple(LayerObservation(
        layer.name, layer.min_z, layer.max_z,
        _frames(walls, truth, FrameRole.HOLDOUT, start_id=100))
        for layer, walls in zip(layers, (lower, upper)))
    candidates = tuple(Hypothesis(p.x, p.y, p.yaw, .9, 1., 0., i, (), ())
                       for i, p in enumerate((truth, SE2(6., 1., 0.))))
    return dict(layers=layers, observations=observations, candidates=candidates,
                reference_odom=IDENTITY, navigation_map_hash=nav_hash,
                source_map_sha256='a' * 64, config=SearchConfig(),
                thresholds=ValidationThresholds())


def test_upper_geometry_separates_identical_lower_rooms(twin_rooms):
    result = compare_height_layers(**twin_rooms)
    a, b = result['candidates']
    assert a['cluster_id'] == 0
    assert a['layers']['low']['score'] == pytest.approx(b['layers']['low']['score'])
    assert a['layers']['high']['score'] > b['layers']['high']['score']
    assert result['score_margin'] > 0
    assert not result['navigation_accepted']  # synthetic separation is not acceptance


def test_unknown_upper_map_cannot_vote_for_known_candidate(twin_rooms):
    layer = twin_rooms['layers'][1]
    grid = layer.grid
    # Erase the twin's upper map observations, preserving the true room.
    for y in range(grid.info.height):
        for x in range(round(4. / grid.info.resolution), grid.info.width):
            grid.data[y * grid.info.width + x] = -1
    twin_rooms['layers'] = (twin_rooms['layers'][0], replace(layer, grid_hash=map_hash(grid)))
    result = compare_height_layers(**twin_rooms)
    assert not result['comparable']
    assert result['score_margin'] is None
    assert all(row['balanced_score'] is None for row in result['candidates'])


@pytest.mark.parametrize('field', ['navigation_map_hash', 'source_map_sha256'])
def test_wrong_map_provenance_is_rejected(twin_rooms, field):
    twin_rooms[field] = 'b' * 64
    with pytest.raises(ContractError, match='provenance'):
        compare_height_layers(**twin_rooms)


def test_mutated_map_is_rejected(twin_rooms):
    twin_rooms['layers'][1].grid.data[0] = 100
    with pytest.raises(ContractError, match='provenance'):
        compare_height_layers(**twin_rooms)


def test_mismatched_height_is_rejected(twin_rooms):
    a, b = twin_rooms['observations']
    twin_rooms['observations'] = (a, replace(b, min_z=.5))
    with pytest.raises(ContractError, match='height interval'):
        compare_height_layers(**twin_rooms)


def test_training_data_cannot_be_reused(twin_rooms):
    twin_rooms['train_stamps'] = [twin_rooms['observations'][0].frames[0].stamp_ns]
    with pytest.raises(ContractError, match='HOLDOUT'):
        compare_height_layers(**twin_rooms)


def test_layers_cannot_use_different_source_poses(twin_rooms):
    a, b = twin_rooms['observations']
    frames = tuple(replace(f, T_odom_base=SE2(.1, 0., 0.)) for f in b.frames)
    twin_rooms['observations'] = (a, replace(b, frames=frames))
    with pytest.raises(ContractError, match='source stamps'):
        compare_height_layers(**twin_rooms)


def test_repeated_observation_does_not_inflate_score(twin_rooms):
    baseline = compare_height_layers(**twin_rooms)
    twin_rooms['observations'] = tuple(replace(obs, frames=obs.frames[:1])
                                       for obs in twin_rooms['observations'])
    single = compare_height_layers(**twin_rooms)
    assert single['score_margin'] == baseline['score_margin']


def test_overlapping_height_layers_are_rejected(twin_rooms):
    a, b = twin_rooms['layers']
    twin_rooms['layers'] = (a, replace(b, min_z=.3))
    with pytest.raises(ContractError, match='overlapping'):
        compare_height_layers(**twin_rooms)
