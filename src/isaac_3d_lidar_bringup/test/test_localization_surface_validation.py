"""Independent observations, geometric observability, and bounded work."""
import math
import time

import numpy as np
import pytest
from scipy.spatial import cKDTree

from isaac_3d_lidar_bringup import localization_surface_validation as validation
from isaac_3d_lidar_bringup.localization_surface_check import SurfaceCheckConfig
from isaac_3d_lidar_bringup.localization_surface_validation import (
    decide_surface,
    refine_surface_pose,
    SurfaceRefineConfig,
    validate_surface_evidence,
)

CONFIG = SurfaceCheckConfig(min_band_points=20, min_points=100)
POSES = ((0., 0., 0.), (6., 0., 0.))


def room():
    a, z = np.meshgrid(np.linspace(-1, 1, 31), np.linspace(.3, 1.8, 31))
    return np.vstack((np.c_[a.ravel(), np.full(a.size, 2.), z.ravel()],
                      np.c_[np.full(a.size, 1.), a.ravel(), z.ravel()],
                      np.c_[np.full(a.size, -1.), a.ravel(), z.ravel()]))


def run(vertices, train, held, deadline=None):
    if deadline is None:
        deadline = time.monotonic() + 10
    return validate_surface_evidence(vertices, train, held, POSES,
                                     deadline=deadline, check_config=CONFIG)


def test_asymmetric_walls_constrain_position_and_heading():
    vertices = room()
    result = run(vertices, vertices[::2], vertices[1::2])
    assert result.supported, result
    assert len(result.support_extents) == 5


def _wall_scan(rng, count, offsets):
    """Points sampled continuously (not on mesh vertices) on walls x = offset."""
    return np.vstack([np.c_[np.full(count, x), rng.uniform(-1, 1, count),
                            rng.uniform(.3, 1.8, count)] for x in offsets])


def _wall_mesh(offsets):
    # 5 cm vertex spacing, like the nvblox mesh.
    y, z = np.meshgrid(np.arange(-5, 5.001, .05), np.arange(.3, 1.801, .05))
    return np.vstack([np.c_[np.full(y.size, x), y.ravel(), z.ravel()] for x in offsets])


def test_one_wall_cannot_constrain_translation_along_it():
    rng = np.random.default_rng(1)
    result = run(_wall_mesh((1.,)), _wall_scan(rng, 3000, (1.,)),
                 _wall_scan(rng, 3000, (1.,)))
    assert not result.supported
    assert result.reason in ('WIDE_3D_SUPPORT', 'UNBOUNDED_3D_SUPPORT')


def test_a_straight_corridor_cannot_constrain_travel_along_it():
    rng = np.random.default_rng(2)
    result = run(_wall_mesh((1., -1.)), _wall_scan(rng, 2000, (1., -1.)),
                 _wall_scan(rng, 2000, (1., -1.)))
    assert not result.supported
    assert result.reason in ('WIDE_3D_SUPPORT', 'UNBOUNDED_3D_SUPPORT')


def test_temporally_inconsistent_winner_is_rejected():
    vertices = room()
    result = run(vertices, vertices[::2], vertices[1::2]-[6, 0, 0])
    assert not result.supported
    assert result.reason == 'INDEPENDENT_LEADER_DISAGREEMENT'


def test_symmetric_surfaces_do_not_nominate_unique_winner():
    vertices = room()
    result = run(np.vstack((vertices, vertices + [6, 0, 0])), vertices[::2], vertices[1::2])
    assert not result.supported and result.reason == 'TRAIN_LEADER_GAP_TOO_SMALL'


def test_missing_holdout_or_expired_budget_never_passes():
    vertices = room()
    result = run(vertices, vertices, np.zeros((0, 3)))
    assert not result.supported and result.reason == 'HOLDOUT_TOO_FEW_POINTS'
    assert run(vertices, vertices, vertices, deadline=0).reason == 'DEADLINE'


def _room_scan(rng, count=4000):
    """Scan the three room walls continuously (not on mesh vertices)."""
    a, z = rng.uniform(-1, 1, count), rng.uniform(.3, 1.8, count)
    side = rng.integers(0, 3, count)
    return np.where(side[:, None] == 0, np.c_[a, np.full(count, 2.), z],
                    np.where(side[:, None] == 1, np.c_[np.full(count, 1.), a, z],
                             np.c_[np.full(count, -1.), a, z]))


def test_refinement_converges_nearby_and_flags_a_limit_or_budget_stop():
    rng = np.random.default_rng(3)
    tree = cKDTree(room())
    points = _room_scan(rng)
    pose, converged = refine_surface_pose(tree, points, (.04, -.03, math.radians(2)),
                                          SurfaceRefineConfig(), lambda: False)
    assert math.hypot(pose[0], pose[1]) <= .015 and abs(pose[2]) <= math.radians(1)
    assert converged
    tight = SurfaceRefineConfig(max_shift_m=.05)
    _pose, converged = refine_surface_pose(tree, points, (.3, 0., 0.), tight, lambda: False)
    assert not converged                    # stopped on its limit, still climbing
    budget = SurfaceRefineConfig(max_evaluations=3)
    _pose, converged = refine_surface_pose(tree, points, (.3, 0., 0.), budget, lambda: False)
    assert not converged


def test_candidates_converging_to_one_solution_are_merged():
    rng = np.random.default_rng(7)
    result = validate_surface_evidence(
        room(), _room_scan(rng), _room_scan(rng),
        ((.03, .02, 0.), (6., 0., 0.), (-.03, .01, 0.)),
        deadline=time.monotonic() + 10, check_config=CONFIG,
        refine_config=SurfaceRefineConfig())
    assert result.origins == (0, 1)
    assert len(result.poses) == 2


def test_refinement_is_applied_to_every_candidate():
    rng = np.random.default_rng(4)
    vertices = room()
    result = validate_surface_evidence(
        vertices, _room_scan(rng), _room_scan(rng), ((.03, .02, 0.), (6., 0., 0.)),
        deadline=time.monotonic() + 10, check_config=CONFIG,
        refine_config=SurfaceRefineConfig())
    assert result.supported, result
    assert math.hypot(*result.poses[0][:2]) <= .015
    assert result.poses[1] != (6., 0., 0.) or result.train.composite[1] == 0


# Production split: ranking at 0.05 m without an absolute floor; fit at 0.10 m.
RANK = SurfaceCheckConfig(tolerance_m=.05, min_band_points=20, min_points=100,
                          min_leader_composite=.01)


class _Thresholds:
    min_score, min_coverage, min_known = .65, .65, 30


def _decide(monkeypatch, metrics):
    rng = np.random.default_rng(5)
    monkeypatch.setattr(validation, 'score_pose', lambda *args: dict(metrics))
    return decide_surface(room(), _room_scan(rng), _room_scan(rng), POSES, None, (), None,
                          type('C', (), {'refine_beams': 60})(), _Thresholds(), .35,
                          deadline=time.monotonic() + 10, check_config=RANK,
                          fit_config=SurfaceCheckConfig(min_band_points=20, min_points=100))


GOOD = {'score': .72, 'coverage': 1., 'known': 120, 'conflict': .29}


def test_decision_accepts_3d_evidence_with_a_sane_2d_fit(monkeypatch):
    decision = _decide(monkeypatch, GOOD)
    assert decision.accepted, decision
    assert math.hypot(*decision.pose[:2]) <= .015
    assert decision.metrics_2d['fit_3d'] >= .7


@pytest.mark.parametrize('change, reason', [
    ({'score': .6}, '2D_SCORE_TOO_LOW'),
    ({'coverage': .5}, '2D_COVERAGE_TOO_LOW'),
    ({'known': 10}, '2D_TOO_FEW_KNOWN'),
    ({'conflict': .4}, '2D_GROSS_CONFLICT'),
])
def test_decision_refuses_a_gross_2d_contradiction(monkeypatch, change, reason):
    decision = _decide(monkeypatch, {**GOOD, **change})
    assert not decision.accepted and decision.reason == reason


def test_decision_refuses_a_leader_with_a_poor_absolute_fit(monkeypatch):
    rng = np.random.default_rng(6)
    monkeypatch.setattr(validation, 'score_pose', lambda *args: dict(GOOD))
    # Half the scan is clutter the map does not contain.
    clutter = np.c_[rng.uniform(-.8, .8, 4000), rng.uniform(-.5, 1.5, 4000),
                    rng.uniform(.3, 1.8, 4000)]
    decision = decide_surface(
        room(), np.vstack((_room_scan(rng), clutter)), np.vstack((_room_scan(rng), clutter)),
        POSES, None, (), None, type('C', (), {'refine_beams': 60})(), _Thresholds(), .35,
        deadline=time.monotonic() + 10, check_config=RANK,
        fit_config=SurfaceCheckConfig(min_band_points=20, min_points=100))
    assert not decision.accepted
    assert decision.reason == 'LEADER_FIT_TOO_LOW'
