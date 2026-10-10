"""Independent observations, geometric observability, and bounded work."""
import math
import time

import numpy as np
import pytest
from scipy.spatial import cKDTree

from isaac_3d_lidar_bringup import localization_surface_validation as validation
from isaac_3d_lidar_bringup.localization_surface_check import SurfaceCheckConfig, SurfaceResult
from isaac_3d_lidar_bringup.localization_surface_search import SurfaceSearchResult
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


def _decide_with_search(monkeypatch, poses, found):
    rng = np.random.default_rng(5)
    monkeypatch.setattr(validation, 'score_pose', lambda *args: dict(GOOD))
    monkeypatch.setattr(validation, 'search_surface', lambda *args: found)
    return decide_surface(room(), _room_scan(rng), _room_scan(rng), poses, None, (), None,
                          type('C', (), {'refine_beams': 60})(), _Thresholds(), .35,
                          deadline=time.monotonic() + 10, check_config=RANK,
                          fit_config=SurfaceCheckConfig(min_band_points=20, min_points=100),
                          search=('field', 'positions', 'config'))


def test_the_3d_search_adds_a_place_the_2d_candidates_missed(monkeypatch):
    missed = ((6., 0., 0.), (9., 0., 0.))           # the true place is not among them
    found = SurfaceSearchResult(True, '', ((.02, .01, 0., .6),), .6, 100)
    decision = _decide_with_search(monkeypatch, missed, found)
    assert decision.accepted, decision
    assert math.hypot(*decision.pose[:2]) <= .015
    assert decision.validation.origins[decision.leader] == 2      # the search's seed


def test_an_incomplete_3d_search_refuses(monkeypatch):
    found = SurfaceSearchResult(False, 'TOO_MANY_CANDIDATES', (), .6, 100)
    decision = _decide_with_search(monkeypatch, POSES, found)
    assert not decision.accepted
    assert decision.reason == '3D_SEARCH_TOO_MANY_CANDIDATES'


# --- See-through evidence (stage B) -------------------------------------------

def _wall(x0, y0, x1, y1, z0=0.0, z1=2.0, step=.04):
    length = math.hypot(x1 - x0, y1 - y0)
    s, z = np.meshgrid(np.linspace(0, 1, max(2, int(length / step))),
                       np.arange(z0, z1 + 1e-9, step))
    return np.c_[x0 + s.ravel() * (x1 - x0), y0 + s.ravel() * (y1 - y0), z.ravel()]


CONFLICT = validation.SurfaceConflictConfig()


def test_rays_through_mapped_structure_count_and_added_objects_do_not():
    occupancy = validation.surface_occupancy(_wall(3, -2, 3, 2), CONFLICT)
    origin = (0., 0., .2)
    beyond = np.c_[np.full(50, 6.0), np.linspace(-1, 1, 50), np.full(50, 1.0)]
    added = np.c_[np.full(50, 1.5), np.linspace(-1, 1, 50), np.full(50, 1.0)]
    on_wall = np.c_[np.full(50, 3.0), np.linspace(-1, 1, 50), np.full(50, 1.0)]
    assert validation.see_through_ratio(occupancy, beyond, origin, (0, 0, 0), CONFLICT) == 1.
    # A new object shortens rays: never a contradiction.
    assert validation.see_through_ratio(occupancy, added, origin, (0, 0, 0), CONFLICT) == 0.
    assert validation.see_through_ratio(occupancy, on_wall, origin, (0, 0, 0), CONFLICT) == 0.
    # The same rays placed where the wall is not in their way.
    assert validation.see_through_ratio(occupancy, beyond, origin, (0, 10, 0), CONFLICT) == 0.


def test_floor_and_ceiling_grazing_is_not_a_contradiction():
    floor = np.array([(x, y, 0.) for x in np.arange(0, 8, .04) for y in np.arange(-1, 1, .04)])
    occupancy = validation.surface_occupancy(np.r_[floor, floor + (0, 0, 2.3)], CONFLICT)
    low = np.c_[np.linspace(2, 7, 60), np.zeros(60), np.full(60, .15)]
    high = np.c_[np.linspace(2, 7, 60), np.zeros(60), np.full(60, 2.25)]
    for points in (low, high):
        ratio = validation.see_through_ratio(occupancy, points, (0, 0, .2), (0, 0, 0), CONFLICT)
        assert ratio == 0.


def test_rerank_excludes_contradicted_candidates():
    check = SurfaceCheckConfig(min_composite_gap=.15, min_leader_composite=.5)
    base = SurfaceResult(False, 'LEADER_GAP_TOO_SMALL', 0, (.70, .68, .40), .02, (), (), 0)
    assert validation._rerank(base, [0, 2], check).resolved
    assert validation._rerank(base, [0, 2], check).composite_gap == pytest.approx(.30)
    single = validation._rerank(base, [1], check)
    assert single.resolved and single.leader == 1
    assert validation._rerank(base, [], check).reason == 'ALL_CANDIDATES_SEE_THROUGH'


def test_a_twin_room_whose_rays_cross_a_closed_wall_is_excluded():
    # Rooms A (y 0-4) and B (y 10-14) look alike from inside; A's right wall
    # has a door through which the scan sees a far wall at x = 8, B's right
    # wall is closed.  The inlier share cannot tell them apart; the rays
    # through B's closed wall can.
    def room(y0, door):
        right = ([_wall(4, y0, 4, y0 + 1), _wall(4, y0 + 3, 4, y0 + 4)] if door
                 else [_wall(4, y0, 4, y0 + 4)])
        return [_wall(0, y0, 0, y0 + 4), _wall(0, y0, 4, y0), _wall(0, y0 + 4, 4, y0 + 4),
                *right]
    surface = np.concatenate(room(0, True) + room(10, False) + [_wall(8, -1, 8, 15)])
    rng = np.random.default_rng(2)
    walls = np.concatenate(room(0, True))
    walls = walls[rng.choice(len(walls), 9000, replace=False)]
    far = _wall(8, 1.3, 8, 2.7)                # seen through the door
    seen = np.r_[walls, far] + rng.normal(0, .005, (len(walls) + len(far), 3))
    scan = seen - (2, 2, 0)                    # robot at (2, 2, 0) facing +x
    keep = np.hypot(scan[:, 0], scan[:, 1]) > .6
    scan = scan[keep]
    train, held = scan[::2], scan[1::2]
    poses = [(2., 2., 0.), (2., 12., 0.)]
    check = SurfaceCheckConfig(tolerance_m=.05, min_points=1000, min_band_points=100,
                               min_leader_composite=.01)
    blind = validation.validate_surface_evidence(
        surface, train, held, poses, deadline=time.monotonic() + 60, check_config=check)
    assert blind.reason == 'TRAIN_LEADER_GAP_TOO_SMALL'
    seeing = validation.validate_surface_evidence(
        surface, train, held, poses, deadline=time.monotonic() + 60, check_config=check,
        sensor_origin=(0., 0., .2))
    assert seeing.see_through[0][0] < .02 and seeing.see_through[0][1] > .08
    assert seeing.leader == 0
    assert seeing.train.resolved and seeing.holdout.resolved


def test_candidates_converging_to_one_place_are_unique_not_refused():
    # Two starts a few cm apart refine to the same pose and merge: one
    # distinct place, nothing competes; support and fit still decide.
    rng = np.random.default_rng(9)
    scan = _room_scan(rng)
    result = validation.validate_surface_evidence(
        room(), scan, _room_scan(rng), ((0.02, 0.01, 0.), (-0.02, 0., 0.01)),
        deadline=time.monotonic() + 30, check_config=RANK,
        refine_config=SurfaceRefineConfig())
    assert len(result.poses) == 1
    assert result.supported, result.reason
    assert result.train.composite_gap == pytest.approx(result.train.composite[0])


def _fence(period=.25, length=6.):
    """Two rows of posts, one period apart: shifting by a period changes nothing."""
    posts = []
    for x in np.arange(-length / 2, length / 2 + 1e-9, period):
        for y in (-1., 1.):
            a, z = np.meshgrid(np.linspace(-.04, .04, 5), np.arange(.3, 1.8, .04))
            posts += [np.c_[x + a.ravel(), np.full(a.size, y), z.ravel()],
                      np.c_[np.full(a.size, x), y + a.ravel(), z.ravel()]]
    return np.vstack(posts)


def test_a_disconnected_nearby_optimum_is_a_competitor():
    # One candidate converges to one of two optima a period apart.  The gap
    # test has nobody to compare it with and the support check measures only
    # the connected dip around it, so without the nearby search it is taken.
    rng = np.random.default_rng(4)
    fence = _fence()
    scan = fence[np.abs(fence[:, 0]) < 1.5]
    scan = scan + rng.normal(0, .005, scan.shape)
    train, held = scan[::2], scan[1::2]
    check = SurfaceCheckConfig(tolerance_m=.05, min_points=500, min_band_points=50,
                               min_leader_composite=.01)
    kwargs = dict(deadline=time.monotonic() + 60, check_config=check,
                  refine_config=SurfaceRefineConfig())
    blind = validation.validate_surface_evidence(fence, train, held, ((.01, 0., 0.),),
                                                 nearby_config=None, **kwargs)
    assert blind.supported, blind.reason       # the hazard: accepted
    seeing = validation.validate_surface_evidence(fence, train, held, ((.01, 0., 0.),), **kwargs)
    assert seeing.reason == 'NEARBY_COMPETITOR'


def test_a_unique_room_has_no_nearby_competitor():
    rng = np.random.default_rng(9)
    result = validation.validate_surface_evidence(
        room(), _room_scan(rng), _room_scan(rng), ((0.02, 0.01, 0.),),
        deadline=time.monotonic() + 30, check_config=RANK,
        refine_config=SurfaceRefineConfig())
    assert result.supported, result.reason


def test_a_candidate_starts_refinement_from_its_best_local_peak():
    # A coarse node 0.2 m and 6 deg off: the sharp-field seed lands next to
    # the true pose before the exact refinement starts.
    rng = np.random.default_rng(11)
    scan = _room_scan(rng)
    tree = validation.surface_tree(room())
    seed = validation.best_local_seed(tree, scan, (0.2, -0.15, math.radians(6)),
                                      validation.SurfaceNearbyConfig())
    assert math.hypot(seed[0], seed[1]) <= .06
    assert abs(seed[2]) <= math.radians(3)
