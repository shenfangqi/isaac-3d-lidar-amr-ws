"""Analytic before/after range observations, independent of occupancy scoring."""
from dataclasses import replace
import time

import pytest

from isaac_3d_lidar_bringup.localization_active_probe import infer_after_translation
from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError, Hypothesis, SearchResult, SE2,
)
from isaac_3d_lidar_bringup.localization_hypotheses import (
    map_hash, validate_hypotheses, ValidationThresholds,
)
from test_localization_hypotheses import _rectangle, _grid, _frames, FAST, IDENTITY


def scene(symmetric=False):
    walls = _rectangle(.1, .1, 3, 2.1) + _rectangle(5.1, .1, 8, 2.1)
    walls += [((2.4, .5), (2.4, 1.7))]
    if symmetric:
        walls += [((7.4, .5), (7.4, 1.7))]
    grid = _grid(walls, 8.2, 2.3)
    first, second = SE2(1, 1.1, 0), SE2(1.6, 1.1, 0)
    before_train = _frames(walls, first, 'TRAIN', start_id=0, range_max=1.1)
    before_hold = _frames(walls, first, 'HOLDOUT', start_id=10, range_max=1.1)
    after_train = _frames(walls, second, 'TRAIN', start_id=20, view_id=1,
                          odom=SE2(.6, 0, 0), range_max=1.1)
    after_hold = _frames(walls, second, 'HOLDOUT', start_id=30, view_id=1,
                         odom=SE2(.6, 0, 0), range_max=1.1)
    # Deliberately put the incorrect initial candidate first.
    candidates = tuple(Hypothesis(x, 1.1, 0, .9, 1, 0, i, (), ()) for i, x in enumerate((6, 1)))
    previous = SearchResult('s1', map_hash(grid), True, candidates, 2, 0., '')
    return grid, previous, (before_train, before_hold, after_train, after_hold)


def infer(grid, previous, groups, **kwargs):
    return infer_after_translation(grid, previous, IDENTITY, *groups, SE2(.6, 0, 0),
                                   FAST, ValidationThresholds(),
                                   odom_continuous=True, deadline=time.monotonic() + 20, **kwargs)


def test_translation_resolves_ambiguity_and_outputs_current_pose():
    grid, previous, groups = scene()
    initial = validate_hypotheses(grid, previous, *groups[:2], FAST, ValidationThresholds())
    assert not initial.accepted and initial.reason == 'AMBIGUOUS_LOCATION'
    outcome = infer(grid, previous, groups)
    assert outcome.decision.accepted, outcome.decision
    assert outcome.current_pose.x == pytest.approx(1.6, abs=.12)
    assert outcome.current_pose.y == pytest.approx(1.1, abs=.12)
    assert not outcome.full_search_used


def test_translation_cannot_break_true_symmetry():
    grid, previous, groups = scene(symmetric=True)
    outcome = infer(grid, previous, groups)
    assert not outcome.decision.accepted
    assert outcome.current_pose is None


@pytest.mark.parametrize('bad', ['reuse', 'missing', 'session', 'stationary', 'order'])
def test_inference_rejects_invalid_evidence(bad):
    grid, previous, groups = scene()
    groups = list(groups)
    if bad == 'reuse':
        groups[1] = groups[0]
    elif bad == 'missing':
        groups[3] = ()
    elif bad == 'session':
        groups[3] = tuple(replace(f, session='another') for f in groups[3])
    elif bad == 'stationary':
        groups[2:] = [tuple(replace(f, T_odom_base=IDENTITY) for f in g) for g in groups[2:]]
    else:
        groups[2] = tuple(replace(f, stamp_ns=f.stamp_ns - 100) for f in groups[2])
    with pytest.raises(ContractError):
        infer(grid, previous, groups)


def test_expired_deadline_never_accepts():
    grid, previous, groups = scene()
    outcome = infer_after_translation(grid, previous, IDENTITY, *groups, SE2(.6, 0, 0),
                                      FAST, ValidationThresholds(),
                                      odom_continuous=True, deadline=0.)
    assert not outcome.decision.accepted and outcome.current_pose is None


def test_discontinuous_odometry_never_combines_views():
    grid, previous, groups = scene()
    with pytest.raises(ContractError):
        infer_after_translation(grid, previous, IDENTITY, *groups, SE2(.6, 0, 0),
                                FAST, ValidationThresholds(),
                                odom_continuous=False, deadline=time.monotonic() + 1)


def test_refuted_candidates_trigger_full_search_with_both_views(monkeypatch):
    from isaac_3d_lidar_bringup import localization_active_probe as module
    from isaac_3d_lidar_bringup.localization_hypotheses import QualityDecision
    grid, previous, groups = scene()
    calls = []

    def validate(grid, result, train, holdout, *args):
        calls.append((tuple(f.id for f in train), tuple(f.id for f in holdout)))
        return QualityDecision(False, 'NO_VALID_CANDIDATE')

    def search(grid, train, *args):
        assert train == groups[0] + groups[2]
        return previous
    monkeypatch.setattr(module, 'validate_hypotheses', validate)
    monkeypatch.setattr(module, 'search_multiview', search)
    outcome = infer(grid, previous, groups)
    assert outcome.full_search_used and len(calls) == 2
    assert calls[0] == calls[1]
    assert calls[0][1] == tuple(f.id for f in groups[1] + groups[3])
    assert outcome.current_pose is None


def test_one_good_view_cannot_average_away_a_conflict(monkeypatch):
    from isaac_3d_lidar_bringup import localization_active_probe as module
    grid, previous, groups = scene()
    original = module.score_pose

    def score(grid, pose, frames, *args):
        metrics = original(grid, pose, frames, *args)
        if frames == groups[3]:
            metrics['conflict'] = .4
        return metrics
    monkeypatch.setattr(module, 'score_pose', score)
    monkeypatch.setattr(module, 'search_multiview', lambda *args: previous)
    outcome = infer(grid, previous, groups)
    assert not outcome.decision.accepted
    assert outcome.full_search_used and outcome.current_pose is None
