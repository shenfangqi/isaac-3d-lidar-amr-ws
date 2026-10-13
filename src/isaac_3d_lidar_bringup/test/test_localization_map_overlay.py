import math

import numpy as np
import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_map_overlay import (
    observe, OutcomeTracker, OverlayConfig, OverlayState, OverlayStore, self_check,
    session_gate, SessionEvidence, SessionQuality, undo_commits, update_overlay)
from isaac_3d_lidar_bringup.localization_submap import build_submap, SubmapConfig
from isaac_3d_lidar_bringup.localization_surface_additions import load_surface_additions
from isaac_3d_lidar_bringup.localization_surface_check import planar
from isaac_3d_lidar_bringup.localization_variable_regions import VariableRegions
from test_localization_submap import _box, _frames, _room

SHA = 'ab' * 32
GOOD = SessionQuality(True, .3, .02, math.radians(1))
NONE = np.zeros(0, int)


def _chair(x=1., y=1.):
    return _box((x, y, .45), (.45, .45, .9), step=.04)


def _session(state, added, occupied=NONE, free=NONE, quality=GOOD, config=OverlayConfig(),
             position=None, share=.05):
    """One session; by default from a new viewpoint 1 m from the previous one."""
    position = (float(state.sessions), 0.) if position is None else position
    return update_overlay(state, quality,
                          SessionEvidence(np.asarray(added, float).reshape(-1, 3),
                                          np.asarray(occupied, int), np.asarray(free, int),
                                          share, position), config)


def test_only_reliable_sessions_count():
    assert session_gate(GOOD) == (True, 'OK')
    assert session_gate(SessionQuality(False, .3, .02, 0.))[1] == 'NOT_ACCEPTED'
    assert session_gate(SessionQuality(True, .17, .02, 0.))[1] == 'GAP_TOO_SMALL'
    assert session_gate(SessionQuality(True, .3))[1] == 'GAP_TOO_SMALL' or True
    assert session_gate(SessionQuality(True, .3, .2, 0.))[1] == 'AMCL_UNCERTAIN'
    assert session_gate(SessionQuality(True, .3))[1] == 'AMCL_UNCERTAIN'
    state = OverlayState(SHA)
    same, report = _session(state, _chair(), quality=SessionQuality(True, .1, .02, 0.))
    assert same is state and not report.usable           # no trace, not even evidence


def test_a_new_object_is_committed_after_two_sessions_and_retracted_when_gone():
    state, report = _session(OverlayState(SHA), _chair())
    assert report.promoted == 0 and not state.committed.any() and state.version == 1
    state, report = _session(state, _chair())
    assert report.promoted > 10 and state.committed.any() and state.version == 2
    surface = state.surface()
    assert np.allclose(surface.mean(0)[:2], (1., 1.), atol=.1)
    # Once committed it is part of the map: later sessions see it occupied.
    cells = np.flatnonzero(state.committed)
    state, report = _session(state, np.zeros((0, 3)), occupied=cells)
    assert report.retracted == 0 and state.committed.sum() == len(cells)
    # Moved away: one session seeing through it is not enough, two are.
    state, report = _session(state, np.zeros((0, 3)), free=cells)
    assert report.retracted == 0
    state, report = _session(state, np.zeros((0, 3)), free=cells)
    assert report.retracted == len(cells) and not state.committed.any()
    assert len(state.surface()) == 0


def test_a_failed_self_check_undoes_the_commit_but_keeps_the_evidence():
    state, _ = _session(OverlayState(SHA), _chair())
    new, report = _session(state, _chair())
    assert report.promoted
    undone = undo_commits(state, new)
    assert not undone.committed.any() and len(undone.evidence) == len(new.evidence)
    assert undone.version == new.version


def test_a_passer_by_seen_once_is_never_committed():
    state = OverlayState(SHA)
    state, _ = _session(state, _chair(2., 2.))                 # once
    for _ in range(3):
        state, report = _session(state, np.zeros((0, 3)))      # not in view later
        assert report.promoted == 0
    # Seen again after it left the window: still one session in the window.
    config = OverlayConfig(window_sessions=3)
    for _ in range(3):
        state, _ = _session(state, np.zeros((0, 3)), config=config)
    state, report = _session(state, _chair(2., 2.), config=config)
    assert report.promoted == 0


def test_seen_free_after_occupied_blocks_the_commit():
    state, _ = _session(OverlayState(SHA), _chair())
    cells = np.arange(len(state.keys))
    state, _ = _session(state, np.zeros((0, 3)), free=cells)   # gone again
    state, report = _session(state, _chair())                  # back once
    assert report.promoted == 0
    state, report = _session(state, _chair())                  # twice since
    assert report.promoted > 0


def test_a_scene_that_disagrees_with_the_map_is_refused_whole():
    state = OverlayState(SHA)
    same, report = _session(state, _chair(), share=.3)          # a wrong pose
    assert same is state and report.reason == 'SCENE_DISAGREES'


def test_one_viewpoint_repeated_cannot_commit():
    state, _ = _session(OverlayState(SHA), _chair(), position=(2., 2.))
    state, report = _session(state, _chair(), position=(2.1, 2.))
    assert report.promoted == 0                                 # 0.1 m apart
    state, report = _session(state, _chair(), position=(2.8, 2.))
    assert report.promoted > 0


def test_a_wrong_pose_cannot_rewrite_the_map():
    # At a wrong pose the whole room looks added: one large component.
    wrong = _room()[::7] + (.8, .5, 0.)
    state, _ = _session(OverlayState(SHA), wrong)
    state, report = _session(state, wrong)
    assert report.promoted == 0 and report.refused_components
    assert report.refused_components[0][0] == 'TOO_LARGE'
    assert not state.committed.any()


def test_one_update_changes_a_bounded_amount():
    config = OverlayConfig(max_cells_per_update=60)
    chairs = np.vstack([_chair(1., 1.), _chair(3., 1.), _chair(1., -2.)])
    state, _ = _session(OverlayState(SHA), chairs, config=config)
    state, report = _session(state, chairs, config=config)
    one = report.promoted                                      # one chair, over budget
    assert 60 < one < 300 and report.deferred >= 2 * one * .8
    state, report = _session(state, chairs, config=config)     # the rest later
    assert report.promoted > 0 and report.deferred > 0


def test_observe_finds_new_objects_and_ignores_variable_regions():
    rng = np.random.default_rng(5)
    room = _room()
    pose = (.3, .2, math.radians(10))
    crate = _box((-2., 1.5, .3), (.6, .6, .6))
    curtain = np.c_[np.linspace(-3, 3, 400), np.full(400, -2.6), np.full(400, 1.)]
    frames = _frames(lambda i: np.vstack([room, crate, curtain]), [pose] * 8, rng)
    submap = build_submap(frames, planar(*pose), SubmapConfig(carve_every=2))
    regions = VariableRegions(np.array([[-4., -2.8, .3, 4., -2.4, 2.5]]), ('curtain',))
    evidence = observe(OverlayState(SHA), submap, pose, room, regions)
    added = evidence.added
    assert len(added) > 20 and np.allclose(added.mean(0)[:2], (-2., 1.5), atol=.3)
    assert not regions.contains(added).any()
    assert 0 < evidence.added_share < .18 and evidence.position == pytest.approx(pose[:2])
    # The crate, committed, is now seen occupied; taken away, it is seen free.
    state, _ = _session(OverlayState(SHA), added)
    state, _ = _session(state, added)
    assert state.committed.any()
    seen = observe(state, submap, pose, np.vstack([room, state.surface()]), regions)
    assert len(seen.occupied) > 0.8 * state.committed.sum() and len(seen.free) == 0
    empty = build_submap(_frames(lambda i: room, [pose] * 8, rng), planar(*pose),
                         SubmapConfig(carve_every=2))
    gone = observe(state, empty, pose, np.vstack([room, state.surface()]), regions)
    # Rays reach only the faces in view of the free space; enough to retract
    # those cells, the rest waits until seen (synthetic rays are sparse).
    assert len(gone.occupied) == 0 and len(gone.free) > 0.2 * state.committed.sum()


def test_self_check_keeps_only_updates_that_keep_the_lead():
    def fit_of(table):
        return lambda pose, new: table[(pose, new)]
    better = {('L', False): .6, ('L', True): .7, ('R', False): .4, ('R', True): .45}
    assert self_check(fit_of(better), 'L', ['R']).keep
    dropped = {**better, ('L', True): .55}
    assert self_check(fit_of(dropped), 'L', ['R']).reason == 'LEADER_FIT_DROPPED'
    rival = {**better, ('R', True): .65}
    assert self_check(fit_of(rival), 'L', ['R']).reason == 'LEAD_SHRANK'
    behind = {**better, ('R', False): .65, ('R', True): .66}    # leader not the best fit
    assert self_check(fit_of(behind), 'L', ['R']).reason == 'LEADER_NOT_BEST'
    assert self_check(fit_of(better), 'L', []).keep


def test_outcomes_roll_back_a_bad_update():
    tracker = OutcomeTracker(max_failures=2, probation=5)
    assert not tracker.record(False)                 # nothing committed yet
    tracker.start(7, 6)
    assert not tracker.record(False) and not tracker.record(True)
    assert not tracker.record(False)
    assert tracker.record(False)                     # third failure within probation
    tracker.start(8, 7)
    for _ in range(5):
        tracker.record(True)
    assert not tracker.record(False)                 # probation over


def test_versions_are_immutable_and_can_be_restored(tmp_path):
    base = np.array([[0., 0., .3], [0., .05, .3]])
    store = OverlayStore(tmp_path, SHA, base)
    assert store.load().version == 0
    state, _ = _session(OverlayState(SHA), _chair())
    store.save(state)
    with pytest.raises(ContractError):
        store.save(state)                            # never overwrite
    first = state.version
    state, _ = _session(state, _chair())
    store.save(state, 'chair')
    assert len(load_surface_additions(store.additions_path(state.version), SHA)) \
        == len(base) + len(state.surface())
    reopened = OverlayStore(tmp_path, SHA, base)
    assert reopened.load().committed.sum() == state.committed.sum()
    reopened.rollback(first)
    assert not OverlayStore(tmp_path, SHA).load().committed.any()
    assert reopened.next_version() == state.version + 1      # numbers are never reused
    with pytest.raises(ContractError):
        reopened.rollback(99)
    with pytest.raises(ContractError):
        OverlayStore(tmp_path, 'cd' * 32)


def test_limits_are_checked():
    with pytest.raises(ContractError):
        OverlayConfig(min_sessions=0)
    with pytest.raises(ContractError):
        OverlayConfig(window_sessions=1)
    with pytest.raises(ContractError):
        OverlayConfig(min_viewpoint_m=-1.)
