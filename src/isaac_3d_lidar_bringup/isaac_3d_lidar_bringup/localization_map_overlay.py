"""
Automatic map overlay: static structure learned from reliable sessions (#54).

Pure logic, no ROS.  The saved mesh never changes.  Structure the scene has
and the map lacks (a moved chair, a bed the mapping run missed) is kept in
an overlay of 10 cm cells, each holding the surface points last observed in
it.  Committed cells join the 3D fit like surface additions: fitted, never
solid (``localization_surface_additions``); exported they *are* a
surface-additions file, so the runtime loads an overlay version unchanged.

Safety comes from constraints on the update itself, not from validating
every update offline (issue #54):

1. Only reliable sessions count (``session_gate``): an accepted 3D decision
   with at least ``min_gap_3d`` and an AMCL verification within limits.  A
   refused session leaves no trace, not even candidate evidence.
   A session whose submap disagrees with the map too much
   (``max_added_share``) is refused as a whole: that is what a wrong pose
   looks like (the reliability gate alone does not see it).
2. A cell is committed only after ``min_sessions`` distinct sessions saw it
   occupied within the last ``window_sessions`` reliable sessions, from
   viewpoints at least ``min_viewpoint_m`` apart, and none of them saw it
   free since; a committed cell is retracted after
   ``retract_sessions`` sessions saw it free (rays through it) and none saw it
   occupied since.  A cell not in view gets no evidence.
3. Variable regions are never learned: they already describe change.
4. One update commits whole components: none larger than ``max_extent_m``
   (a wrong pose makes the whole scene look added; refused), and beyond the
   first one at most ``max_cells_per_update`` cells in total (the rest wait
   for the next session).
5. ``self_check``: with the new overlay the session's leader must stay the
   best fit and keep its fit and its lead over the rivals, else the update
   is not kept.  It cannot catch a wrong leader (the overlay is made from
   that leader's own view and only helps it); 1, 2 and 4 must.
6. ``OutcomeTracker``: too many failed localizations after an update roll
   back to the version before it.

Versions are immutable; ``OverlayStore`` (files) keeps every one with its
parent, so any version can be restored.
"""

from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path

import numpy as np

from .localization_contracts import ContractError
from .localization_submap import compare_submap, CompareConfig, components
from .localization_surface_additions import save_surface_additions
from .localization_surface_validation import _voxel_keys


@dataclass(frozen=True)
class OverlayConfig:
    """Update limits.  Provisional: from the 2026-10-09..12 home captures."""

    cell_m: float = .1
    min_gap_3d: float = .2
    max_amcl_xy_std_m: float = .05
    max_amcl_yaw_std_rad: float = math.radians(2)
    min_sessions: int = 2
    retract_sessions: int = 2
    window_sessions: int = 8
    min_component_cells: int = 4
    max_extent_m: float = 2.5
    max_cells_per_update: int = 400
    max_points_per_cell: int = 40
    hit_m: float = .1
    # Session-level: at the true poses of 2026-10-09..12 at most 13% of the
    # in-band submap voxels were added (mesh and regions only), at the
    # 2D candidates beyond 1.2 m at least 22% (14 sessions, 40 wrong poses).
    max_added_share: float = .18
    # A cell must be seen occupied from viewpoints this far apart, so the
    # same wrong pose repeated cannot commit what it sees.
    min_viewpoint_m: float = .5

    def __post_init__(self):
        values = (self.cell_m, self.min_gap_3d, self.max_amcl_xy_std_m,
                  self.max_amcl_yaw_std_rad, self.max_extent_m, self.hit_m,
                  self.max_added_share)
        counts = (self.min_sessions, self.retract_sessions, self.window_sessions,
                  self.min_component_cells, self.max_cells_per_update,
                  self.max_points_per_cell)
        if (any(not math.isfinite(v) or v <= 0 for v in values)
                or any(type(v) is not int or v < 1 for v in counts)
                or not math.isfinite(self.min_viewpoint_m) or self.min_viewpoint_m < 0
                or self.window_sessions < max(self.min_sessions, self.retract_sessions)):
            raise ContractError('invalid overlay limits')


@dataclass(frozen=True)
class SessionQuality:
    """What the localization of one session established (runtime or replay)."""

    accepted: bool
    gap_3d: float = float('nan')
    amcl_xy_std_m: float = float('nan')
    amcl_yaw_std_rad: float = float('nan')


def session_gate(quality, config=OverlayConfig()):
    """Return ``(usable, reason)``: whether this session may update the overlay."""
    if not quality.accepted:
        return False, 'NOT_ACCEPTED'
    if not (quality.gap_3d >= config.min_gap_3d):
        return False, 'GAP_TOO_SMALL'
    if not (quality.amcl_xy_std_m <= config.max_amcl_xy_std_m
            and quality.amcl_yaw_std_rad <= config.max_amcl_yaw_std_rad):
        return False, 'AMCL_UNCERTAIN'
    return True, 'OK'


@dataclass(frozen=True)
class OverlayState:
    """
    One overlay version.

    ``keys`` are the tracked cells (candidates and committed), ``committed``
    their state and ``points``/``point_cell`` the surface points per cell.
    ``evidence`` rows are ``(cell index, session, kind)`` with kind 1 for
    occupied and -1 for free; ``sessions`` counts reliable sessions so far.
    Every reliable session makes a new version (its evidence changes even
    when the committed cells do not); ``parent`` is the version before.
    """

    mesh_sha256: str
    version: int = 0
    parent: int = -1
    sessions: int = 0
    keys: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int64))
    committed: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    point_cell: np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    evidence: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), int))
    # Robot map position of every reliable session, row ``session - 1``.
    positions: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))

    def surface(self):
        """Return the committed overlay points (map frame), fitted like surface additions."""
        return self.points[self.committed[self.point_cell]] if len(self.points) \
            else np.zeros((0, 3))


@dataclass(frozen=True)
class UpdateReport:
    usable: bool
    reason: str
    observed_cells: int = 0
    free_cells: int = 0
    promoted: int = 0
    retracted: int = 0
    deferred: int = 0
    refused_components: tuple = ()


def _member(sorted_keys, keys):
    if not len(sorted_keys) or not len(keys):
        return np.full(len(keys), -1)
    slot = np.minimum(np.searchsorted(sorted_keys, keys), len(sorted_keys) - 1)
    return np.where(sorted_keys[slot] == keys, slot, -1)


def _cell_keys(points, config):
    return _voxel_keys(np.asarray(points, float).reshape(-1, 3), config.cell_m)


@dataclass(frozen=True)
class SessionEvidence:
    """
    What one session saw at its decided map pose.

    ``added`` are submap points far from the map as localization uses it
    and outside the variable regions, ``added_share`` their share of the
    in-band submap voxels; ``occupied``/``free`` are tracked cells (indexes
    into ``OverlayState.keys``) seen occupied or seen through; ``position``
    is the robot's map position.
    """

    added: np.ndarray
    occupied: np.ndarray
    free: np.ndarray
    added_share: float
    position: tuple


def observe(state, submap, pose, surface, regions=None, config=OverlayConfig(),
            compare=CompareConfig()):
    """
    Return the :class:`SessionEvidence` of one session at map ``pose``.

    ``surface`` is the map as localization uses it: the mesh (or its tree
    with the variable regions) plus the current additions and overlay.
    """
    result = compare_submap(submap, pose, surface, compare)
    outside = (np.ones(len(result.world), bool) if regions is None
               else ~regions.contains(result.world))
    band = ((result.world[:, 2] >= compare.min_z_m) & (result.world[:, 2] <= compare.max_z_m)
            & outside)
    added = result.world[result.added & outside]
    share = float(len(added) / band.sum()) if band.any() else 1.
    position = (float(pose[0]), float(pose[1]))
    if not len(state.keys):
        return SessionEvidence(added, np.zeros(0, int), np.zeros(0, int), share, position)
    from scipy.spatial import cKDTree
    distance = (cKDTree(result.world).query(state.points, distance_upper_bound=config.hit_m)[0]
                if len(result.world) else np.full(len(state.points), np.inf))
    occupied = np.unique(state.point_cell[np.isfinite(distance)])
    tracked = compare_submap(submap, pose, state.points, replace(compare, hit_m=config.hit_m))
    free = _member(state.keys, _cell_keys(tracked.removed, config))
    free = np.setdiff1d(np.unique(free[free >= 0]), occupied)
    return SessionEvidence(added, occupied, free, share, position)


def _recent(state, config):
    """
    Count the recent evidence of every tracked cell.

    Over the newest run of one kind within the window: occupied and free
    session counts and the spread of the occupied viewpoints (largest
    distance between their robot positions).
    """
    occupied = np.zeros(len(state.keys), int)
    free = np.zeros(len(state.keys), int)
    spread = np.zeros(len(state.keys))
    rows = state.evidence[state.evidence[:, 1] > state.sessions - config.window_sessions]
    newest, stopped, seen_from = {}, set(), {}
    for cell, session, kind in rows[np.argsort(-rows[:, 1], kind='stable')]:
        if cell in stopped:
            continue
        if newest.setdefault(cell, kind) != kind:
            stopped.add(cell)
            continue
        if kind > 0:
            occupied[cell] += 1
            seen_from.setdefault(cell, []).append(state.positions[session - 1])
        else:
            free[cell] += 1
    for cell, positions in seen_from.items():
        positions = np.asarray(positions)
        spread[cell] = float(np.max(np.linalg.norm(
            positions[:, None, :] - positions[None, :, :], axis=2)))
    return occupied, free, spread


def update_overlay(state, quality, evidence, config=OverlayConfig()):
    """
    Return ``(new_state, UpdateReport)`` after one session.

    An unusable session returns ``state`` unchanged.  Otherwise its evidence
    is recorded and cells are promoted or retracted under the limits; the
    result is the next version, whose parent is ``state``.
    """
    usable, reason = session_gate(quality, config)
    if usable and not evidence.added_share <= config.max_added_share:
        usable, reason = False, 'SCENE_DISAGREES'
    if not usable:
        return state, UpdateReport(False, reason)
    session = state.sessions + 1
    added = np.asarray(evidence.added, float).reshape(-1, 3)
    occupied, free = evidence.occupied, evidence.free

    # New and refreshed cells from this session's added points.
    added_keys = _cell_keys(added, config)
    keys = np.union1d(state.keys, added_keys)
    remap = np.searchsorted(keys, state.keys)
    committed = np.zeros(len(keys), bool)
    committed[remap] = state.committed
    point_cell = remap[state.point_cell] if len(state.points) else np.zeros(0, int)
    rows = state.evidence.copy()
    if len(rows):
        rows[:, 0] = remap[rows[:, 0]]
    seen = np.unique(np.searchsorted(keys, added_keys))
    occupied = np.union1d(remap[np.asarray(occupied, int)], seen)
    free = np.setdiff1d(remap[np.asarray(free, int)], occupied)
    # Replace the points of cells observed occupied now by the new ones.
    keep = ~np.isin(point_cell, seen)
    cell_of_added = np.searchsorted(keys, added_keys)
    fresh = []
    for cell in seen:
        part = added[cell_of_added == cell]
        if len(part) > config.max_points_per_cell:
            part = part[np.linspace(0, len(part) - 1, config.max_points_per_cell).astype(int)]
        fresh.append((part, np.full(len(part), cell)))
    points = np.vstack([state.points[keep]] + [p for p, _ in fresh])
    point_cell = np.concatenate([point_cell[keep]] + [c for _, c in fresh]).astype(int)
    rows = np.vstack([rows,
                      np.c_[occupied, np.full(len(occupied), session),
                            np.ones(len(occupied), int)],
                      np.c_[free, np.full(len(free), session),
                            -np.ones(len(free), int)]]).astype(int)
    positions = np.vstack([state.positions.reshape(-1, 2), [evidence.position]])
    draft = OverlayState(state.mesh_sha256, state.version, state.parent, session, keys,
                         committed, points, point_cell, rows, positions)

    recent_occupied, recent_free, spread = _recent(draft, config)
    promote = ((~committed) & (recent_occupied >= config.min_sessions) & (recent_free == 0)
               & (spread >= config.min_viewpoint_m))
    retract = committed & (recent_free >= config.retract_sessions) & (recent_occupied == 0)

    # Limits: whole components only, none too large, a bounded total.
    centres = _centres(keys[promote], config)
    labels = components(centres, config.cell_m * 1.5) if len(centres) else np.zeros(0, int)
    chosen, refused, budget, deferred = [], [], config.max_cells_per_update, 0
    order = sorted(np.unique(labels), key=lambda label: -(labels == label).sum())
    for label in order:
        part = np.flatnonzero(promote)[labels == label]
        extent = np.ptp(centres[labels == label][:, :2], axis=0).max() + config.cell_m
        if len(part) < config.min_component_cells:
            continue
        if extent > config.max_extent_m:
            refused.append(('TOO_LARGE', int(len(part)), round(float(extent), 2)))
            continue
        # The first component within the extent limit always fits, so an
        # object larger than the budget is not deferred forever.
        if len(part) > budget and chosen:
            deferred += len(part)
            continue
        chosen.append(part)
        budget -= len(part)
    chosen = np.concatenate(chosen) if chosen else np.zeros(0, int)
    retracted = np.flatnonzero(retract)[:max(budget, 0)]
    committed = committed.copy()
    committed[chosen] = True
    committed[retracted] = False
    new = replace(draft, committed=committed, version=state.version + 1,
                  parent=state.version)
    return new, UpdateReport(True, 'OK', len(occupied), len(free), int(len(chosen)),
                             int(len(retracted)), int(deferred), tuple(refused))


def undo_commits(old, new):
    """Return ``new`` with the committed cells of ``old`` (failed self-check); keep evidence."""
    committed = np.zeros(len(new.keys), bool)
    index = _member(new.keys, old.keys[old.committed])
    committed[index[index >= 0]] = True
    return replace(new, committed=committed)


def _centres(keys, config):
    mask = (1 << 21) - 1
    cells = np.stack([(keys >> 42) & mask, (keys >> 21) & mask, keys & mask], 1)
    return (cells - (1 << 20) + .5) * config.cell_m


@dataclass(frozen=True)
class SelfCheck:
    keep: bool
    reason: str
    leader_fit: tuple
    lead: tuple


def self_check(fit, leader, rivals, tolerance=.005):
    """
    Keep an update only if the session's leader keeps its fit and its lead.

    ``fit(pose, new)`` scores the session cloud at ``pose`` against the old
    (``new=False``) or new map; callers refine locally inside it.  ``rivals``
    are the other candidates of the session's own decision.
    """
    old_leader, new_leader = fit(leader, False), fit(leader, True)
    old_rival = max((fit(r, False) for r in rivals), default=0.)
    new_rival = max((fit(r, True) for r in rivals), default=0.)
    old_lead, new_lead = old_leader - old_rival, new_leader - new_rival
    pair = (round(old_leader, 4), round(new_leader, 4))
    leads = (round(old_lead, 4), round(new_lead, 4))
    if old_lead <= 0 or new_lead <= 0:
        return SelfCheck(False, 'LEADER_NOT_BEST', pair, leads)
    if new_leader < old_leader - tolerance:
        return SelfCheck(False, 'LEADER_FIT_DROPPED', pair, leads)
    if new_lead < old_lead - tolerance:
        return SelfCheck(False, 'LEAD_SHRANK', pair, leads)
    return SelfCheck(True, 'OK', pair, leads)


@dataclass
class OutcomeTracker:
    """
    Roll back an update that made localization fail.

    After an update commits cells, at most ``max_failures`` of the first
    ``probation`` localizations may fail (refused or AMCL verification
    failed); one more and ``record`` returns True: restore ``parent``, the
    version before the update.
    """

    max_failures: int = 2
    probation: int = 10
    version: int = 0
    parent: int = -1
    attempts: int = 0
    failures: int = 0

    def start(self, version, parent):
        self.version, self.parent, self.attempts, self.failures = version, parent, 0, 0

    def record(self, success):
        """Record one localization; return True if the version must be rolled back."""
        if self.parent < 0 or self.attempts >= self.probation:
            return False
        self.attempts += 1
        self.failures += 0 if success else 1
        return self.failures > self.max_failures


class OverlayStore:
    """
    Immutable overlay versions in one directory.

    ``state_vNNNN.npz`` holds the full state, ``additions_vNNNN.npz`` the
    surface-additions file the runtime loads (``base`` additions, e.g. the
    operator's bed patch, plus the committed overlay) and ``manifest.json``
    the versions, their parents and the current one.
    """

    def __init__(self, directory, mesh_sha256, base_additions=None):
        self.directory = Path(directory)
        self.mesh_sha256 = mesh_sha256
        self.base = (np.zeros((0, 3)) if base_additions is None
                     else np.asarray(base_additions, float).reshape(-1, 3))
        self.directory.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.directory / 'manifest.json'
        if self.manifest_path.exists():
            self.manifest = json.loads(self.manifest_path.read_text())
            if self.manifest['mesh_sha256'] != mesh_sha256:
                raise ContractError('overlay store belongs to another mesh')
        else:
            self.manifest = dict(schema=1, mesh_sha256=mesh_sha256, current=None,
                                 versions=[])

    def _write_manifest(self):
        temporary = self.manifest_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.manifest, indent=1) + '\n')
        temporary.replace(self.manifest_path)

    def save(self, state, note=''):
        """Store a new version (never overwrite) and make it current."""
        if state.mesh_sha256 != self.mesh_sha256:
            raise ContractError('overlay state belongs to another mesh')
        known = {row['version'] for row in self.manifest['versions']}
        if state.version in known:
            raise ContractError(f'overlay version {state.version} exists')
        path = self.directory / f'state_v{state.version:04d}.npz'
        np.savez_compressed(path, mesh_sha256=state.mesh_sha256, version=state.version,
                            parent=state.parent, sessions=state.sessions, keys=state.keys,
                            committed=state.committed, points=state.points,
                            point_cell=state.point_cell, evidence=state.evidence,
                            positions=state.positions)
        surface = np.vstack([self.base, state.surface()])
        if len(surface):
            save_surface_additions(self.additions_path(state.version), surface,
                                   self.mesh_sha256, f'overlay v{state.version} {note}')
        self.manifest['versions'].append(dict(
            version=state.version, parent=state.parent, sessions=state.sessions,
            committed_cells=int(state.committed.sum()), points=int(len(surface)), note=note))
        self.manifest['current'] = state.version
        self._write_manifest()

    def next_version(self):
        """Return the first unused version number (it goes on after a rollback)."""
        return 1 + max((row['version'] for row in self.manifest['versions']), default=-1)

    def additions_path(self, version):
        return self.directory / f'additions_v{version:04d}.npz'

    def load(self, version=None):
        version = self.manifest['current'] if version is None else version
        if version is None:
            return OverlayState(self.mesh_sha256)
        with np.load(self.directory / f'state_v{version:04d}.npz') as data:
            if str(data['mesh_sha256']) != self.mesh_sha256:
                raise ContractError('overlay state belongs to another mesh')
            return OverlayState(str(data['mesh_sha256']), int(data['version']),
                                int(data['parent']), int(data['sessions']), data['keys'],
                                data['committed'], data['points'], data['point_cell'],
                                data['evidence'], data['positions'])

    def rollback(self, version):
        """Make an existing version current again (its files are unchanged)."""
        if version not in {row['version'] for row in self.manifest['versions']}:
            raise ContractError(f'no overlay version {version}')
        self.manifest['current'] = version
        self._write_manifest()
