"""
Independent 3D evidence checks and the 3D localization decision.

This module does not authorize READY by itself: the manager still seeds
AMCL and requires its own post-seed checks.  Temporal TRAIN/HOLDOUT
separation, candidate completeness, source transforms and map identity
belong to the caller.
"""
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial import cKDTree

from .localization_contracts import ContractError, SE2
from .localization_hypotheses import score_pose
from .localization_surface_check import SurfaceCheckConfig, check_surfaces, planar


@dataclass(frozen=True)
class SurfaceSupportConfig:
    sigma_m: float = .05
    score_loss: float = .03
    step_m: float = .025
    range_m: float = .4
    max_extent_m: float = .20
    step_rad: float = math.radians(1)
    range_rad: float = math.radians(12)
    max_extent_rad: float = math.radians(5)
    max_points: int = 12000

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in self.__dict__.values()):
            raise ContractError('support limits must be finite and positive')
        if (type(self.max_points) is not int or self.max_points < 1
                or self.step_m > self.max_extent_m or self.max_extent_m >= self.range_m
                or self.step_rad > self.max_extent_rad or self.max_extent_rad >= self.range_rad):
            raise ContractError('invalid support sampling limits')


@dataclass(frozen=True)
class SurfaceRefineConfig:
    """
    Local 3D refinement of every candidate before ranking.

    2D-refined poses sit a few cm off the 3D optimum (2026-10-10 labelled
    captures), so every candidate, not only the leader, climbs the smooth
    fit until it converges; a competitor stopped early would be
    under-scored.  The leader must stay within ``leader_shift_m`` /
    ``leader_turn_rad`` of the candidate it started from.
    """

    sigma_m: float = .05
    steps_m: tuple = (.04, .02, .01)
    steps_rad: tuple = (math.radians(2), math.radians(1), math.radians(.5))
    max_shift_m: float = 1.0
    max_turn_rad: float = math.radians(45)
    max_evaluations: int = 600
    leader_shift_m: float = .15
    leader_turn_rad: float = math.radians(6)
    same_xy_m: float = .10
    same_yaw_rad: float = math.radians(5)
    # 3000 points refine to the same poses as 8000 at 2.5x less cost
    # (2026-10-10 capture_02 benchmark).
    max_points: int = 3000

    def __post_init__(self):
        values = (self.sigma_m, self.max_shift_m, self.max_turn_rad,
                  self.leader_shift_m, self.leader_turn_rad, self.same_xy_m,
                  self.same_yaw_rad) + tuple(self.steps_m) + tuple(self.steps_rad)
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ContractError('refinement limits must be finite and positive')
        if (len(self.steps_m) != len(self.steps_rad) or not self.steps_m
                or type(self.max_points) is not int or self.max_points < 1
                or type(self.max_evaluations) is not int or self.max_evaluations < 1):
            raise ContractError('invalid refinement schedule')


def _subsample(points, count):
    if len(points) > count:
        points = points[np.linspace(0, len(points) - 1, count).astype(int)]
    return points


def _smooth_fit(tree, homogeneous, sigma):
    def fit(pose):
        world = (planar(*pose) @ homogeneous.T).T[:, :3]
        distances, _ = tree.query(world, distance_upper_bound=sigma * 4)
        return float(np.exp(-.5 * (distances / sigma) ** 2).mean())
    return fit


def _turn(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def refine_surface_pose(tree, points, pose, config, expired):
    """
    Coordinate pattern search on the smooth 3D fit around ``pose``.

    Returns ``(pose, converged)``.  Not converged: the search hit its
    shift/turn limit or its evaluation budget, or ran out of time, so the
    pose may be short of its local optimum.
    """
    points = _subsample(np.asarray(points, float), config.max_points)
    fit = _smooth_fit(tree, np.c_[points, np.ones(len(points))], config.sigma_m)
    start = np.asarray(pose, float)
    best, best_fit = start.copy(), fit(start)
    evaluations, blocked = 1, False
    for step_m, step_rad in zip(config.steps_m, config.steps_rad):
        improved = True
        while improved:
            if expired() or evaluations >= config.max_evaluations:
                return tuple(float(v) for v in best), False
            improved = False
            for delta in ((step_m, 0, 0), (-step_m, 0, 0), (0, step_m, 0),
                          (0, -step_m, 0), (0, 0, step_rad), (0, 0, -step_rad)):
                trial = best + delta
                if (math.hypot(*(trial[:2] - start[:2])) > config.max_shift_m
                        or _turn(trial[2], start[2]) > config.max_turn_rad):
                    blocked = True
                    continue
                value = fit(trial)
                evaluations += 1
                if value > best_fit + 1e-9:
                    best, best_fit, improved = trial, value, True
    # A blocked move only matters if the final pose is on the limit.
    on_limit = blocked and (
        math.hypot(*(best[:2] - start[:2])) > config.max_shift_m - min(config.steps_m)
        or _turn(best[2], start[2]) > config.max_turn_rad - min(config.steps_rad))
    return tuple(float(v) for v in best), not on_limit


@dataclass(frozen=True)
class SurfaceValidation:
    supported: bool
    reason: str
    leader: int
    train: object
    holdout: object
    support_extents: tuple = ()
    center_fit: float = 0.
    poses: tuple = ()
    # Index of each pose in the input order (``poses`` then ``seed_poses``).
    origins: tuple = ()


def validate_surface_evidence(vertices, train_points, holdout_points, poses, *,
                              deadline, check_config=SurfaceCheckConfig(),
                              support_config=SurfaceSupportConfig(),
                              refine_config=None, seed_poses=()):
    """
    Require independent ranking agreement and bounded x/y/yaw support.

    Supports four planar axes (including diagonals) and yaw, using smooth
    mesh distance. A sampled connected support is evidence under this model,
    not a proof of global identifiability or physical collision clearance.

    With ``refine_config`` every candidate (``poses`` then ``seed_poses``)
    is first refined on the TRAIN points only, and candidates that converge
    to the same solution are merged; ranking and support then use the
    refined poses (returned in ``poses``).  A candidate that does not
    converge refuses the result, since it could be under-scored, and the
    leader must stay near the candidate it started from.
    """
    if not math.isfinite(deadline):
        raise ContractError('finite validation deadline required')

    def expired():
        return time.monotonic() >= deadline
    if expired():
        return SurfaceValidation(False, 'DEADLINE', -1, None, None)
    tree = cKDTree(vertices)
    starts = tuple(tuple(float(v) for v in pose) for pose in tuple(poses) + tuple(seed_poses))
    origins = tuple(range(len(starts)))
    unconverged = False
    if refine_config is not None:
        train_xyz = np.asarray(train_points, float).reshape(-1, 3)
        train_xyz = train_xyz[np.hypot(train_xyz[:, 0], train_xyz[:, 1])
                              >= check_config.min_range_m]
        refined = [refine_surface_pose(tree, train_xyz, pose, refine_config, expired)
                   for pose in starts]
        if expired():
            return SurfaceValidation(False, 'DEADLINE', -1, None, None)
        unconverged = not all(converged for _, converged in refined)
        merged = []                     # indices into ``refined``, first kept
        for index, (pose, _converged) in enumerate(refined):
            if not any(math.hypot(pose[0] - refined[k][0][0], pose[1] - refined[k][0][1])
                       <= refine_config.same_xy_m
                       and _turn(pose[2], refined[k][0][2]) <= refine_config.same_yaw_rad
                       for k in merged):
                merged.append(index)
        poses = tuple(refined[k][0] for k in merged)
        starts = tuple(starts[k] for k in merged)
        origins = tuple(merged)
    else:
        poses = starts
    if len(poses) < 2:
        return SurfaceValidation(False, 'TOO_FEW_DISTINCT_CANDIDATES', -1, None, None,
                                 poses=poses, origins=origins)
    train = check_surfaces(vertices, train_points, poses, check_config)
    if expired():
        return SurfaceValidation(False, 'DEADLINE', -1, train, None)
    held = check_surfaces(vertices, holdout_points, poses, check_config)

    def result(reason, extents=(), fit=0.):
        return SurfaceValidation(not reason, reason, train.leader, train, held,
                                 tuple(extents), fit, poses, origins)
    if expired():
        return result('DEADLINE')
    if not train.resolved:
        return result('TRAIN_' + train.reason)
    if not held.resolved:
        return result('HOLDOUT_' + held.reason)
    if train.leader != held.leader:
        return result('INDEPENDENT_LEADER_DISAGREEMENT')
    if unconverged:
        return result('CANDIDATE_NOT_CONVERGED')
    if refine_config is not None and (
            math.hypot(poses[train.leader][0] - starts[train.leader][0],
                       poses[train.leader][1] - starts[train.leader][1])
            > refine_config.leader_shift_m
            or _turn(poses[train.leader][2], starts[train.leader][2])
            > refine_config.leader_turn_rad):
        return result('LEADER_FAR_FROM_ITS_CANDIDATE')
    points = np.asarray(holdout_points, float)
    mask = np.zeros(len(points), bool)
    for used, (lo, hi) in zip(held.bands_used, check_config.bands):
        if used:
            mask |= (points[:, 2] >= lo) & (points[:, 2] < hi)
    mask &= np.hypot(points[:, 0], points[:, 1]) >= check_config.min_range_m
    points = points[mask]
    cfg = support_config
    if len(points) > cfg.max_points:
        points = points[np.linspace(0, len(points)-1, cfg.max_points).astype(int)]
    homogeneous = np.c_[points, np.ones(len(points))]
    center = np.asarray(poses[train.leader], float)

    def fit(pose):
        world = (planar(*pose) @ homogeneous.T).T[:, :3]
        distances, _ = tree.query(world, distance_upper_bound=cfg.sigma_m*4)
        return float(np.exp(-.5*(distances/cfg.sigma_m)**2).mean())
    center_fit = fit(center)
    floor = center_fit - cfg.score_loss
    extents = []
    axes = [(math.cos(a), math.sin(a), 0.) for a in
            (0., math.pi/4, math.pi/2, 3*math.pi/4)] + [(0., 0., 1.)]
    for index, axis in enumerate(axes):
        step = cfg.step_rad if index == 4 else cfg.step_m
        limit = cfg.range_rad if index == 4 else cfg.range_m
        maximum = cfg.max_extent_rad if index == 4 else cfg.max_extent_m
        extent = 0.
        for sign in (1., -1.):
            distance = step
            while distance <= limit + 1e-9:
                if expired():
                    return result('DEADLINE', extents, center_fit)
                if fit(center + np.asarray(axis)*sign*distance) < floor:
                    break
                extent += step
                distance += step
            else:
                return result('UNBOUNDED_3D_SUPPORT', extents, center_fit)
        extents.append(extent)
        if extent > maximum + 1e-9:
            return result('WIDE_3D_SUPPORT', extents, center_fit)
    return result('', extents, center_fit)


@dataclass(frozen=True)
class SurfaceDecision:
    """3D localization decision with the 2D sanity metrics of its leader."""

    accepted: bool
    reason: str
    leader: int
    pose: tuple
    validation: SurfaceValidation
    metrics_2d: dict


# Ranking of refined candidates: the gap is measured at 0.05 m, the scale of
# the refinement; the leader's absolute fit keeps the 0.10 m check.  On the
# three labelled captures (2026-10-10) refined twins scored gaps of 0.161-0.244
# at 0.05 m but only 0.123-0.182 at 0.10 m; both thresholds are unchanged and
# provisional.
RANK_CHECK = SurfaceCheckConfig(tolerance_m=.05, min_leader_composite=.01)
FIT_CHECK = SurfaceCheckConfig()


def decide_surface(vertices, train_points, holdout_points, poses, grid, holdout_frames,
                   reference, search_config, thresholds, max_conflict, *, deadline,
                   check_config=RANK_CHECK, fit_config=FIT_CHECK,
                   support_config=SurfaceSupportConfig(),
                   refine_config=SurfaceRefineConfig(), seed_poses=()):
    """
    Accept a pose from independent 3D evidence plus a 2D sanity check.

    ``poses`` are the refined 2D candidates and ``seed_poses`` the coarse
    seeds of competitors an incomplete search left unrefined; together they
    must cover every cluster that could still compete.  The 3D evidence
    replaces the 2D margin and corridor checks, which the 2D band cannot
    satisfy in the confined twins.  The leader, at its refined pose, must
    still fit the 2D map on the HOLDOUT scans: score, coverage and known
    beams at the production thresholds, and a conflict ratio no larger than
    ``max_conflict`` (a gross-contradiction cap; the labelled true poses
    reached 0.275 against the 0.25 2D gate).
    """
    validation = validate_surface_evidence(
        vertices, train_points, holdout_points, poses, deadline=deadline,
        check_config=check_config, support_config=support_config,
        refine_config=refine_config, seed_poses=seed_poses)
    if not validation.supported:
        return SurfaceDecision(False, validation.reason, validation.leader, (),
                               validation, {})
    pose = validation.poses[validation.leader]
    fit = check_surfaces(vertices, holdout_points, (pose, pose), fit_config).composite
    if not fit or fit[0] < fit_config.min_leader_composite:
        return SurfaceDecision(False, 'LEADER_FIT_TOO_LOW', validation.leader, pose,
                               validation, {'fit_3d': fit[0] if fit else 0.})
    metrics = score_pose(grid, SE2(*pose), holdout_frames, reference, search_config,
                         search_config.refine_beams)
    summary = {k: metrics[k] for k in ('score', 'coverage', 'known', 'conflict')}
    summary['fit_3d'] = fit[0]
    reason = ('2D_SCORE_TOO_LOW' if metrics['score'] < thresholds.min_score
              else '2D_COVERAGE_TOO_LOW' if metrics['coverage'] < thresholds.min_coverage
              else '2D_TOO_FEW_KNOWN' if metrics['known'] < thresholds.min_known
              else '2D_GROSS_CONFLICT' if metrics['conflict'] > max_conflict
              else '')
    return SurfaceDecision(not reason, reason, validation.leader, pose, validation, summary)


def run_surface_decision_job(vertices, train_points, holdout_points, poses, grid,
                             holdout_frames, reference, search_config, thresholds,
                             max_conflict, check_config, fit_config, timeout_s,
                             seed_poses=()):
    """Worker entry point for :func:`decide_surface`."""
    return decide_surface(vertices, train_points, holdout_points, poses, grid,
                          holdout_frames, reference, search_config, thresholds,
                          max_conflict, deadline=time.monotonic() + timeout_s,
                          check_config=check_config, fit_config=fit_config,
                          seed_poses=seed_poses)
