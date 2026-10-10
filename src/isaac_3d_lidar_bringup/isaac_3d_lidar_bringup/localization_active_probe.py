"""Joint before/after probe inference. Offline core, not a movement executor."""
from dataclasses import dataclass
import math
import time

from .localization_contracts import ContractError, FrameRole, RejectReason, SE2
from .localization_hypotheses import (
    QualityDecision, recheck_hypotheses, search_multiview,
    validate_hypotheses, seed_pose_at_current_time, score_pose)


@dataclass(frozen=True)
class ProbeInference:
    result: object
    decision: QualityDecision
    current_pose: object
    full_search_used: bool


def infer_after_translation(grid, previous, previous_reference,
                            before_train, before_holdout, after_train, after_holdout,
                            current_odom, config, thresholds, *,
                            odom_continuous, deadline, cancel_token=None,
                            min_translation_m=.15):
    """
    Recheck all candidates using two stationary viewpoints and one odom chain.

    The caller must attest continuity from the guarded motion trace, validate
    source freshness and current stopped status. Old holdout frames remain
    holdout. No saved pose prior is used; no READY or motion authority is issued.
    """
    if odom_continuous is not True:
        raise ContractError('continuous guarded odometry is required')
    if not math.isfinite(min_translation_m) or min_translation_m <= 0:
        raise ContractError('minimum translation must be positive')
    if deadline is None or not math.isfinite(deadline):
        raise ContractError('a bounded inference deadline is required')
    groups = tuple(tuple(g) for g in (before_train, before_holdout, after_train, after_holdout))
    if any(not g for g in groups):
        raise ContractError('both viewpoints need TRAIN and independent HOLDOUT')
    frames = sum(groups, ())
    if (len({f.id for f in frames}) != len(frames)
            or len({f.stamp_ns for f in frames}) != len(frames)):
        raise ContractError('all inference frames must be independent')
    for group, role in zip(groups, (FrameRole.TRAIN, FrameRole.HOLDOUT)*2):
        if any(f.role != role or f.session != previous.session for f in group):
            raise ContractError('incorrect frame role or session')
    before, after = groups[0] + groups[1], groups[2] + groups[3]
    if {f.view_id for f in before} & {f.view_id for f in after}:
        raise ContractError('translation requires a new view ID')
    if max(f.stamp_ns for f in before) >= min(f.stamp_ns for f in after):
        raise ContractError('after frames must follow before frames')
    for view in (before, after):
        anchor = view[0].T_odom_base
        if any(math.hypot(f.T_odom_base.x-anchor.x, f.T_odom_base.y-anchor.y) > .05
               or abs(anchor.inverse().compose(f.T_odom_base).yaw) > math.radians(3)
               for f in view):
            raise ContractError('each viewpoint must be stationary')
    first, last = before[0].T_odom_base, after[0].T_odom_base
    if math.hypot(last.x-first.x, last.y-first.y) < min_translation_m:
        raise ContractError('probe produced insufficient viewpoint translation')
    train, holdout = groups[0] + groups[2], groups[1] + groups[3]

    def validate(result):
        verdict = validate_hypotheses(grid, result, train, holdout, config,
                                      thresholds, deadline, cancel_token)
        if not verdict.accepted:
            return verdict
        winner = verdict.winner
        pose = SE2(winner.x, winner.y, winner.yaw)
        # A high score at one location must not average away a contradiction
        # at the other. Margin/observability remain joint, absolute fit does not.
        for held_view in (groups[1], groups[3]):
            if cancel_token is not None and cancel_token():
                return QualityDecision(False, RejectReason.CANCELED.value)
            if time.monotonic() >= deadline:
                return QualityDecision(False, RejectReason.SEARCH_INCOMPLETE.value)
            metrics = score_pose(grid, pose, held_view, train[0].T_odom_base,
                                 config, config.refine_beams)
            if (metrics['score'] < thresholds.min_score
                    or metrics['coverage'] < thresholds.min_coverage
                    or metrics['known'] < thresholds.min_known
                    or metrics['conflict'] > thresholds.max_conflict):
                return QualityDecision(False, RejectReason.NO_VALID_CANDIDATE.value)
        if cancel_token is not None and cancel_token():
            return QualityDecision(False, RejectReason.CANCELED.value)
        if time.monotonic() >= deadline:
            return QualityDecision(False, RejectReason.SEARCH_INCOMPLETE.value)
        return verdict
    full = not previous.complete or not previous.hypotheses
    result = (search_multiview(grid, train, config, deadline, cancel_token)
              if full else recheck_hypotheses(grid, previous, previous_reference,
                                              train, config, deadline, cancel_token))
    decision = validate(result)
    if not full and decision.reason == RejectReason.NO_VALID_CANDIDATE.value:
        # A refuted candidate set cannot justify a winner. Search the entire
        # map and test every new candidate against BOTH held-out viewpoints.
        full = True
        result = search_multiview(grid, train, config, deadline, cancel_token)
        decision = validate(result)
    winner = decision.winner
    pose = (seed_pose_at_current_time(SE2(winner.x, winner.y, winner.yaw), train[0].T_odom_base,
                                      current_odom) if decision.accepted else None)
    return ProbeInference(result, decision, pose, full)
