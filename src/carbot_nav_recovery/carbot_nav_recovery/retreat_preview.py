"""Connect current runtime history to bounded, non-actuating retreat checks."""

from dataclasses import asdict
import math
import time

from .failure_evidence import failure_gate
from .swept_footprint import SnapshotFreshness
from .trace_retreat import RetreatLimits, evaluate_trace_retreat


def evaluate_runtime_retreat(state, local_map, static_map, visibility, *,
                             now_ros_sec, now_monotonic_sec, footprint,
                             stopping_distance_m):
    """Preview geometry only; budget is read but no attempt is started.

    A returned clear candidate still lacks measured braking calibration,
    authenticated source freshness, controller failure and command ownership.
    The hypothetical stopping allowance is reported, never called calibrated.
    """
    if (not math.isfinite(stopping_distance_m)
            or not 0 < stopping_distance_m <= 0.20):
        raise ValueError('preview stopping allowance must be in (0, 0.20]')
    report = {
        'schema': 'carbot_trace_retreat_preview_v1',
        'goal_id': state.goal_id,
        'localization_epoch': (state.context.localization_epoch
                               if state.context else None),
        'motion_eligible': False,
        'controller_gate': failure_gate(
            None, state.context, None, now_ros_sec=now_ros_sec,
            now_monotonic_sec=now_monotonic_sec),
        'costmap_update_verified': False,
        'visibility_provenance_verified': False,
        'braking_calibrated': False,
        'assumed_stopping_distance_m': stopping_distance_m,
        'departure_verified': False,
        'candidates': [],
    }
    reason = None
    if state.context is None:
        reason = 'NO_ACTIVE_GOAL'
    elif state.history.context != state.context or len(state.history.samples) < 2:
        reason = 'NO_CURRENT_HISTORY'
    elif local_map is None:
        reason = 'NO_LOCAL_COSTMAP'
    elif static_map is None:
        reason = 'NO_STATIC_MAP'
    elif visibility is None:
        reason = 'NO_OBSERVED_REAR_CLEARANCE'
    elif state.budget is None:
        reason = 'NO_GOAL_BUDGET'
    if reason:
        report['reason'] = reason
        return report
    distance, seconds = state.budget.remaining(state.context, now_monotonic_sec)
    if state.budget.active or state.budget.attempts >= 2:
        report['reason'] = 'RECOVERY_ACTIVE_OR_ATTEMPTS_EXHAUSTED'
        return report
    # The observer rechecks its source/receive/TF gates before calling this.
    # A provisional localization flag permits geometry, never execution.
    freshness = SnapshotFreshness(
        now_ros_sec, now_monotonic_sec, 0.5, 0.5, True, True)
    candidates, reason = evaluate_trace_retreat(
        state.history, state.context, state.history.samples[-1].pose,
        local_map, static_map, visibility, footprint, freshness,
        RetreatLimits(stopping_distance_m=stopping_distance_m),
        distance_remaining_m=distance, time_remaining_sec=seconds,
        deadline_monotonic=time.monotonic() + 0.10)
    report['reason'] = reason
    report['candidates'] = [asdict(candidate) for candidate in candidates]
    return report
