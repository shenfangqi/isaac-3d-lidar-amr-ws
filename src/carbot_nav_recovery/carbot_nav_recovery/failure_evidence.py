"""Structured controller-failure contract, independent of log messages.

An in-process controller/BT adapter must bind its failure to the navigation
goal, localization context and exact FollowPath request. Passive status and
Humble's empty FollowPath result cannot construct this evidence.
"""

from dataclasses import dataclass
from enum import Enum
import math

from .trace_retreat import TraceContext


class FailureCause(str, Enum):
    COLLISION_PREDICTED = 'COLLISION_PREDICTED'
    ROTATION_BLOCKED = 'ROTATION_BLOCKED'
    NO_PROGRESS = 'NO_PROGRESS'
    TF_ERROR = 'TF_ERROR'
    INVALID_PATH = 'INVALID_PATH'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class ControllerFailureEvidence:
    context: TraceContext
    follow_path_id: str
    controller_id: str
    cause: FailureCause
    source_stamp_sec: float
    received_monotonic_sec: float
    collision_checked: bool


def failure_gate(evidence, context, follow_path_id, *, now_ros_sec,
                 now_monotonic_sec, max_age_sec=0.5):
    """Allow only evaluation of a fresh, request-bound collision failure.

    OK does not grant control or authorize motion. The producer must be the
    controller adapter; this API does not authenticate a ROS publisher.
    """
    if evidence is None:
        return 'NO_STRUCTURED_CONTROLLER_FAILURE'
    if not isinstance(evidence, ControllerFailureEvidence):
        return 'INVALID_CONTROLLER_EVIDENCE'
    if context is None or evidence.context != context:
        return 'FAILURE_CONTEXT_MISMATCH'
    if (not follow_path_id or evidence.follow_path_id != follow_path_id
            or not evidence.controller_id):
        return 'FAILURE_REQUEST_MISMATCH'
    times = (evidence.source_stamp_sec, evidence.received_monotonic_sec,
             now_ros_sec, now_monotonic_sec, max_age_sec)
    if (not all(math.isfinite(value) for value in times)
            or evidence.source_stamp_sec <= 0 or not 0 < max_age_sec <= 0.5):
        return 'INVALID_FAILURE_TIME'
    if (not 0 <= now_ros_sec - evidence.source_stamp_sec <= max_age_sec
            or not 0 <= now_monotonic_sec
            - evidence.received_monotonic_sec <= max_age_sec):
        return 'STALE_CONTROLLER_FAILURE'
    if (not isinstance(evidence.cause, FailureCause)
            or evidence.cause not in (FailureCause.COLLISION_PREDICTED,
                                      FailureCause.ROTATION_BLOCKED)
            or evidence.collision_checked is not True):
        return 'FAILURE_NOT_COLLISION_VERIFIED'
    return 'OK'
