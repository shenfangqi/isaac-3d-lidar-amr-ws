"""Safety geometry for non-actuating Carbot recovery evaluation."""

from .swept_footprint import (
    CostmapSnapshot,
    ObservedFreeSpaceSnapshot,
    Pose2D,
    SnapshotFreshness,
    check_snapshot_freshness,
    check_snapshot_data_freshness,
    check_swept_path,
    check_observed_free_path,
    evaluate_rotation_candidates,
    rank_rotation_candidates,
)

from .trace_retreat import (
    PoseHistory, RetreatLimits, TraceContext, TraceLimits,
    evaluate_trace_retreat,
)
from .recovery_budget import RecoveryBudget
from .strategy_selection import recovery_options
from .speed_advisor import (
    BrakingProfile, PathRisk, SpeedAdvisory, advise_speed,
    braking_speed_limit, curvature_speed_limit, evaluate_path_risk,
    maximum_path_curvature, stopping_distance,
)

__all__ = [
    'PoseHistory', 'RetreatLimits', 'TraceContext', 'TraceLimits',
    'evaluate_trace_retreat', 'RecoveryBudget', 'recovery_options',
    'CostmapSnapshot',
    'ObservedFreeSpaceSnapshot',
    'Pose2D',
    'SnapshotFreshness',
    'check_snapshot_freshness',
    'check_snapshot_data_freshness',
    'check_swept_path',
    'check_observed_free_path',
    'evaluate_rotation_candidates',
    'rank_rotation_candidates',
    'BrakingProfile', 'PathRisk', 'SpeedAdvisory', 'advise_speed',
    'braking_speed_limit', 'curvature_speed_limit', 'evaluate_path_risk',
    'maximum_path_curvature', 'stopping_distance',
]
