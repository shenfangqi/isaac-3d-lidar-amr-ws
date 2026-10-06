"""Read-only speed advice for curvature and predicted path collisions.

The functions in this module are ROS independent and never publish commands.
They intentionally require a physically accepted braking profile before
returning a numeric speed recommendation.  Geometry may still be inspected
without a profile, but the result remains ``CALIBRATION_REQUIRED``.
"""

from dataclasses import dataclass
import math
import time
from typing import Mapping, Optional, Sequence, Tuple

from .swept_footprint import (
    CostmapSnapshot,
    Pose2D,
    SnapshotFreshness,
    check_snapshot_freshness,
    check_swept_path,
)


Point = Tuple[float, float]


@dataclass(frozen=True)
class BrakingProfile:
    """Externally measured limits used for validation-only speed advice."""

    physical_acceptance_complete: bool
    evidence_directory: str
    minimum_deceleration_mps2: float
    command_latency_sec: float
    position_margin_m: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float

    def __post_init__(self):
        """Reject incomplete or nonphysical profile values."""
        values = (
            self.minimum_deceleration_mps2,
            self.command_latency_sec,
            self.position_margin_m,
            self.max_linear_speed_mps,
            self.max_angular_speed_radps,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('braking profile values must be finite')
        if (self.minimum_deceleration_mps2 <= 0.0
                or self.command_latency_sec < 0.0
                or self.position_margin_m < 0.0
                or self.max_linear_speed_mps <= 0.0
                or self.max_angular_speed_radps <= 0.0):
            raise ValueError('braking profile limits are invalid')
        if self.physical_acceptance_complete:
            if not self.evidence_directory.startswith('/'):
                raise ValueError(
                    'accepted profile requires an absolute evidence directory')

    @classmethod
    def from_mapping(cls, values: Mapping):
        """Parse a strict profile without silently supplying physical data."""
        required = {
            'physical_acceptance_complete',
            'evidence_directory',
            'minimum_deceleration_mps2',
            'command_latency_sec',
            'position_margin_m',
            'max_linear_speed_mps',
            'max_angular_speed_radps',
        }
        if set(values) != required:
            raise ValueError('braking profile fields do not match schema')
        if type(values['physical_acceptance_complete']) is not bool:
            raise ValueError('physical_acceptance_complete must be boolean')
        if not isinstance(values['evidence_directory'], str):
            raise ValueError('evidence_directory must be a string')
        return cls(**values)


@dataclass(frozen=True)
class PathRisk:
    """Geometry-only description of the path remaining in the local window."""

    distance_to_unsafe_m: float
    unsafe_reason: str
    horizon_length_m: float
    maximum_curvature_inv_m: float
    evaluated_pose_count: int


@dataclass(frozen=True)
class SpeedAdvisory:
    """A non-actuating recommendation with stable diagnostic fields."""

    reason: str
    recommended_speed_mps: Optional[float]
    braking_speed_limit_mps: Optional[float]
    curvature_speed_limit_mps: Optional[float]
    current_speed_mps: float
    stopping_distance_m: Optional[float]
    risk: PathRisk
    braking_calibrated: bool
    speed_reduction_required: bool
    validation_only: bool = True
    motion_eligible: bool = False


def path_length(poses: Sequence[Pose2D]) -> float:
    """Return cumulative XY distance along a pose sequence."""
    return sum(math.hypot(end.x - start.x, end.y - start.y)
               for start, end in zip(poses, poses[1:]))


def maximum_path_curvature(poses: Sequence[Pose2D]) -> float:
    """Return maximum three-point curvature, ignoring duplicate samples."""
    maximum = 0.0
    for first, middle, last in zip(poses, poses[1:], poses[2:]):
        a = math.hypot(middle.x - first.x, middle.y - first.y)
        b = math.hypot(last.x - middle.x, last.y - middle.y)
        c = math.hypot(last.x - first.x, last.y - first.y)
        if min(a, b, c) <= 1.0e-6:
            continue
        cross = ((middle.x - first.x) * (last.y - first.y)
                 - (middle.y - first.y) * (last.x - first.x))
        maximum = max(maximum, abs(2.0 * cross / (a * b * c)))
    return maximum


def stopping_distance(speed_mps: float, profile: BrakingProfile) -> float:
    """Return latency, constant-deceleration distance and position margin."""
    if not math.isfinite(speed_mps) or speed_mps < 0.0:
        raise ValueError('speed must be finite and nonnegative')
    return (speed_mps * profile.command_latency_sec
            + speed_mps * speed_mps
            / (2.0 * profile.minimum_deceleration_mps2)
            + profile.position_margin_m)


def braking_speed_limit(available_distance_m: float,
                        profile: BrakingProfile) -> float:
    """Solve the accepted stopping model for its maximum initial speed."""
    if not math.isfinite(available_distance_m):
        raise ValueError('available distance must be finite')
    distance = available_distance_m - profile.position_margin_m
    if distance <= 0.0:
        return 0.0
    acceleration = profile.minimum_deceleration_mps2
    latency = profile.command_latency_sec
    limit = acceleration * (
        math.sqrt(latency * latency + 2.0 * distance / acceleration)
        - latency)
    return min(profile.max_linear_speed_mps, max(0.0, limit))


def curvature_speed_limit(curvature_inv_m: float,
                          profile: BrakingProfile) -> float:
    """Limit linear speed so path yaw rate stays within the accepted bound."""
    if not math.isfinite(curvature_inv_m) or curvature_inv_m < 0.0:
        raise ValueError('curvature must be finite and nonnegative')
    if curvature_inv_m <= 1.0e-9:
        return profile.max_linear_speed_mps
    return min(profile.max_linear_speed_mps,
               profile.max_angular_speed_radps / curvature_inv_m)


def _interpolate(start: Pose2D, end: Pose2D, fraction: float) -> Pose2D:
    yaw_delta = math.atan2(math.sin(end.yaw - start.yaw),
                           math.cos(end.yaw - start.yaw))
    return Pose2D(
        start.x + (end.x - start.x) * fraction,
        start.y + (end.y - start.y) * fraction,
        start.yaw + yaw_delta * fraction,
    )


def evaluate_path_risk(
    snapshot: CostmapSnapshot,
    footprint: Sequence[Point],
    poses: Sequence[Pose2D],
    safety_margin_m: float = 0.0,
    *,
    deadline_monotonic: Optional[float] = None,
) -> PathRisk:
    """Find the safe path prefix using full swept-footprint checks.

    A blocked segment is bisected to costmap-scale precision.  The returned
    distance is a path distance, not Euclidean range to a raw laser point.
    """
    if len(poses) < 2:
        raise ValueError('path requires at least two poses')
    values = tuple(value for pose in poses for value in
                   (pose.x, pose.y, pose.yaw))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('path poses must be finite')
    total = path_length(poses)
    curvature = maximum_path_curvature(poses)
    travelled = 0.0
    start_check = check_swept_path(
        snapshot, footprint, (poses[0],), safety_margin_m,
        deadline_monotonic=deadline_monotonic)
    if not start_check.safe:
        return PathRisk(0.0, start_check.reason, total, curvature, len(poses))

    for start, end in zip(poses, poses[1:]):
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            return PathRisk(travelled, 'COMPUTE_BUDGET_EXCEEDED', total,
                            curvature, len(poses))
        segment_length = math.hypot(end.x - start.x, end.y - start.y)
        result = check_swept_path(
            snapshot, footprint, (start, end), safety_margin_m,
            deadline_monotonic=deadline_monotonic)
        if result.safe:
            travelled += segment_length
            continue
        low, high = 0.0, 1.0
        # Stop when the remaining uncertainty is at most half a grid cell.
        while ((high - low) * segment_length > snapshot.resolution * 0.5
               and high - low > 1.0e-6):
            if (deadline_monotonic is not None
                    and time.monotonic() >= deadline_monotonic):
                return PathRisk(
                    travelled + low * segment_length,
                    'COMPUTE_BUDGET_EXCEEDED', total, curvature, len(poses))
            middle = (low + high) * 0.5
            prefix = check_swept_path(
                snapshot, footprint, (start, _interpolate(start, end, middle)),
                safety_margin_m, deadline_monotonic=deadline_monotonic)
            if prefix.safe:
                low = middle
            else:
                high = middle
        return PathRisk(travelled + low * segment_length, result.reason,
                        total, curvature, len(poses))
    return PathRisk(total, 'HORIZON_CLEAR', total, curvature, len(poses))


def advise_speed(
    snapshot: CostmapSnapshot,
    footprint: Sequence[Point],
    poses: Sequence[Pose2D],
    freshness: SnapshotFreshness,
    current_speed_mps: float,
    profile: Optional[BrakingProfile],
    safety_margin_m: float = 0.0,
    *,
    deadline_monotonic: Optional[float] = None,
) -> SpeedAdvisory:
    """Evaluate a local path without modifying controller output."""
    if not math.isfinite(current_speed_mps) or current_speed_mps < 0.0:
        raise ValueError('current speed must be finite and nonnegative')
    fresh = check_snapshot_freshness(snapshot, freshness)
    if not fresh.safe:
        risk = PathRisk(0.0, fresh.reason, 0.0, 0.0, len(poses))
        return SpeedAdvisory(
            fresh.reason, None, None, None, current_speed_mps, None, risk,
            bool(profile and profile.physical_acceptance_complete), False)

    risk = evaluate_path_risk(
        snapshot, footprint, poses, safety_margin_m,
        deadline_monotonic=deadline_monotonic)
    calibrated = bool(profile and profile.physical_acceptance_complete)
    if not calibrated:
        return SpeedAdvisory(
            'CALIBRATION_REQUIRED', None, None, None, current_speed_mps,
            None, risk, False, False)

    braking_limit = braking_speed_limit(risk.distance_to_unsafe_m, profile)
    curve_limit = curvature_speed_limit(
        risk.maximum_curvature_inv_m, profile)
    recommended = min(profile.max_linear_speed_mps,
                      braking_limit, curve_limit)
    stopping = stopping_distance(current_speed_mps, profile)
    tolerance = 1.0e-6
    reduction = current_speed_mps > recommended + tolerance
    if risk.unsafe_reason == 'COMPUTE_BUDGET_EXCEEDED':
        # Only the checked prefix is known: keep its braking limit, but do
        # not report an unchecked path as a predicted collision (#18).
        reason = 'COMPUTE_BUDGET_EXCEEDED'
    elif risk.distance_to_unsafe_m <= profile.position_margin_m:
        reason = 'PREDICTED_COLLISION'
    elif current_speed_mps > curve_limit + tolerance:
        reason = 'CURVATURE_SPEED_UNSAFE'
    elif stopping > risk.distance_to_unsafe_m + tolerance:
        reason = 'PREDICTED_COLLISION'
    else:
        reason = 'OK'
    return SpeedAdvisory(
        reason, recommended, braking_limit, curve_limit, current_speed_mps,
        stopping, risk, True, reduction)
