"""Bounded reverse traversal of measured poses; never sends commands.

The history belongs to one user goal and localization epoch. A remembered
route supplies geometry, never evidence that the space is still free. Every
candidate is checked against current local/static maps and independent recent
sensor-cleared coverage, including its braking extension.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .swept_footprint import (
    CheckResult, CostmapSnapshot, ObservedFreeSpaceSnapshot, Pose2D,
    SnapshotFreshness, check_observed_free_path, check_snapshot_freshness,
    check_swept_path,
)


@dataclass(frozen=True)
class TraceContext:
    goal_id: str
    localization_epoch: str
    frame_id: str

    def __post_init__(self):
        if not all((self.goal_id, self.localization_epoch, self.frame_id)):
            raise ValueError('trace requires goal, epoch and frame')


@dataclass(frozen=True)
class Breadcrumb:
    pose: Pose2D
    stamp_sec: float


@dataclass(frozen=True)
class TraceLimits:
    max_age_sec: float = 20.0
    max_gap_sec: float = 0.5
    max_samples: int = 512
    max_speed_m_s: float = 0.15
    max_yaw_rate_rad_s: float = 0.75
    position_noise_m: float = 0.01
    yaw_noise_rad: float = 0.03
    lateral_tolerance_m: float = 0.01

    def __post_init__(self):
        values = tuple(vars(self).values())
        if any(not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError('trace limits must be finite and positive')
        if not isinstance(self.max_samples, int) or self.max_samples < 2:
            raise ValueError('max_samples must be an integer >= 2')


class PoseHistory:
    """Record actual forward tracking poses, preserving turns and timing.

    Call invalidate on cancel, manual takeover, lift, localization loss or
    estimator reset. A context change clears the old route automatically.
    Pausing sampling is allowed, but a gap is never bridged on resumption.
    """

    def __init__(self, limits=TraceLimits()):
        self.limits = limits
        self.context = None
        self._samples = []
        self.last_reset_reason = 'NO_HISTORY'

    @property
    def samples(self):
        return tuple(self._samples)

    def invalidate(self, reason):
        self._samples.clear()
        self.context = None
        self.last_reset_reason = reason

    def append(self, pose, stamp_sec, context, *, localization_valid=False):
        if not localization_valid:
            self.invalidate('LOCALIZATION_INVALID')
            return False
        if not all(math.isfinite(v) for v in
                   (pose.x, pose.y, pose.yaw, stamp_sec)) or stamp_sec <= 0:
            self.invalidate('INVALID_POSE')
            return False
        if context != self.context:
            self.invalidate('CONTEXT_CHANGED')
            self.context = context
        if self._samples:
            previous = self._samples[-1]
            dt = stamp_sec - previous.stamp_sec
            yaw_delta = wrap(pose.yaw - previous.pose.yaw)
            pose = Pose2D(pose.x, pose.y, previous.pose.yaw + yaw_delta)
            distance = math.hypot(pose.x - previous.pose.x,
                                  pose.y - previous.pose.y)
            reason = None
            if dt <= 0 or dt > self.limits.max_gap_sec:
                reason = 'TRACE_TIME_GAP'
            elif (distance > self.limits.max_speed_m_s * dt
                  + self.limits.position_noise_m
                  or abs(yaw_delta) > self.limits.max_yaw_rate_rad_s * dt
                  + self.limits.yaw_noise_rad):
                reason = 'POSE_JUMP'
            elif not forward_segment(previous.pose, pose, self.limits):
                reason = 'NON_FORWARD_TRACE'
            if reason:
                self.invalidate(reason)
                self.context = context
                self._samples.append(Breadcrumb(pose, stamp_sec))
                return False
        self._samples.append(Breadcrumb(pose, stamp_sec))
        self._samples = [sample for sample in self._samples
                         if stamp_sec - sample.stamp_sec
                         <= self.limits.max_age_sec][-self.limits.max_samples:]
        return True


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def forward_segment(start, end, limits):
    dx, dy = end.x - start.x, end.y - start.y
    heading = (start.yaw + end.yaw) * 0.5
    longitudinal = dx * math.cos(heading) + dy * math.sin(heading)
    lateral = -dx * math.sin(heading) + dy * math.cos(heading)
    return (longitudinal >= -1e-6
            and abs(lateral) <= limits.lateral_tolerance_m)


@dataclass(frozen=True)
class RetreatLimits:
    # These are development bounds, not measured hardware stopping distances.
    # The caller must supply a calibrated positive braking allowance.
    stopping_distance_m: float
    max_distance_m: float = 0.20
    min_distance_m: float = 0.03
    max_duration_sec: float = 30.0
    linear_speed_m_s: float = 0.05
    angular_speed_rad_s: float = 0.40
    current_position_tolerance_m: float = 0.02
    current_yaw_tolerance_rad: float = 0.05
    endpoint_rotation_rad: float = math.pi
    safety_margin_m: float = 0.0

    def __post_init__(self):
        values = vars(self)
        if any(not math.isfinite(v) or v < 0 for v in values.values()):
            raise ValueError('retreat limits must be finite and nonnegative')
        if (self.stopping_distance_m <= 0 or self.linear_speed_m_s <= 0
                or self.angular_speed_rad_s <= 0
                or self.min_distance_m <= 0
                or self.max_distance_m < self.min_distance_m
                or self.max_duration_sec <= 0
                or self.endpoint_rotation_rad <= 0):
            raise ValueError('invalid retreat range, speed or braking')
        if self.linear_speed_m_s > 0.10 or self.angular_speed_rad_s > 0.50:
            raise ValueError('retreat speeds exceed real navigation limits')
        if self.max_distance_m > 0.20 or self.max_duration_sec > 30.0:
            raise ValueError('retreat exceeds per-goal recovery bounds')


@dataclass(frozen=True)
class RetreatCandidate:
    context: TraceContext
    path: Tuple[Pose2D, ...]
    distance_m: float
    minimum_duration_sec: float
    check: CheckResult
    obstacle_source: str = ''
    strategy: str = 'RETRACE_MEASURED_PATH'

    @property
    def geometry_clear(self):
        return self.check.safe


def evaluate_trace_retreat(
    history: PoseHistory,
    context: TraceContext,
    current_pose: Pose2D,
    local_map: CostmapSnapshot,
    static_map: CostmapSnapshot,
    observations: Optional[ObservedFreeSpaceSnapshot],
    footprint,
    freshness: SnapshotFreshness,
    limits: RetreatLimits,
    *,
    distance_remaining_m: float,
    time_remaining_sec: float,
    deadline_monotonic=None,
    held_trace_max_age_sec=None,
):
    """Return nearest-first candidates and a stable overall rejection reason.

    This is geometry evaluation, not motion eligibility. The executor still
    needs controller failure attribution, exclusive ownership, actual stopped
    odometry, per-cycle rechecks and a calibrated total timeout. The original
    goal ID travels with every candidate; no replacement goal is generated.
    """
    if (not math.isfinite(distance_remaining_m)
            or not math.isfinite(time_remaining_sec)
            or distance_remaining_m <= 0 or time_remaining_sec <= 0):
        return (), 'BUDGET_EXHAUSTED'
    fresh = check_snapshot_freshness(local_map, freshness)
    if not fresh.safe:
        return (), fresh.reason
    if history.context != context:
        return (), 'TRACE_CONTEXT_MISMATCH'
    if any(grid.frame_id != context.frame_id
           for grid in (local_map, static_map)):
        return (), 'FRAME_MISMATCH'
    samples = history.samples
    if len(samples) < 2:
        return (), 'NO_HISTORY'
    ages = [freshness.now_ros_sec - sample.stamp_sec for sample in samples]
    max_trace_age = history.limits.max_gap_sec
    if held_trace_max_age_sec is not None:
        if not math.isfinite(held_trace_max_age_sec) or not 0 < held_trace_max_age_sec <= 2.0:
            return (), 'INVALID_HELD_TRACE_AGE'
        max_trace_age = held_trace_max_age_sec
    if min(ages) < 0 or ages[-1] > max_trace_age:
        return (), 'STALE_TRACE'
    samples = tuple(sample for sample, age in zip(samples, ages)
                    if age <= history.limits.max_age_sec)
    if len(samples) < 2:
        return (), 'NO_HISTORY'
    last = samples[-1].pose
    if (math.hypot(current_pose.x - last.x, current_pose.y - last.y)
            > limits.current_position_tolerance_m
            or abs(wrap(current_pose.yaw - last.yaw))
            > limits.current_yaw_tolerance_rad):
        return (), 'TRACE_START_MISMATCH'
    if observations is None:
        return (), 'NO_OBSERVED_REAR_CLEARANCE'
    if observations.frame_id != context.frame_id:
        return (), 'FRAME_MISMATCH'

    # Include the short measured-pose discrepancy in all checks; never jump
    # directly to an old breadcrumb or shortcut across a corner.
    current = Pose2D(current_pose.x, current_pose.y,
                     last.yaw + wrap(current_pose.yaw - last.yaw))
    if not forward_segment(last, current, history.limits):
        return (), 'TRACE_START_MISMATCH'
    path = [current, last]
    distance = math.hypot(current.x - last.x, current.y - last.y)
    angle = abs(current.yaw - last.yaw)
    output = []
    for sample in reversed(samples[:-1]):
        previous = path[-1]
        path.append(sample.pose)
        distance += math.hypot(previous.x - sample.pose.x,
                               previous.y - sample.pose.y)
        angle += abs(previous.yaw - sample.pose.yaw)
        reserved_distance = distance + limits.stopping_distance_m
        if reserved_distance > min(limits.max_distance_m,
                                   distance_remaining_m):
            break
        if distance < limits.min_distance_m:
            continue
        duration = (distance / limits.linear_speed_m_s
                    + angle / limits.angular_speed_rad_s)
        if duration > min(limits.max_duration_sec, time_remaining_sec):
            break
        target = path[-1]
        stop = Pose2D(
            target.x - limits.stopping_distance_m * math.cos(target.yaw),
            target.y - limits.stopping_distance_m * math.sin(target.yaw),
            target.yaw)
        protected_path = tuple(path) + (stop,)
        result, source = _check_retreat(
            local_map, static_map, observations, footprint, protected_path,
            freshness, limits, deadline_monotonic)
        output.append(RetreatCandidate(
            context, tuple(path), distance, duration, result, source))
        if result.reason == 'COMPUTE_BUDGET_EXCEEDED':
            break
    if any(candidate.geometry_clear for candidate in output):
        return tuple(output), 'CANDIDATE_AVAILABLE'
    return tuple(output), ('NO_SAFE_TRACE_RETREAT' if output
                           else 'BUDGET_OR_HISTORY_TOO_SHORT')


def _check_retreat(local, static, observations, footprint, path,
                   freshness, limits, deadline):
    for label, grid in (('local_costmap', local), ('static_map', static)):
        result = check_swept_path(
            grid, footprint, path, limits.safety_margin_m,
            deadline_monotonic=deadline)
        if not result.safe:
            return result, label
    observed = check_observed_free_path(
        observations, footprint, path, freshness, limits.safety_margin_m,
        deadline_monotonic=deadline)
    if not observed.safe:
        return observed, 'recent_sensor_visibility'
    # Both sides of the historical stopping point must remain rotatable now.
    # The last pose is braking overrun; the intended endpoint is penultimate.
    target = path[-2]
    for direction in (-1, 1):
        rotation = (target, Pose2D(target.x, target.y,
                                   target.yaw + direction
                                   * limits.endpoint_rotation_rad))
        for label, grid in (('local_costmap', local), ('static_map', static)):
            result = check_swept_path(
                grid, footprint, rotation, limits.safety_margin_m,
                deadline_monotonic=deadline)
            if not result.safe:
                if result.reason == 'COMPUTE_BUDGET_EXCEEDED':
                    return result, label
                return CheckResult(False, 'REFUGE_NOT_ROTATABLE',
                                   blocked_cell=result.blocked_cell), label
        visible_rotation = check_observed_free_path(
            observations, footprint, rotation, freshness,
            limits.safety_margin_m, deadline_monotonic=deadline)
        if not visible_rotation.safe:
            return visible_rotation, 'recent_sensor_visibility'
    return CheckResult(True, 'GEOMETRY_CLEAR_EXECUTION_NOT_AUTHORIZED'), ''
