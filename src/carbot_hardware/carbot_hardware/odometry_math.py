"""Pure differential-drive odometry math used by the hardware node."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the odometry frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class MotionDelta:
    """Integrated body motion for one pair of cumulative tick samples."""

    pose: Pose2D
    distance_m: float
    yaw_rad: float
    left_distance_m: float
    right_distance_m: float


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def integrate_tick_delta(
    pose: Pose2D,
    left_delta_ticks: int,
    right_delta_ticks: int,
    meters_per_tick: float,
    track_separation_m: float,
) -> MotionDelta:
    """Integrate one differential-drive increment using an exact planar arc."""

    if meters_per_tick <= 0.0:
        raise ValueError("meters_per_tick must be positive")
    if track_separation_m <= 0.0:
        raise ValueError("track_separation_m must be positive")

    left_m = left_delta_ticks * meters_per_tick
    right_m = right_delta_ticks * meters_per_tick
    distance_m = 0.5 * (left_m + right_m)
    yaw_delta = (right_m - left_m) / track_separation_m
    next_yaw_unwrapped = pose.yaw + yaw_delta

    if abs(yaw_delta) < 1.0e-9:
        next_x = pose.x + distance_m * math.cos(pose.yaw)
        next_y = pose.y + distance_m * math.sin(pose.yaw)
    else:
        arc_radius = distance_m / yaw_delta
        next_x = pose.x + arc_radius * (
            math.sin(next_yaw_unwrapped) - math.sin(pose.yaw)
        )
        next_y = pose.y - arc_radius * (
            math.cos(next_yaw_unwrapped) - math.cos(pose.yaw)
        )

    return MotionDelta(
        pose=Pose2D(next_x, next_y, wrap_angle(next_yaw_unwrapped)),
        distance_m=distance_m,
        yaw_rad=yaw_delta,
        left_distance_m=left_m,
        right_distance_m=right_m,
    )
