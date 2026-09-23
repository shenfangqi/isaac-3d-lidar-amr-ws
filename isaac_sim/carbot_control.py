"""Pure Carbot differential-drive command limiting and conversion helpers."""

from dataclasses import dataclass
import math


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def approach(current, target, maximum_change):
    return current + clamp(target - current, -maximum_change, maximum_change)


@dataclass(frozen=True)
class ControlLimits:
    effective_track_separation_m: float
    effective_sprocket_radius_m: float
    encoder_counts_per_revolution: int
    max_wheel_velocity_rad_s: float
    max_linear_velocity_mps: float
    max_angular_velocity_rad_s: float
    max_linear_acceleration_mps2: float
    max_angular_acceleration_rad_s2: float
    cmd_vel_timeout_s: float
    right_turn_command_scale: float
    right_turn_response_gain: float
    right_straight_trim: float = 1.0

    @classmethod
    def from_parameters(cls, parameters):
        kinematics = parameters["kinematics"]
        control = parameters["control"]
        observed_motion = parameters["hardware_response"]["observed_motion"]
        return cls(
            effective_track_separation_m=kinematics[
                "effective_track_separation_m"
            ],
            effective_sprocket_radius_m=kinematics[
                "effective_sprocket_radius_m"
            ],
            encoder_counts_per_revolution=kinematics[
                "encoder_counts_per_revolution"
            ],
            max_wheel_velocity_rad_s=control["max_wheel_velocity_rad_s"],
            max_linear_velocity_mps=control["max_linear_velocity_mps"],
            max_angular_velocity_rad_s=control["max_angular_velocity_rad_s"],
            max_linear_acceleration_mps2=control[
                "max_linear_acceleration_mps2"
            ],
            max_angular_acceleration_rad_s2=control[
                "max_angular_acceleration_rad_s2"
            ],
            cmd_vel_timeout_s=control["cmd_vel_timeout_s"],
            right_turn_command_scale=control["right_turn_command_scale"],
            right_turn_response_gain=observed_motion[
                "right_turn_response_gain_vs_left"
            ],
            right_straight_trim=control["ideal_sim_right_straight_trim"],
        )


@dataclass(frozen=True)
class LimitedCommand:
    requested_linear_mps: float
    requested_angular_rad_s: float
    applied_linear_mps: float
    applied_angular_rad_s: float
    left_wheel_rad_s: float
    right_wheel_rad_s: float
    wheel_scale: float
    watchdog_active: bool


class CarbotCommandLimiter:
    def __init__(self, limits):
        self.limits = limits
        self.linear_mps = 0.0
        self.angular_rad_s = 0.0

    def reset(self):
        self.linear_mps = 0.0
        self.angular_rad_s = 0.0

    def update(self, linear_mps, angular_rad_s, command_age_s, dt_s):
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")

        watchdog_active = command_age_s > self.limits.cmd_vel_timeout_s
        requested_linear = 0.0 if watchdog_active else linear_mps
        requested_angular = 0.0 if watchdog_active else angular_rad_s
        requested_linear = clamp(
            requested_linear,
            -self.limits.max_linear_velocity_mps,
            self.limits.max_linear_velocity_mps,
        )
        requested_angular = clamp(
            requested_angular,
            -self.limits.max_angular_velocity_rad_s,
            self.limits.max_angular_velocity_rad_s,
        )

        self.linear_mps = approach(
            self.linear_mps,
            requested_linear,
            self.limits.max_linear_acceleration_mps2 * dt_s,
        )
        self.angular_rad_s = approach(
            self.angular_rad_s,
            requested_angular,
            self.limits.max_angular_acceleration_rad_s2 * dt_s,
        )

        compensated_angular = self.angular_rad_s
        if compensated_angular < 0.0:
            compensated_angular *= self.limits.right_turn_command_scale
            compensated_angular *= self.limits.right_turn_response_gain
        left, right = body_to_wheels(
            self.linear_mps, compensated_angular, self.limits
        )
        right *= self.limits.right_straight_trim
        left, right, wheel_scale = saturate_wheels(
            left, right, self.limits.max_wheel_velocity_rad_s
        )
        applied_linear, applied_angular = wheels_to_body(
            left, right, self.limits
        )
        return LimitedCommand(
            requested_linear_mps=requested_linear,
            requested_angular_rad_s=requested_angular,
            applied_linear_mps=applied_linear,
            applied_angular_rad_s=applied_angular,
            left_wheel_rad_s=left,
            right_wheel_rad_s=right,
            wheel_scale=wheel_scale,
            watchdog_active=watchdog_active,
        )


def body_to_wheels(linear_mps, angular_rad_s, limits):
    half_separation = limits.effective_track_separation_m / 2.0
    radius = limits.effective_sprocket_radius_m
    left = (linear_mps - angular_rad_s * half_separation) / radius
    right = (linear_mps + angular_rad_s * half_separation) / radius
    return left, right


def wheels_to_body(left_rad_s, right_rad_s, limits):
    radius = limits.effective_sprocket_radius_m
    linear = radius * (left_rad_s + right_rad_s) / 2.0
    angular = (
        radius
        * (right_rad_s - left_rad_s)
        / limits.effective_track_separation_m
    )
    return linear, angular


def saturate_wheels(left_rad_s, right_rad_s, maximum_rad_s):
    peak = max(abs(left_rad_s), abs(right_rad_s))
    scale = 1.0 if peak <= maximum_rad_s else maximum_rad_s / peak
    return left_rad_s * scale, right_rad_s * scale, scale


def radians_to_ticks(radians, encoder_counts_per_revolution):
    return round(radians * encoder_counts_per_revolution / (2.0 * math.pi))


def ticks_to_radians(ticks, encoder_counts_per_revolution):
    return ticks * 2.0 * math.pi / encoder_counts_per_revolution
