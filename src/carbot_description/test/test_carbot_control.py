import importlib.util
import math
from pathlib import Path

import pytest
import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
CONTROL_PATH = WORKSPACE / "isaac_sim" / "carbot_control.py"
PARAMETER_PATH = DESCRIPTION_ROOT / "config" / "carbot_parameters.yaml"

spec = importlib.util.spec_from_file_location("carbot_control", CONTROL_PATH)
carbot_control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(carbot_control)


@pytest.fixture
def limits():
    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    return carbot_control.ControlLimits.from_parameters(parameters)


def test_body_limits_and_acceleration_are_applied_before_wheel_limit(limits):
    limiter = carbot_control.CarbotCommandLimiter(limits)
    command = limiter.update(5.0, 20.0, command_age_s=0.0, dt_s=0.02)
    assert command.requested_linear_mps == limits.max_linear_velocity_mps
    assert command.requested_angular_rad_s == limits.max_angular_velocity_rad_s
    assert command.applied_linear_mps == pytest.approx(0.01)
    assert command.applied_angular_rad_s == pytest.approx(0.05)


def test_coupled_saturation_preserves_curvature(limits):
    limiter = carbot_control.CarbotCommandLimiter(limits)
    limiter.linear_mps = limits.max_linear_velocity_mps
    limiter.angular_rad_s = limits.max_angular_velocity_rad_s
    command = limiter.update(
        limits.max_linear_velocity_mps,
        limits.max_angular_velocity_rad_s,
        command_age_s=0.0,
        dt_s=0.02,
    )
    assert command.wheel_scale < 1.0
    assert max(
        abs(command.left_wheel_rad_s), abs(command.right_wheel_rad_s)
    ) == pytest.approx(limits.max_wheel_velocity_rad_s)
    assert command.applied_angular_rad_s / command.applied_linear_mps == (
        pytest.approx(
            limits.max_angular_velocity_rad_s / limits.max_linear_velocity_mps
        )
    )


def test_watchdog_replaces_stale_command_with_zero_target(limits):
    limiter = carbot_control.CarbotCommandLimiter(limits)
    limiter.linear_mps = 0.2
    limiter.angular_rad_s = -0.2
    command = limiter.update(0.4, 1.0, command_age_s=0.501, dt_s=0.02)
    assert command.watchdog_active
    assert command.requested_linear_mps == 0.0
    assert command.requested_angular_rad_s == 0.0
    assert command.applied_linear_mps < 0.2
    assert command.applied_angular_rad_s > -0.2


def test_ideal_sim_has_no_straight_trim(limits):
    limiter = carbot_control.CarbotCommandLimiter(limits)
    limiter.linear_mps = 0.2
    command = limiter.update(0.2, 0.0, command_age_s=0.0, dt_s=0.02)
    assert command.left_wheel_rad_s == pytest.approx(command.right_wheel_rad_s)
    assert command.applied_angular_rad_s == pytest.approx(0.0)


def test_encoder_tick_semantics_are_positive_and_reversible(limits):
    counts = limits.encoder_counts_per_revolution
    assert carbot_control.radians_to_ticks(2.0 * math.pi, counts) == 1560
    assert carbot_control.radians_to_ticks(-2.0 * math.pi, counts) == -1560
    assert carbot_control.ticks_to_radians(1560, counts) == pytest.approx(
        2.0 * math.pi
    )


def test_positive_linear_command_produces_positive_track_rates(limits):
    left, right = carbot_control.body_to_wheels(0.1, 0.0, limits)
    assert left > 0.0
    assert right > 0.0
    assert left == pytest.approx(right)


def test_positive_angular_command_has_faster_right_track(limits):
    left, right = carbot_control.body_to_wheels(0.0, 0.5, limits)
    assert left < 0.0 < right


def test_calibrated_right_turn_matches_left_turn_in_isaac(limits):
    left_limiter = carbot_control.CarbotCommandLimiter(limits)
    right_limiter = carbot_control.CarbotCommandLimiter(limits)
    left = left_limiter.update(0.0, 0.3, command_age_s=0.0, dt_s=1.0)
    right = right_limiter.update(0.0, -0.3, command_age_s=0.0, dt_s=1.0)
    assert right.applied_angular_rad_s == pytest.approx(
        -left.applied_angular_rad_s
    )
