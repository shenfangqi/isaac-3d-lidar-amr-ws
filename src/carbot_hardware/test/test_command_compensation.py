import math

import pytest

from carbot_hardware.command_compensation import compensate_angular_z


def test_right_turn_is_scaled():
    assert compensate_angular_z(-0.3, 0.896) == pytest.approx(-0.2688)


def test_left_turn_and_zero_are_unchanged():
    assert compensate_angular_z(0.3, 0.896) == pytest.approx(0.3)
    assert compensate_angular_z(0.0, 0.896) == 0.0


@pytest.mark.parametrize("value", [math.inf, -math.inf, math.nan])
def test_nonfinite_command_is_rejected(value):
    with pytest.raises(ValueError):
        compensate_angular_z(value, 0.896)


@pytest.mark.parametrize("scale", [0.0, -0.1, 1.01, math.nan])
def test_invalid_scale_is_rejected(scale):
    with pytest.raises(ValueError):
        compensate_angular_z(-0.3, scale)
