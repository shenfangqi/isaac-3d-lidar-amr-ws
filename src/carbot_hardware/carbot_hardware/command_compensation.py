"""Pure helpers for calibrated Carbot velocity command compensation."""

import math


def compensate_angular_z(angular_z, right_turn_scale):
    """Scale negative (right-turn) yaw commands and preserve other values."""
    angular_z = float(angular_z)
    right_turn_scale = float(right_turn_scale)
    if not math.isfinite(angular_z):
        raise ValueError("angular_z must be finite")
    if (
        not math.isfinite(right_turn_scale)
        or not 0.0 < right_turn_scale <= 1.0
    ):
        raise ValueError("right_turn_scale must be finite and in (0, 1]")
    return angular_z * right_turn_scale if angular_z < 0.0 else angular_z
