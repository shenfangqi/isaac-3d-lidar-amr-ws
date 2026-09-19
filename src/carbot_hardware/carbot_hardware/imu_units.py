"""Unit conversion helpers for IMU messages."""

STANDARD_GRAVITY_MPS2 = 9.80665


def acceleration_g_to_mps2(values, gravity_mps2=STANDARD_GRAVITY_MPS2):
    """Convert a three-axis acceleration vector from g to SI units."""
    if gravity_mps2 <= 0.0:
        raise ValueError("gravity_mps2 must be positive")
    if len(values) != 3:
        raise ValueError("acceleration vector must contain exactly three values")
    return tuple(float(value) * gravity_mps2 for value in values)


def scale_covariance(covariance, scale):
    """Scale a 3x3 covariance when each underlying value is scaled."""
    if len(covariance) != 9:
        raise ValueError("covariance must contain exactly nine values")
    scale_squared = float(scale) ** 2
    return tuple(float(value) * scale_squared for value in covariance)
