import pytest

from carbot_hardware.imu_units import (
    STANDARD_GRAVITY_MPS2,
    acceleration_g_to_mps2,
    scale_covariance,
)


def test_acceleration_g_is_converted_to_ros_si_units():
    assert acceleration_g_to_mps2((0.0, -0.5, 1.0)) == pytest.approx(
        (0.0, -STANDARD_GRAVITY_MPS2 / 2.0, STANDARD_GRAVITY_MPS2)
    )


def test_acceleration_conversion_rejects_invalid_inputs():
    with pytest.raises(ValueError):
        acceleration_g_to_mps2((1.0, 2.0))
    with pytest.raises(ValueError):
        acceleration_g_to_mps2((1.0, 2.0, 3.0), gravity_mps2=0.0)


def test_covariance_is_scaled_by_gravity_squared():
    source = (1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0)
    converted = scale_covariance(source, 2.0)
    assert converted == pytest.approx(
        (4.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 12.0)
    )
