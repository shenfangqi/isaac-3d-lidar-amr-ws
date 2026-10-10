import pytest

from carbot_hardware.imu_units import (
    STANDARD_GRAVITY_MPS2,
    StationaryBiasEstimator,
    acceleration_g_to_mps2,
    covariance_with_fallback_diagonal,
    scale_covariance,
    shift_ros_stamp,
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


def test_timestamp_correction_normalizes_across_second_boundary():
    assert shift_ros_stamp(100, 5_000_000, -0.009782937) == (
        99,
        995_217_063,
    )


def test_timestamp_correction_rejects_invalid_values():
    with pytest.raises(ValueError):
        shift_ros_stamp(1, 1_000_000_000, 0.0)
    with pytest.raises(ValueError):
        shift_ros_stamp(0, 0, -0.1)
    with pytest.raises(ValueError):
        shift_ros_stamp(1, 0, float("nan"))


def test_stationary_bias_estimator_uses_only_bounded_stationary_samples():
    estimator = StationaryBiasEstimator(3, 5, 0.03)
    assert estimator.update(0.004, stationary=False) == pytest.approx(0.004)
    assert estimator.update(0.2, stationary=True) == pytest.approx(0.2)
    assert estimator.update(0.003, stationary=True) == pytest.approx(0.0)
    assert estimator.update(0.005, stationary=True) == pytest.approx(0.001)
    assert not estimator.ready
    assert estimator.update(0.004, stationary=True) == pytest.approx(0.0)
    assert estimator.ready
    assert estimator.bias == pytest.approx(0.004)


def test_stationary_bias_estimator_matches_the_sorted_window_median():
    # The sorted window is kept incrementally (200 Hz on the Jetson); the
    # result must equal the median of exactly the last window samples.
    import random
    from collections import deque
    from statistics import median
    rng = random.Random(13)
    estimator = StationaryBiasEstimator(4, 50, 0.03)
    window = deque(maxlen=50)
    for _ in range(2000):
        value = rng.choice([rng.uniform(-0.04, 0.04), 0.01, -0.0, 0.0])
        stationary = rng.random() < 0.8
        corrected = estimator.update(value, stationary)
        if stationary and abs(value) <= 0.03:
            window.append(value)
        expected = median(window) if window else 0.0
        assert estimator.bias == expected
        assert corrected == value - expected
        assert estimator.sample_count == len(window)


def test_unknown_covariance_gets_explicit_fallback_diagonal():
    assert covariance_with_fallback_diagonal(
        [0.0] * 9, [1.0, 2.0, 3.0]
    ) == pytest.approx((1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0))


def test_reported_covariance_is_preserved():
    source = [0.0] * 9
    source[8] = 0.25
    assert covariance_with_fallback_diagonal(source, [1.0, 2.0, 3.0]) == tuple(
        source
    )
