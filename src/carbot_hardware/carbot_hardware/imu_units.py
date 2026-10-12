"""Unit and timestamp conversion helpers for IMU messages."""

from bisect import bisect_left, insort
from collections import deque
import math

STANDARD_GRAVITY_MPS2 = 9.80665


class StationaryBiasEstimator:
    """Estimate a scalar sensor bias from bounded stationary samples."""

    def __init__(self, min_samples, window_samples, max_abs_value):
        if min_samples <= 0:
            raise ValueError("min_samples must be positive")
        if window_samples < min_samples:
            raise ValueError("window_samples must be at least min_samples")
        if max_abs_value <= 0.0 or not math.isfinite(max_abs_value):
            raise ValueError("max_abs_value must be finite and positive")
        self._min_samples = int(min_samples)
        self._window = int(window_samples)
        self._samples = deque()
        # The same samples kept sorted, so the median costs O(1) per IMU
        # message instead of a full sort (200 Hz on the Jetson).
        self._ordered = []
        self._max_abs_value = float(max_abs_value)

    @property
    def sample_count(self):
        return len(self._samples)

    @property
    def ready(self):
        return self.sample_count >= self._min_samples

    @property
    def bias(self):
        if not self._samples:
            return 0.0
        ordered = self._ordered
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    def update(self, value, stationary):
        value = float(value)
        if (
            stationary
            and math.isfinite(value)
            and abs(value) <= self._max_abs_value
        ):
            if len(self._samples) == self._window:
                oldest = self._samples.popleft()
                del self._ordered[bisect_left(self._ordered, oldest)]
            self._samples.append(value)
            insort(self._ordered, value)
        return value - self.bias


def covariance_with_fallback_diagonal(covariance, diagonal):
    """Use an explicit diagonal when a sensor reports unknown covariance."""
    if len(covariance) != 9 or len(diagonal) != 3:
        raise ValueError("covariance must be 3x3 and diagonal must have 3 values")
    source = tuple(float(value) for value in covariance)
    if any(value != 0.0 for value in source):
        return source
    result = [0.0] * 9
    for index, value in enumerate(diagonal):
        value = float(value)
        if value <= 0.0 or not math.isfinite(value):
            raise ValueError("fallback covariance diagonal must be finite and positive")
        result[index * 4] = value
    return tuple(result)


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


def shift_ros_stamp(sec, nanosec, correction_s):
    """Add a signed correction to a ROS timestamp and normalize the result."""
    correction_s = float(correction_s)
    if not math.isfinite(correction_s):
        raise ValueError("timestamp correction must be finite")
    if not 0 <= int(nanosec) < 1_000_000_000:
        raise ValueError("nanosec must be in [0, 1e9)")
    corrected_ns = (
        int(sec) * 1_000_000_000
        + int(nanosec)
        + round(correction_s * 1_000_000_000)
    )
    if corrected_ns < 0:
        raise ValueError("corrected ROS timestamp must be non-negative")
    return divmod(corrected_ns, 1_000_000_000)
