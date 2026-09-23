"""Deterministic evidence-constrained actuator and observation models."""

from collections import deque
from dataclasses import dataclass
import math
import random

import yaml


def load_evidence_profile(path):
    """Load and minimally validate the generated evidence profile."""
    with open(path, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if profile.get("schema_version") != 1:
        raise ValueError("unsupported evidence profile schema")
    if not str(profile.get("status", "")).startswith("EVIDENCE_CONSTRAINED"):
        raise ValueError("evidence profile status is not accepted")
    return profile


def _approach_exponential(current, target, dt_s, time_constant_s):
    if time_constant_s <= 0.0:
        return target
    alpha = 1.0 - math.exp(-dt_s / time_constant_s)
    return current + alpha * (target - current)


class EvidenceActuatorModel:
    """Apply measured delay, finite-duration gain and braking response."""

    def __init__(self, profile):
        self.profile = profile
        self.elapsed_s = 0.0
        self.linear_mps = 0.0
        self.angular_rad_s = 0.0
        self._history = deque([(0.0, 0.0, 0.0)])

    def reset(self):
        self.elapsed_s = 0.0
        self.linear_mps = 0.0
        self.angular_rad_s = 0.0
        self._history.clear()
        self._history.append((0.0, 0.0, 0.0))

    def update(self, linear_mps, angular_rad_s, dt_s):
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        self.elapsed_s += dt_s
        self._history.append(
            (self.elapsed_s, float(linear_mps), float(angular_rad_s))
        )
        sample_time = self.elapsed_s - self.profile["command_latency_s"]
        while len(self._history) > 1 and self._history[1][0] <= sample_time:
            self._history.popleft()
        _, delayed_linear, delayed_angular = self._history[0]
        linear_gain = self.profile["linear_gain"][
            "forward" if delayed_linear >= 0.0 else "reverse"
        ]
        yaw_gain = self.profile["yaw_gain"][
            "left" if delayed_angular >= 0.0 else "right_after_compensation"
        ]
        target_linear = delayed_linear * linear_gain
        target_angular = delayed_angular * yaw_gain
        stopping = abs(target_linear) < 1.0e-12 and abs(target_angular) < 1.0e-12
        tau = self.profile[
            "stop_time_constant_s" if stopping else "rise_time_constant_s"
        ]
        self.linear_mps = _approach_exponential(
            self.linear_mps, target_linear, dt_s, tau
        )
        self.angular_rad_s = _approach_exponential(
            self.angular_rad_s, target_angular, dt_s, tau
        )
        threshold = self.profile["stationary_threshold_mps"]
        if stopping and abs(self.linear_mps) < threshold:
            self.linear_mps = 0.0
        angular_threshold = 2.0 * threshold / 0.254
        if stopping and abs(self.angular_rad_s) < angular_threshold:
            self.angular_rad_s = 0.0
        return self.linear_mps, self.angular_rad_s


@dataclass(frozen=True)
class EncoderSample:
    sequence: int
    device_stamp_us: int
    left_ticks: int
    right_ticks: int
    duplicate: bool = False


class EncoderObservationModel:
    """Quantize wheel motion and reproduce sparse drops/duplicates."""

    def __init__(self, profile):
        self.profile = profile
        self.random = random.Random(profile["seed"])
        self.period_s = 1.0 / profile["publish_rate_hz"]
        self.elapsed_s = 0.0
        self.next_publish_s = self.period_s
        self.left_revolutions = 0.0
        self.right_revolutions = 0.0
        self.sequence = 0
        self.last_sample = None

    def update(self, left_rad_s, right_rad_s, dt_s):
        self.elapsed_s += dt_s
        self.left_revolutions += left_rad_s * dt_s / (2.0 * math.pi)
        self.right_revolutions += right_rad_s * dt_s / (2.0 * math.pi)
        if self.elapsed_s + 1.0e-12 < self.next_publish_s:
            return None
        self.next_publish_s += self.period_s
        if self.random.random() < self.profile["drop_fraction"]:
            return None
        if (
            self.last_sample is not None
            and self.random.random() < self.profile["duplicate_fraction"]
        ):
            return EncoderSample(
                sequence=self.last_sample.sequence,
                device_stamp_us=self.last_sample.device_stamp_us,
                left_ticks=self.last_sample.left_ticks,
                right_ticks=self.last_sample.right_ticks,
                duplicate=True,
            )
        self.sequence += 1
        counts = self.profile["counts_per_revolution"]
        sample = EncoderSample(
            sequence=self.sequence,
            device_stamp_us=round(self.elapsed_s * 1.0e6),
            left_ticks=round(self.left_revolutions * counts),
            right_ticks=round(self.right_revolutions * counts),
        )
        self.last_sample = sample
        return sample


class ImuObservationModel:
    """Generate deterministic MID-360 yaw-rate residual bias and noise."""

    def __init__(self, profile):
        self.profile = profile
        self.random = random.Random(profile["seed"])

    def yaw_rate(self, true_yaw_rate_rad_s):
        return (
            true_yaw_rate_rad_s
            + self.profile["gyro_z_residual_bias_rad_s"]
            + self.random.gauss(
                0.0, self.profile["gyro_z_noise_stddev_rad_s"]
            )
        )
