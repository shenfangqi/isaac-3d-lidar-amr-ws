"""Map ESP32 device time onto the Jetson clock (issue #97).

The ESP32 stamps each sample with a monotonic device clock whose intervals are
precise, but its ROS header follows a clock synchronized once per micro-ROS
session.  After a Jetson cold boot NTP steps the Jetson clock by hours after
that sync, and the slew-limited firmware resync never catches up (#95).

Transport only delays a message, never advances it, so the lower envelope of
``receive - device`` over a recent window is the clock offset plus the minimum
transport latency.  ``device + offset`` is then a sample time on the Jetson
clock that keeps the device's interval precision and is never in the future.
"""

from collections import deque
from dataclasses import dataclass


STAMP_MODES = ("esp32_header", "arrival", "device_clock")


def resolve_stamp_mode(stamp_mode: str, stamp_with_arrival_time: bool) -> str:
    """Return the wheel-odometry stamp mode; empty keeps the legacy flag."""

    if not stamp_mode:
        return "arrival" if stamp_with_arrival_time else "esp32_header"
    if stamp_mode not in STAMP_MODES:
        raise ValueError(
            f"stamp_mode must be one of {', '.join(STAMP_MODES)}: {stamp_mode}"
        )
    return stamp_mode


@dataclass(frozen=True)
class ClockMapperConfig:
    """Window and step-detection limits for DeviceClockMapper."""

    # Long enough to contain a low-latency sample, short enough to follow
    # crystal drift (50 ppm x 10 s = 0.5 ms).
    window_us: int = 10_000_000
    # A forward Jetson clock step shows as every sample arriving this late ...
    forward_jump_ns: int = 500_000_000
    # ... for this long in receive time.  A delayed batch (345 ms observed
    # with Wi-Fi power save on) arrives within milliseconds, so it never
    # spans this hold.
    forward_hold_ns: int = 500_000_000
    # A sample this much earlier than the current envelope means the Jetson
    # clock stepped backwards.
    backward_jump_ns: int = 50_000_000
    # Device time this far behind the newest sample means the device clock
    # restarted; smaller regressions are reordered packets.
    device_reset_us: int = 1_000_000


class DeviceClockMapper:
    """Track the device-to-Jetson clock offset for one ESP32 stream."""

    def __init__(self, config: ClockMapperConfig = ClockMapperConfig()):
        self._config = config
        self.resets = 0
        self.last_reset_reason = None
        self._boot_id = None
        self._clear()

    def _clear(self):
        # (device_us, raw_ns) with raw_ns increasing: front is the minimum.
        self._envelope = deque()
        self._newest_device_us = None
        self._last_stamp_ns = None
        self._late_since_ns = None

    def _reset(self, reason):
        self.resets += 1
        self.last_reset_reason = reason
        self._clear()

    @property
    def offset_ns(self):
        """Current Jetson-minus-device offset, or None before any sample."""
        return self._envelope[0][1] if self._envelope else None

    def update(self, boot_id: int, device_us: int, receive_ns: int) -> int:
        """Add one received sample and return its Jetson-clock stamp in ns."""

        config = self._config
        raw_ns = receive_ns - device_us * 1000

        if self._boot_id is not None and boot_id != self._boot_id:
            self._reset("boot_id_changed")
        self._boot_id = boot_id

        if self._newest_device_us is not None:
            if device_us < self._newest_device_us - config.device_reset_us:
                self._reset("device_time_regressed")
            elif device_us <= self._newest_device_us:
                # Duplicate or reordered packet: map it, do not learn from it.
                return min(device_us * 1000 + self.offset_ns, receive_ns)

        offset_ns = self.offset_ns
        if offset_ns is not None:
            residual_ns = raw_ns - offset_ns
            if residual_ns < -config.backward_jump_ns:
                self._reset("jetson_clock_stepped_back")
            elif residual_ns > config.forward_jump_ns:
                if self._late_since_ns is None:
                    self._late_since_ns = receive_ns
                elif receive_ns - self._late_since_ns >= config.forward_hold_ns:
                    self._reset("jetson_clock_stepped_forward")
            else:
                self._late_since_ns = None

        while self._envelope and self._envelope[-1][1] >= raw_ns:
            self._envelope.pop()
        self._envelope.append((device_us, raw_ns))
        while device_us - self._envelope[0][0] > config.window_us:
            self._envelope.popleft()
        self._newest_device_us = device_us

        stamp_ns = device_us * 1000 + self.offset_ns
        if self._last_stamp_ns is not None:
            stamp_ns = max(stamp_ns, self._last_stamp_ns + 1)
        # The current sample is inside the envelope, so this only guards the
        # monotonic adjustment above.
        stamp_ns = min(stamp_ns, receive_ns)
        self._last_stamp_ns = stamp_ns
        return stamp_ns
