"""Thread-safe command lease used by the Jetson web teleop node."""

from dataclasses import dataclass
import math
import threading
import time


@dataclass(frozen=True)
class VelocityCommand:
    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def moving(self) -> bool:
        return self.linear_x != 0.0 or self.angular_z != 0.0


class CommandLease:
    """Accept bounded commands only while armed and their lease is fresh."""

    def __init__(
        self, max_linear, max_angular, timeout_s, clock=time.monotonic
    ):
        if max_linear <= 0.0 or max_angular <= 0.0 or timeout_s <= 0.0:
            raise ValueError("velocity limits and timeout must be positive")
        self._max_linear = float(max_linear)
        self._max_angular = float(max_angular)
        self._timeout_s = float(timeout_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._armed = False
        self._command = VelocityCommand()
        self._deadline = 0.0

    @property
    def armed(self):
        with self._lock:
            return self._armed

    def set_armed(self, armed):
        with self._lock:
            self._armed = bool(armed)
            self._command = VelocityCommand()
            self._deadline = 0.0

    def submit(self, linear_x, angular_z):
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            raise ValueError("velocity must be finite")
        if abs(linear_x) > self._max_linear:
            raise ValueError("linear velocity exceeds configured limit")
        if abs(angular_z) > self._max_angular:
            raise ValueError("angular velocity exceeds configured limit")

        with self._lock:
            if not self._armed:
                raise RuntimeError("web teleop is not armed")
            self._command = VelocityCommand(linear_x, angular_z)
            self._deadline = self._clock() + self._timeout_s

    def sample(self):
        with self._lock:
            if not self._armed or self._clock() > self._deadline:
                self._command = VelocityCommand()
            return self._command
