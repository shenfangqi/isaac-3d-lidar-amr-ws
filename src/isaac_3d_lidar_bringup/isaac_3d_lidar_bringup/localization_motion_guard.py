"""
Issue #13 PR3 motion guard core: the only startup-time velocity authority.

Pure logic, no ROS.  ``localization_motion_guard_node`` feeds it requests,
odometry, sweep verdicts and the localization emergency flag, calls
``tick`` every 50 ms and publishes the returned angular rate.

Invariants (spec 4.3, 6.3):

* A ROTATE is accepted only after a STOP handshake opened the session, only
  from that session with a newer sequence, only while stopped, and only if
  motion is permitted (policy guarded + ACCEPTED profile with matching
  hashes), the budget admits it, odometry is fresh and a fresh sweep
  verdict allows at least the requested angle.
* A repeated sequence only renews the lease; it never resets the target or
  the budget.  Older sequences are ignored.  Other sessions are a control
  conflict.
* Every tick re-checks lease, odometry age, sweep verdict, emergency,
  centre drift and budget; any failure outputs zero immediately.
* ``stopped`` is reported only from fresh odometry; RELEASE happens only
  once stopped and is terminal.
"""

from dataclasses import dataclass
import hashlib
import math

from .localization_contracts import (
    ContractError,
    DEFAULT_MAX_PROBE_SPEED_RAD_S,
    encode_motion_profile,
    GuardState,
    MotionOperation,
    MotionRequest,
    MotionStatus,
    RejectReason,
    SCHEMA_VERSION,
)
from .localization_rotation_policy import (
    CENTER_DRIFT_MARGIN,
    predicted_stop_angle,
    ProbeBudget,
    RotationProgress,
)


def profile_hash(profile):
    """Hash a ROTATE request must carry to bind it to the loaded profile."""
    return hashlib.sha256(encode_motion_profile(profile).encode()).hexdigest()


@dataclass(frozen=True)
class GuardConfig:
    """Guard timing and thresholds; motion values come from the profile."""

    request_timeout_s: float = 0.30
    sensor_freshness_s: float = 0.5
    sweep_freshness_s: float = 0.5
    max_speed_rad_s: float = DEFAULT_MAX_PROBE_SPEED_RAD_S
    max_accel_rad_s2: float = 2.0
    stop_linear_mps: float = 0.02
    stop_angular_rps: float = 0.03
    stopped_window_s: float = 0.5
    # FAST-LIO odometry has short speed spikes at rest; like the manager,
    # only motion lasting this long resets the stopped window.
    stationary_grace_s: float = 0.3
    yaw_margin_rad: float = math.radians(5.0)
    max_odom_position_jump_m: float = 0.20
    max_odom_yaw_jump_rad: float = 0.35
    # /carbot/status arrives at 2 Hz: three periods.
    chassis_status_max_age_s: float = 1.5
    # Odometry wobbles backwards by ~0.002 rad (2026-10-07); that region is
    # where the body just was, so a verdict may fall this short of the
    # remaining angle and still cover it.
    sweep_tolerance_rad: float = 0.02

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ContractError(f'{name} must be positive')
        if self.sensor_freshness_s > 0.5:
            raise ContractError('sensor_freshness_s cannot exceed 0.5 s')
        if self.max_speed_rad_s > DEFAULT_MAX_PROBE_SPEED_RAD_S:
            raise ContractError('max_speed_rad_s exceeds the probe limit')


@dataclass(frozen=True)
class OdomSample:
    """One odometry message: source stamp plus local receipt time."""

    stamp_ns: int
    x: float
    y: float
    yaw: float
    linear: float
    angular: float
    receipt_mono: float


@dataclass(frozen=True)
class SweepVerdict:
    """Result of evaluating ``delta_yaw`` from the pose at ``receipt_mono``."""

    delta_yaw: float
    allowed: bool
    reason: str
    receipt_mono: float


@dataclass(frozen=True)
class MotionPermission:
    """Launch-time verdict on whether this guard may ever move."""

    permitted: bool
    profile_hash: str
    latency_s: float = 0.0
    stop_tail_rad: float = 0.0
    center_drift_m: float = 0.0


class MotionGuardCore:
    """State machine behind the guard node; see the module docstring."""

    def __init__(self, config, permission, budget_limits):
        self.config = config
        self.permission = permission
        self.budget_limits = budget_limits
        self.state = GuardState.IDLE
        self.session = ''
        self.sequence = -1
        self.reason = ''
        self.budget = None
        self._lease_mono = None
        self._last_request = None
        self._pending = None
        self._odom = None
        self._sweep = None
        self._emergency = None          # None = never received
        self._chassis = None            # (connected, blocked, receipt)
        self._still_since = None
        self._moving_since = None
        self._release_pending = False
        self._progress = None
        self._segment_start = None
        self._target = 0.0
        self._command = 0.0
        self._last_tick = None
        # Sessions ended by an odometry reset/jump may never handshake again.
        self._dead_sessions = set()

    # -- inputs -----------------------------------------------------------

    def on_request(self, request, receipt_mono):
        """Apply one validated MotionRequest."""
        if self.state == GuardState.RELEASED:
            return
        if request.operation == MotionOperation.STOP:
            self._on_stop(request, receipt_mono)
            return
        if not self.session or request.session != self.session:
            # No handshake yet, or another session: never move.
            self._halt(RejectReason.CONTROL_CONFLICT)
            return
        if request.sequence < self.sequence:
            return
        if request.sequence == self.sequence:
            if request == self._last_request:
                self._lease_mono = receipt_mono
            else:
                self._halt(RejectReason.CONTROL_CONFLICT)
            return
        self.sequence = request.sequence
        self._last_request = request
        self._lease_mono = receipt_mono
        if request.operation == MotionOperation.RELEASE:
            # Repeats of this sequence only renew the lease, so remember the
            # release and complete it in tick once stopped.
            self._halt(None)
            self._release_pending = True
            return
        if self.state == GuardState.ROTATING:
            self._halt(RejectReason.CONTROL_CONFLICT)
            return
        # STOPPING is fine: _try_start waits for the settle window.
        self._release_pending = False
        self._pending = request

    def _on_stop(self, request, receipt_mono):
        self._release_pending = False
        if request.session in self._dead_sessions:
            self._halt(RejectReason.CONTROL_CONFLICT)
            return
        if request.session != self.session:
            # A new session (or a restart): fresh budget, no old actions.
            self.session = request.session
            self.budget = ProbeBudget(self.budget_limits, receipt_mono)
            self.reason = ''
        elif request.sequence < self.sequence:
            return
        self.sequence = request.sequence
        self._last_request = request
        self._lease_mono = receipt_mono
        self._halt(None)

    def on_odom(self, sample):
        previous = self._odom
        if previous is not None:
            if sample.stamp_ns == previous.stamp_ns:
                return
            jumped = (
                sample.stamp_ns < previous.stamp_ns
                or math.hypot(sample.x - previous.x, sample.y - previous.y)
                > self.config.max_odom_position_jump_m
                or abs(math.atan2(math.sin(sample.yaw - previous.yaw),
                                  math.cos(sample.yaw - previous.yaw)))
                > self.config.max_odom_yaw_jump_rad)
            if jumped:
                self._odom = sample
                self._invalidate_session(RejectReason.ODOM_JUMP)
                return
        self._odom = sample
        if self._progress is not None:
            step = self._progress.update(sample.yaw)
            if self.budget is not None:
                self.budget.charge(abs(step), 0.0)
        moving = (abs(sample.linear) > self.config.stop_linear_mps
                  or abs(sample.angular) > self.config.stop_angular_rps)
        if moving:
            if self._moving_since is None:
                self._moving_since = sample.receipt_mono
            if (sample.receipt_mono - self._moving_since
                    >= self.config.stationary_grace_s):
                self._still_since = None
        else:
            self._moving_since = None
            if self._still_since is None:
                self._still_since = sample.receipt_mono

    def on_sweep(self, verdict):
        self._sweep = verdict

    def on_chassis(self, connected, blocked, receipt_mono):
        """Apply one /carbot/status sample (agent link and motion block)."""
        self._chassis = (bool(connected), bool(blocked), receipt_mono)
        if blocked or not connected:
            self._halt(RejectReason.CHASSIS_BLOCKED)

    def on_emergency(self, active):
        self._emergency = bool(active)
        if self._emergency:
            self._halt(RejectReason.LOCALIZATION_FAULT)

    # -- queries ----------------------------------------------------------

    def sweep_query(self):
        """Angle the node should evaluate from the current pose, or None."""
        if self.state == GuardState.ROTATING:
            remaining = self._sweep_remaining()
            return remaining if remaining is not None else None
        if self._pending is not None:
            return self._pending.delta_yaw_rad
        return None

    def status(self, now_mono):
        def age(stamp):
            return None if stamp is None else round(
                max(0.0, now_mono - stamp), 3)
        progress = 0.0 if self._progress is None else (
            self._progress.signed_progress)
        travel = 0.0 if self.budget is None else self.budget.abs_yaw
        travel = max(travel, abs(progress))
        stopped = (self.state != GuardState.ROTATING
                   and self._stopped(now_mono))
        return MotionStatus(
            SCHEMA_VERSION, self.session, max(self.sequence, 0), self.state,
            self.reason, progress, travel, stopped,
            (('chassis', age(None if self._chassis is None
                             else self._chassis[2])),
             ('emergency', None if self._emergency is None else 0.0),
             ('odom', age(None if self._odom is None
                          else self._odom.receipt_mono)),
             ('request', age(self._lease_mono)),
             ('sweep', age(None if self._sweep is None
                           else self._sweep.receipt_mono))))

    # -- control ----------------------------------------------------------

    def tick(self, now_mono):
        """Advance the state machine; return the angular rate to publish."""
        dt = 0.0 if self._last_tick is None else max(
            0.0, min(now_mono - self._last_tick, 0.2))
        self._last_tick = now_mono
        if self.state == GuardState.RELEASED:
            return 0.0
        if self._pending is not None:
            self._try_start(self._pending, now_mono)
        if self.state == GuardState.ROTATING:
            self._check_rotation(now_mono)
        if self.state == GuardState.STOPPING and self._stopped(now_mono):
            self.state = GuardState.STOPPED
        if (self._release_pending and self.state == GuardState.STOPPED
                and self._stopped(now_mono)):
            self._release_pending = False
            self.state = GuardState.RELEASED
            self._command = 0.0
            return 0.0
        if self.state != GuardState.ROTATING:
            self._command = 0.0
            return 0.0
        target_rate = math.copysign(self.config.max_speed_rad_s,
                                    self._target)
        step = self.config.max_accel_rad_s2 * dt
        self._command += max(-step, min(step, target_rate - self._command))
        if self.budget is not None and self._command != 0.0:
            self.budget.charge(0.0, dt)
        return self._command

    def _try_start(self, request, now_mono):
        if not self._lease_alive(now_mono):
            self._pending = None
            return
        reason = None
        if not self.permission.permitted or (
                request.profile_hash != self.permission.profile_hash):
            reason = RejectReason.PROFILE_INVALID
        elif self._emergency is not False:
            reason = RejectReason.LOCALIZATION_FAULT
        elif not self._chassis_ok(now_mono):
            reason = RejectReason.CHASSIS_BLOCKED
        elif not self._odom_fresh(now_mono):
            reason = RejectReason.SENSOR_STALE
        elif not self._stopped(now_mono):
            return                       # wait for the settle window
        elif self.budget.admit(request.delta_yaw_rad, now_mono):
            reason = RejectReason.MOTION_BUDGET_EXHAUSTED
        else:
            verdict = self._sweep_reason(request.delta_yaw_rad, now_mono)
            if verdict is None:
                return                   # verdict for this angle pending
            if verdict:
                reason = RejectReason(verdict)
        self._pending = None
        if reason is not None:
            self.reason = reason.value
            return
        self.reason = ''
        self._target = request.delta_yaw_rad
        self._progress = RotationProgress(self._odom.yaw)
        self._segment_start = (self._odom.x, self._odom.y)
        self.budget.start_segment()
        self.state = GuardState.ROTATING

    def _check_rotation(self, now_mono):
        config = self.config
        if not self._lease_alive(now_mono):
            self._halt(RejectReason.CANCELED)
            return
        if self._emergency is not False:
            self._halt(RejectReason.LOCALIZATION_FAULT)
            return
        if not self._chassis_ok(now_mono):
            self._halt(RejectReason.CHASSIS_BLOCKED)
            return
        if not self._odom_fresh(now_mono):
            self._halt(RejectReason.SENSOR_STALE)
            return
        if self.budget.exhausted(now_mono):
            self._halt(RejectReason.MOTION_BUDGET_EXHAUSTED)
            return
        drift = math.hypot(self._odom.x - self._segment_start[0],
                           self._odom.y - self._segment_start[1])
        # Same margin as the sweep padding: the base origin circles the
        # rotation centre, so the measured drift scales with the angle.
        if drift > self.permission.center_drift_m * CENTER_DRIFT_MARGIN:
            self._halt(RejectReason.ODOM_JUMP)
            return
        remaining = self._target - self._progress.signed_progress
        stop_angle = predicted_stop_angle(
            self._odom.angular, self.permission.latency_s,
            self.permission.stop_tail_rad, config.yaw_margin_rad)
        if remaining * self._target <= 0.0 or abs(remaining) <= stop_angle:
            self._halt(None)             # segment complete
            return
        verdict = self._sweep_reason(self._sweep_remaining(), now_mono)
        if verdict is None:
            self._halt(RejectReason.SENSOR_STALE)
        elif verdict:
            self._halt(RejectReason(verdict))

    def _sweep_remaining(self):
        """
        Remaining angle the sweep must cover, never more than the target.

        Odometry can wobble slightly backwards at the start of a turn, which
        would make target - progress exceed the target (and pi/2) and void
        a verdict that covers the whole target (2026-10-07 real robot).
        """
        remaining = self._target - self._progress.signed_progress
        if remaining * self._target <= 0.0:
            return None
        return math.copysign(min(abs(remaining), abs(self._target)),
                             self._target)

    def _sweep_reason(self, angle, now_mono):
        """'' if a fresh verdict allows ``angle``; reason; None if none."""
        verdict = self._sweep
        if angle is None:
            return ''                    # nothing left to sweep
        if verdict is None or not (
                0.0 <= now_mono - verdict.receipt_mono
                <= self.config.sweep_freshness_s):
            return None
        if verdict.delta_yaw * angle <= 0.0 or (
                abs(verdict.delta_yaw) + self.config.sweep_tolerance_rad
                < abs(angle)):
            return None
        if verdict.allowed:
            return ''
        return verdict.reason or RejectReason.UNKNOWN_SWEEP.value

    # -- helpers ----------------------------------------------------------

    def _halt(self, reason):
        self._command = 0.0
        self._pending = None
        self._progress = None if self.state != GuardState.ROTATING else (
            self._progress)
        if reason is not None:
            self.reason = reason.value
        if self.state in (GuardState.ROTATING, GuardState.STOPPED,
                          GuardState.IDLE):
            self.state = GuardState.STOPPING

    def _invalidate_session(self, reason):
        """Odometry reset or jump: stop and require a new STOP handshake."""
        self._halt(reason)
        if self.session:
            self._dead_sessions.add(self.session)
        self.session = ''
        self.sequence = -1
        self._last_request = None
        self._lease_mono = None
        self.budget = None

    def _lease_alive(self, now_mono):
        return (self._lease_mono is not None
                and 0.0 <= now_mono - self._lease_mono
                <= self.config.request_timeout_s)

    def _chassis_ok(self, now_mono):
        """Fresh status, agent connected and motion not blocked."""
        if self._chassis is None:
            return False
        connected, blocked, receipt = self._chassis
        return (connected and not blocked
                and 0.0 <= now_mono - receipt
                <= self.config.chassis_status_max_age_s)

    def _odom_fresh(self, now_mono):
        return (self._odom is not None
                and 0.0 <= now_mono - self._odom.receipt_mono
                <= self.config.sensor_freshness_s)

    def _stopped(self, now_mono):
        return (self._odom_fresh(now_mono)
                and self._still_since is not None
                and now_mono - self._still_since
                >= self.config.stopped_window_s)


class ProbeLink:
    """
    Manager side of the lease protocol for one localization session.

    Every call that changes intent uses a new, strictly increasing
    sequence; ``request()`` is re-sent at 10 Hz unchanged, which the guard
    treats as a lease renewal only.
    """

    def __init__(self, session):
        self.session = session
        self.sequence = 0
        self._request = None
        self.stop()

    def _set(self, operation, delta=0.0, speed=0.0, profile=''):
        self.sequence += 1
        self._request = MotionRequest(SCHEMA_VERSION, self.session,
                                      self.sequence, operation, delta,
                                      speed, profile)
        return self._request

    def stop(self):
        return self._set(MotionOperation.STOP)

    def rotate(self, delta_yaw, speed, profile_hash):
        return self._set(MotionOperation.ROTATE, delta_yaw, speed,
                         profile_hash)

    def release(self):
        return self._set(MotionOperation.RELEASE)

    def request(self):
        return self._request

    @property
    def operation(self):
        return self._request.operation

    def owns(self, status):
        """Whether ``status`` reports on this link's latest request."""
        return (status is not None and status.session == self.session
                and status.sequence == self.sequence)
