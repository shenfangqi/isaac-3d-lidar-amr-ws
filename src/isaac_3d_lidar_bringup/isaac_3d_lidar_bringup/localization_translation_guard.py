"""
Offline-testable leased linear probe controller, not wired to a ROS node.

The eventual single velocity-authority adapter must feed source-validated
odom, chassis/emergency status, and a newly evaluated remaining-path preview
on every tick. No publisher exists here. A rotation profile cannot authorize
this controller; its permission must come from a linear stopping calibration.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
import math

from .localization_contracts import ContractError, SE2, normalize_angle
from .localization_motion_guard import PoseSpeed
from .localization_translation_policy import TranslationPreview


@dataclass(frozen=True)
class LinearProfile:
    """Explicit linear calibration identity; defaults never imply acceptance."""

    geometry_hash: str
    extrinsics_hash: str
    control_chain_hash: str
    evidence_ids: tuple
    max_speed_mps: float
    reaction_s: float
    stop_tail_m: float
    watchdog_s: float
    lateral_error_m: float
    status: str = 'ESTIMATED'

    def __post_init__(self):
        for value in (self.geometry_hash, self.extrinsics_hash, self.control_chain_hash):
            if (not isinstance(value, str) or len(value) != 64
                    or any(c not in '0123456789abcdef' for c in value)):
                raise ContractError('profile needs full geometry/extrinsics/control SHA256 hashes')
        object.__setattr__(self, 'evidence_ids', tuple(self.evidence_ids))
        if not self.evidence_ids or any(
                not isinstance(v, str) or not v for v in self.evidence_ids):
            raise ContractError('linear calibration needs evidence IDs')
        if self.status not in ('ESTIMATED', 'REVIEWED', 'ACCEPTED'):
            raise ContractError('invalid linear profile status')
        for name in ('max_speed_mps', 'reaction_s', 'stop_tail_m', 'watchdog_s',
                     'lateral_error_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ContractError(f'{name} must be positive')
        if self.max_speed_mps > .10:
            raise ContractError('probe speed exceeds 0.10 m/s development ceiling')

    @property
    def digest(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True,
                                         separators=(',', ':')).encode()).hexdigest()


@dataclass(frozen=True)
class LinearGuardConfig:
    lease_s: float = .30
    freshness_s: float = .50
    chassis_freshness_s: float = 1.5
    stopped_window_s: float = .5
    speed_window_s: float = .5
    stop_linear_mps: float = .01
    stop_angular_rps: float = .03
    max_acceleration_mps2: float = .1
    max_segment_m: float = .6
    max_total_travel_m: float = 1.5
    max_segment_s: float = 20.
    max_session_s: float = 360.
    stall_window_s: float = 3.
    min_progress_m: float = .015
    max_yaw_error_rad: float = .08
    max_odom_step_m: float = .20
    stop_margin_m: float = .02
    max_segments: int = 3

    def __post_init__(self):
        if any(isinstance(getattr(self, name), bool)
               or not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0
               for name in self.__dataclass_fields__):
            raise ContractError('linear guard limits must be positive')
        if self.freshness_s > .5 or type(self.max_segments) is not int:
            raise ContractError('invalid freshness or segment count')


class LinearProbeGuard:
    """
    Fail-closed pure state machine returning a linear speed recommendation.

    STOP opens a session while stationary. MOVE is accepted only with a new
    sequence and a matching accepted linear profile. Renewals cannot alter
    the target. Faults latch until the controller is explicitly reconstructed.
    RELEASE is terminal. External adapter must ensure one command publisher.
    """

    def __init__(self, profile=None, expected_hashes=None, config=LinearGuardConfig()):
        self.config = config
        self.profile = profile
        self.permitted = (isinstance(profile, LinearProfile) and profile.status == 'ACCEPTED'
                          and expected_hashes == (profile.geometry_hash, profile.extrinsics_hash,
                                                  profile.control_chain_hash))
        self.state, self.reason = 'IDLE', ''
        self.session, self.sequence = '', -1
        self._intent = None
        self._lease = self._session_start = self._last_tick = None
        self._odom = self._still_since = self._preview = None
        self._measured_speed = None
        self._emergency = self._chassis = None
        self._speed = PoseSpeed(config.speed_window_s)
        self._release = False
        self._start = None
        self._segment_start = self._progress_time = None
        self.progress_m = self.total_travel_m = self._progress_anchor = 0.
        self.segments = 0
        self.target_m = self.speed_mps = self.command_mps = 0.

    def _fault(self, reason):
        self.state, self.reason, self.command_mps = 'FAULT', reason, 0.

    def request(self, operation, session, sequence, now, *, distance_m=0., speed_mps=0.,
                profile_hash=''):
        if (operation not in ('STOP', 'MOVE', 'RELEASE') or not isinstance(session, str)
                or not session or type(sequence) is not int or sequence < 0
                or not math.isfinite(now)):
            raise ContractError('invalid linear request')
        if operation == 'MOVE':
            if not (math.isfinite(distance_m) and 0 < distance_m <= self.config.max_segment_m
                    and math.isfinite(speed_mps) and 0 < speed_mps <= .1):
                raise ContractError('invalid linear target/speed')
        elif distance_m != 0 or speed_mps != 0 or profile_hash:
            raise ContractError('STOP/RELEASE cannot contain a motion target')
        if self.state in ('FAULT', 'RELEASED'):
            return
        intent = (operation, session, sequence, distance_m, speed_mps, profile_hash)
        if not self.session:
            if operation != 'STOP':
                self._fault('STOP_HANDSHAKE_REQUIRED')
                return
            self.session, self._session_start = session, now
        if self.session != session:
            self._fault('CONTROL_CONFLICT')
            return
        if sequence < self.sequence:
            return
        if sequence == self.sequence:
            if intent != self._intent:
                self._fault('CONTROL_CONFLICT')
            else:
                self._lease = now
            return
        self.sequence, self._intent, self._lease = sequence, intent, now
        if operation in ('STOP', 'RELEASE'):
            self.state, self.command_mps = 'STOPPING', 0.
            self._release = operation == 'RELEASE'
            return
        if (not self.permitted or profile_hash != self.profile.digest
                or speed_mps > self.profile.max_speed_mps):
            self._fault('LINEAR_PROFILE_INVALID')
            return
        if self.state != 'STOPPED' or not self._stopped(now):
            self._fault('NOT_STOPPED')
            return
        if distance_m <= self.stop_extension_m + self.config.min_progress_m:
            self._fault('INSUFFICIENT_PROBE_DISTANCE')
            return
        if (self.segments >= self.config.max_segments
                or self.total_travel_m + distance_m + self.stop_extension_m
                > self.config.max_total_travel_m):
            self._fault('MOTION_BUDGET_EXHAUSTED')
            return
        self.target_m, self.speed_mps = distance_m, speed_mps
        self._start = SE2(self._odom.x, self._odom.y, self._odom.yaw)
        self._segment_start = self._progress_time = now
        self.progress_m = self._progress_anchor = 0.
        self.segments += 1
        self.state = 'MOVING'

    @property
    def stop_extension_m(self):
        if not self.permitted:
            return math.inf
        p, c = self.profile, self.config
        # Conservative serial upper bound: observation/lease delay, device
        # watchdog, calibrated reaction and residual stopping tail.
        return (p.max_speed_mps * (c.freshness_s + c.lease_s + p.watchdog_s + p.reaction_s)
                + p.stop_tail_m + c.stop_margin_m)

    def on_odom(self, sample):
        values = (sample.x, sample.y, sample.yaw, sample.linear, sample.angular,
                  sample.receipt_mono)
        if sample.stamp_ns <= 0 or not all(math.isfinite(v) for v in values):
            self._fault('INVALID_ODOMETRY')
            return
        previous = self._odom
        if previous is not None:
            if sample.stamp_ns == previous.stamp_ns:
                return
            step = math.hypot(sample.x - previous.x, sample.y - previous.y)
            if (sample.stamp_ns < previous.stamp_ns or step > self.config.max_odom_step_m
                    or abs(normalize_angle(sample.yaw - previous.yaw)) > .35):
                self._fault('ODOM_JUMP')
                return
            if self.state in ('MOVING', 'STOPPING') and self._start is not None:
                self.total_travel_m += step
        self._odom = sample
        speed = self._speed.update(sample.stamp_ns / 1e9, sample.x, sample.y)
        self._measured_speed = speed
        if (self.state == 'MOVING' and speed is not None
                and speed > self.profile.max_speed_mps + self.config.stop_linear_mps):
            self._fault('OVERSPEED')
        if (speed is None or speed > self.config.stop_linear_mps
                or abs(sample.angular) > self.config.stop_angular_rps):
            self._still_since = None
        elif self._still_since is None:
            self._still_since = sample.receipt_mono
        if self._start is not None:
            delta = self._start.inverse().compose(SE2(sample.x, sample.y, sample.yaw))
            self.progress_m = delta.x
            if self.state == 'MOVING' and (
                    abs(delta.y) > self.profile.lateral_error_m
                    or abs(delta.yaw) > self.config.max_yaw_error_rad
                    or delta.x < -self.config.stop_margin_m):
                self._fault('PATH_DEVIATION')

    def on_preview(self, preview, receipt_mono):
        if not isinstance(preview, TranslationPreview) or not math.isfinite(receipt_mono):
            self._fault('INVALID_SWEEP')
            return
        self._preview = (preview, receipt_mono)

    def on_emergency(self, active, receipt_mono):
        self._emergency = (active is False, receipt_mono)
        if active is not False:
            self._fault('LOCALIZATION_FAULT')

    def on_chassis(self, connected, blocked, receipt_mono):
        self._chassis = (connected is True and blocked is False, receipt_mono)
        if not self._chassis[0]:
            self._fault('CHASSIS_BLOCKED')

    def _fresh(self, receipt, now, limit):
        return receipt is not None and 0 <= now - receipt <= limit

    def _stopped(self, now):
        return (self._odom is not None
                and self._fresh(self._odom.receipt_mono, now, self.config.freshness_s)
                and self._still_since is not None
                and now - self._still_since >= self.config.stopped_window_s)

    def tick(self, now):
        if not math.isfinite(now) or self._last_tick is not None and now < self._last_tick:
            self._fault('CLOCK_RESET')
            return 0.
        dt = 0. if self._last_tick is None else min(now - self._last_tick, .2)
        self._last_tick = now
        if self.state in ('FAULT', 'RELEASED', 'IDLE'):
            return 0.
        if self.state == 'STOPPING':
            if self._stopped(now):
                self.state = 'RELEASED' if self._release else 'STOPPED'
            return 0.
        if self.state == 'STOPPED':
            return 0.
        reason = self._motion_fault(now)
        if reason:
            self._fault(reason)
            return 0.
        if self.progress_m >= self.target_m - self.stop_extension_m:
            self.command_mps, self.state = 0., 'STOPPING'
            return 0.
        if self.progress_m - self._progress_anchor >= self.config.min_progress_m:
            self._progress_anchor, self._progress_time = self.progress_m, now
        if now - self._progress_time > self.config.stall_window_s:
            self._fault('NO_PROGRESS')
            return 0.
        self.command_mps = min(
            self.speed_mps, self.command_mps + self.config.max_acceleration_mps2 * dt)
        return self.command_mps

    def _motion_fault(self, now):
        c = self.config
        if not self._fresh(self._lease, now, c.lease_s):
            return 'LEASE_EXPIRED'
        if self._emergency is None or not self._emergency[0]:
            return 'LOCALIZATION_FAULT'
        # Emergency is a latched state topic; its last false message need not
        # be periodic. Chassis/odom/scan must remain live.
        if (self._chassis is None or not self._chassis[0]
                or not self._fresh(self._chassis[1], now, c.chassis_freshness_s)):
            return 'CHASSIS_BLOCKED'
        if self._odom is None or not self._fresh(self._odom.receipt_mono, now, c.freshness_s):
            return 'SENSOR_STALE'
        if (now - self._session_start > c.max_session_s
                or now - self._segment_start > c.max_segment_s
                or self.total_travel_m >= c.max_total_travel_m):
            return 'MOTION_BUDGET_EXHAUSTED'
        if self._preview is None:
            return 'SWEEP_STALE'
        preview, receipt = self._preview
        if (not self._fresh(receipt, now, c.freshness_s)
                or abs(self._odom.stamp_ns - preview.snapshot_stamp_ns) / 1e9 > c.freshness_s):
            return 'SWEEP_STALE'
        pose = SE2(self._odom.x, self._odom.y, self._odom.yaw)
        # The adapter must resweep at the current pose; stale geometry cannot
        # follow a moving robot merely by refreshing the message receipt.
        if (math.hypot(preview.start.x - pose.x, preview.start.y - pose.y) > .005
                or abs(normalize_angle(preview.start.yaw - pose.yaw)) > .01):
            return 'SWEEP_POSE_MISMATCH'
        if not preview.geometry_clear:
            return preview.reason or 'UNKNOWN_SWEEP'
        if (preview.distance_m + 1e-9 < max(0., self.target_m - self.progress_m)
                or preview.stop_extension_m + 1e-9 < self.stop_extension_m):
            return 'SWEEP_TOO_SHORT'
        return ''


def combine_commands(rotation_state, angular_rad_s, linear_state, linear_mps):
    """
    Merge the rotation and translation cores into one command.

    Returns ``(linear, angular, conflict)``.  The two may never move at the
    same time: if both are active, both outputs are zero and ``conflict`` is
    True so the caller can fault both cores.  ``rotation_state`` is the
    rotation core's state value ('ROTATING' while active).
    """
    rotating = rotation_state == 'ROTATING' or angular_rad_s != 0.0
    moving = linear_state == 'MOVING' or linear_mps != 0.0
    if rotating and moving:
        return 0.0, 0.0, True
    return (float(linear_mps) if moving else 0.0,
            float(angular_rad_s) if rotating else 0.0, False)
