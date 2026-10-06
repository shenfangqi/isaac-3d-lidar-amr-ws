"""
Issue #13 PR3 motion guard core (spec C01, C03, C05, C06, C08, C10).

A kinematic robot follows the commanded rate exactly; odometry arrives every
50 ms.  No ROS graph is used.
"""

import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    encode_motion_status,
    GuardState,
    MotionOperation,
    MotionRequest,
)
from isaac_3d_lidar_bringup.localization_motion_guard import (
    GuardConfig,
    MotionGuardCore,
    MotionPermission,
    OdomSample,
    SweepVerdict,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (
    ProbeBudgetLimits,
    RotationProgress,
)


PROFILE_HASH = 'c0ffee' * 4
SESSION = 'session0000000001'
PERMITTED = MotionPermission(True, PROFILE_HASH, latency_s=0.1,
                             stop_tail_rad=0.05, center_drift_m=0.03)


class Sim:
    """Guard + kinematic robot + 10 Hz request lease from a manager."""

    def __init__(self, permission=PERMITTED, limits=ProbeBudgetLimits(),
                 sweep_allowed=True):
        self.guard = MotionGuardCore(GuardConfig(), permission, limits)
        self.now = 100.0
        self.yaw = 0.0
        self.x = self.y = 0.0
        self.rate = 0.0
        self.stamp = 1_000_000_000
        self.sweep_allowed = sweep_allowed
        self.sweep_reason = 'UNKNOWN_SWEEP'
        self.feed_odom = True
        self.feed_sweep = True
        self.feed_chassis = True
        self.chassis_blocked = False
        self.lease = None               # request re-sent at 10 Hz
        self.sequence = 0
        self.commands = []
        self.guard.on_emergency(False)
        self._next_lease = self.now

    def request(self, operation, delta=0.0, session=SESSION, sequence=None,
                speed=None, profile=PROFILE_HASH):
        if sequence is None:
            self.sequence += 1
            sequence = self.sequence
        moving = operation == MotionOperation.ROTATE
        message = MotionRequest(
            1, session, sequence, operation, delta,
            (0.4 if speed is None else speed) if moving else 0.0,
            profile if moving else '')
        self.guard.on_request(message, self.now)
        self.lease = message
        return message

    def step(self, count=1, dt=0.05):
        for _ in range(count):
            self.now += dt
            self.yaw += self.rate * dt
            self.stamp += int(dt * 1e9)
            if self.lease is not None and self.now >= self._next_lease:
                self.guard.on_request(self.lease, self.now)
                self._next_lease = self.now + 0.1
            if self.feed_odom:
                self.guard.on_odom(OdomSample(
                    self.stamp, self.x, self.y,
                    math.atan2(math.sin(self.yaw), math.cos(self.yaw)),
                    0.0, self.rate, self.now))
            if self.feed_chassis:
                self.guard.on_chassis(True, self.chassis_blocked, self.now)
            query = self.guard.sweep_query()
            if self.feed_sweep and query is not None:
                self.guard.on_sweep(SweepVerdict(
                    query, self.sweep_allowed,
                    '' if self.sweep_allowed else self.sweep_reason,
                    self.now))
            self.rate = self.guard.tick(self.now)
            self.commands.append(self.rate)
        return self

    def handshake(self):
        self.request(MotionOperation.STOP)
        self.step(15)
        assert self.guard.state == GuardState.STOPPED
        return self

    @property
    def status(self):
        return self.guard.status(self.now)


def test_rotation_completes_early_by_the_predicted_stop_angle():
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(60))
    sim.step(2)
    assert sim.guard.state == GuardState.ROTATING
    sim.step(80)

    assert sim.guard.state == GuardState.STOPPED
    assert max(sim.commands) == pytest.approx(0.4)
    # Zero was commanded before the target, leaving room for the stop tail.
    assert sim.yaw < math.radians(60)
    assert sim.yaw > math.radians(60) - 0.04 - 0.05 - math.radians(5) - 0.03
    assert sim.status.stopped
    encode_motion_status(sim.status)            # contract-valid status


def test_yaw_wrap_and_noise():
    # C01: crossing +pi/-pi is continuous; noise spends budget only.
    progress = RotationProgress(math.pi - 0.05)
    progress.update(-math.pi + 0.05)
    assert progress.signed_progress == pytest.approx(0.10)
    noisy = RotationProgress(0.0)
    for value in (0.01, -0.01) * 50:
        noisy.update(value)
    assert noisy.signed_progress == pytest.approx(-0.01)
    assert noisy.abs_travel == pytest.approx(1.99)

    sim = Sim()
    sim.yaw = math.pi - 0.2             # placed facing just short of +pi
    sim.handshake()
    sim.request(MotionOperation.ROTATE, math.radians(45))
    sim.step(60)
    assert sim.guard.state == GuardState.STOPPED
    assert sim.status.signed_progress_rad > math.radians(25)


def test_cancel_and_lease_loss():
    # C03: STOP mid-rotation or a silent manager zeroes within the lease
    # limit plus one cycle; stopped needs real odometry afterwards.
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(10)
    sim.request(MotionOperation.STOP)
    sim.step(1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.state == GuardState.STOPPING

    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(10)
    sim.lease = None                   # manager stops refreshing
    sim.step(int((0.30 + 0.05) / 0.05) + 1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == 'CANCELED'

    sim.feed_odom = False               # no data: never claim stopped
    sim.step(20)
    assert not sim.status.stopped


@pytest.mark.parametrize('fault, reason', [
    ('odom', 'SENSOR_STALE'),
    ('sweep_missing', 'SENSOR_STALE'),
    ('sweep_rejects', 'OBSTACLE_IN_SWEEP'),
    ('emergency', 'LOCALIZATION_FAULT'),
])
def test_stale_tf_odom_emergency(fault, reason):
    # C05: each fault alone stops the rotation in that cycle (odometry
    # after its freshness limit) and never resumes on its own.
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(5)
    assert sim.guard.state == GuardState.ROTATING
    if fault == 'odom':
        sim.feed_odom = False
        sim.step(int(0.5 / 0.05) + 1)
    elif fault == 'sweep_missing':
        sim.feed_sweep = False
        sim.step(int(0.5 / 0.05) + 1)
    elif fault == 'sweep_rejects':
        sim.sweep_allowed = False
        sim.sweep_reason = 'OBSTACLE_IN_SWEEP'
        sim.step(1)
    else:
        sim.guard.on_emergency(True)
        sim.step(1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == reason
    sim.feed_odom = sim.feed_sweep = sim.sweep_allowed = True
    sim.step(30)
    assert sim.guard.state != GuardState.ROTATING
    assert max(sim.commands[-30:]) == 0.0


@pytest.mark.parametrize('fault', ['blocked', 'stale'])
def test_chassis_block_or_silence_stops(fault):
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(5)
    if fault == 'blocked':
        sim.chassis_blocked = True
        sim.step(1)
    else:
        sim.feed_chassis = False
        sim.step(int(1.5 / 0.05) + 1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == 'CHASSIS_BLOCKED'


def test_missing_chassis_status_forbids_motion():
    sim = Sim()
    sim.feed_chassis = False
    sim.handshake()
    sim.request(MotionOperation.ROTATE, math.radians(30))
    sim.step(10)
    assert max(sim.commands) == 0.0
    assert sim.guard.reason == 'CHASSIS_BLOCKED'


def test_unknown_emergency_state_forbids_motion():
    sim = Sim()
    sim.guard._emergency = None        # never received
    sim.handshake()
    sim.request(MotionOperation.ROTATE, math.radians(30))
    sim.step(10)
    assert max(sim.commands) == 0.0
    assert sim.guard.reason == 'LOCALIZATION_FAULT'


def test_duplicate_request():
    # C06: a repeat renews the lease without restarting; stale, mismatched
    # and foreign requests never move the robot.
    sim = Sim().handshake()
    first = sim.request(MotionOperation.ROTATE, math.radians(60))
    sim.step(30)
    progress = sim.status.signed_progress_rad
    sim.guard.on_request(first, sim.now)     # same sequence again
    sim.step(1)
    assert sim.status.signed_progress_rad >= progress

    sim.step(60)
    assert sim.guard.state == GuardState.STOPPED
    sim.guard.on_request(first, sim.now)     # replay after completion
    sim.step(20)
    assert max(sim.commands[-20:]) == 0.0
    assert sim.guard.budget.segments == 1

    stale = sim.request(MotionOperation.ROTATE, math.radians(30),
                        sequence=1)
    sim.step(10)
    assert max(sim.commands[-10:]) == 0.0
    assert stale.sequence < sim.guard.sequence

    sim.request(MotionOperation.ROTATE, math.radians(30),
                session='othersession00001')
    sim.step(10)
    assert max(sim.commands[-10:]) == 0.0
    assert sim.guard.reason == 'CONTROL_CONFLICT'

    fresh = Sim()                              # no STOP handshake yet
    fresh.request(MotionOperation.ROTATE, math.radians(30))
    fresh.step(20)
    assert max(fresh.commands) == 0.0


def test_conflicting_content_on_same_sequence_halts():
    sim = Sim().handshake()
    first = sim.request(MotionOperation.ROTATE, math.radians(60))
    sim.step(5)
    sim.request(MotionOperation.ROTATE, math.radians(-60),
                sequence=first.sequence)
    sim.step(1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == 'CONTROL_CONFLICT'


@pytest.mark.parametrize('permission, profile', [
    (MotionPermission(False, PROFILE_HASH), PROFILE_HASH),
    (PERMITTED, 'deadbeef' * 3),
])
def test_profile_mismatch(permission, profile):
    # C08: no accepted profile, or a request bound to another profile.
    sim = Sim(permission=permission).handshake()
    sim.request(MotionOperation.ROTATE, math.radians(30), profile=profile)
    sim.step(20)
    assert max(sim.commands) == 0.0
    assert sim.guard.reason == 'PROFILE_INVALID'


def test_unsafe_sweep_refuses_to_start():
    sim = Sim(sweep_allowed=False).handshake()
    sim.request(MotionOperation.ROTATE, math.radians(30))
    sim.step(10)
    assert max(sim.commands) == 0.0
    assert sim.guard.reason == 'UNKNOWN_SWEEP'


@pytest.mark.parametrize('limits, deltas', [
    (ProbeBudgetLimits(max_segments=2), [0.5, 0.5, 0.5]),
    # Each 0.5 rad probe travels about 0.32 rad (zero is commanded early).
    (ProbeBudgetLimits(max_total_abs_yaw_rad=1.0), [0.5, 0.5, 0.5]),
    (ProbeBudgetLimits(max_motion_time_s=2.0), [0.9, 0.9, 0.9]),
    (ProbeBudgetLimits(max_session_s=6.0), [0.9, 0.9, 0.9]),
])
def test_budget_and_clock_reset(limits, deltas):
    # C10: each budget alone refuses the segment that would exceed it.
    sim = Sim(limits=limits).handshake()
    accepted = 0
    for delta in deltas:
        before = sim.guard.budget.segments
        sim.request(MotionOperation.ROTATE, delta)
        sim.step(80)
        accepted += sim.guard.budget.segments - before
    assert accepted < len(deltas)
    assert sim.guard.reason == 'MOTION_BUDGET_EXHAUSTED'
    assert sim.guard.state != GuardState.ROTATING


def test_odometry_clock_reset_never_replays_old_actions():
    sim = Sim().handshake()
    rotate = sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(5)
    sim.stamp -= 5_000_000_000           # ROS time jumps backwards
    sim.step(1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == 'ODOM_JUMP'
    assert sim.guard.session == ''

    sim.guard.on_request(rotate, sim.now)    # old lease refresh
    sim.request(MotionOperation.STOP)        # same (dead) session again
    sim.step(20)
    sim.request(MotionOperation.ROTATE, math.radians(30))
    sim.step(20)
    assert max(sim.commands[-40:]) == 0.0

    sim.request(MotionOperation.STOP, session='newsession000002')
    sim.step(15)
    assert sim.guard.state == GuardState.STOPPED
    assert sim.guard.budget.segments == 0


def test_release_only_after_stopped_and_is_terminal():
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(60))
    sim.step(10)
    sim.request(MotionOperation.RELEASE)     # still rotating: just stop
    sim.step(1)
    assert sim.guard.state == GuardState.STOPPING
    sim.step(15)
    sim.request(MotionOperation.RELEASE)
    sim.step(1)
    assert sim.guard.state == GuardState.RELEASED

    sim.request(MotionOperation.STOP, session='newsession000003')
    sim.request(MotionOperation.ROTATE, math.radians(30),
                session='newsession000003')
    sim.step(20)
    assert sim.guard.state == GuardState.RELEASED
    assert max(sim.commands[-20:]) == 0.0


def test_translation_beyond_profile_drift_stops():
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    sim.step(5)
    sim.x = 0.05                             # robot slid sideways
    sim.step(1)
    assert sim.commands[-1] == 0.0
    assert sim.guard.reason == 'ODOM_JUMP'


def _chassis_model():
    import importlib.util
    from pathlib import Path

    import yaml

    workspace = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location(
        'carbot_control', workspace / 'isaac_sim' / 'carbot_control.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = yaml.safe_load((
        workspace / 'src/carbot_description/config/carbot_parameters.yaml'
    ).read_text(encoding='utf-8'))
    limits = module.ControlLimits.from_parameters(parameters)
    return module.CarbotCommandLimiter(limits), limits


def test_guard_crash_watchdog():
    # C04 (simulation only): the guard dies mid-rotation, so the chassis
    # keeps receiving nothing after its last nonzero command.  The canonical
    # chassis model's cmd_vel watchdog must stop the robot on its own.  The
    # real ESP32 watchdog still needs its own physical acceptance test.
    limiter, limits = _chassis_model()
    sim = Sim().handshake()
    sim.request(MotionOperation.ROTATE, math.radians(90))
    dt = 0.02
    last_command, last_command_time = 0.0, sim.now
    for _ in range(60):                       # 1.2 s of normal rotation
        sim.step(1, dt=dt)
        last_command, last_command_time = sim.rate, sim.now
        sim.rate = limiter.update(0.0, last_command, 0.0,
                                  dt).applied_angular_rad_s
    assert sim.guard.state == GuardState.ROTATING
    crash_time, crash_yaw = sim.now, sim.yaw
    speed = abs(limiter.angular_rad_s)

    stopped_at = None
    while sim.now - crash_time < 3.0:         # guard process gone
        sim.now += dt
        command = limiter.update(0.0, last_command,
                                 sim.now - last_command_time, dt)
        sim.yaw += command.applied_angular_rad_s * dt
        if stopped_at is None and abs(command.applied_angular_rad_s) < 1e-9:
            stopped_at = sim.now
    decel_time = speed / limits.max_angular_acceleration_rad_s2
    assert stopped_at is not None
    assert stopped_at - crash_time <= (
        limits.cmd_vel_timeout_s + decel_time + 2 * dt)
    extra = abs(sim.yaw - crash_yaw)
    assert extra <= speed * (limits.cmd_vel_timeout_s + dt) + (
        speed ** 2 / (2 * limits.max_angular_acceleration_rad_s2)) + 1e-6
