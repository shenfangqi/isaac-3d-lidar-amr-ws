import pytest

from carbot_hardware.web_teleop_core import CommandLease, VelocityCommand


class Clock:
    now = 10.0

    def __call__(self):
        return self.now


def test_command_requires_arming_and_expires_to_zero():
    clock = Clock()
    lease = CommandLease(0.10, 0.40, 0.30, clock=clock)

    with pytest.raises(RuntimeError):
        lease.submit(0.10, 0.0)

    lease.set_armed(True)
    lease.submit(0.10, 0.40)
    assert lease.sample() == VelocityCommand(0.10, 0.40)

    clock.now += 0.31
    assert lease.sample() == VelocityCommand()
    assert lease.armed


@pytest.mark.parametrize(
    "linear,angular",
    [(0.101, 0.0), (-0.101, 0.0), (0.0, 0.401), (0.0, -0.401)],
)
def test_command_rejects_values_outside_limits(linear, angular):
    lease = CommandLease(0.10, 0.40, 0.30)
    lease.set_armed(True)
    with pytest.raises(ValueError):
        lease.submit(linear, angular)


def test_disarm_clears_an_active_command():
    lease = CommandLease(0.10, 0.40, 0.30)
    lease.set_armed(True)
    lease.submit(-0.10, -0.40)
    lease.set_armed(False)
    assert lease.sample() == VelocityCommand()
    assert not lease.armed
