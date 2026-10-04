"""Accounting across strategy switches, cancellation and stopping."""

from carbot_nav_recovery.recovery_budget import RecoveryBudget
from carbot_nav_recovery.trace_retreat import TraceContext


CONTEXT = TraceContext('goal', 'epoch', 'map')


def test_attempts_and_travel_do_not_reset_after_success():
    budget = RecoveryBudget(CONTEXT)
    assert budget.begin(CONTEXT, 10) == 'OK'
    assert budget.observe(CONTEXT, 12, 0.08) == 'OK'
    assert budget.finish(CONTEXT, 13, stopped=True) == 'OK'
    assert budget.begin(CONTEXT, 100) == 'OK'
    assert budget.observe(CONTEXT, 102, 0.07) == 'OK'
    assert budget.finish(CONTEXT, 103, stopped=True) == 'OK'
    distance, seconds = budget.remaining(CONTEXT, 104)
    assert abs(distance - 0.05) < 1e-9
    assert seconds == 24
    assert budget.begin(CONTEXT, 105) == 'BUDGET_EXHAUSTED'


def test_waiting_and_braking_consume_time_and_distance():
    budget = RecoveryBudget(CONTEXT)
    budget.begin(CONTEXT, 10)
    result = budget.finish(CONTEXT, 35, stopped=False)
    assert result == 'WAITING_FOR_STANDSTILL'
    assert budget.active
    assert budget.observe(CONTEXT, 40, 0.21) == 'BUDGET_EXHAUSTED'
    assert budget.remaining(CONTEXT, 41) == (0, 0)


def test_context_clock_and_invalid_odometry_fail_closed():
    for cause in ('context', 'clock', 'odometry', 'cancel'):
        budget = RecoveryBudget(CONTEXT)
        budget.begin(CONTEXT, 10)
        if cause == 'context':
            budget.remaining(TraceContext('new-goal', 'epoch', 'map'), 11)
        elif cause == 'clock':
            budget.remaining(CONTEXT, 9)
        elif cause == 'odometry':
            budget.observe(CONTEXT, 11, float('nan'))
        else:
            budget.invalidate('CANCELED')
        assert budget.remaining(CONTEXT, 12) == (0, 0)
        assert budget.begin(CONTEXT, 13) != 'OK'
