"""Per-goal accounting shared by rotation, translation and trace retreat.

This ledger has no actuator interface. Start accounting before evaluation;
finish only after measured standstill. Waiting, checks and braking therefore
consume the same wall-clock budget as motion. Recovery success never resets
the ledger for the original navigation goal.
"""

import math


class RecoveryBudget:
    """Fail closed on stale contexts, time reversal or invalid odometry."""

    def __init__(self, context):
        self.context = context
        self.attempts = 0
        self.distance_m = 0.0
        self.elapsed_sec = 0.0
        self.active = False
        self.invalid_reason = ''
        self._last_time = None

    def invalidate(self, reason):
        self.invalid_reason = reason or 'INVALIDATED'

    def _tick(self, now):
        if (not math.isfinite(now)
                or (self._last_time is not None and now < self._last_time)):
            self.invalidate('CLOCK_INVALID')
            return
        if self.active:
            self.elapsed_sec += now - self._last_time
        self._last_time = now

    def remaining(self, context, now):
        self._tick(now)
        if context != self.context:
            self.invalidate('CONTEXT_CHANGED')
        if self.invalid_reason:
            return 0.0, 0.0
        return (max(0.0, 0.20 - self.distance_m),
                max(0.0, 30 - self.elapsed_sec))

    def begin(self, context, now):
        distance, seconds = self.remaining(context, now)
        if self.invalid_reason:
            return self.invalid_reason
        if self.active:
            return 'ATTEMPT_ALREADY_ACTIVE'
        if self.attempts >= 2 or distance <= 0 or seconds <= 0:
            return 'BUDGET_EXHAUSTED'
        self.attempts += 1
        self.active = True
        return 'OK'

    def observe(self, context, now, traveled_distance_m):
        """Charge absolute measured travel, including overshoot and braking.

        Caller supplies each odometry increment once, not net displacement.
        Missing odometry must invalidate the ledger, never be charged as zero.
        """
        self.remaining(context, now)
        if (not self.active or not math.isfinite(traveled_distance_m)
                or traveled_distance_m < 0):
            self.invalidate('INVALID_MOTION_ACCOUNTING')
        else:
            self.distance_m += traveled_distance_m
        if self.invalid_reason:
            return self.invalid_reason
        if self.distance_m >= 0.20 or self.elapsed_sec >= 30:
            return 'BUDGET_EXHAUSTED'
        return 'OK'

    def finish(self, context, now, *, stopped):
        self.remaining(context, now)
        if not stopped:
            return 'WAITING_FOR_STANDSTILL'
        self.active = False
        return self.invalid_reason or 'OK'
