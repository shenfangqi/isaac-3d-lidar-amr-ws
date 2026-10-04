"""Non-actuating navigation context and progress observation.

Passive ROS observations are insufficient to take control of navigation.
This context deliberately never sets motion_eligible or declares a verified
collision deadlock. Controller failure attribution is a separate input that
the future controller/BT adapter must provide.
"""

import math
import uuid

from .recovery_budget import RecoveryBudget
from .trace_retreat import PoseHistory, TraceContext, wrap


class RecoveryContext:
    """Bind provisional measured history and recovery accounting to a goal."""

    def __init__(self, frame_id='map'):
        self.frame_id = frame_id
        self.session_id = uuid.uuid4().hex
        self.epoch = 0
        self.goal_id = None
        self.context = None
        self.history = PoseHistory()
        self.budget = None
        self.reason = 'NO_ACTIVE_GOAL'
        self._anchor = None
        self._anchor_time = None
        self._last_time = None
        self.no_progress_sec = 0.0

    def set_goal(self, goal_id):
        if goal_id == self.goal_id:
            return
        self.invalidate('GOAL_CHANGED')
        self.goal_id = goal_id
        if goal_id is None:
            self.context = None
            self.budget = None
            self.reason = 'NO_ACTIVE_GOAL'
            return
        self.context = TraceContext(goal_id, self._epoch_id(), self.frame_id)
        self.budget = RecoveryBudget(self.context)

    def _epoch_id(self):
        return '{}:{}'.format(self.session_id, self.epoch)

    def invalidate(self, reason):
        # Invalidation never grants a new budget to the same user goal.
        had_evidence = (self.history.context is not None
                        or self._anchor is not None)
        self.history.invalidate(reason)
        self.reason = reason
        self._anchor = None
        self._anchor_time = None
        self.no_progress_sec = 0.0
        if had_evidence:
            self.epoch += 1
            if self.budget is not None:
                self.budget.invalidate(reason)
            if self.goal_id is not None:
                self.context = TraceContext(
                    self.goal_id, self._epoch_id(), self.frame_id)

    def observe(self, pose, source_sec, now_monotonic, *,
                localization_ready, feedback_fresh, odometry_fresh,
                tf_fresh, command_fresh, linear_command, angular_command,
                linear_speed, angular_speed):
        values = (source_sec, now_monotonic, linear_command, angular_command,
                  linear_speed, angular_speed)
        if not all(math.isfinite(value) for value in values):
            self.invalidate('INVALID_RUNTIME_DATA')
            return
        if self._last_time is not None and now_monotonic < self._last_time:
            self.invalidate('CLOCK_REVERSED')
            self._last_time = now_monotonic
            return
        self._last_time = now_monotonic
        gates = (
            (self.context is not None, 'NO_ACTIVE_GOAL'),
            (localization_ready, 'LOCALIZATION_NOT_READY'),
            (feedback_fresh, 'GOAL_FEEDBACK_STALE'),
            (odometry_fresh, 'ODOMETRY_STALE'),
            (tf_fresh, 'TF_STALE'),
            (command_fresh, 'NAV_COMMAND_STALE'),
        )
        for passed, reason in gates:
            if not passed:
                self.invalidate(reason)
                return
        if linear_command < -1e-6:
            self.invalidate('REVERSE_OR_EXISTING_RECOVERY')
            return
        if not self.history.append(pose, source_sec, self.context,
                                   localization_valid=True):
            reason = self.history.last_reset_reason
            self.invalidate(reason)
            return
        # Compare to a fixed progress anchor, not to the immediately preceding
        # noisy sample. Actual rotation counts as progress too.
        progressed = self._anchor is None or (
            math.hypot(pose.x - self._anchor.x, pose.y - self._anchor.y) >= 0.02
            or abs(wrap(pose.yaw - self._anchor.yaw)) >= 0.05)
        commanded = abs(linear_command) >= 0.01 or abs(angular_command) >= 0.05
        stationary = abs(linear_speed) < 0.01 and abs(angular_speed) < 0.03
        if progressed or not commanded or not stationary:
            self._anchor = pose
            self._anchor_time = now_monotonic
        self.no_progress_sec = now_monotonic - self._anchor_time
        self.reason = ('NO_PROGRESS_OBSERVED' if self.no_progress_sec >= 3.0
                       else 'OBSERVING')

    def report(self):
        return {
            'schema': 'carbot_recovery_runtime_v1',
            'goal_id': self.goal_id,
            'localization_epoch': (self.context.localization_epoch
                                   if self.context else None),
            'frame_id': self.frame_id,
            'reason': self.reason,
            'trace_samples': len(self.history.samples),
            'trace_provisional': True,
            'no_progress_sec': self.no_progress_sec,
            'deadlock_verified': False,
            'controller_failure_evidence': 'NOT_AVAILABLE',
            'motion_eligible': False,
        }
