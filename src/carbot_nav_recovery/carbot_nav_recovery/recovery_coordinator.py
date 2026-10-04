"""Bounded recovery coordinator; commands leave only through behavior_server.

Execution requires an explicit physical acceptance profile. Missing profile,
coverage, fresh update evidence or ownership fails closed. No Twist publisher
exists here. The C++ behavior owns cancellation and the command watchdog.
"""

import copy
import json
import math
import time
from pathlib import Path

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from tf2_ros import TransformException
from carbot_recovery_interfaces.msg import ControllerFailure, CostmapEvidence
from carbot_recovery_interfaces.srv import RecoveryStep

from .runtime_observer import RecoveryRuntimeObserver
from .ros_snapshots import raw_costmap
from .sensor_visibility import scan_visibility
from .swept_footprint import Pose2D, SnapshotFreshness, check_swept_path
from .trace_retreat import RetreatLimits, evaluate_trace_retreat, wrap
from .validation_visualizer import _transform_pose, _yaw


def load_acceptance_profile(filename):
    if not filename:
        return None
    profile = json.loads(Path(filename).read_text())
    required = ('braking_distance_m', 'braking_yaw_rad',
                'position_margin_m', 'max_odom_gap_sec')
    evidence = profile.get('evidence_directory')
    numeric_values_valid = all(
        not isinstance(profile.get(key), bool)
        and isinstance(profile.get(key), (float, int))
        and math.isfinite(profile[key])
        and profile[key] > 0
        for key in required)
    if (profile.get('physical_acceptance_complete') is not True
            or not isinstance(evidence, str)
            or not Path(evidence).is_absolute()
            or not Path(evidence).is_dir()
            or not numeric_values_valid
            or profile['braking_distance_m'] > 0.10
            or profile['braking_yaw_rad'] > 0.5
            or profile['max_odom_gap_sec'] > 0.5):
        raise ValueError('invalid physical acceptance profile')
    return profile


class RecoveryCoordinator(RecoveryRuntimeObserver):
    def __init__(self):
        super().__init__()
        self.declare_parameter('costmap_footprint_padding_m', 0.01)
        padding = float(
            self.get_parameter('costmap_footprint_padding_m').value)
        if not math.isfinite(padding) or padding < 0.0 or padding > 0.05:
            raise ValueError('invalid costmap_footprint_padding_m')
        # Nav2 pads each footprint coordinate away from zero before giving it
        # to costmap layers.  Recovery collision checks must use that same
        # polygon; comparing the evidence layer's padded footprint with the
        # physical, unpadded polygon otherwise rejects every real update.
        self._footprint = tuple((
            x + math.copysign(padding, x) if x else x,
            y + math.copysign(padding, y) if y else y,
        ) for x, y in self._footprint)
        self.declare_parameter('acceptance_profile', '')
        filename = self.get_parameter('acceptance_profile').value
        self._profile = load_acceptance_profile(filename)
        self._failure = None
        self._evidence = None
        self._scan = None
        self._session = None
        self._step_deadline = None
        self._controller_active = True
        self._completed_recovery_count = 0
        self._planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.create_subscription(ControllerFailure,
                                 '/carbot_nav_recovery/controller_failure',
                                 self._on_failure, 10)
        self.create_subscription(CostmapEvidence,
                                 '/carbot_nav_recovery/costmap_evidence',
                                 self._on_evidence, 1)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                                 self._on_controller_status, 10)
        self.create_service(RecoveryStep, '/carbot_nav_recovery/step', self._step)
        self.create_timer(0.05, self._watchdog)

    def _on_scan(self, message):
        self._scan = (message, time.monotonic())

    def _on_controller_status(self, message):
        self._controller_active = any(s.status in (1, 2, 3) for s in message.status_list)

    def _on_goals(self, action_name, message):
        previous_goal = self.state.goal_id
        super()._on_goals(action_name, message)
        if self.state.goal_id != previous_goal:
            # Nav2 recovery counters are scoped to one navigation goal.  A
            # count carried from a previous goal could hide an unexpected
            # recovery in the next goal.
            self._completed_recovery_count = 0

    def _on_feedback(self, action_name, message):
        goal = action_name + ':' + bytes(message.goal_id.uuid).hex()
        if goal != self.state.goal_id:
            return
        if self._session:
            self._completed_recovery_count = message.feedback.number_of_recoveries
            self._feedback = time.monotonic()
        elif message.feedback.number_of_recoveries <= self._completed_recovery_count:
            self._feedback = time.monotonic()
        else:
            super()._on_feedback(action_name, message)

    def _on_evidence(self, message):
        try:
            snapshot = raw_costmap(message.grid, time.monotonic())
            footprint = tuple((float(p.x), float(p.y)) for p in message.footprint.points)
            if (not message.layers_current or snapshot.frame_id != self.state.frame_id
                    or len(footprint) != len(self._footprint)
                    or any(math.hypot(a[0]-b[0], a[1]-b[1]) > 0.002
                           for a, b in zip(footprint, self._footprint))):
                raise ValueError('costmap update or footprint mismatch')
            self._evidence = snapshot
        except (ValueError, TypeError):
            self._evidence = None

    def _on_failure(self, message):
        context = self.state.context
        if (context is None or message.goal_id != context.goal_id
                or message.localization_epoch != context.localization_epoch
                or message.cause != ControllerFailure.COLLISION
                or not message.collision_checked
                or message.controller_id != 'FollowPath'
                or message.header.frame_id != context.frame_id):
            self._failure = None
            return
        # Preserve the actually measured trace before the controller stops
        # publishing commands. Safety inputs continue to be checked separately.
        self._failure = (message, time.monotonic(), copy.deepcopy(self.state.history),
                         context, self.state.budget, self._localization_epoch)

    def _observe(self):
        if getattr(self, '_session', None) is not None:
            return
        failure = getattr(self, '_failure', None)
        if failure and time.monotonic()-failure[1] <= 0.5:
            return
        super()._observe()

    def _invalidate(self, reason):
        if getattr(self, '_session', None) is not None:
            self._session['failure'] = reason
            self._cancel_planning(self._session)
        if hasattr(self, '_failure'):
            self._failure = None
        super()._invalidate(reason)

    def _cancel_planning(self, session):
        session['planning_canceled'] = True
        handle = session.get('plan_handle')
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def _plan_accepted(self, future, session):
        try:
            handle = future.result()
            session['plan_handle'] = handle
            if session.get('planning_canceled') and handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            session['failure'] = 'PLANNER_TRANSPORT_ERROR'

    def _watchdog(self):
        session = self._session
        if session and time.monotonic() - session['last_request'] > 0.25:
            session['failure'] = 'BEHAVIOR_HEARTBEAT_LOST'
            session['budget'].invalidate('BEHAVIOR_HEARTBEAT_LOST')
            self._cancel_planning(session)

    def _fresh(self, stamp, received, limit=0.5):
        ros = self.get_clock().now().nanoseconds * 1e-9
        source = stamp.sec + stamp.nanosec * 1e-9
        return (source > 0 and 0 <= ros-source <= limit
                and 0 <= time.monotonic()-received <= limit)

    def _safety(self, session):
        if session.get('failure'):
            raise ValueError(session['failure'])
        if self._controller_active:
            raise ValueError('CONTROLLER_STILL_ACTIVE')
        if self.state.goal_id != session['context'].goal_id:
            raise ValueError('GOAL_CHANGED')
        if (self._localization_epoch != session['localization_epoch']
                or not self._localization or not self._localization[0]
                or time.monotonic()-self._localization[1] > 0.5):
            raise ValueError('LOCALIZATION_INVALID')
        if self._evidence is None or self._maps['static'] is None:
            raise ValueError('NO_CURRENT_COSTMAP')
        grid = self._evidence
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if not (0 <= now_ros-grid.stamp_sec <= 0.5
                and 0 <= time.monotonic()-grid.received_monotonic_sec <= 0.5):
            raise ValueError('COSTMAP_UPDATE_STALE')
        if not self._scan or not self._fresh(self._scan[0].header.stamp, self._scan[1]):
            raise ValueError('SCAN_STALE')
        if not self._odometry or not self._fresh(
                self._odometry[0].header.stamp, self._odometry[1],
                self._profile['max_odom_gap_sec']):
            raise ValueError('ODOMETRY_STALE')
        # No controller action may still be executing when behavior takes over.
        # Graph identity is an additional guard, not a replacement for BT sequencing.
        for topic, allowed in (
                ('/cmd_vel_nav', {'controller_server', 'behavior_server'}),
                ('/cmd_vel_command', {'velocity_smoother'}),
                ('/cmd_vel', {'cmd_vel_compensator'})):
            publishers = self.get_publishers_info_by_topic(topic)
            names = [p.node_name for p in publishers]
            # A Nav2 behavior server owns one publisher endpoint per loaded
            # behavior, so several endpoints legitimately share its node
            # name.  Downstream stages must remain single-publisher chains.
            duplicate_allowed = topic == '/cmd_vel_nav'
            if (set(names) != allowed
                    or any(p.node_namespace != '/' for p in publishers)
                    or (not duplicate_allowed and len(publishers) != 1)):
                raise ValueError('COMMAND_OWNERSHIP_INVALID')
        odom = self._odometry[0]
        tf = self._buffer.lookup_transform(
            self.state.frame_id, self.get_parameter('base_frame').value,
            Time.from_msg(odom.header.stamp), timeout=Duration(seconds=0.0))
        pose = _transform_pose(tf)
        speed = odom.twist.twist
        stopped = math.hypot(speed.linear.x, speed.linear.y) < 0.005 and abs(speed.angular.z) < 0.02
        if not all(math.isfinite(v) for v in (speed.linear.x, speed.linear.y, speed.angular.z)):
            raise ValueError('INVALID_ODOMETRY')
        return pose, stopped

    def _step(self, request, response):
        self._step_deadline = time.monotonic()+0.10
        response.status = RecoveryStep.Response.FAILED
        response.reason = 'EXECUTION_NOT_ACCEPTED'
        if self._profile is None:
            response.reason = 'PHYSICAL_ACCEPTANCE_REQUIRED'
            return response
        try:
            if request.operation == RecoveryStep.Request.START:
                self._start(request)
            session = self._session
            if session is None or session['token'] != request.token:
                raise ValueError('SESSION_MISMATCH')
            session['last_request'] = time.monotonic()
            if request.operation == RecoveryStep.Request.STOP:
                self._cancel_planning(session)
                session['failure'] = 'STOP_REQUESTED'
                if session['budget'].active:
                    session['budget'].invalidate('STOP_WITHOUT_CONFIRMED_STANDSTILL')
                self._session = None
                self._failure = None
                self.state.history.invalidate('RECOVERY_FINISHED')
                response.reason = 'STOPPED'
                return response
            pose, stopped = self._safety(session)
            self._account(session, pose)
            response.distance_used_m = session['budget'].distance_m
            response.status = RecoveryStep.Response.RUNNING
            response.reason = session['phase']
            if session['phase'] == 'WAIT_STOP':
                if not stopped:
                    session['stopped_since'] = None
                elif session['stopped_since'] is None:
                    session['stopped_since'] = time.monotonic()
                elif time.monotonic()-session['stopped_since'] >= 0.3:
                    session['phase'] = 'SELECT'
                    session['angles'] = [15, -15, 30, -30, 60, -60, 90, -90]
                    session['candidates'] = []
                    session['translations'] = None
                    session['visibility'] = None
                    session['translation_check'] = None
                    session['retreats_evaluated'] = False
            elif session['phase'] == 'SELECT':
                self._select(session, pose)
            elif session['phase'] == 'WAIT_PLAN':
                self._plan_progress(session)
            elif session['phase'] == 'MOVE':
                self._drive(session, pose, response)
            elif session['phase'] == 'FINAL_STOP':
                if stopped:
                    if session['stopped_since'] is None:
                        session['stopped_since'] = time.monotonic()
                    elif time.monotonic()-session['stopped_since'] >= 0.3:
                        session['budget'].finish(session['context'], time.monotonic(), stopped=True)
                        # BtNavigator increments number_of_recoveries only
                        # after this action returns success.  Record the
                        # expected increment now so its next feedback does not
                        # invalidate the budget needed for a second attempt.
                        if not session.get('recovery_count_committed'):
                            self._completed_recovery_count = max(
                                self._completed_recovery_count,
                                session['recovery_count_before'] + 1)
                            session['recovery_count_committed'] = True
                        response.status = RecoveryStep.Response.SUCCEEDED
                        response.reason = 'REPLAN_ORIGINAL_GOAL'
                else:
                    session['stopped_since'] = None
            if time.monotonic() > self._step_deadline:
                raise ValueError('CONTROL_COMPUTE_DEADLINE')
        except Exception as error:
            # Service failures must never return a partially populated command.
            response.command = Twist()
            response.status = RecoveryStep.Response.FAILED
            response.reason = str(error)
            if self._session:
                self._session['failure'] = str(error)
                self._cancel_planning(self._session)
        return response

    def _start(self, request):
        if self._session is not None:
            raise ValueError('RECOVERY_ALREADY_ACTIVE')
        if not self._failure:
            raise ValueError('NO_CONTROLLER_FAILURE')
        msg, received, history, context, budget, epoch = self._failure
        if (not self._fresh(msg.header.stamp, received)
                or msg.path_fingerprint != request.failed_path_fingerprint
                or not request.token or budget is None
                or not context.goal_id.startswith('navigate_to_pose:')
                or request.original_goal.header.frame_id != context.frame_id):
            raise ValueError('FAILURE_REQUEST_MISMATCH_OR_STALE')
        reason = budget.begin(context, time.monotonic())
        if reason != 'OK':
            raise ValueError(reason)
        q = request.original_goal.pose.orientation
        p = request.original_goal.pose.position
        if (not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-5):
            budget.invalidate('INVALID_ORIGINAL_GOAL')
            raise ValueError('INVALID_ORIGINAL_GOAL')
        self._session = dict(token=request.token, context=context, budget=budget,
                             history=history, localization_epoch=epoch,
                             original_goal=request.original_goal,
                             last_request=time.monotonic(), phase='WAIT_STOP',
                             stopped_since=None, last_pose=None, last_stamp=None,
                             recovery_count_before=self._completed_recovery_count)
        self._failure = None

    def _account(self, session, pose):
        stamp = self._odometry[0].header.stamp
        key = (stamp.sec, stamp.nanosec)
        delta = 0.0
        if key != session['last_stamp']:
            if session['last_stamp'] is not None and key < session['last_stamp']:
                raise ValueError('ODOMETRY_TIME_REVERSED')
            previous = session['last_pose']
            if previous:
                delta = math.hypot(pose.x-previous.x, pose.y-previous.y)
                if delta > 0.03 or abs(wrap(pose.yaw-previous.yaw)) > 0.15:
                    raise ValueError('POSE_JUMP')
            session['last_pose'], session['last_stamp'] = pose, key
        reason = session['budget'].observe(session['context'], time.monotonic(), delta)
        if reason != 'OK':
            raise ValueError(reason)

    def _visibility(self, pose):
        scan, received = self._scan
        transform = self._buffer.lookup_transform(
            self.state.frame_id, scan.header.frame_id, Time.from_msg(scan.header.stamp),
            timeout=Duration(seconds=0.0))
        radius = (max(math.hypot(x, y) for x, y in self._footprint)
                  + self._profile['position_margin_m']
                  + 0.20 + self._profile['braking_distance_m'])
        bounds = (pose.x-radius, pose.y-radius,
                  pose.x+radius, pose.y+radius)
        return scan_visibility(scan, _transform_pose(transform), self._evidence,
                               received, deadline=self._step_deadline,
                               world_bounds=bounds)

    def _clear(self, path):
        margin = self._profile['position_margin_m']
        for grid in (self._evidence, self._maps['static']):
            result = check_swept_path(grid, self._footprint, path, margin,
                                      deadline_monotonic=self._step_deadline)
            if not result.safe:
                raise ValueError(result.reason)

    def _select(self, session, pose):
        candidates = session['candidates']
        # Costmaps alone cannot establish that the near-field sweep is still
        # clear.  Build and periodically refresh finite-ray visibility before
        # considering rotation, translation, or trace-retreat candidates.
        visibility = session.get('visibility')
        if (visibility is None
                or time.monotonic()-visibility.received_monotonic_sec > 0.30):
            session['visibility'] = self._visibility(pose)
            return
        # Keep signed/unwrapped headings; CW and CCW are separate sweeps.
        if session['angles']:
            angle = session['angles'].pop(0)
            target = Pose2D(pose.x, pose.y, pose.yaw+math.radians(angle))
            overrun = Pose2D(target.x, target.y, target.yaw+math.copysign(
                self._profile['braking_yaw_rad'], angle))
            try:
                self._clear((pose, target, overrun))
                from .swept_footprint import check_observed_free_path
                checked = check_observed_free_path(
                    session['visibility'], self._footprint,
                    (pose, target, overrun),
                    SnapshotFreshness(
                        self.get_clock().now().nanoseconds*1e-9,
                        time.monotonic(), 0.5, 0.5, True, True),
                    self._profile['position_margin_m'],
                    deadline_monotonic=self._step_deadline)
                if checked.safe:
                    candidates.append(('ROTATE', (pose, target)))
            except ValueError:
                pass
            return
        if candidates and session['translations'] is None:
            self._request_plan(session)
            return

        if session['translations'] is None:
            session['translations'] = [-0.05, 0.05, -0.10, 0.10]
        if session['translation_check'] is None and session['translations']:
            offset = session['translations'].pop(0)
            remaining = session['budget'].remaining(
                session['context'], time.monotonic())
            if abs(offset)+self._profile['braking_distance_m'] <= remaining[0]:
                target = Pose2D(pose.x+offset*math.cos(pose.yaw),
                                pose.y+offset*math.sin(pose.yaw), pose.yaw)
                extended = offset+math.copysign(
                    self._profile['braking_distance_m'], offset)
                stop = Pose2D(pose.x+extended*math.cos(pose.yaw),
                              pose.y+extended*math.sin(pose.yaw), pose.yaw)
                from .swept_footprint import check_observed_free_path
                try:
                    self._clear((pose, target, stop))
                    checked = check_observed_free_path(
                        session['visibility'], self._footprint,
                        (pose, target, stop),
                        SnapshotFreshness(
                            self.get_clock().now().nanoseconds*1e-9,
                            time.monotonic(), 0.5, 0.5, True, True),
                        self._profile['position_margin_m'],
                        deadline_monotonic=self._step_deadline)
                    if checked.safe:
                        session['translation_check'] = {
                            'offset': offset, 'path': (pose, target),
                            'target': target, 'directions': [1, -1],
                            'safe_direction': False}
                except ValueError:
                    pass
            return

        check = session['translation_check']
        if check is not None and check['directions']:
            direction = check['directions'].pop(0)
            target = check['target']
            try:
                self._clear((target, Pose2D(
                    target.x, target.y, target.yaw+direction*math.pi)))
                check['safe_direction'] = True
            except ValueError:
                pass
            return
        if check is not None:
            if check['safe_direction']:
                offset = check['offset']
                candidates.append((
                    'FORWARD' if offset > 0 else 'RETRACE', check['path']))
            session['translation_check'] = None
            return
        if session['translations']:
            return

        if not session['retreats_evaluated']:
            visibility = session['visibility']
            remaining = session['budget'].remaining(
                session['context'], time.monotonic())
            now = self.get_clock().now().nanoseconds*1e-9
            history = session['history']
            retreats, _ = evaluate_trace_retreat(
                history, session['context'], pose, self._evidence,
                self._maps['static'], visibility, self._footprint,
                SnapshotFreshness(now, time.monotonic(), 0.5, 0.5, True, True),
                RetreatLimits(stopping_distance_m=self._profile['braking_distance_m'],
                              safety_margin_m=self._profile['position_margin_m']),
                distance_remaining_m=remaining[0], time_remaining_sec=remaining[1],
                deadline_monotonic=self._step_deadline,
                held_trace_max_age_sec=2.0)
            candidates.extend(('RETRACE', c.path) for c in retreats if c.geometry_clear)
            session['retreats_evaluated'] = True
            return
        if not candidates:
            raise ValueError('NO_SAFE_RECOVERY_CANDIDATE')
        session['candidates'] = candidates
        self._request_plan(session)

    def _request_plan(self, session):
        if not session['candidates'] or not self._planner.server_is_ready():
            raise ValueError('NO_FEASIBLE_DEPARTURE')
        strategy, path = session['candidates'].pop(0)
        session['strategy'], session['path'] = strategy, path
        target = path[-1]
        goal = ComputePathToPose.Goal()
        goal.goal = session['original_goal']
        goal.use_start = True
        goal.planner_id = 'GridBased'
        goal.start = PoseStamped()
        goal.start.header.frame_id = self.state.frame_id
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x, goal.start.pose.position.y = target.x, target.y
        goal.start.pose.orientation.z = math.sin(target.yaw/2)
        goal.start.pose.orientation.w = math.cos(target.yaw/2)
        session['plan_future'] = self._planner.send_goal_async(goal)
        session['planning_canceled'] = False
        session['plan_handle'] = None
        session['plan_future'].add_done_callback(
            lambda future: self._plan_accepted(future, session))
        session['plan_result'] = None
        session['phase'] = 'WAIT_PLAN'

    def _plan_progress(self, session):
        future = session['plan_future']
        if not future.done():
            return
        handle = future.result()
        if not handle.accepted:
            self._request_plan(session)
            return
        if session['plan_result'] is None:
            session['plan_result'] = handle.get_result_async()
        if not session['plan_result'].done():
            return
        result = session['plan_result'].result()
        poses = result.result.path.poses
        target = session['path'][-1]
        if (result.status != 4 or len(poses) < 2
                or result.result.path.header.frame_id != self.state.frame_id
                or math.hypot(poses[0].pose.position.x-target.x,
                              poses[0].pose.position.y-target.y) > 0.03):
            self._request_plan(session)
            return
        departure = [target]
        length = 0.0
        for p in poses:
            point = Pose2D(p.pose.position.x, p.pose.position.y,
                           target.yaw + wrap(_yaw(p.pose.orientation)-target.yaw))
            length += math.hypot(point.x-departure[-1].x, point.y-departure[-1].y)
            departure.append(point)
            if length >= 0.10:
                break
        try:
            if length < 0.03:
                raise ValueError('NO_DEPARTURE_PROGRESS')
            self._clear(tuple(departure))
        except ValueError:
            self._request_plan(session)
            return
        session['index'] = 1
        session['phase'] = 'MOVE'

    def _drive(self, session, pose, response):
        path = session['path']
        pose = Pose2D(pose.x, pose.y, path[0].yaw+wrap(pose.yaw-path[0].yaw))
        index = session['index']
        target = path[index]
        distance = math.hypot(target.x-pose.x, target.y-pose.y)
        yaw_error = wrap(target.yaw-pose.yaw)
        if distance < 0.008 and abs(yaw_error) < 0.02:
            index += 1
            if index == len(path):
                session['phase'] = 'FINAL_STOP'
                session['stopped_since'] = None
                return
            session['index'] = index
            target = path[index]
            distance = math.hypot(target.x-pose.x, target.y-pose.y)
            yaw_error = wrap(target.yaw-pose.yaw)
        protected = (pose,) + tuple(path[index:])
        from .swept_footprint import check_observed_free_path
        visibility = self._visibility(pose)
        if session['strategy'] == 'ROTATE':
            end = path[-1]
            sign = math.copysign(1, end.yaw-path[0].yaw)
            protected += (Pose2D(end.x, end.y, end.yaw+sign*self._profile['braking_yaw_rad']),)
            if distance > 0.02:
                raise ValueError('ROTATION_POSITION_DRIFT')
            response.command.angular.z = max(-0.4, min(0.4, yaw_error*2))
        else:
            end = path[-1]
            direction = 1 if session['strategy'] == 'FORWARD' else -1
            protected += (Pose2D(end.x+direction*self._profile['braking_distance_m']*math.cos(end.yaw),
                                end.y+direction*self._profile['braking_distance_m']*math.sin(end.yaw), end.yaw),)
            dx, dy = target.x-pose.x, target.y-pose.y
            lateral = -dx*math.sin(pose.yaw)+dy*math.cos(pose.yaw)
            longitudinal = dx*math.cos(pose.yaw)+dy*math.sin(pose.yaw)
            if abs(lateral) > 0.015 or direction*longitudinal < -0.01 or abs(yaw_error) > 0.25:
                raise ValueError('TRACE_TRACKING_ERROR')
            response.command.linear.x = direction*min(0.05, distance)
            response.command.angular.z = max(-0.3, min(0.3, yaw_error*2+direction*lateral*3))
        checked = check_observed_free_path(
            visibility, self._footprint, protected,
            SnapshotFreshness(self.get_clock().now().nanoseconds*1e-9,
                              time.monotonic(), 0.5, 0.5, True, True),
            self._profile['position_margin_m'],
            deadline_monotonic=self._step_deadline)
        if not checked.safe:
            raise ValueError(checked.reason)
        self._clear(protected)
        # Check the actual commanded arc, including response/watchdog latency,
        # not merely the ideal breadcrumb polyline.
        velocity, angular = response.command.linear.x, response.command.angular.z
        duration = 0.25
        distance_left, time_left = session['budget'].remaining(
            session['context'], time.monotonic())
        if time_left <= duration + 0.5:
            raise ValueError('STOP_TIME_RESERVE_EXHAUSTED')
        if velocity and distance_left <= abs(velocity)*duration+self._profile['braking_distance_m']:
            raise ValueError('STOP_DISTANCE_RESERVE_EXHAUSTED')
        predicted = [pose]
        for i in range(1, 6):
            dt = duration*i/5
            yaw = pose.yaw+angular*dt
            if abs(angular) < 1e-8:
                x = pose.x+velocity*dt*math.cos(pose.yaw)
                y = pose.y+velocity*dt*math.sin(pose.yaw)
            else:
                x = pose.x+velocity/angular*(math.sin(yaw)-math.sin(pose.yaw))
                y = pose.y-velocity/angular*(math.cos(yaw)-math.cos(pose.yaw))
            predicted.append(Pose2D(x, y, yaw))
        end = predicted[-1]
        travel_sign = math.copysign(1, velocity) if velocity else 0
        yaw_sign = math.copysign(1, angular) if angular else 0
        predicted.append(Pose2D(
            end.x+travel_sign*self._profile['braking_distance_m']*math.cos(end.yaw),
            end.y+travel_sign*self._profile['braking_distance_m']*math.sin(end.yaw),
            end.yaw+yaw_sign*self._profile['braking_yaw_rad']))
        self._clear(tuple(predicted))
        check = check_observed_free_path(
            visibility, self._footprint, tuple(predicted),
            SnapshotFreshness(self.get_clock().now().nanoseconds*1e-9,
                              time.monotonic(), 0.5, 0.5, True, True),
            self._profile['position_margin_m'],
            deadline_monotonic=self._step_deadline)
        if not check.safe:
            raise ValueError(check.reason)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
