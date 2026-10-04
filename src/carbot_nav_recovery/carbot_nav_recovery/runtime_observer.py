"""Read-only ROS adapter for provisional navigation history and progress."""

import json
import math
import time
from dataclasses import asdict

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .runtime_context import RecoveryContext
from .retreat_preview import evaluate_runtime_retreat
from .ros_snapshots import occupancy_snapshot, raw_costmap
from .validation_visualizer import _transform_pose
from carbot_recovery_interfaces.msg import RuntimeState


class RecoveryRuntimeObserver(Node):
    """Observe goals/TF/odom without claiming command ownership."""

    def __init__(self):
        super().__init__('carbot_recovery_runtime_observer')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('preview_stopping_distance_m', 0.02)
        self.declare_parameter(
            'footprint_xy', [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133])
        values = self.get_parameter('footprint_xy').value
        if (not 6 <= len(values) <= 128 or len(values) % 2
                or not all(math.isfinite(v) for v in values)):
            raise ValueError('invalid footprint_xy')
        self._footprint = tuple(zip(values[::2], values[1::2]))
        self._maps = {'local': None, 'static': None, 'visibility': None}
        self._map_errors = {}
        self._static_identity = None
        self.state = RecoveryContext(self.get_parameter('global_frame').value)
        self._goals = {}
        self._feedback = None
        self._localization = None
        self._localization_epoch = None
        self._localization_source_stamp = 0
        self._invalidation_stamp = 0
        self._odometry = None
        self._command = None
        self._last_processed_stamp = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Costmap, '/local_costmap/costmap_raw',
            lambda msg: self._on_map('local', msg), 1)
        self.create_subscription(
            OccupancyGrid, '/map',
            lambda msg: self._on_map('static', msg), status_qos)
        self.create_subscription(
            OccupancyGrid, '/carbot_nav_recovery/observed_free',
            lambda msg: self._on_map('visibility', msg), sensor_qos)
        for action_name, action_type in (
                ('navigate_to_pose', NavigateToPose),
                ('navigate_through_poses', NavigateThroughPoses)):
            self.create_subscription(
                GoalStatusArray, '/' + action_name + '/_action/status',
                lambda msg, name=action_name: self._on_goals(name, msg),
                status_qos)
            self.create_subscription(
                action_type.Impl.FeedbackMessage,
                '/' + action_name + '/_action/feedback',
                lambda msg, name=action_name: self._on_feedback(name, msg), 10)
        self.create_subscription(
            String, '/automatic_localization/status', self._on_localization, 10)
        self.create_subscription(
            Odometry, '/odom', self._on_odometry, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel_nav', self._on_command, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            lambda _: self._invalidate('MANUAL_RELOCALIZATION'), 10)
        self.create_service(Trigger, '~/invalidate_history', self._invalidate_srv)
        self.create_service(Trigger, '~/get_trace', self._get_trace)
        self._status = self.create_publisher(
            String, '/carbot_nav_recovery/runtime_status', 10)
        self._runtime = self.create_publisher(
            RuntimeState, '/carbot_nav_recovery/runtime', 1)
        self._trace = self.create_publisher(
            Marker, '/carbot_nav_recovery/measured_trace', 10)
        self._retreat_status = self.create_publisher(
            String, '/carbot_nav_recovery/retreat_preview', 10)
        self._retreat_markers = self.create_publisher(
            MarkerArray, '/carbot_nav_recovery/retreat_candidates', 10)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._evaluate_retreat)

    def _on_map(self, kind, message):
        try:
            received = time.monotonic()
            snapshot = (raw_costmap(message, received) if kind == 'local'
                        else occupancy_snapshot(
                            message, received, visibility=kind == 'visibility'))
            if snapshot.frame_id != self.state.frame_id:
                raise ValueError('grid frame does not match history frame')
            if kind == 'static':
                identity = (snapshot.width, snapshot.height,
                            snapshot.resolution, snapshot.origin_x,
                            snapshot.origin_y, snapshot.frame_id,
                            snapshot.costs)
                if identity != self._static_identity:
                    self._invalidate('STATIC_MAP_CHANGED')
                    self._static_identity = identity
            self._maps[kind] = snapshot
            self._map_errors.pop(kind, None)
        except (ValueError, TypeError, OverflowError) as error:
            self._maps[kind] = None
            self._map_errors[kind] = str(error)
            if kind == 'static':
                self._invalidate('INVALID_STATIC_MAP')

    def _evaluate_retreat(self):
        try:
            self._observe()
            report = evaluate_runtime_retreat(
                self.state, self._maps['local'], self._maps['static'],
                self._maps['visibility'],
                now_ros_sec=self.get_clock().now().nanoseconds * 1e-9,
                now_monotonic_sec=time.monotonic(), footprint=self._footprint,
                stopping_distance_m=float(
                    self.get_parameter('preview_stopping_distance_m').value))
        except (ValueError, TypeError, OverflowError) as error:
            report = {'reason': 'INVALID_RETREAT_INPUT', 'detail': str(error),
                      'motion_eligible': False, 'candidates': []}
        report['map_input_errors'] = dict(self._map_errors)
        self._retreat_status.publish(String(
            data=json.dumps(report, allow_nan=False)))
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for index, candidate in enumerate(report['candidates']):
            marker = Marker()
            marker.header.frame_id = self.state.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'retreat_geometry_only'
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01
            marker.color.r, marker.color.a = 1.0, 0.8
            marker.color.g = 0.65 if candidate['check']['safe'] else 0.0
            marker.lifetime = Duration(seconds=0.75).to_msg()
            marker.points = [Point(x=p['x'], y=p['y'], z=0.05)
                             for p in candidate['path']]
            output.markers.append(marker)
        self._retreat_markers.publish(output)

    def _on_goals(self, action_name, message):
        self._goals[action_name] = {
            action_name + ':' + bytes(item.goal_info.goal_id.uuid).hex()
            for item in message.status_list if item.status in (1, 2)}
        active = set().union(*self._goals.values())
        goal = next(iter(active)) if len(active) == 1 else None
        if goal != self.state.goal_id:
            self._feedback = None
            self._last_processed_stamp = None
        self.state.set_goal(goal)
        if len(active) > 1:
            self._invalidate('MULTIPLE_ACTIVE_GOALS')

    def _on_feedback(self, action_name, message):
        goal = action_name + ':' + bytes(message.goal_id.uuid).hex()
        if goal != self.state.goal_id:
            return
        self._feedback = time.monotonic()
        if message.feedback.number_of_recoveries > 0:
            # Existing Nav2 recoveries may rotate or reverse outside this
            # observer. Their trajectory must not become forward history.
            self._invalidate('EXISTING_NAV2_RECOVERY')
            self._feedback = None

    def _on_localization(self, message):
        try:
            data = json.loads(message.data)
            source_stamp = data['source_stamp_ns']
            epoch = (data['instance_id'], data['evidence_epoch_ns'])
            if (not isinstance(source_stamp, int)
                    or not isinstance(epoch[0], str) or not epoch[0]
                    or not isinstance(epoch[1], int)):
                raise ValueError('invalid localization identity')
            age = (self.get_clock().now().nanoseconds - source_stamp) * 1e-9
            if (not 0 <= age <= 1.0
                    or source_stamp <= self._invalidation_stamp):
                raise ValueError('stale localization source')
            if epoch != self._localization_epoch:
                self._invalidate('LOCALIZATION_EPOCH_CHANGED')
                self._localization_epoch = epoch
                self._localization_source_stamp = 0
            if source_stamp <= self._localization_source_stamp:
                raise ValueError('repeated localization source')
            self._localization_source_stamp = source_stamp
            ready = (data.get('state') == 'READY'
                     and data.get('ready') is True
                     and data.get('navigation_activated') is True
                     and data.get('validation_only') is False)
        except (ValueError, TypeError, AttributeError, KeyError):
            ready = False
        self._localization = (ready, time.monotonic())
        if not ready:
            self._invalidate('LOCALIZATION_NOT_READY')

    def _on_odometry(self, message):
        self._odometry = (message, time.monotonic())

    def _on_command(self, message):
        self._command = (message, time.monotonic())

    def _invalidate(self, reason):
        self.state.invalidate(reason)
        self._last_processed_stamp = None
        if reason in ('MANUAL_RELOCALIZATION', 'EXTERNAL_INVALIDATION'):
            self._localization = None
            self._invalidation_stamp = self.get_clock().now().nanoseconds

    def _invalidate_srv(self, request, response):
        self._invalidate('EXTERNAL_INVALIDATION')
        self._feedback = None
        response.success = True
        response.message = 'Provisional measured history invalidated.'
        return response

    def _get_trace(self, request, response):
        # Recheck age gates at request time; a timer's previous result cannot
        # establish freshness for a later consumer.
        try:
            self._observe()
        except (ValueError, TypeError, OverflowError):
            self._invalidate('INVALID_RUNTIME_DATA')
        response.success = len(self.state.history.samples) >= 2
        response.message = json.dumps({
            **self.state.report(),
            'context': (asdict(self.state.history.context)
                        if self.state.history.context else None),
            'samples': [asdict(sample) for sample in self.state.history.samples],
        }, allow_nan=False)
        return response

    def _tick(self):
        try:
            self._observe()
        except (ValueError, TypeError, OverflowError) as error:
            self._invalidate('INVALID_RUNTIME_DATA')
            self.get_logger().warning(str(error))
        self._status.publish(String(data=json.dumps(
            self.state.report(), allow_nan=False)))
        runtime = RuntimeState()
        runtime.header.stamp = self.get_clock().now().to_msg()
        runtime.header.frame_id = self.state.frame_id
        runtime.goal_id = self.state.goal_id or ''
        runtime.localization_epoch = (self.state.context.localization_epoch
                                      if self.state.context else '')
        runtime.ready = self.state.reason in ('OBSERVING', 'NO_PROGRESS_OBSERVED')
        self._runtime.publish(runtime)
        marker = Marker()
        marker.header.frame_id = self.state.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'provisional_measured_trace'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = (Marker.ADD if self.state.history.samples
                         else Marker.DELETE)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.a = 1.0, 0.65, 1.0
        marker.lifetime = Duration(seconds=0.5).to_msg()
        marker.points = [Point(x=s.pose.x, y=s.pose.y, z=0.03)
                         for s in self.state.history.samples]
        self._trace.publish(marker)

    def _observe(self):
        now = time.monotonic()

        def recent(received, limit=0.5):
            return received is not None and 0 <= now - received <= limit

        reason = None
        if self.state.goal_id is None:
            reason = 'NO_ACTIVE_GOAL'
        elif (not self._localization or not self._localization[0]
              or not recent(self._localization[1], 1.0)):
            reason = 'LOCALIZATION_NOT_READY'
        elif not recent(self._feedback, 1.0):
            reason = 'GOAL_FEEDBACK_STALE'
        elif not self._odometry or not recent(self._odometry[1]):
            reason = 'ODOMETRY_STALE'
        elif not self._command or not recent(self._command[1]):
            reason = 'NAV_COMMAND_STALE'
        if reason:
            self._invalidate(reason)
            return
        odometry = self._odometry[0]
        stamp = Time.from_msg(odometry.header.stamp)
        source_sec = stamp.nanoseconds * 1e-9
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if source_sec <= 0 or not 0 <= age <= 0.5:
            self._invalidate('ODOMETRY_SOURCE_STALE')
            return
        if stamp.nanoseconds == self._last_processed_stamp:
            return
        try:
            transform = self._buffer.lookup_transform(
                self.state.frame_id, self.get_parameter('base_frame').value,
                stamp, timeout=Duration(seconds=0.0))
        except TransformException:
            # Pending TF can arrive on the next executor callback. Stop using
            # the previous trace immediately rather than bridge an unseen gap.
            self._invalidate('TF_UNAVAILABLE')
            return
        command = self._command[0]
        speed = odometry.twist.twist
        self.state.observe(
            _transform_pose(transform), source_sec, now,
            localization_ready=True, feedback_fresh=True, odometry_fresh=True,
            tf_fresh=True, command_fresh=True,
            linear_command=command.linear.x, angular_command=command.angular.z,
            linear_speed=math.hypot(speed.linear.x, speed.linear.y),
            angular_speed=speed.angular.z)
        self._last_processed_stamp = stamp.nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryRuntimeObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
