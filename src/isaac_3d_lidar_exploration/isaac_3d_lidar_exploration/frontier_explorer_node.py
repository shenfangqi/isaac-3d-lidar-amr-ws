import json
import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .frontier import build_frontier_candidates, world_to_cell


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        defaults = {
            'map_topic': '/nvblox_node/static_occupancy_grid',
            'global_frame': 'odom',
            'robot_base_frame': 'base_link',
            'navigate_action': '/navigate_to_pose',
            'cmd_vel_topic': '/cmd_vel',
            'reject_unexpected_cmd_vel_publishers': True,
            'allowed_cmd_vel_publishers': [
                'velocity_smoother',
                'behavior_server',
            ],
            'start_enabled': False,
            'planning_period_sec': 2.0,
            'map_stale_timeout_sec': 5.0,
            'goal_timeout_sec': 120.0,
            'blacklist_duration_sec': 180.0,
            'blacklist_radius_m': 0.75,
            'free_threshold': 20,
            'occupied_threshold': 65,
            'min_frontier_length_m': 0.30,
            'goal_clearance_m': 0.55,
            'goal_unknown_clearance_m': 0.40,
            'goal_search_radius_m': 1.00,
            'boundary_margin_m': 0.60,
            'min_goal_distance_m': 0.80,
            'max_goal_distance_m': 15.0,
            'information_gain_weight': 1.0,
            'distance_weight': 0.35,
            'completion_cycles': 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        for name in defaults:
            setattr(self, '_' + name, self.get_parameter(name).value)

        map_qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_callback, map_qos
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, '~/frontiers', 1
        )
        self._status_publisher = self.create_publisher(
            String, '~/status', latched_qos
        )
        self._enable_service = self.create_service(
            SetBool, '~/set_enabled', self._set_enabled_callback
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigate_client = ActionClient(
            self, NavigateToPose, self._navigate_action
        )

        self._latest_map = None
        self._map_received_time = None
        self._goal_handle = None
        self._goal_response_pending = False
        self._goal_result_pending = False
        self._goal_start_time = None
        self._current_goal = None
        self._cancel_requested = False
        self._cancel_reason = None
        self._blacklist = []
        self._enabled = bool(self._start_enabled)
        self._no_frontier_cycles = 0
        self._last_status = None
        self._timer = self.create_timer(
            float(self._planning_period_sec), self._planning_tick
        )

        initial_state = 'enabled' if self._enabled else 'disabled'
        self._publish_status(initial_state)
        self.get_logger().info(
            f'Frontier explorer ready on {self._map_topic}; '
            f'enabled={self._enabled}. Motion is delegated to Nav2.'
        )

    def _map_callback(self, message):
        self._latest_map = message
        self._map_received_time = self.get_clock().now()

    def _set_enabled_callback(self, request, response):
        if request.data and self._reject_unexpected_cmd_vel_publishers:
            allowed = set(self._allowed_cmd_vel_publishers)
            unexpected = sorted({
                publisher.node_name
                for publisher in self.get_publishers_info_by_topic(
                    self._cmd_vel_topic
                )
                if publisher.node_name not in allowed
            })
            if unexpected:
                self._enabled = False
                self._publish_status(
                    'enable_rejected',
                    unexpected_cmd_vel_publishers=unexpected,
                )
                response.success = False
                response.message = (
                    'Unexpected cmd_vel publishers: '
                    + ', '.join(unexpected)
                )
                return response

        self._enabled = bool(request.data)
        self._no_frontier_cycles = 0
        if self._enabled:
            self._publish_status('enabled')
            response.message = 'Frontier exploration enabled'
        else:
            self._cancel_current_goal('disabled', blacklist=False)
            self._publish_status('disabled')
            response.message = (
                'Frontier exploration disabled; active goal canceled'
            )
        response.success = True
        return response

    def _publish_status(self, state, **details):
        payload = {'state': state, 'enabled': self._enabled}
        payload.update(details)
        serialized = json.dumps(payload, sort_keys=True)
        if serialized == self._last_status:
            return
        self._last_status = serialized
        message = String()
        message.data = serialized
        self._status_publisher.publish(message)

    @staticmethod
    def _origin_yaw(map_message):
        orientation = map_message.info.origin.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )

    def _robot_position(self):
        transform = self._tf_buffer.lookup_transform(
            self._global_frame,
            self._robot_base_frame,
            Time(),
            timeout=Duration(seconds=0.25),
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _prune_blacklist(self, now_ns):
        self._blacklist = [
            entry for entry in self._blacklist if entry[2] > now_ns
        ]

    def _is_blacklisted(self, candidate):
        return any(
            math.hypot(candidate.x - x, candidate.y - y)
            < self._blacklist_radius_m
            for x, y, _ in self._blacklist
        )

    def _blacklist_current_goal(self):
        if self._current_goal is None:
            return
        expiry = (
            self.get_clock().now().nanoseconds
            + int(self._blacklist_duration_sec * 1e9)
        )
        self._blacklist.append(
            (self._current_goal[0], self._current_goal[1], expiry)
        )

    def _current_goal_is_occupied(self):
        if self._latest_map is None or self._current_goal is None:
            return False
        info = self._latest_map.info
        row, col = world_to_cell(
            self._current_goal[0],
            self._current_goal[1],
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            self._origin_yaw(self._latest_map),
        )
        if not (0 <= row < info.height and 0 <= col < info.width):
            return True
        value = self._latest_map.data[row * info.width + col]
        return value >= self._occupied_threshold

    def _cancel_current_goal(self, reason, blacklist=True):
        if self._goal_handle is None or self._cancel_requested:
            return
        if blacklist:
            self._blacklist_current_goal()
        self._cancel_requested = True
        self._cancel_reason = reason
        self.get_logger().warning(f'Canceling exploration goal: {reason}')
        self._goal_handle.cancel_goal_async()

    def _planning_tick(self):
        now = self.get_clock().now()
        self._prune_blacklist(now.nanoseconds)

        if self._goal_result_pending and self._goal_handle is not None:
            elapsed = (now - self._goal_start_time).nanoseconds / 1e9
            if self._map_received_time is None:
                self._cancel_current_goal('map_missing', blacklist=False)
                return
            else:
                map_age = (
                    now - self._map_received_time
                ).nanoseconds / 1e9
                if map_age > self._map_stale_timeout_sec:
                    self._cancel_current_goal(
                        'map_stale', blacklist=False
                    )
                    return
            if elapsed > self._goal_timeout_sec:
                self._cancel_current_goal('goal_timeout')
            elif self._current_goal_is_occupied():
                self._cancel_current_goal('goal_became_occupied')
            return
        if self._goal_response_pending:
            return
        if not self._enabled:
            return
        if self._latest_map is None or self._map_received_time is None:
            self._publish_status('waiting_for_map')
            return
        map_age = (now - self._map_received_time).nanoseconds / 1e9
        if map_age > self._map_stale_timeout_sec:
            self._publish_status('map_stale', map_age_sec=round(map_age, 2))
            return
        if not self._navigate_client.server_is_ready():
            self._publish_status('waiting_for_nav2')
            return

        try:
            robot_x, robot_y = self._robot_position()
        except TransformException as exception:
            self.get_logger().warning(
                f'Waiting for {self._global_frame} -> '
                f'{self._robot_base_frame}: {exception}',
                throttle_duration_sec=3.0,
            )
            self._publish_status('waiting_for_tf')
            return

        info = self._latest_map.info
        clusters, candidates = build_frontier_candidates(
            data=self._latest_map.data,
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            origin_yaw=self._origin_yaw(self._latest_map),
            robot_x=robot_x,
            robot_y=robot_y,
            free_threshold=self._free_threshold,
            occupied_threshold=self._occupied_threshold,
            min_frontier_length_m=self._min_frontier_length_m,
            goal_clearance_m=self._goal_clearance_m,
            goal_unknown_clearance_m=self._goal_unknown_clearance_m,
            goal_search_radius_m=self._goal_search_radius_m,
            boundary_margin_m=self._boundary_margin_m,
            min_goal_distance_m=self._min_goal_distance_m,
            max_goal_distance_m=self._max_goal_distance_m,
            information_gain_weight=self._information_gain_weight,
            distance_weight=self._distance_weight,
        )
        candidates = [
            candidate for candidate in candidates
            if not self._is_blacklisted(candidate)
        ]
        self._publish_markers(candidates)

        if not clusters:
            self._no_frontier_cycles += 1
            self._publish_status(
                'checking_complete', cycle=self._no_frontier_cycles
            )
            if self._no_frontier_cycles >= self._completion_cycles:
                self._enabled = False
                self._publish_status('complete')
                self.get_logger().info(
                    'Exploration complete: no frontiers remain'
                )
            return

        self._no_frontier_cycles = 0
        if not candidates:
            self._publish_status(
                'no_safe_frontier',
                clusters=len(clusters),
                blacklisted=len(self._blacklist),
            )
            return

        self._send_goal(candidates[0], robot_x, robot_y, len(clusters))

    def _send_goal(self, candidate, robot_x, robot_y, cluster_count):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = candidate.x
        goal_pose.pose.position.y = candidate.y
        yaw = math.atan2(candidate.y - robot_y, candidate.x - robot_x)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self._current_goal = (candidate.x, candidate.y)
        self._goal_response_pending = True
        self._cancel_requested = False
        self._cancel_reason = None
        future = self._navigate_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        self._publish_status(
            'sending_goal',
            x=round(candidate.x, 3),
            y=round(candidate.y, 3),
            clearance_m=round(candidate.clearance_m, 3),
            unknown_clearance_m=round(
                candidate.unknown_clearance_m, 3
            ),
            frontier_clusters=cluster_count,
        )

    def _goal_response_callback(self, future):
        self._goal_response_pending = False
        try:
            goal_handle = future.result()
        except Exception as exception:  # noqa: BLE001
            self.get_logger().error(
                f'NavigateToPose request failed: {exception}'
            )
            self._blacklist_current_goal()
            self._current_goal = None
            return
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 rejected frontier goal')
            self._blacklist_current_goal()
            self._current_goal = None
            return

        self._goal_handle = goal_handle
        self._goal_result_pending = True
        self._goal_start_time = self.get_clock().now()
        if not self._enabled:
            self._cancel_current_goal('disabled', blacklist=False)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)
        self._publish_status('navigating')

    def _goal_result_callback(self, future):
        try:
            status = future.result().status
        except Exception as exception:  # noqa: BLE001
            self.get_logger().error(
                f'NavigateToPose result failed: {exception}'
            )
            status = GoalStatus.STATUS_ABORTED

        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        canceled_by_operator = (
            status == GoalStatus.STATUS_CANCELED
            and self._cancel_reason == 'disabled'
        )
        if succeeded:
            self.get_logger().info('Reached frontier goal')
        elif not canceled_by_operator and not self._cancel_requested:
            self._blacklist_current_goal()
            self.get_logger().warning(
                f'Frontier goal ended with action status {status}'
            )

        self._goal_handle = None
        self._goal_result_pending = False
        self._goal_start_time = None
        self._current_goal = None
        self._cancel_requested = False
        self._cancel_reason = None
        if self._enabled:
            self._publish_status('goal_finished', succeeded=succeeded)
        else:
            self._publish_status('disabled')

    def _publish_markers(self, candidates):
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        points = Marker()
        points.header.frame_id = self._global_frame
        points.header.stamp = self.get_clock().now().to_msg()
        points.ns = 'safe_frontiers'
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.10
        points.scale.y = 0.10
        points.color.r = 0.1
        points.color.g = 1.0
        points.color.b = 0.2
        points.color.a = 0.9
        for candidate in candidates:
            point = Point()
            point.x = candidate.x
            point.y = candidate.y
            point.z = 0.05
            points.points.append(point)
        marker_array.markers.append(points)
        self._marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
