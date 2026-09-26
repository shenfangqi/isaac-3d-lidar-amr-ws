#!/usr/bin/env python3
"""Keep discovery alive and relay low-bandwidth RViz data to workstation."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


DYNAMIC_TF_TOPIC = '/carbot_workstation/tf'
STATIC_TF_TOPIC = '/carbot_workstation/tf_static'
ROBOT_DESCRIPTION_TOPIC = '/carbot_workstation/robot_description'
VOXEL_MARKER_TOPIC = '/carbot_workstation/tsdf_layer_marker'
VOXEL_MARKER_MAX_POINTS = 6000
WHEEL_JOINT_NAMES = tuple(
    f'{side}_{group}_wheel_joint'
    for side in ('left', 'right')
    for group in (
        'front_idler',
        'front_road_1',
        'front_road_2',
        'rear_road_1',
        'rear_road_2',
        'rear_drive',
    )
)


def compact_cube_list_marker(message: Marker, max_points: int) -> Marker:
    """Uniformly thin a CUBE_LIST marker while preserving point colors."""
    if max_points < 1:
        raise ValueError('max_points must be at least 1')
    point_count = len(message.points)
    if message.type != Marker.CUBE_LIST or point_count <= max_points:
        return message

    stride = (point_count + max_points - 1) // max_points
    indices = range(0, point_count, stride)
    output = deepcopy(message)
    output.points = [message.points[index] for index in indices]
    if len(message.colors) == point_count:
        output.colors = [message.colors[index] for index in indices]
    return output


class WorkstationDdsAnchor(Node):
    """Relay locally discovered TF through the workstation-visible participant."""

    def __init__(self) -> None:
        super().__init__('carbot_workstation_dds_anchor')
        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pending_dynamic = {}
        self._logged_first_voxel_marker = False
        self._dynamic_publisher = self.create_publisher(
            TFMessage, DYNAMIC_TF_TOPIC, dynamic_qos
        )
        self._static_publisher = self.create_publisher(
            TFMessage, STATIC_TF_TOPIC, static_qos
        )
        self._description_publisher = self.create_publisher(
            String, ROBOT_DESCRIPTION_TOPIC, static_qos
        )
        self._joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', sensor_qos
        )
        self._voxel_marker_publisher = self.create_publisher(
            Marker, VOXEL_MARKER_TOPIC, marker_qos
        )
        self._dynamic_subscription = self.create_subscription(
            TFMessage, '/tf', self._dynamic_callback, dynamic_qos
        )
        self._static_subscription = self.create_subscription(
            TFMessage, '/tf_static', self._static_callback, static_qos
        )
        self._description_subscription = self.create_subscription(
            String,
            '/robot_description',
            self._description_callback,
            static_qos,
        )
        self._voxel_marker_subscription = self.create_subscription(
            Marker,
            '/carbot_jetson/nvblox_mesh_voxels',
            self._voxel_marker_callback,
            marker_qos,
        )
        # Publish at most 20 batches per second.  Each child frame retains its
        # newest transform, so a busy odometry publisher cannot hide map->odom.
        self._timer = self.create_timer(0.05, self._publish_pending_dynamic)
        # The tracked wheel visuals are rotationally symmetric and currently
        # have no individual encoders.  Zero joint positions are sufficient
        # for robot_state_publisher to provide their link transforms in RViz.
        # This publisher is visual-only and never enters the control path.
        self._joint_state_timer = self.create_timer(
            0.1, self._publish_visual_joint_states
        )
        self.get_logger().info(
            'Workstation DDS discovery anchor is ready; relaying TF on '
            f'{DYNAMIC_TF_TOPIC} and {STATIC_TF_TOPIC}, plus robot model on '
            f'{ROBOT_DESCRIPTION_TOPIC}, visual wheel joint states, and a '
            f'bounded voxel map on {VOXEL_MARKER_TOPIC}'
        )

    def _dynamic_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._pending_dynamic[transform.child_frame_id] = transform

    def _static_callback(self, message: TFMessage) -> None:
        if message.transforms:
            self._static_publisher.publish(message)

    def _description_callback(self, message: String) -> None:
        if message.data:
            self._description_publisher.publish(message)

    def _voxel_marker_callback(self, message: Marker) -> None:
        output = compact_cube_list_marker(
            message, VOXEL_MARKER_MAX_POINTS
        )
        self._voxel_marker_publisher.publish(output)
        if not self._logged_first_voxel_marker:
            self.get_logger().info(
                f'First full voxel marker relayed with {len(output.points)} '
                f'of {len(message.points)} points'
            )
            self._logged_first_voxel_marker = True

    def _publish_pending_dynamic(self) -> None:
        if not self._pending_dynamic:
            return
        message = TFMessage()
        message.transforms = list(self._pending_dynamic.values())
        self._pending_dynamic.clear()
        self._dynamic_publisher.publish(message)

    def _publish_visual_joint_states(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.name = list(WHEEL_JOINT_NAMES)
        message.position = [0.0] * len(WHEEL_JOINT_NAMES)
        self._joint_state_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = WorkstationDdsAnchor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
