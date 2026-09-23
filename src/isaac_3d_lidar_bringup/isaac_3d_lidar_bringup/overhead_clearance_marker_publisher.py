"""Publish Isaac overhead-clearance fixtures as RViz markers."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml


DEFAULT_CONFIG = (
    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/"
    "configs/carbot/common.yaml"
)


class OverheadClearanceMarkerPublisher(Node):
    """Show simulation collision beams without affecting navigation costs."""

    def __init__(self):
        """Load the shared fixture configuration and start publishing."""
        super().__init__("overhead_clearance_marker_publisher")
        self.declare_parameter("config_path", DEFAULT_CONFIG)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter(
            "topic", "/overhead_clearance_markers"
        )

        config_path = Path(self.get_parameter("config_path").value)
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self._obstacles = config["simulation_overhead_obstacles"]
        self._frame_id = self.get_parameter("frame_id").value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("topic").value,
            qos,
        )
        self._timer = self.create_timer(1.0, self._publish_markers)
        self._publish_markers()
        self.get_logger().info(
            f"Publishing {len(self._obstacles)} overhead-clearance "
            "fixtures for RViz"
        )

    def _base_marker(self, obstacle, marker_id, marker_type, namespace):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(obstacle["map_x_m"])
        marker.pose.position.y = float(obstacle["map_y_m"])
        marker.pose.orientation.w = 1.0
        red, green, blue = obstacle["color_rgb"]
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        return marker

    def _beam_marker(self, obstacle, marker_id):
        marker = self._base_marker(
            obstacle, marker_id, Marker.CUBE, "overhead_beams"
        )
        marker.pose.position.z = float(obstacle["clearance_m"]) + (
            float(obstacle["thickness_m"]) / 2.0
        )
        marker.scale.x = float(obstacle["size_x_m"])
        marker.scale.y = float(obstacle["size_y_m"])
        marker.scale.z = float(obstacle["thickness_m"])
        marker.color.a = 0.78
        return marker

    def _label_marker(self, obstacle, marker_id):
        marker = self._base_marker(
            obstacle, marker_id, Marker.TEXT_VIEW_FACING, "overhead_labels"
        )
        marker.pose.position.z = (
            float(obstacle["clearance_m"])
            + float(obstacle["thickness_m"])
            + 0.18
        )
        marker.scale.z = 0.16
        marker.color.a = 1.0
        result = "PASS" if obstacle["name"] == "pass" else "BLOCKED"
        marker.text = (
            f"{result}: {float(obstacle['clearance_m']):.2f} m clearance"
        )
        return marker

    def _publish_markers(self):
        marker_array = MarkerArray()
        for index, obstacle in enumerate(self._obstacles):
            marker_array.markers.extend(
                [
                    self._beam_marker(obstacle, index),
                    self._label_marker(obstacle, index),
                ]
            )
        self._publisher.publish(marker_array)


def main(args=None):
    """Run the overhead-clearance marker publisher."""
    rclpy.init(args=args)
    node = OverheadClearanceMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
