#!/usr/bin/env python3
"""Validate the live Carbot ROS interface contract without moving the robot."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InterfaceValidator(Node):
    def __init__(self):
        super().__init__("carbot_ros_interface_validator")
        self.arrivals = {name: [] for name in (
            "/cmd_vel", "/odom", "/joint_states", "/tf", "/tf_static",
            "/clock", "/livox/lidar", "/livox/lidar_nvblox", "/scan",
        )}
        self.frames = {}
        self.tf_edges = set()
        self.padded_shape = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        self.zero_publisher = self.create_publisher(Twist, "/cmd_vel", RELIABLE_QOS)
        self.create_timer(0.1, self._publish_zero)
        self.create_subscription(Twist, "/cmd_vel", self._callback("/cmd_vel"), RELIABLE_QOS)
        self.create_subscription(Odometry, "/odom", self._odom, RELIABLE_QOS)
        self.create_subscription(JointState, "/joint_states", self._joint, SENSOR_QOS)
        self.create_subscription(TFMessage, "/tf", self._tf, RELIABLE_QOS)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static, STATIC_QOS)
        self.create_subscription(Clock, "/clock", self._callback("/clock"), SENSOR_QOS)
        self.create_subscription(PointCloud2, "/livox/lidar", self._lidar, SENSOR_QOS)
        self.create_subscription(
            PointCloud2, "/livox/lidar_nvblox", self._padded, RELIABLE_QOS
        )
        self.create_subscription(LaserScan, "/scan", self._scan, SENSOR_QOS)

    def _record(self, topic):
        self.arrivals[topic].append(time.monotonic())

    def _callback(self, topic):
        def callback(_message):
            self._record(topic)
        return callback

    def _publish_zero(self):
        self.zero_publisher.publish(Twist())

    def _odom(self, message):
        self._record("/odom")
        self.frames["/odom"] = (message.header.frame_id, message.child_frame_id)

    def _joint(self, message):
        self._record("/joint_states")
        self.frames["/joint_states"] = message.header.frame_id
        self.frames["joint_count"] = len(message.name)

    def _add_tf(self, topic, message):
        self._record(topic)
        for transform in message.transforms:
            self.tf_edges.add(
                (transform.header.frame_id, transform.child_frame_id)
            )

    def _tf(self, message):
        self._add_tf("/tf", message)

    def _tf_static(self, message):
        self._add_tf("/tf_static", message)

    def _lidar(self, message):
        self._record("/livox/lidar")
        self.frames["/livox/lidar"] = message.header.frame_id

    def _padded(self, message):
        self._record("/livox/lidar_nvblox")
        self.frames["/livox/lidar_nvblox"] = message.header.frame_id
        self.padded_shape = (message.width, message.height)

    def _scan(self, message):
        self._record("/scan")
        self.frames["/scan"] = message.header.frame_id


def _rate(samples):
    if len(samples) < 2 or samples[-1] <= samples[0]:
        return 0.0
    return (len(samples) - 1) / (samples[-1] - samples[0])


def _qos_name(value):
    return str(value).rsplit(".", 1)[-1]


def main():
    rclpy.init()
    node = InterfaceValidator()
    deadline = time.monotonic() + 8.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    expected_types = {
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/odom": "nav_msgs/msg/Odometry",
        "/joint_states": "sensor_msgs/msg/JointState",
        "/tf": "tf2_msgs/msg/TFMessage",
        "/tf_static": "tf2_msgs/msg/TFMessage",
        "/clock": "rosgraph_msgs/msg/Clock",
        "/livox/lidar": "sensor_msgs/msg/PointCloud2",
        "/livox/lidar_nvblox": "sensor_msgs/msg/PointCloud2",
        "/scan": "sensor_msgs/msg/LaserScan",
    }
    discovered = dict(node.get_topic_names_and_types())
    failures = []
    for topic, expected in expected_types.items():
        actual = discovered.get(topic, [])
        if actual != [expected]:
            failures.append(f"{topic} type={actual}, expected {expected}")

    minimum_rates = {
        "/cmd_vel": 5.0,
        "/odom": 10.0,
        "/joint_states": 10.0,
        "/tf": 10.0,
        "/clock": 10.0,
        "/livox/lidar": 5.0,
        "/livox/lidar_nvblox": 5.0,
        "/scan": 5.0,
    }
    bounded_rates = {
        "/cmd_vel": (8.0, 12.0),
        "/livox/lidar": (8.0, 12.0),
        "/livox/lidar_nvblox": (8.0, 12.0),
        "/scan": (8.0, 12.0),
    }
    print("[ROS-IF][RATES]", flush=True)
    for topic, samples in node.arrivals.items():
        rate = _rate(samples)
        suffix = "static" if topic == "/tf_static" else f"{rate:.2f} Hz"
        print(f"  {topic}: count={len(samples)}, rate={suffix}", flush=True)
        if topic == "/tf_static":
            if not samples:
                failures.append("/tf_static did not deliver its latched message")
        elif rate < minimum_rates[topic]:
            failures.append(
                f"{topic} rate {rate:.2f} Hz below {minimum_rates[topic]:.2f} Hz"
            )
        elif topic in bounded_rates and not (
            bounded_rates[topic][0] <= rate <= bounded_rates[topic][1]
        ):
            failures.append(
                f"{topic} rate {rate:.2f} Hz outside "
                f"[{bounded_rates[topic][0]:.2f}, {bounded_rates[topic][1]:.2f}] Hz"
            )

    expected_frames = {
        "/odom": ("odom", "base_footprint"),
        "/joint_states": "base_link",
        "/livox/lidar": "front_3d_lidar",
        "/livox/lidar_nvblox": "front_3d_lidar",
        "/scan": "base_footprint",
    }
    for topic, expected in expected_frames.items():
        if node.frames.get(topic) != expected:
            failures.append(
                f"{topic} frame={node.frames.get(topic)!r}, expected {expected!r}"
            )
    if node.frames.get("joint_count") != 12:
        failures.append(
            f"/joint_states count={node.frames.get('joint_count')}, expected 12"
        )
    if node.padded_shape != (1000, 40):
        failures.append(
            f"/livox/lidar_nvblox shape={node.padded_shape}, expected (1000, 40)"
        )

    expected_tf_links = [
        ("map", "odom"),
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "lidar_link"),
        ("lidar_link", "livox_frame"),
        ("lidar_link", "front_3d_lidar"),
    ]
    missing_tf_links = [
        link
        for link in expected_tf_links
        if not node.tf_buffer.can_transform(link[0], link[1], rclpy.time.Time())
    ]
    if missing_tf_links:
        failures.append(f"missing TF links: {missing_tf_links}")

    expected_qos = {
        "/cmd_vel": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        "/odom": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        "/joint_states": (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
        "/tf": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        "/tf_static": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL),
        "/clock": (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
        "/livox/lidar": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        "/livox/lidar_nvblox": (ReliabilityPolicy.RELIABLE, DurabilityPolicy.VOLATILE),
        "/scan": (ReliabilityPolicy.BEST_EFFORT, DurabilityPolicy.VOLATILE),
    }
    print("[ROS-IF][QOS]", flush=True)
    for topic, expected in expected_qos.items():
        endpoints = node.get_publishers_info_by_topic(topic)
        profiles = [(item.qos_profile.reliability, item.qos_profile.durability) for item in endpoints]
        printable = [f"{_qos_name(r)}/{_qos_name(d)}" for r, d in profiles]
        print(f"  {topic}: {printable}", flush=True)
        if expected not in profiles:
            failures.append(
                f"{topic} publisher QoS lacks "
                f"{_qos_name(expected[0])}/{_qos_name(expected[1])}"
            )
    if not node.get_subscriptions_info_by_topic("/cmd_vel"):
        failures.append("/cmd_vel has no subscriber")

    print(f"[ROS-IF][FRAMES] {node.frames}", flush=True)
    print(
        f"[ROS-IF][TF] required links present={not missing_tf_links}",
        flush=True,
    )
    if failures:
        for failure in failures:
            print(f"[ROS-IF][FAIL] {failure}", flush=True)
        exit_code = 1
    else:
        print(
            "[ROS-IF] PASS: live types, QoS, rates, frames, 1000x40 cloud, "
            "and map->odom->base_footprint->base_link->lidar frames verified.",
            flush=True,
        )
        exit_code = 0

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
