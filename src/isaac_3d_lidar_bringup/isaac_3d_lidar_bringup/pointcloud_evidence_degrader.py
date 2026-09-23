"""Delay and degrade Isaac MID-360 clouds using measured real-sensor evidence."""

from collections import deque
import math
import random
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def degrade_xyz_data(message, dropout_fraction, noise_stddev_m, random_source):
    """Return a degraded copy of PointCloud2 data without changing its schema."""
    offsets = {field.name: field.offset for field in message.fields}
    if not {"x", "y", "z"}.issubset(offsets):
        raise ValueError("PointCloud2 must contain x/y/z fields")
    byte_order = ">" if message.is_bigendian else "<"
    output = bytearray(message.data)
    count = message.width * message.height
    for index in range(count):
        base = index * message.point_step
        xyz = [
            struct.unpack_from(byte_order + "f", output, base + offsets[name])[0]
            for name in ("x", "y", "z")
        ]
        if random_source.random() < dropout_fraction:
            for name in ("x", "y", "z"):
                struct.pack_into(
                    byte_order + "f", output, base + offsets[name], math.nan
                )
            continue
        distance = math.sqrt(sum(value * value for value in xyz))
        if distance > 1.0e-9 and all(math.isfinite(value) for value in xyz):
            radial_noise = random_source.gauss(0.0, noise_stddev_m)
            for name, value in zip(("x", "y", "z"), xyz):
                struct.pack_into(
                    byte_order + "f",
                    output,
                    base + offsets[name],
                    value + radial_noise * value / distance,
                )
    return bytes(output)


class PointcloudEvidenceDegrader(Node):
    """Model measured cloud transport latency, dropout and range noise."""

    def __init__(self):
        super().__init__("pointcloud_evidence_degrader")
        self.declare_parameter("input_topic", "/livox/lidar_ground_truth")
        self.declare_parameter("output_topic", "/livox/lidar")
        self.declare_parameter("latency_mean_s", 0.1109323606)
        self.declare_parameter("latency_stddev_s", 0.0011617102)
        self.declare_parameter("dropout_fraction", 0.02)
        self.declare_parameter("range_noise_stddev_m", 0.03)
        self.declare_parameter("seed", 20260925)
        self._random = random.Random(
            int(self.get_parameter("seed").value)
        )
        self._queue = deque()
        self._publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self._receive,
            qos_profile_sensor_data,
        )
        self.create_timer(0.002, self._release_due)

    def _receive(self, message):
        message.data = degrade_xyz_data(
            message,
            float(self.get_parameter("dropout_fraction").value),
            float(self.get_parameter("range_noise_stddev_m").value),
            self._random,
        )
        latency = max(
            0.0,
            self._random.gauss(
                float(self.get_parameter("latency_mean_s").value),
                float(self.get_parameter("latency_stddev_s").value),
            ),
        )
        due_ns = self.get_clock().now().nanoseconds + round(latency * 1.0e9)
        self._queue.append((due_ns, message))

    def _release_due(self):
        now_ns = self.get_clock().now().nanoseconds
        while self._queue and self._queue[0][0] <= now_ns:
            _, message = self._queue.popleft()
            self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudEvidenceDegrader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
