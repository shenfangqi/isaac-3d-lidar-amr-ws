#!/usr/bin/env python3
"""Read-only ROS freshness checks used by jetson_nav_preflight.sh."""

import sys
import time

import rclpy
from carbot_msgs.msg import WheelTicks
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def main() -> int:
    rclpy.init()
    node = rclpy.create_node('carbot_nav_preflight_probe')
    received = {
        '/wheel_ticks': False,
        '/odom': False,
        '/livox/lidar': False,
        '/scan': False,
    }

    subscriptions = []
    for topic, message_type in (
        ('/wheel_ticks', WheelTicks),
        ('/odom', Odometry),
        ('/livox/lidar', PointCloud2),
        ('/scan', LaserScan),
    ):
        subscriptions.append(node.create_subscription(
            message_type,
            topic,
            lambda _message, name=topic: received.__setitem__(name, True),
            qos_profile_sensor_data,
        ))

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        dynamic_tf = tf_buffer.can_transform(
            'odom', 'base_footprint', rclpy.time.Time())
        lidar_tf = tf_buffer.can_transform(
            'base_footprint', 'livox_frame', rclpy.time.Time())
        if all(received.values()) and dynamic_tf and lidar_tf:
            break

    checks = list(received.items()) + [
        ('odom -> base_footprint', tf_buffer.can_transform(
            'odom', 'base_footprint', rclpy.time.Time())),
        ('base_footprint -> livox_frame', tf_buffer.can_transform(
            'base_footprint', 'livox_frame', rclpy.time.Time())),
    ]
    failures = 0
    for name, ok in checks:
        if ok:
            print(f'PASS: {name} delivered fresh data')
        else:
            print(f'FAIL: {name} did not deliver fresh data within 8 seconds',
                  file=sys.stderr)
            failures += 1

    # Keep references alive until all spinning is complete.
    del subscriptions, tf_listener
    node.destroy_node()
    rclpy.shutdown()
    return min(failures, 125)


if __name__ == '__main__':
    raise SystemExit(main())
