#!/usr/bin/env python3
"""Read-only ROS freshness checks used by jetson_nav_preflight.sh."""

import sys
import time

import rclpy
from carbot_msgs.msg import WheelTicks
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from livox_ros_driver2.msg import CustomMsg
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def main() -> int:
    rclpy.init()
    node = rclpy.create_node('carbot_nav_preflight_probe')
    received = {
        '/wheel_ticks': False,
        '/odom': False,
        '/livox/lidar': False,
        '/scan': False,
        '/scan_localization': False,
    }
    cmd_vel_received = False
    cmd_vel_nonzero = False
    emergency_received = False
    emergency_stop = True

    def cmd_vel_callback(message):
        nonlocal cmd_vel_received, cmd_vel_nonzero
        cmd_vel_received = True
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if any(abs(value) > 1.0e-6 for value in values):
            cmd_vel_nonzero = True

    def emergency_callback(message):
        nonlocal emergency_received, emergency_stop
        emergency_received = True
        emergency_stop = message.data

    subscriptions = []
    for topic, message_type in (
        ('/wheel_ticks', WheelTicks),
        ('/odom', Odometry),
        ('/livox/lidar', CustomMsg),
        ('/scan', LaserScan),
        ('/scan_localization', LaserScan),
    ):
        subscriptions.append(node.create_subscription(
            message_type,
            topic,
            lambda _message, name=topic: received.__setitem__(name, True),
            qos_profile_sensor_data,
        ))
    subscriptions.append(node.create_subscription(
        Twist, '/cmd_vel', cmd_vel_callback, 10
    ))
    fault_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions.append(node.create_subscription(
        Bool,
        '/localization/emergency_stop',
        emergency_callback,
        fault_qos,
    ))

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    map_state_client = node.create_client(GetState, '/map_server/get_state')
    # The base adapter deliberately withholds odom -> base_footprint during
    # its 20-second startup plausibility window.  Leave enough margin for a
    # cold FAST-LIO start and DDS discovery without allowing an unbounded wait.
    deadline = time.monotonic() + 35.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        dynamic_tf = tf_buffer.can_transform(
            'odom', 'base_footprint', rclpy.time.Time())
        lidar_tf = tf_buffer.can_transform(
            'base_footprint', 'livox_frame', rclpy.time.Time())
        topology_ready = (
            node.count_publishers('/cmd_vel') == 1
            and node.count_subscribers('/cmd_vel') >= 1
            and node.count_publishers('/cmd_vel_command') == 0
            and node.count_subscribers('/cmd_vel_command') == 1
            and node.count_publishers('/odom') == 1
        )
        if (all(received.values()) and dynamic_tf and lidar_tf
                and topology_ready and emergency_received
                and map_state_client.service_is_ready()):
            break

    map_state = None
    if map_state_client.service_is_ready():
        future = map_state_client.call_async(GetState.Request())
        state_deadline = time.monotonic() + 8.0
        while not future.done() and time.monotonic() < state_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if future.done() and future.result() is not None:
            map_state = future.result().current_state.label

    checks = list(received.items()) + [
        ('odom -> base_footprint', tf_buffer.can_transform(
            'odom', 'base_footprint', rclpy.time.Time())),
        ('base_footprint -> livox_frame', tf_buffer.can_transform(
            'base_footprint', 'livox_frame', rclpy.time.Time())),
        ('map_server is inactive', map_state in ('unconfigured', 'inactive')),
        ('/cmd_vel has exactly one compensator publisher',
         node.count_publishers('/cmd_vel') == 1),
        ('/cmd_vel has a base-controller subscriber',
         node.count_subscribers('/cmd_vel') >= 1),
        ('/cmd_vel_command has no publisher before Nav2 activation',
         node.count_publishers('/cmd_vel_command') == 0),
        ('/cmd_vel_command has exactly one compensator subscriber',
         node.count_subscribers('/cmd_vel_command') == 1),
        ('/odom has exactly one publisher', node.count_publishers('/odom') == 1),
        ('/cmd_vel stayed zero before Nav2 activation', not cmd_vel_nonzero),
        ('localization emergency stop is false',
         emergency_received and not emergency_stop),
    ]
    failures = 0
    for name, ok in checks:
        if ok:
            print(f'PASS: {name} delivered fresh data')
        else:
            print(f'FAIL: {name}', file=sys.stderr)
            failures += 1

    if not cmd_vel_nonzero:
        command_state = 'zero_only' if cmd_vel_received else 'silent'
        print(f'PASS: /cmd_vel is {command_state}')

    # Keep references alive until all spinning is complete.
    del subscriptions, tf_listener
    node.destroy_node()
    rclpy.shutdown()
    return min(failures, 125)


if __name__ == '__main__':
    raise SystemExit(main())
