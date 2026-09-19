#!/usr/bin/env python3
"""Initialize saved-map Nav2 in a deterministic, non-motion sequence."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


LOCALIZATION_MANAGER = '/lifecycle_manager_localization/manage_nodes'
NAVIGATION_MANAGER = '/lifecycle_manager_navigation/manage_nodes'
LIFECYCLE_NODES = (
    'map_server',
    'amcl',
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--discovery-settle', type=float, default=3.0)
    return parser.parse_args()


def spin_until(node, predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise RuntimeError(f'timed out waiting for {description}')


def call_manager(node, service_name, timeout_s):
    client = node.create_client(ManageLifecycleNodes, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f'lifecycle service unavailable: {service_name}')
    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.STARTUP
    future = client.call_async(request)
    spin_until(node, future.done, timeout_s, service_name)
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f'lifecycle startup failed: {service_name}')


def assert_active(node, node_name, timeout_s):
    service_name = f'/{node_name}/get_state'
    client = node.create_client(GetState, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f'lifecycle state service unavailable: {service_name}')
    future = client.call_async(GetState.Request())
    spin_until(node, future.done, timeout_s, service_name)
    response = future.result()
    if response is None or response.current_state.label != 'active':
        state = 'unavailable' if response is None else response.current_state.label
        raise RuntimeError(f'{node_name} is {state}, expected active')


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('carbot_navigation_initializer')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    amcl_received = False

    def amcl_callback(_message):
        nonlocal amcl_received
        amcl_received = True

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', amcl_callback, qos
    )
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', qos
    )
    try:
        time.sleep(args.discovery_settle)
        call_manager(node, LOCALIZATION_MANAGER, args.timeout)
        spin_until(
            node,
            lambda: publisher.get_subscription_count() >= 1,
            args.timeout,
            '/initialpose subscriber discovery',
        )

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = args.x
        message.pose.pose.position.y = args.y
        message.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        message.pose.covariance[0] = 0.03
        message.pose.covariance[7] = 0.03
        message.pose.covariance[35] = 0.02
        for _ in range(10):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.2)

        spin_until(
            node,
            lambda: amcl_received and tf_buffer.can_transform(
                'map', 'odom', rclpy.time.Time()
            ),
            args.timeout,
            'AMCL pose and map -> odom',
        )
        call_manager(node, NAVIGATION_MANAGER, args.timeout)
        for node_name in LIFECYCLE_NODES:
            assert_active(node, node_name, args.timeout)
        print(
            'NAVIGATION_INITIALIZED '
            f'x={args.x:.3f} y={args.y:.3f} yaw={args.yaw:.4f} '
            'localization=active navigation=active goal_sent=false',
            flush=True,
        )
    finally:
        del subscription, tf_listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
