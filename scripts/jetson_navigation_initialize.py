#!/usr/bin/env python3
"""Initialize manual saved-map Nav2 in a deterministic, non-motion sequence."""

import argparse
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
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=300.0)
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
    initial_pose = None

    def initial_pose_callback(message):
        nonlocal initial_pose
        initial_pose = message

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/initialpose', initial_pose_callback, qos
    )
    try:
        time.sleep(args.discovery_settle)
        call_manager(node, LOCALIZATION_MANAGER, args.timeout)
        spin_until(
            node,
            lambda: any(
                endpoint.node_name == 'manual_map_localizer'
                for endpoint in node.get_subscriptions_info_by_topic(
                    '/initialpose'
                )
            ),
            args.timeout,
            'manual_map_localizer /initialpose subscription',
        )
        print(
            'WAITING_FOR_INITIAL_POSE map_server=active '
            'use_rviz_2d_pose_estimate=true goal_sent=false',
            flush=True,
        )
        spin_until(
            node,
            lambda: initial_pose is not None and tf_buffer.can_transform(
                'map', 'odom', rclpy.time.Time()
            ),
            args.timeout,
            'RViz Initial Pose and map -> odom',
        )
        call_manager(node, NAVIGATION_MANAGER, args.timeout)
        for node_name in LIFECYCLE_NODES:
            assert_active(node, node_name, args.timeout)
        pose = initial_pose.pose.pose
        print(
            'NAVIGATION_INITIALIZED '
            f'x={pose.position.x:.3f} y={pose.position.y:.3f} '
            'localization=manual navigation=active goal_sent=false',
            flush=True,
        )
    finally:
        del subscription, tf_listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
