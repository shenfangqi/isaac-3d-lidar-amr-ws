#!/usr/bin/env python3
"""Reliable lifecycle and health operations for physical Carbot navigation."""

import argparse
import sys
import time

from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


LOCALIZATION_MANAGER = '/lifecycle_manager_localization/manage_nodes'
NAVIGATION_MANAGER = '/lifecycle_manager_navigation/manage_nodes'
REQUIRED_NAVIGATION_NODES = (
    'map_server',
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)
REQUIRED_ACTION_SERVICES = (
    '/navigate_to_pose/_action/send_goal',
    '/spin/_action/send_goal',
    '/backup/_action/send_goal',
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'command',
        choices=(
            'activate-localization',
            'wait-for-pose',
            'activate-navigation',
            'verify-compensator',
            'health',
        ),
    )
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--settle', type=float, default=3.0)
    return parser.parse_args()


def spin_until(node, predicate, timeout_s, description):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
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


def lifecycle_state(node, node_name, timeout_s):
    service_name = f'/{node_name}/get_state'
    client = node.create_client(GetState, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f'lifecycle state unavailable: {node_name}')
    future = client.call_async(GetState.Request())
    spin_until(node, future.done, timeout_s, service_name)
    response = future.result()
    if response is None:
        raise RuntimeError(f'empty lifecycle response: {node_name}')
    return response.current_state.label


def endpoint_counts(node, topic):
    return (
        len(node.get_publishers_info_by_topic(topic)),
        len(node.get_subscriptions_info_by_topic(topic)),
    )


def wait_for_map(node, timeout_s):
    received = False

    def callback(_message):
        nonlocal received
        received = True

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscription = node.create_subscription(
        OccupancyGrid, '/map', callback, qos
    )
    try:
        spin_until(node, lambda: received, timeout_s, '/map')
    finally:
        node.destroy_subscription(subscription)


def wait_for_map_tf(node, timeout_s):
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    emergency_stop = False
    fault_reason = ''

    def emergency_callback(message):
        nonlocal emergency_stop
        emergency_stop = message.data

    def reason_callback(message):
        nonlocal fault_reason
        fault_reason = message.data

    fault_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    emergency_subscription = node.create_subscription(
        Bool,
        '/localization/emergency_stop',
        emergency_callback,
        fault_qos,
    )
    reason_subscription = node.create_subscription(
        String,
        '/localization/fault_reason',
        reason_callback,
        fault_qos,
    )
    try:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if emergency_stop:
                detail = fault_reason or 'unspecified localization fault'
                raise RuntimeError(
                    f'localization emergency stop is latched: {detail}'
                )
            if buffer.can_transform(
                'map', 'base_footprint', rclpy.time.Time()
            ):
                break
        else:
            raise RuntimeError('timed out waiting for map -> base_footprint')
        transform = buffer.lookup_transform(
            'map', 'base_footprint', rclpy.time.Time()
        )
        position = transform.transform.translation
        print(
            'MAP_POSE '
            f'x={position.x:.3f} y={position.y:.3f}',
            flush=True,
        )
    finally:
        del listener
        node.destroy_subscription(emergency_subscription)
        node.destroy_subscription(reason_subscription)


def assert_fresh_scan(node, timeout_s):
    received = False

    def callback(_message):
        nonlocal received
        received = True

    qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    subscription = node.create_subscription(LaserScan, '/scan', callback, qos)
    try:
        spin_until(node, lambda: received, timeout_s, '/scan')
    finally:
        node.destroy_subscription(subscription)


def assert_cmd_vel_safe(node, duration_s):
    received = False
    nonzero = False

    def callback(message):
        nonlocal nonzero, received
        received = True
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if any(abs(value) > 1.0e-6 for value in values):
            nonzero = True

    subscription = node.create_subscription(Twist, '/cmd_vel', callback, 10)
    try:
        deadline = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if nonzero:
                raise RuntimeError(
                    '/cmd_vel produced a nonzero message during the safety check'
                )
    finally:
        node.destroy_subscription(subscription)
    return 'zero_only' if received else 'silent'


def assert_localization_safe(node, timeout_s):
    emergency_stop = None
    fault_reason = ''

    def emergency_callback(message):
        nonlocal emergency_stop
        emergency_stop = message.data

    def reason_callback(message):
        nonlocal fault_reason
        fault_reason = message.data

    fault_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    emergency_subscription = node.create_subscription(
        Bool, '/localization/emergency_stop', emergency_callback, fault_qos
    )
    reason_subscription = node.create_subscription(
        String, '/localization/fault_reason', reason_callback, fault_qos
    )
    try:
        spin_until(
            node,
            lambda: emergency_stop is not None,
            timeout_s,
            '/localization/emergency_stop',
        )
        if emergency_stop:
            detail = fault_reason or 'unspecified localization fault'
            raise RuntimeError(
                f'localization emergency stop is latched: {detail}'
            )
    finally:
        node.destroy_subscription(emergency_subscription)
        node.destroy_subscription(reason_subscription)


def verify_compensator(node, timeout_s, settle_s):
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert_localization_safe(node, timeout_s)
    expected = {
        '/cmd_vel': (1, 1),
        '/cmd_vel_command': (0, 1),
    }
    for topic, counts in expected.items():
        observed = endpoint_counts(node, topic)
        print(
            f'TOPOLOGY {topic} publishers={observed[0]} '
            f'subscriptions={observed[1]}',
            flush=True,
        )
        if observed[0] != counts[0] or observed[1] < counts[1]:
            raise RuntimeError(
                f'{topic} topology is {observed}, expected exactly '
                f'{counts[0]} publisher(s) and at least '
                f'{counts[1]} subscription(s)'
            )
    command_state = assert_cmd_vel_safe(node, 2.0)
    print(f'COMPENSATOR_READY cmd_vel={command_state}', flush=True)


def health(node, timeout_s, settle_s):
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    service_names = {
        name for name, _types in node.get_service_names_and_types()
    }
    lifecycle_nodes = list(REQUIRED_NAVIGATION_NODES)
    if '/amcl/get_state' in service_names:
        lifecycle_nodes.insert(1, 'amcl')
    for node_name in lifecycle_nodes:
        state = lifecycle_state(node, node_name, timeout_s)
        print(f'LIFECYCLE {node_name}={state}', flush=True)
        if state != 'active':
            raise RuntimeError(f'{node_name} is {state}, expected active')

    service_names = {
        name for name, _types in node.get_service_names_and_types()
    }
    for service_name in REQUIRED_ACTION_SERVICES:
        if service_name not in service_names:
            raise RuntimeError(f'missing action service: {service_name}')
    print('ACTIONS navigate_to_pose=ready spin=ready backup=ready', flush=True)

    expected = {
        '/cmd_vel': (1, 1),
        '/cmd_vel_command': (1, 1),
        '/goal_pose': (1, 1),
    }
    for topic, counts in expected.items():
        observed = endpoint_counts(node, topic)
        print(
            f'TOPOLOGY {topic} publishers={observed[0]} '
            f'subscriptions={observed[1]}',
            flush=True,
        )
        if observed[0] != counts[0] or observed[1] < counts[1]:
            raise RuntimeError(
                f'{topic} topology is {observed}, expected exactly '
                f'{counts[0]} publisher(s) and at least '
                f'{counts[1]} subscription(s)'
            )

    assert_fresh_scan(node, timeout_s)
    print('FRESH /scan', flush=True)
    wait_for_map_tf(node, timeout_s)
    command_state = assert_cmd_vel_safe(node, 3.0)
    print(f'SAFE /cmd_vel={command_state}', flush=True)


def main():
    args = parse_args()
    if args.timeout <= 0 or args.settle < 0:
        raise SystemExit('timeout must be positive and settle non-negative')

    rclpy.init()
    node = rclpy.create_node('carbot_navigation_control')
    try:
        if args.command == 'activate-localization':
            call_manager(node, LOCALIZATION_MANAGER, args.timeout)
            wait_for_map(node, args.timeout)
            print('LOCALIZATION_ACTIVE map=ready', flush=True)
        elif args.command == 'wait-for-pose':
            wait_for_map_tf(node, args.timeout)
        elif args.command == 'activate-navigation':
            call_manager(node, NAVIGATION_MANAGER, args.timeout)
            print('NAVIGATION_ACTIVE', flush=True)
        elif args.command == 'verify-compensator':
            verify_compensator(node, args.timeout, args.settle)
        else:
            health(node, args.timeout, args.settle)
            print('NAVIGATION_HEALTHY goal_sent=false', flush=True)
    except RuntimeError as error:
        print(f'ERROR: {error}', file=sys.stderr, flush=True)
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
