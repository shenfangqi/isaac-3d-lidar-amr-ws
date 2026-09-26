#!/usr/bin/env python3
"""Send one bounded Nav2 goal and cancel it if it exceeds a hard timeout."""

import argparse
import math
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--discovery-settle', type=float, default=3.0)
    return parser.parse_args()


def wait_future(node, future, deadline):
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main():
    args = parse_args()
    if not 0.0 < args.timeout <= 30.0:
        raise SystemExit('timeout must be in (0, 30] seconds')
    if not 0.0 <= args.discovery_settle <= 10.0:
        raise SystemExit('discovery-settle must be in [0, 10] seconds')

    rclpy.init()
    node = rclpy.create_node('carbot_safe_nav_goal')
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('NavigateToPose action server is unavailable')

        # Fast DDS over the Jetson's UDP-only profile may report the action
        # endpoints before a fresh process can exchange service data reliably.
        # Keep the client alive briefly instead of immediately sending a goal.
        settle_deadline = time.monotonic() + args.discovery_settle
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

        send_future = client.send_goal_async(goal)
        if not wait_future(node, send_future, time.monotonic() + 20.0):
            raise RuntimeError('timed out waiting for navigation goal response')
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('navigation goal was rejected')
        print('GOAL_ACCEPTED', flush=True)

        result_future = handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        if wait_future(node, result_future, deadline):
            wrapped = result_future.result()
            print(f'GOAL_RESULT_STATUS={wrapped.status}', flush=True)
            return 0 if wrapped.status == 4 else 1

        print('GOAL_TIMEOUT_CANCELLING', flush=True)
        cancel_future = handle.cancel_goal_async()
        if not wait_future(node, cancel_future, time.monotonic() + 5.0):
            raise RuntimeError('timed out while cancelling navigation goal')
        print('GOAL_CANCELLED', flush=True)
        return 124
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
