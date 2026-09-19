#!/usr/bin/env python3
"""Send one bounded Nav2 Spin action and cancel on timeout."""

import argparse
import math
import sys
import time

import rclpy
from nav2_msgs.action import Spin
from rclpy.action import ActionClient


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaw-deg', type=float, required=True)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--discovery-settle', type=float, default=5.0)
    return parser.parse_args()


def wait_future(node, future, deadline):
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main():
    args = parse_args()
    if not 1.0 <= abs(args.yaw_deg) <= 90.0:
        raise SystemExit('absolute yaw-deg must be in [1, 90]')
    if not 0.0 < args.timeout <= 30.0:
        raise SystemExit('timeout must be in (0, 30] seconds')
    if not 0.0 <= args.discovery_settle <= 10.0:
        raise SystemExit('discovery-settle must be in [0, 10] seconds')

    rclpy.init()
    node = rclpy.create_node('carbot_safe_spin')
    client = ActionClient(node, Spin, '/spin')
    try:
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('Spin action server is unavailable')

        settle_deadline = time.monotonic() + args.discovery_settle
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        goal = Spin.Goal()
        goal.target_yaw = math.radians(args.yaw_deg)
        whole_seconds = int(args.timeout)
        goal.time_allowance.sec = whole_seconds
        goal.time_allowance.nanosec = int((args.timeout - whole_seconds) * 1e9)

        send_future = client.send_goal_async(goal)
        if not wait_future(node, send_future, time.monotonic() + 20.0):
            raise RuntimeError('timed out waiting for spin goal response')
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('spin goal was rejected')
        print('SPIN_ACCEPTED', flush=True)

        result_future = handle.get_result_async()
        deadline = time.monotonic() + args.timeout + 2.0
        if wait_future(node, result_future, deadline):
            wrapped = result_future.result()
            print(f'SPIN_RESULT_STATUS={wrapped.status}', flush=True)
            return 0 if wrapped.status == 4 else 1

        print('SPIN_TIMEOUT_CANCELLING', flush=True)
        cancel_future = handle.cancel_goal_async()
        if not wait_future(node, cancel_future, time.monotonic() + 5.0):
            raise RuntimeError('timed out while cancelling spin goal')
        print('SPIN_CANCELLED', flush=True)
        return 124
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
