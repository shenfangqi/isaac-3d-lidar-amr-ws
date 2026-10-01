#!/usr/bin/env python3
"""Wait for the guarded automatic-localization terminal state."""

import argparse
import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def parse_args():
    """Parse the maximum wait time."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=150.0)
    return parser.parse_args()


def main():
    """Wait for READY or a stopped terminal state."""
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('automatic_localization_waiter')
    result = {
        'code': None,
        'status': None,
        'manual_pose_required': False,
    }

    def on_status(message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = status.get('state')
        if state == 'CANDIDATE_READY':
            result['code'] = 5
            result['status'] = status
        elif state == 'READY':
            result['code'] = 0
            result['status'] = status
        elif state == 'WAIT_MANUAL_POSE':
            result['status'] = status
            if not result['manual_pose_required']:
                print(
                    'STOPPED: click 2D Pose Estimate once in RViz; '
                    'navigation remains inactive while the pose is checked.',
                    flush=True,
                )
            result['manual_pose_required'] = True
        elif state == 'FAULT_STOPPED':
            result['code'] = 4
            result['status'] = status

    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscription = node.create_subscription(
        String, '/automatic_localization/status', on_status, qos
    )
    deadline = time.monotonic() + args.timeout
    while (
        rclpy.ok()
        and result['code'] is None
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.2)

    if result['code'] is None:
        if result['manual_pose_required']:
            print(json.dumps(result['status'], sort_keys=True), flush=True)
            exit_code = 3
        else:
            print('AUTOMATIC_LOCALIZATION_TIMEOUT', flush=True)
            exit_code = 2
    else:
        print(json.dumps(result['status'], sort_keys=True), flush=True)
        exit_code = result['code']

    del subscription
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
