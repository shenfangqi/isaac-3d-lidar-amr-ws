#!/usr/bin/env python3
"""Reject navigation startup when stationary FAST-LIO odometry is unstable."""

import argparse
import math
import time

from nav_msgs.msg import Odometry
import rclpy


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def unwrap(values):
    output = [values[0]]
    for value in values[1:]:
        output.append(output[-1] + math.atan2(
            math.sin(value - output[-1]),
            math.cos(value - output[-1]),
        ))
    return output


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--max-position-range', type=float, default=0.03)
    parser.add_argument('--max-yaw-range-deg', type=float, default=1.0)
    parser.add_argument('--max-step', type=float, default=0.01)
    parser.add_argument('--min-rate', type=float, default=8.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('carbot_lio_stability_check')
    messages = []
    subscription = node.create_subscription(
        Odometry, '/odom', messages.append, 50)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        if len(messages) < 2:
            raise RuntimeError('insufficient /odom samples')
        stamps = [
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1.0e-9
            for message in messages
        ]
        elapsed = stamps[-1] - stamps[0]
        rate = (len(messages) - 1) / elapsed if elapsed > 0.0 else 0.0
        xs = [message.pose.pose.position.x for message in messages]
        ys = [message.pose.pose.position.y for message in messages]
        yaws = unwrap([
            yaw_from_quaternion(message.pose.pose.orientation)
            for message in messages
        ])
        position_range = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        yaw_range_deg = math.degrees(max(yaws) - min(yaws))
        max_step = max(
            math.hypot(
                second.pose.pose.position.x - first.pose.pose.position.x,
                second.pose.pose.position.y - first.pose.pose.position.y,
            )
            for first, second in zip(messages, messages[1:])
        )
        print(
            'LIO_STABILITY '
            f'samples={len(messages)} rate={rate:.3f}_hz '
            f'position_range={position_range:.6f}_m '
            f'yaw_range={yaw_range_deg:.6f}_deg '
            f'max_step={max_step:.6f}_m',
            flush=True,
        )
        failures = []
        if rate < args.min_rate:
            failures.append('odometry rate below minimum')
        if position_range > args.max_position_range:
            failures.append('position range exceeds limit')
        if yaw_range_deg > args.max_yaw_range_deg:
            failures.append('yaw range exceeds limit')
        if max_step > args.max_step:
            failures.append('single odometry step exceeds limit')
        if failures:
            raise RuntimeError('; '.join(failures))
        print('PASS: stationary FAST-LIO odometry is stable', flush=True)
    finally:
        del subscription
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
