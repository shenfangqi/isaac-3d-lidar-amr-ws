#!/usr/bin/env python3
"""Publish a bounded real-robot velocity pulse followed by repeated zeroes."""

import argparse
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--linear', type=float, default=0.0)
    parser.add_argument('--angular', type=float, default=0.0)
    parser.add_argument('--duration', type=float, required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    if abs(args.linear) > 0.10:
        raise SystemExit('linear velocity exceeds 0.10 m/s safety limit')
    if abs(args.angular) > 0.50:
        raise SystemExit('angular velocity exceeds 0.50 rad/s jog safety limit')
    if not 0.0 < args.duration <= 3.0:
        raise SystemExit('duration must be in (0, 3.0] seconds')
    if args.linear and args.angular:
        raise SystemExit('combined linear and angular jogs are not allowed')

    rclpy.init()
    node = rclpy.create_node('carbot_safe_jog')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    interrupted = False

    def request_stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    command = Twist()
    command.linear.x = args.linear
    command.angular.z = args.angular
    zero = Twist()

    try:
        # A new Fast DDS participant can need several seconds to discover the
        # micro-ROS subscriber over Wi-Fi. Wait without publishing; retain the
        # exact-one-publisher and at-least-one-subscriber gates below.
        discovery_deadline = time.monotonic() + 5.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.count_subscribers('/cmd_vel') >= 1:
                break
        if node.count_subscribers('/cmd_vel') < 1:
            raise RuntimeError('no /cmd_vel subscriber')
        if node.count_publishers('/cmd_vel') != 1:
            raise RuntimeError('another /cmd_vel publisher is present')

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and not interrupted:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for _ in range(20):
            publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    if interrupted:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
