#!/usr/bin/env python3
"""Run one bounded, odometry-terminated L-shaped mapping route."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def angle_delta(current, start):
    return math.atan2(math.sin(current - start), math.cos(current - start))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--first-distance', type=float, default=0.50)
    parser.add_argument('--turn-deg', type=float, default=90.0)
    parser.add_argument('--second-distance', type=float, default=0.30)
    parser.add_argument('--confirm', required=True)
    return parser.parse_args()


class RouteNode:
    def __init__(self):
        self.node = rclpy.create_node('carbot_supervised_mapping_route')
        self.odom = None
        self.last_odom_monotonic = 0.0
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

    def _odom_callback(self, message):
        self.odom = message
        self.last_odom_monotonic = time.monotonic()

    def spin_once(self, timeout=0.05):
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def wait_ready(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.spin_once(0.1)
            if (self.odom is not None
                    and self.publisher.get_subscription_count() == 1
                    and self.node.count_publishers('/cmd_vel') == 1):
                return
        raise RuntimeError(
            'route preflight failed: require fresh odom, one cmd_vel '
            'publisher (this node), and one subscriber')

    def require_fresh_odom(self):
        if (self.odom is None
                or time.monotonic() - self.last_odom_monotonic > 0.30):
            raise RuntimeError('odometry is stale; stopping route')

    def send(self, linear=0.0, angular=0.0):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def stop(self, duration=1.0):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.send()
            self.spin_once(0.05)

    def forward(self, distance):
        self.require_fresh_odom()
        start_x = self.odom.pose.pose.position.x
        start_y = self.odom.pose.pose.position.y
        deadline = time.monotonic() + distance / 0.05 + 5.0
        while True:
            self.spin_once(0.05)
            self.require_fresh_odom()
            dx = self.odom.pose.pose.position.x - start_x
            dy = self.odom.pose.pose.position.y - start_y
            traveled = math.hypot(dx, dy)
            if traveled >= distance:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f'forward phase timed out at {traveled:.3f} m')
            self.send(linear=0.10)
        self.stop()
        print(f'FORWARD_DONE target={distance:.3f} actual={traveled:.3f}',
              flush=True)

    def turn(self, degrees):
        self.require_fresh_odom()
        start_yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        target = math.radians(degrees)
        direction = 1.0 if target > 0.0 else -1.0
        deadline = time.monotonic() + 15.0
        turned = 0.0
        while True:
            self.spin_once(0.05)
            self.require_fresh_odom()
            current_yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
            turned = angle_delta(current_yaw, start_yaw)
            if direction * turned >= abs(target):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f'turn phase timed out at {math.degrees(turned):.1f} deg')
            self.send(angular=direction * 0.50)
        self.stop()
        print(
            f'TURN_DONE target={degrees:.1f} '
            f'actual={math.degrees(turned):.1f}',
            flush=True,
        )

    def pose_text(self):
        self.require_fresh_odom()
        pose = self.odom.pose.pose
        return (
            f'x={pose.position.x:.3f} y={pose.position.y:.3f} '
            f'yaw_deg={math.degrees(yaw_from_quaternion(pose.orientation)):.1f}'
        )


def main():
    args = parse_args()
    if args.confirm != 'SUPERVISED_ROUTE':
        raise SystemExit('--confirm must be SUPERVISED_ROUTE')
    if not 0.10 <= args.first_distance <= 0.60:
        raise SystemExit('first-distance must be in [0.10, 0.60] m')
    if not 30.0 <= abs(args.turn_deg) <= 100.0:
        raise SystemExit('absolute turn-deg must be in [30, 100]')
    if not 0.10 <= args.second_distance <= 0.40:
        raise SystemExit('second-distance must be in [0.10, 0.40] m')

    rclpy.init()
    route = RouteNode()
    try:
        route.wait_ready()
        print(f'ROUTE_START {route.pose_text()}', flush=True)
        route.forward(args.first_distance)
        route.turn(args.turn_deg)
        route.forward(args.second_distance)
        print(f'ROUTE_COMPLETE {route.pose_text()}', flush=True)
        return 0
    finally:
        try:
            route.stop(duration=2.0)
        finally:
            route.node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
