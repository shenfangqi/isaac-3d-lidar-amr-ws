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
        self.publisher = self.node.create_publisher(
            Twist, '/cmd_vel_command', 10)
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
                    and self.node.count_publishers('/cmd_vel_command') == 1
                    and self.node.count_publishers('/cmd_vel') == 1
                    and self.node.count_subscribers('/cmd_vel') >= 1):
                return
        raise RuntimeError(
            'route preflight failed: require fresh odom, this node as the '
            'only /cmd_vel_command publisher, one compensator publisher, '
            'and the ESP /cmd_vel subscriber')

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
        target_yaw = start_yaw + math.radians(degrees)
        tolerance = math.radians(1.5)
        remaining = math.radians(degrees)
        for _attempt in range(3):
            deadline = time.monotonic() + 12.0
            while time.monotonic() < deadline:
                self.spin_once(0.05)
                self.require_fresh_odom()
                current_yaw = yaw_from_quaternion(
                    self.odom.pose.pose.orientation)
                remaining = angle_delta(target_yaw, current_yaw)
                yaw_rate = self.odom.twist.twist.angular.z
                braking_angle = max(tolerance, abs(yaw_rate) * 0.35)
                if abs(remaining) <= braking_angle:
                    break
                self.send(angular=math.copysign(0.40, remaining))
            else:
                raise RuntimeError(
                    'turn phase timed out with '
                    f'{math.degrees(remaining):.1f} deg remaining')
            self.stop(duration=1.5)
            current_yaw = yaw_from_quaternion(
                self.odom.pose.pose.orientation)
            remaining = angle_delta(target_yaw, current_yaw)
            if abs(remaining) <= tolerance:
                break
        if abs(remaining) > tolerance:
            raise RuntimeError(
                'turn correction failed with '
                f'{math.degrees(remaining):.1f} deg remaining')
        turned = angle_delta(
            yaw_from_quaternion(self.odom.pose.pose.orientation), start_yaw)
        print(
            f'TURN_DONE target={degrees:.1f} '
            f'actual={math.degrees(turned):.1f} '
            f'error={math.degrees(remaining):.1f}',
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
