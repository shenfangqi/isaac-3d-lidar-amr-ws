#!/usr/bin/env python3
"""Reliably publish an AMCL initial pose after subscriber discovery."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    parser.add_argument('--xy-variance', type=float, default=0.03)
    parser.add_argument('--yaw-variance', type=float, default=0.02)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('carbot_initial_pose_setter')
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', qos
    )
    try:
        deadline = time.monotonic() + 8.0
        while publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise RuntimeError('no /initialpose subscriber discovered within 8 seconds')
            rclpy.spin_once(node, timeout_sec=0.1)

        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = args.x
        message.pose.pose.position.y = args.y
        message.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        message.pose.covariance[0] = args.xy_variance
        message.pose.covariance[7] = args.xy_variance
        message.pose.covariance[35] = args.yaw_variance

        for _ in range(10):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.2)

        print(
            f'INITIAL_POSE_PUBLISHED subscribers={publisher.get_subscription_count()} '
            f'x={args.x:.3f} y={args.y:.3f} yaw={args.yaw:.4f}',
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
