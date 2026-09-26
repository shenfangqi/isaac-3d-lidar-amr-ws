#!/usr/bin/env python3
"""Compute and summarize one Nav2 path without commanding motion."""

import argparse
import math
import time

import rclpy
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True)
    return parser.parse_args()


def wait_future(node, future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return future.done()


def main():
    args = parse_args()
    rclpy.init()
    node = rclpy.create_node('carbot_compute_path_summary')
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('ComputePathToPose action server unavailable')
        settle_deadline = time.monotonic() + 3.0
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x = args.x
        goal.goal.pose.position.y = args.y
        goal.goal.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(args.yaw / 2.0)
        goal.planner_id = 'GridBased'
        goal.use_start = False

        send_future = client.send_goal_async(goal)
        if not wait_future(node, send_future, 20.0):
            raise RuntimeError('timed out waiting for path goal response')
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('path goal rejected')
        result_future = handle.get_result_async()
        if not wait_future(node, result_future, 20.0):
            raise RuntimeError('timed out waiting for path result')
        wrapped = result_future.result()
        poses = wrapped.result.path.poses
        if not poses:
            raise RuntimeError(f'planner returned no poses, status={wrapped.status}')

        length = 0.0
        for first, second in zip(poses, poses[1:]):
            length += math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        approach_yaw = args.yaw
        if len(poses) >= 2:
            first = poses[-2].pose.position
            second = poses[-1].pose.position
            approach_yaw = math.atan2(second.y - first.y, second.x - first.x)
        start = poses[0].pose.position
        end = poses[-1].pose.position
        print(
            f'PATH status={wrapped.status} poses={len(poses)} '
            f'length={length:.3f} '
            f'start=({start.x:.3f},{start.y:.3f}) '
            f'end=({end.x:.3f},{end.y:.3f}) '
            f'approach_yaw={approach_yaw:.3f} '
            f'approach_deg={math.degrees(approach_yaw):.1f}',
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
