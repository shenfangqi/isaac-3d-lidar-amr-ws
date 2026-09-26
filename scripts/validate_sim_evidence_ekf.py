#!/usr/bin/env python3
"""Drive the isolated Isaac evidence mode and score EKF against ground truth."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Validator(Node):
    def __init__(self):
        super().__init__("validate_sim_evidence_ekf")
        self.ground_truth = None
        self.filtered = None
        self.ground_truth_samples = 0
        self.filtered_samples = 0
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._ground_truth, 10
        )
        self.create_subscription(Odometry, "/odom", self._filtered, 10)

    def _ground_truth(self, message):
        self.ground_truth = message
        self.ground_truth_samples += 1

    def _filtered(self, message):
        self.filtered = message
        self.filtered_samples += 1

    def command_for_elapsed(self, elapsed_s):
        message = Twist()
        if 0.5 <= elapsed_s < 3.5:
            message.linear.x = 0.05
        elif 5.0 <= elapsed_s < 8.0:
            message.angular.z = 0.5
        return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = Validator()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            elapsed = time.monotonic() - started
            node.publisher.publish(node.command_for_elapsed(elapsed))
            rclpy.spin_once(node, timeout_sec=0.02)
        for _ in range(10):
            node.publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if node.ground_truth is None or node.filtered is None:
        raise SystemExit("missing ground-truth or filtered odometry")
    dx = node.filtered.pose.pose.position.x - node.ground_truth.pose.pose.position.x
    dy = node.filtered.pose.pose.position.y - node.ground_truth.pose.pose.position.y
    yaw_error = math.atan2(
        math.sin(yaw(node.filtered) - yaw(node.ground_truth)),
        math.cos(yaw(node.filtered) - yaw(node.ground_truth)),
    )
    result = {
        "ground_truth_samples": node.ground_truth_samples,
        "filtered_samples": node.filtered_samples,
        "final_position_error_m": math.hypot(dx, dy),
        "final_yaw_error_rad": yaw_error,
        "checks": {
            "both_streams_healthy": min(
                node.ground_truth_samples, node.filtered_samples
            ) > 100,
            "position_error_below_2cm": math.hypot(dx, dy) < 0.02,
            "yaw_error_below_2deg": abs(yaw_error) < math.radians(2.0),
        },
    }
    result["passed"] = all(result["checks"].values())
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
