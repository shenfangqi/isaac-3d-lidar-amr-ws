#!/usr/bin/env python3
"""Read-only monitor for a single real-robot Nav2 goal attempt."""

import argparse
import math
import time

from action_msgs.msg import GoalStatusArray
from carbot_msgs.msg import WheelTicks
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data


class Monitor:
    def __init__(self, node):
        self.node = node
        self.started = time.monotonic()
        self.commands = {"upstream": [], "output": []}
        self.ticks = []
        self.odom = []
        self.statuses = []
        node.create_subscription(
            Twist, "/cmd_vel_command", lambda msg: self._command("upstream", msg), 10
        )
        node.create_subscription(
            Twist, "/cmd_vel", lambda msg: self._command("output", msg), 10
        )
        node.create_subscription(
            WheelTicks, "/wheel_ticks", self._ticks, qos_profile_sensor_data
        )
        node.create_subscription(
            Odometry, "/odom", self._odom, qos_profile_sensor_data
        )
        node.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._status,
            qos_profile_sensor_data,
        )

    def _now(self):
        return time.monotonic() - self.started

    def _command(self, name, msg):
        self.commands[name].append((self._now(), msg.linear.x, msg.angular.z))

    def _ticks(self, msg):
        self.ticks.append((self._now(), msg.left_ticks, msg.right_ticks))

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.odom.append((self._now(), p.x, p.y))

    def _status(self, msg):
        if msg.status_list:
            self.statuses.append((self._now(), msg.status_list[-1].status))


def command_summary(samples):
    nonzero = [s for s in samples if abs(s[1]) > 1e-4 or abs(s[2]) > 1e-4]
    return (
        len(samples),
        len(nonzero),
        max((abs(s[1]) for s in samples), default=0.0),
        max((abs(s[2]) for s in samples), default=0.0),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("carbot_nav_command_monitor")
    monitor = Monitor(node)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    for name in ("upstream", "output"):
        count, nonzero, max_linear, max_angular = command_summary(
            monitor.commands[name]
        )
        print(
            f"CMD_{name.upper()} samples={count} nonzero={nonzero} "
            f"max_linear={max_linear:.4f} max_angular={max_angular:.4f}"
        )
    if len(monitor.ticks) >= 2:
        first, last = monitor.ticks[0], monitor.ticks[-1]
        print(
            f"TICKS samples={len(monitor.ticks)} "
            f"left_delta={last[1] - first[1]} right_delta={last[2] - first[2]}"
        )
    else:
        print(f"TICKS samples={len(monitor.ticks)}")
    if len(monitor.odom) >= 2:
        first, last = monitor.odom[0], monitor.odom[-1]
        distance = math.hypot(last[1] - first[1], last[2] - first[2])
        print(f"ODOM samples={len(monitor.odom)} displacement={distance:.4f}")
    else:
        print(f"ODOM samples={len(monitor.odom)}")
    print("ACTION statuses=" + ",".join(str(status) for _, status in monitor.statuses))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
