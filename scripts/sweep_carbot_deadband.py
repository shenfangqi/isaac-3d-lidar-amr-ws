#!/usr/bin/env python3
"""Run a guarded lifted-track forward/reverse deadband sweep."""

import argparse
import time

from carbot_msgs.msg import CarbotStatus, WheelTicks
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--armed", action="store_true")
    parser.add_argument(
        "--speeds",
        default="0.01,0.02,0.03,0.05,0.08,-0.01,-0.02,-0.03,-0.05,-0.08",
    )
    parser.add_argument("--hold-seconds", type=float, default=1.2)
    parser.add_argument("--zero-seconds", type=float, default=0.8)
    return parser.parse_args()


class Sweep(Node):
    def __init__(self):
        super().__init__("carbot_lifted_deadband_sweep")
        self.latest_ticks = None
        self.latest_status = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(
            WheelTicks, "/wheel_ticks", self._ticks_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CarbotStatus, "/carbot/status", self._status_callback, qos_profile_sensor_data
        )

    def _ticks_callback(self, msg):
        if self.latest_ticks is None or msg.boot_id != self.latest_ticks.boot_id:
            self.latest_ticks = msg
            return
        if msg.sequence > self.latest_ticks.sequence:
            self.latest_ticks = msg

    def _status_callback(self, msg):
        self.latest_status = msg

    def publish(self, linear_mps):
        msg = Twist()
        msg.linear.x = linear_mps
        self.publisher.publish(msg)

    def hold(self, linear_mps, duration_s):
        deadline = time.monotonic() + duration_s
        next_publish = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                self.publish(linear_mps)
                next_publish = time.monotonic() + 0.1
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    args = parse_args()
    speeds = [float(item) for item in args.speeds.split(",")]
    if not args.armed:
        raise SystemExit("refusing motion: pass --armed only with both tracks lifted")
    if any(abs(speed) > 0.10 for speed in speeds):
        raise SystemExit("speed exceeds the guarded lifted-test envelope")

    rclpy.init()
    node = Sweep()
    try:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.publisher.get_subscription_count() >= 1
                and node.latest_ticks is not None
                and node.latest_status is not None
                and node.latest_status.agent_connected
                and node.latest_status.time_synchronized
            ):
                break
        else:
            raise RuntimeError("physical endpoint did not become ready")

        print(
            "READY boot_id=%d cmd_subscribers=%d"
            % (node.latest_ticks.boot_id, node.publisher.get_subscription_count())
        )
        for speed in speeds:
            start = node.latest_ticks
            node.hold(speed, args.hold_seconds)
            end = node.latest_ticks
            node.hold(0.0, args.zero_seconds)
            settled = node.latest_ticks
            print(
                "LEVEL linear_mps=%+.3f active_ticks=(%+d,%+d) "
                "post_zero_ticks=(%+d,%+d)"
                % (
                    speed,
                    end.left_ticks - start.left_ticks,
                    end.right_ticks - start.right_ticks,
                    settled.left_ticks - end.left_ticks,
                    settled.right_ticks - end.right_ticks,
                )
            )
    finally:
        if rclpy.ok():
            for _ in range(10):
                node.publish(0.0)
                rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("ZERO_SENT count=10")


if __name__ == "__main__":
    main()
