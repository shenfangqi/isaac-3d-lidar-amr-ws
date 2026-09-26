#!/usr/bin/env python3
"""Run one guarded lifted-track command and quantify the firmware watchdog stop."""

import argparse
from collections import Counter
import math
import time

from carbot_msgs.msg import CarbotStatus, WheelTicks
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--armed", action="store_true", help="confirm lifted safe setup")
    parser.add_argument(
        "--ground-armed",
        action="store_true",
        help="confirm a clear ground path and an immediately reachable cutoff",
    )
    parser.add_argument("--linear", type=float, default=0.05)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument(
        "--command-seconds",
        type=float,
        default=0.0,
        help="republish at 10 Hz for this duration; zero keeps the watchdog probe",
    )
    parser.add_argument("--observe-seconds", type=float, default=3.0)
    return parser.parse_args()


class Probe(Node):
    def __init__(self):
        super().__init__("carbot_lifted_watchdog_probe")
        self.ticks = []
        self.statuses = []
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(
            WheelTicks, "/wheel_ticks", self._ticks_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CarbotStatus, "/carbot/status", self._status_callback, qos_profile_sensor_data
        )

    def _ticks_callback(self, msg):
        self.ticks.append((time.monotonic_ns(), msg))

    def _status_callback(self, msg):
        self.statuses.append((time.monotonic_ns(), msg))

    def publish_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)


def spin_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    args = parse_args()
    if not (args.armed or args.ground_armed):
        raise SystemExit(
            "refusing motion: pass --armed for lifted tests or --ground-armed "
            "after confirming a clear path and reachable cutoff"
        )
    if not math.isfinite(args.linear) or not math.isfinite(args.angular):
        raise SystemExit("command must be finite")
    if abs(args.linear) > 0.10 or abs(args.angular) > 0.30:
        raise SystemExit("probe command exceeds the guarded lifted-test envelope")

    rclpy.init()
    node = Probe()
    command_ns = None
    try:
        ready = spin_until(
            node,
            lambda: (
                node.cmd_pub.get_subscription_count() >= 1
                and len(node.ticks) >= 5
                and node.statuses
                and node.statuses[-1][1].agent_connected
                and node.statuses[-1][1].time_synchronized
            ),
            15.0,
        )
        if not ready:
            raise RuntimeError(
                "physical endpoint not ready: require a cmd_vel subscriber, ticks, "
                "agent connection, and synchronized time"
            )

        baseline_index = len(node.ticks) - 1
        invalid_before = node.statuses[-1][1].invalid_cmd_count
        print(
            "READY boot_id=%d cmd_subscribers=%d battery_low=%s motion_blocked=%s"
            % (
                node.ticks[-1][1].boot_id,
                node.cmd_pub.get_subscription_count(),
                node.statuses[-1][1].battery_low,
                node.statuses[-1][1].motion_blocked,
            )
        )

        command_ns = time.monotonic_ns()
        node.publish_command(args.linear, args.angular)
        command_deadline = time.monotonic() + args.command_seconds
        next_publish = time.monotonic() + 0.1
        while time.monotonic() < command_deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() >= next_publish:
                node.publish_command(args.linear, args.angular)
                next_publish += 0.1
        command_stop_ns = time.monotonic_ns()
        deadline = time.monotonic() + args.observe_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if rclpy.ok():
            for _ in range(10):
                node.publish_command(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)

    if command_ns is None:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    samples = node.ticks[baseline_index:]
    unique = []
    seen_sequences = set()
    for host_ns, msg in samples:
        if msg.sequence in seen_sequences:
            continue
        seen_sequences.add(msg.sequence)
        unique.append((host_ns, msg))

    movements = []
    for (host_a, msg_a), (host_b, msg_b) in zip(unique, unique[1:]):
        left_delta = msg_b.left_ticks - msg_a.left_ticks
        right_delta = msg_b.right_ticks - msg_a.right_ticks
        if left_delta or right_delta:
            movements.append((host_b, left_delta, right_delta))

    first = unique[0][1]
    last = unique[-1][1]
    sources = Counter(msg.active_command_source for _, msg in node.statuses)
    invalid_after = node.statuses[-1][1].invalid_cmd_count
    final_window = unique[-25:] if len(unique) >= 25 else unique
    final_left_span = final_window[-1][1].left_ticks - final_window[0][1].left_ticks
    final_right_span = final_window[-1][1].right_ticks - final_window[0][1].right_ticks

    print("RESULT samples=%d unique=%d duplicates=%d" % (
        len(samples), len(unique), len(samples) - len(unique)
    ))
    print(
        "RESULT tick_delta left=%d right=%d"
        % (last.left_ticks - first.left_ticks, last.right_ticks - first.right_ticks)
    )
    if movements:
        print(
            "RESULT first_motion_ms=%.1f last_motion_after_command_stop_ms=%.1f "
            "moving_samples=%d"
            % (
                (movements[0][0] - command_ns) / 1.0e6,
                (movements[-1][0] - command_stop_ns) / 1.0e6,
                len(movements),
            )
        )
    else:
        print("RESULT no wheel motion observed")
    print(
        "RESULT final_0.5s_tick_span left=%d right=%d active_sources=%s invalid_delta=%d"
        % (final_left_span, final_right_span, dict(sources), invalid_after - invalid_before)
    )

    left_delta_total = last.left_ticks - first.left_ticks
    right_delta_total = last.right_ticks - first.right_ticks
    expected_left = args.linear - args.angular * 0.254 * 0.5
    expected_right = args.linear + args.angular * 0.254 * 0.5

    def sign_matches(measured, expected):
        if abs(expected) < 1.0e-9:
            return abs(measured) <= 1
        return measured * expected > 0

    wheel_sign_ok = sign_matches(left_delta_total, expected_left) and sign_matches(
        right_delta_total, expected_right
    )
    settled_ok = final_left_span == 0 and final_right_span == 0
    invalid_ok = invalid_after == invalid_before
    print(
        "PASS wheel_tick_sign=%s watchdog_settled=%s command_valid=%s zero_sent=true"
        % (wheel_sign_ok, settled_ok, invalid_ok)
    )

    node.destroy_node()
    rclpy.shutdown()
    if not (wheel_sign_ok and settled_ok and invalid_ok):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
