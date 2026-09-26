#!/usr/bin/env python3
"""Validate repeatable real-robot yaw targets using fused odometry feedback."""

import argparse
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from carbot_msgs.msg import WheelTicks


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--target-deg", type=float, default=30.0)
    parser.add_argument("--pairs", type=int, default=4)
    parser.add_argument("--angular", type=float, default=0.40)
    parser.add_argument("--tolerance-deg", type=float, default=1.5)
    parser.add_argument("--brake-horizon", type=float, default=0.35)
    args = parser.parse_args()
    if not 10.0 <= args.target_deg <= 90.0:
        raise SystemExit("target-deg must be in [10, 90]")
    if not 1 <= args.pairs <= 8:
        raise SystemExit("pairs must be in [1, 8]")
    if not 0.40 <= args.angular <= 0.50:
        raise SystemExit("angular must be in the measured reliable [0.40, 0.50] range")

    rclpy.init()
    node = rclpy.create_node("carbot_closed_loop_yaw_acceptance")
    publisher = node.create_publisher(Twist, "/cmd_vel_command", 10)
    poses = []
    ticks = []
    outputs = []
    node.create_subscription(
        Odometry,
        "/odom",
        lambda message: poses.append(
            (
                time.monotonic(),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                yaw(message),
                float(message.twist.twist.angular.z),
            )
        ),
        10,
    )
    node.create_subscription(
        WheelTicks,
        "/wheel_ticks",
        lambda message: ticks.append(
            (int(message.left_ticks), int(message.right_ticks), int(message.boot_id))
        ),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Twist,
        "/cmd_vel",
        lambda message: outputs.append(
            (float(message.linear.x), float(message.angular.z))
        ),
        10,
    )

    ready = False
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = (
            node.count_subscribers("/cmd_vel_command") >= 1
            and node.count_publishers("/cmd_vel") == 1
            and node.count_subscribers("/cmd_vel") >= 2
            and len(poses) >= 20
            and len(ticks) >= 20
        )
        if ready:
            break
    if not ready:
        raise RuntimeError("preflight failed")
    if len({sample[2] for sample in ticks}) != 1:
        raise RuntimeError("ESP boot_id changed during preflight")
    if ticks[-1][:2] != ticks[-10][:2]:
        raise RuntimeError("tracks are not stationary")
    boot_id = ticks[-1][2]

    def zero(duration):
        message = Twist()
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)

    tolerance = math.radians(args.tolerance_deg)

    def seek(target_yaw):
        attempts = []
        for attempt in range(1, 4):
            start_pose = poses[-1]
            start_ticks = ticks[-1]
            start_outputs = len(outputs)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                stamp, _x, _y, current_yaw, yaw_rate = poses[-1]
                if time.monotonic() - stamp > 0.25:
                    raise RuntimeError("stale /odom during motion")
                error = wrap(target_yaw - current_yaw)
                braking_angle = max(tolerance, abs(yaw_rate) * args.brake_horizon)
                if abs(error) <= braking_angle:
                    break
                message = Twist()
                message.angular.z = math.copysign(args.angular, error)
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.05)
            else:
                raise RuntimeError("yaw target timeout")
            zero(1.5)
            final_pose = poses[-1]
            final_ticks = ticks[-1]
            error = wrap(target_yaw - final_pose[3])
            segment_outputs = outputs[start_outputs:]
            nonzero_outputs = [z for x, z in segment_outputs if abs(x) < 1e-9 and abs(z) > 1e-6]
            attempts.append({
                "attempt": attempt,
                "motion_deg": math.degrees(wrap(final_pose[3] - start_pose[3])),
                "remaining_error_deg": math.degrees(error),
                "tick_delta": [
                    final_ticks[0] - start_ticks[0],
                    final_ticks[1] - start_ticks[1],
                ],
                "actuator_command_mean_rad_s": (
                    sum(nonzero_outputs) / len(nonzero_outputs)
                    if nonzero_outputs else None
                ),
            })
            if abs(error) <= tolerance:
                return attempts
        return attempts

    print("PRECHECK_OK: closed-loop motion starts in 5 seconds", flush=True)
    for remaining in range(5, 0, -1):
        print(f"COUNTDOWN {remaining}", flush=True)
        zero(1.0)

    results = []
    test_start = poses[-1]
    try:
        for pair in range(1, args.pairs + 1):
            baseline = poses[-1]
            direction = 1.0 if pair % 2 else -1.0
            away_target = wrap(baseline[3] + direction * math.radians(args.target_deg))
            away_attempts = seek(away_target)
            return_attempts = seek(baseline[3])
            final_pose = poses[-1]
            result = {
                "pair": pair,
                "direction": "left_then_right" if direction > 0 else "right_then_left",
                "away_attempts": away_attempts,
                "return_attempts": return_attempts,
                "return_yaw_error_deg": math.degrees(wrap(final_pose[3] - baseline[3])),
                "return_position_error_m": math.hypot(
                    final_pose[1] - baseline[1], final_pose[2] - baseline[2]
                ),
            }
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        zero(3.0)
        for _ in range(20):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    test_end = poses[-1]
    report = {
        "controller": {
            "feedback_topic": "/odom",
            "angular_command_rad_s": args.angular,
            "target_deg": args.target_deg,
            "tolerance_deg": args.tolerance_deg,
            "brake_horizon_s": args.brake_horizon,
            "max_correction_attempts": 3,
        },
        "pairs": results,
        "cumulative_yaw_error_deg": math.degrees(wrap(test_end[3] - test_start[3])),
        "cumulative_position_error_m": math.hypot(
            test_end[1] - test_start[1], test_end[2] - test_start[2]
        ),
        "boot_id": boot_id,
        "boot_ids": sorted({sample[2] for sample in ticks}),
        "final_ticks_stationary": ticks[-1][:2] == ticks[-10][:2],
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
