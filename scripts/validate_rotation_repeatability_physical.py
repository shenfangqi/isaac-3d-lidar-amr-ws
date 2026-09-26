#!/usr/bin/env python3
"""Run a supervised repeated in-place rotation return test on real Carbot."""

import argparse
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from carbot_msgs.msg import WheelTicks


def yaw_from_odometry(message):
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--pairs", type=int, default=4)
    parser.add_argument("--angular", type=float, default=0.30)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=2.5)
    parser.add_argument("--candidate-right-scale", type=float, default=0.896)
    args = parser.parse_args()
    if not 1 <= args.pairs <= 8:
        raise SystemExit("pairs must be in [1, 8]")
    if not 0.0 < args.angular <= 0.5:
        raise SystemExit("angular must be in (0, 0.5]")
    if not 0.0 < args.duration <= 3.0:
        raise SystemExit("duration must be in (0, 3.0]")
    if not 1.0 <= args.settle <= 5.0:
        raise SystemExit("settle must be in [1.0, 5.0]")
    if not 0.0 < args.candidate_right_scale <= 1.0:
        raise SystemExit("candidate-right-scale must be in (0, 1]")

    rclpy.init()
    node = rclpy.create_node("carbot_rotation_repeatability_acceptance")
    command_publisher = node.create_publisher(Twist, "/cmd_vel_command", 10)
    marker_publisher = node.create_publisher(String, "/calibration/segment", 10)
    poses = []
    wheel_poses = []
    ticks = []
    actuator_commands = []
    node.create_subscription(
        Odometry,
        "/odom",
        lambda message: poses.append(
            (float(message.pose.pose.position.x),
             float(message.pose.pose.position.y),
             yaw_from_odometry(message))
        ),
        10,
    )
    node.create_subscription(
        Odometry,
        "/wheel/odom",
        lambda message: wheel_poses.append(
            (float(message.pose.pose.position.x),
             float(message.pose.pose.position.y),
             yaw_from_odometry(message))
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
        lambda message: actuator_commands.append(
            (float(message.linear.x), float(message.angular.z))
        ),
        10,
    )

    deadline = time.monotonic() + 30.0
    ready = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = (
            node.count_subscribers("/cmd_vel_command") >= 1
            and node.count_publishers("/cmd_vel") == 1
            and node.count_subscribers("/cmd_vel") >= 2
            and len(poses) >= 10
            and len(wheel_poses) >= 10
            and len(ticks) >= 20
        )
        if ready:
            break
    if not ready:
        raise RuntimeError("preflight failed: control or odometry topics are not ready")
    if len({sample[2] for sample in ticks}) != 1:
        raise RuntimeError("ESP boot_id changed during preflight")
    if ticks[-1][:2] != ticks[-10][:2]:
        raise RuntimeError("wheel ticks changed during stationary preflight")

    boot_id = ticks[-1][2]
    samples = []

    def mark(label):
        message = String()
        message.data = label
        marker_publisher.publish(message)

    def publish(linear, angular, duration):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        first_output = len(actuator_commands)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            command_publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        return actuator_commands[first_output:]

    def stop(duration):
        publish(0.0, 0.0, duration)

    print("PRECHECK_OK: motion starts in 5 seconds", flush=True)
    for remaining in range(5, 0, -1):
        print(f"COUNTDOWN {remaining}", flush=True)
        stop(1.0)

    test_start = poses[-1]
    wheel_test_start = wheel_poses[-1]
    try:
        for pair_index in range(args.pairs):
            directions = ("left", "right") if pair_index % 2 == 0 else ("right", "left")
            pair_start = poses[-1]
            wheel_pair_start = wheel_poses[-1]
            pair_ticks_start = ticks[-1]
            segment_results = []
            for direction in directions:
                angular = (
                    args.angular
                    if direction == "left"
                    else -args.angular * args.candidate_right_scale / 0.896
                )
                label = f"pair_{pair_index + 1}_{direction}"
                mark(label + "_start")
                before = poses[-1]
                wheel_before = wheel_poses[-1]
                outputs = publish(0.0, angular, args.duration)
                stop(args.settle)
                mark(label + "_stop")
                after = poses[-1]
                wheel_after = wheel_poses[-1]
                nonzero = [z for x, z in outputs if abs(x) < 1e-9 and abs(z) > 1e-6]
                segment_results.append({
                    "direction": direction,
                    "ekf_angle_deg": math.degrees(wrap_angle(after[2] - before[2])),
                    "wheel_angle_deg": math.degrees(wrap_angle(wheel_after[2] - wheel_before[2])),
                    "actuator_command_mean_rad_s": (
                        sum(nonzero) / len(nonzero) if nonzero else None
                    ),
                    "actuator_nonzero_samples": len(nonzero),
                })
            pair_end = poses[-1]
            wheel_pair_end = wheel_poses[-1]
            pair_ticks_end = ticks[-1]
            result = {
                "pair": pair_index + 1,
                "order": directions,
                "segments": segment_results,
                "ekf_residual_deg": math.degrees(wrap_angle(pair_end[2] - pair_start[2])),
                "wheel_residual_deg": math.degrees(
                    wrap_angle(wheel_pair_end[2] - wheel_pair_start[2])
                ),
                "ekf_position_drift_m": math.hypot(
                    pair_end[0] - pair_start[0], pair_end[1] - pair_start[1]
                ),
                "wheel_position_drift_m": math.hypot(
                    wheel_pair_end[0] - wheel_pair_start[0],
                    wheel_pair_end[1] - wheel_pair_start[1],
                ),
                "tick_delta": [
                    pair_ticks_end[0] - pair_ticks_start[0],
                    pair_ticks_end[1] - pair_ticks_start[1],
                ],
            }
            samples.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        stop(3.0)
        for _ in range(20):
            command_publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    test_end = poses[-1]
    wheel_test_end = wheel_poses[-1]
    result = {
        "right_turn_command_scale": 0.896,
        "candidate_effective_right_turn_scale": args.candidate_right_scale,
        "state": "warm",
        "pairs": samples,
        "final_ekf_yaw_error_deg": math.degrees(
            wrap_angle(test_end[2] - test_start[2])
        ),
        "final_wheel_yaw_error_deg": math.degrees(
            wrap_angle(wheel_test_end[2] - wheel_test_start[2])
        ),
        "final_ekf_position_error_m": math.hypot(
            test_end[0] - test_start[0], test_end[1] - test_start[1]
        ),
        "boot_id": boot_id,
        "boot_ids": sorted({sample[2] for sample in ticks}),
        "final_ticks_stationary": ticks[-1][:2] == ticks[-10][:2],
    }
    if result["boot_ids"] != [boot_id]:
        raise RuntimeError("ESP boot_id changed during test")
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
