#!/usr/bin/env python3
"""Run the final supervised Carbot command-compensation return test."""

import argparse
import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
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
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("carbot_command_compensation_acceptance")
    publisher = node.create_publisher(Twist, "/cmd_vel_command", 10)
    poses = []
    ticks = []
    actuator_commands = []
    node.create_subscription(
        Odometry,
        "/odom",
        lambda message: poses.append(
            (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                yaw_from_odometry(message),
            )
        ),
        10,
    )
    node.create_subscription(
        WheelTicks,
        "/wheel_ticks",
        lambda message: ticks.append(
            (
                int(message.left_ticks),
                int(message.right_ticks),
                int(message.boot_id),
            )
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

    # With Linger disabled, a fresh SSH login starts the user services and the
    # ESP32 may need about 25 seconds to recreate its micro-ROS session.
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready = (
            node.count_subscribers("/cmd_vel_command") >= 1
            and node.count_publishers("/cmd_vel") == 1
            and node.count_subscribers("/cmd_vel") >= 2
            and len(poses) >= 5
            and len(ticks) >= 20
        )
        if ready:
            break
    if not ready:
        raise RuntimeError(
            "preflight failed: expected compensator, ESP subscriber, fresh "
            f"odom and ticks; command_subscribers="
            f"{node.count_subscribers('/cmd_vel_command')}, "
            f"actuator_publishers={node.count_publishers('/cmd_vel')}, "
            f"actuator_subscribers={node.count_subscribers('/cmd_vel')}, "
            f"odom_samples={len(poses)}, tick_samples={len(ticks)}"
        )
    if len({sample[2] for sample in ticks}) != 1:
        raise RuntimeError("ESP boot_id changed during preflight")
    if ticks[-1][:2] != ticks[-10][:2]:
        raise RuntimeError("wheel ticks changed during stationary preflight")

    marks = {"start": poses[-1]}
    command_windows = {}

    def publish_segment(name, linear, angular, duration):
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        first_output = len(actuator_commands)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        marks[name] = poses[-1]
        command_windows[name] = actuator_commands[first_output:]

    def stop(name, duration=2.0):
        message = Twist()
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        marks[name] = poses[-1]

    print("PRECHECK_OK: motion starts in 5 seconds", flush=True)
    for remaining in range(5, 0, -1):
        print(f"COUNTDOWN {remaining}", flush=True)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        publish_segment("forward", 0.08, 0.0, 3.0)
        stop("forward_stop")
        publish_segment("reverse", -0.08, 0.0, 3.0)
        stop("linear_return")
        publish_segment("left", 0.0, 0.30, 3.0)
        stop("left_stop")
        publish_segment("right", 0.0, -0.30, 3.0)
        stop("final", 3.0)
    finally:
        for _ in range(30):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    start = marks["start"]
    linear_return = marks["linear_return"]
    final = marks["final"]
    left_angle = wrap_angle(marks["left_stop"][2] - linear_return[2])
    right_angle = wrap_angle(final[2] - marks["left_stop"][2])
    right_outputs = [
        angular
        for linear, angular in command_windows["right"]
        if abs(linear) < 1e-9 and angular < -1e-6
    ]
    result = {
        "right_turn_command_scale": 0.896,
        "marks": marks,
        "linear_return_error_m": math.hypot(
            linear_return[0] - start[0], linear_return[1] - start[1]
        ),
        "final_position_error_m": math.hypot(
            final[0] - start[0], final[1] - start[1]
        ),
        "left_angle_deg": math.degrees(left_angle),
        "right_angle_deg": math.degrees(right_angle),
        "turn_pair_residual_deg": math.degrees(left_angle + right_angle),
        "final_yaw_error_deg": math.degrees(wrap_angle(final[2] - start[2])),
        "right_actuator_command_mean_rad_s": (
            sum(right_outputs) / len(right_outputs) if right_outputs else None
        ),
        "tick_start": ticks[0],
        "tick_final": ticks[-1],
        "boot_ids": sorted({sample[2] for sample in ticks}),
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
