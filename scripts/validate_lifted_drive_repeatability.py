#!/usr/bin/env python3
"""Measure lifted left/right encoder repeatability with one persistent publisher."""

import argparse
import json
import statistics
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import qos_profile_sensor_data
from carbot_msgs.msg import WheelTicks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--settle", type=float, default=1.5)
    parser.add_argument("--speeds", type=float, nargs="+", default=(0.02, 0.03, 0.04, 0.05))
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--state", choices=("lifted", "ground"), default="lifted")
    args = parser.parse_args()
    if not args.speeds or any(not 0.0 < speed <= 0.10 for speed in args.speeds):
        raise SystemExit("speeds must be in (0, 0.10]")
    if not 1 <= args.repeats <= 5:
        raise SystemExit("repeats must be in [1, 5]")

    rclpy.init()
    node = rclpy.create_node("carbot_lifted_drive_repeatability")
    publisher = node.create_publisher(Twist, "/cmd_vel_command", 10)
    ticks = []
    actuator = []
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
        lambda message: actuator.append(
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
            and len(ticks) >= 20
        )
        if ready:
            break
    if not ready:
        raise RuntimeError("preflight failed")
    if len({x[2] for x in ticks}) != 1 or ticks[-1][:2] != ticks[-10][:2]:
        raise RuntimeError("ESP restarted or tracks are not stationary")

    boot_id = ticks[-1][2]

    def send(linear, duration):
        message = Twist()
        message.linear.x = float(linear)
        first_output = len(actuator)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
        return actuator[first_output:]

    def stop(duration):
        send(0.0, duration)

    print("PRECHECK_OK: motion starts in 5 seconds", flush=True)
    for remaining in range(5, 0, -1):
        print(f"COUNTDOWN {remaining}", flush=True)
        stop(1.0)

    rows = []
    try:
        for speed in args.speeds:
            for repeat in range(1, args.repeats + 1):
                for direction in (1.0, -1.0):
                    command = direction * speed
                    before = ticks[-1]
                    outputs = send(command, args.duration)
                    stop(args.settle)
                    after = ticks[-1]
                    matching = [
                        x for x, z in outputs
                        if abs(z) < 1e-9 and abs(x - command) < 1e-6
                    ]
                    row = {
                        "speed_mps": speed,
                        "direction": "forward" if direction > 0 else "reverse",
                        "repeat": repeat,
                        "left_ticks": after[0] - before[0],
                        "right_ticks": after[1] - before[1],
                        "actuator_samples": len(matching),
                        "actuator_median_mps": (
                            statistics.median(matching) if matching else None
                        ),
                    }
                    rows.append(row)
                    print(json.dumps(row), flush=True)
    finally:
        stop(3.0)
        for _ in range(20):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)

    result = {
        "boot_id": boot_id,
        "boot_ids": sorted({x[2] for x in ticks}),
        "final_ticks_stationary": ticks[-1][:2] == ticks[-10][:2],
        "state": args.state,
        "rows": rows,
    }
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
