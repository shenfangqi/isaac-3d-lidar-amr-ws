#!/usr/bin/env python3
"""Audit whether a bag can support MID-360/wheel dynamic fusion A/B."""

import argparse
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

from carbot_msgs.msg import WheelTicks


MID360_IMU_TOPIC = "/mid360/imu/data_raw"
WHEEL_TOPIC = "/wheel_ticks"
COMMAND_TOPIC = "/cmd_vel"
FORBIDDEN_IMU_TOPIC = "/imu/data_raw"


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def robust_rate_hz(stamps):
    values = np.asarray(stamps, dtype=float)
    if values.size < 2:
        return None
    intervals = np.diff(values)
    intervals = intervals[intervals > 0.0]
    if intervals.size == 0:
        return None
    return float(1.0 / np.median(intervals))


def motion_ticks(first, last):
    return {
        "left": int(last[0] - first[0]),
        "right": int(last[1] - first[1]),
    }


def audit_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {item.name for item in reader.get_all_topics_and_types()}
    imu_stamps = []
    imu_frames = set()
    wheel = []
    boot_ids = set()
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == MID360_IMU_TOPIC:
            message = deserialize_message(serialized, Imu)
            imu_stamps.append(stamp_seconds(message.header.stamp))
            imu_frames.add(message.header.frame_id)
        elif topic == WHEEL_TOPIC:
            message = deserialize_message(serialized, WheelTicks)
            wheel.append((message.left_ticks, message.right_ticks))
            boot_ids.add(int(message.boot_id))

    deltas = motion_ticks(wheel[0], wheel[-1]) if len(wheel) >= 2 else None
    wheel_moved = bool(
        deltas and (abs(deltas["left"]) > 5 or abs(deltas["right"]) > 5)
    )
    checks = {
        "mid360_imu_present": len(imu_stamps) >= 2,
        "mid360_frame_is_imu_link": imu_frames == {"imu_link"},
        "wheel_ticks_present": len(wheel) >= 2,
        "single_esp_boot": len(boot_ids) == 1,
        "wheel_motion_present": wheel_moved,
        "command_topic_present": COMMAND_TOPIC in topics,
    }
    dynamic_ready = all(checks.values())
    return {
        "sensor_policy": {
            "imu_source": MID360_IMU_TOPIC,
            "esp_imu_allowed": False,
            "esp_imu_topic_seen_but_ignored": FORBIDDEN_IMU_TOPIC in topics,
            "esp_role": "wheel_ticks_and_base_control_only",
        },
        "bag": str(path),
        "imu": {
            "samples": len(imu_stamps),
            "frames": sorted(imu_frames),
            "median_rate_hz": robust_rate_hz(imu_stamps),
        },
        "wheel": {
            "samples": len(wheel),
            "boot_ids": sorted(boot_ids),
            "tick_delta": deltas,
        },
        "checks": checks,
        "dynamic_ab_ready": dynamic_ready,
        "decision": (
            "eligible_for_mid360_wheel_dynamic_ab"
            if dynamic_ready
            else "not_a_driven_ab_bag_do_not_enable_live_fusion"
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit_bag(args.bag)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
