#!/usr/bin/env python3
"""Validate the live MID-360 ROS contract without commanding the robot."""

import argparse
import json
import statistics
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


FULL_FIELDS = ["x", "y", "z", "intensity", "tag", "line", "timestamp"]


class Mid360Validator(Node):
    def __init__(self):
        super().__init__("mid360_live_validator")
        self.receive_times = {"raw": [], "compact": [], "imu": []}
        self.raw_message = None
        self.raw_imu_stamps = {}
        self.adapted_imu_stamps = {}
        self.adapted_imu_message = None
        self.create_subscription(
            PointCloud2,
            "/livox/lidar",
            self._raw_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/mid360/points_xyz",
            lambda message: self._record("compact", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/mid360/imu/data_raw",
            self._adapted_imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/livox/imu",
            self._raw_imu_callback,
            qos_profile_sensor_data,
        )

    def _record(self, stream, _message):
        self.receive_times[stream].append(time.monotonic())

    def _raw_callback(self, message):
        self._record("raw", message)
        self.raw_message = message

    @staticmethod
    def _imu_key(message):
        return (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )

    @staticmethod
    def _stamp_seconds(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def _raw_imu_callback(self, message):
        self.raw_imu_stamps[self._imu_key(message)] = self._stamp_seconds(
            message
        )

    def _adapted_imu_callback(self, message):
        self._record("imu", message)
        self.adapted_imu_stamps[self._imu_key(message)] = self._stamp_seconds(
            message
        )
        self.adapted_imu_message = message


def observed_rate(times):
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


def point_timestamp_summary(message):
    fields = {field.name: field for field in message.fields}
    timestamp = fields["timestamp"]
    count = message.width * message.height
    byte_order = ">" if message.is_bigendian else "<"
    first = struct.unpack_from(
        byte_order + "d", message.data, timestamp.offset
    )[0]
    last = struct.unpack_from(
        byte_order + "d",
        message.data,
        (count - 1) * message.point_step + timestamp.offset,
    )[0]
    return first, last


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=6.0)
    args = parser.parse_args()

    rclpy.init()
    node = Mid360Validator()
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        message = node.raw_message
        if message is None:
            raise RuntimeError("no /livox/lidar message received")
        fields = [field.name for field in message.fields]
        if fields != FULL_FIELDS:
            raise RuntimeError(
                f"raw fields {fields!r} do not match {FULL_FIELDS!r}"
            )

        first_raw_timestamp, last_raw_timestamp = point_timestamp_summary(
            message
        )
        timestamp_unit = (
            "nanoseconds" if abs(first_raw_timestamp) > 1.0e14 else "seconds"
        )
        timestamp_scale = 1.0e-9 if timestamp_unit == "nanoseconds" else 1.0
        first_timestamp = first_raw_timestamp * timestamp_scale
        last_timestamp = last_raw_timestamp * timestamp_scale
        header_timestamp = (
            message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        )
        matching_imu_keys = (
            node.raw_imu_stamps.keys() & node.adapted_imu_stamps.keys()
        )
        imu_stamp_deltas = [
            node.adapted_imu_stamps[key] - node.raw_imu_stamps[key]
            for key in matching_imu_keys
        ]
        if not imu_stamp_deltas or node.adapted_imu_message is None:
            raise RuntimeError("no matching raw/adapted IMU samples received")
        summary = {
            "raw_fields": fields,
            "raw_frame": message.header.frame_id,
            "raw_points": message.width * message.height,
            "raw_point_step": message.point_step,
            "raw_rate_hz": observed_rate(node.receive_times["raw"]),
            "compact_rate_hz": observed_rate(
                node.receive_times["compact"]
            ),
            "imu_rate_hz": observed_rate(node.receive_times["imu"]),
            "imu_matching_samples": len(imu_stamp_deltas),
            "imu_adapted_frame": node.adapted_imu_message.header.frame_id,
            "imu_adapted_minus_raw_stamp_median_s": statistics.median(
                imu_stamp_deltas
            ),
            "imu_orientation_covariance_0": (
                node.adapted_imu_message.orientation_covariance[0]
            ),
            "header_wall_age_s": time.time() - header_timestamp,
            "point_timestamp_unit": timestamp_unit,
            "first_point_timestamp_raw": first_raw_timestamp,
            "last_point_timestamp_raw": last_raw_timestamp,
            "first_point_timestamp": first_timestamp,
            "last_point_timestamp": last_timestamp,
            "point_timestamp_span_s": last_timestamp - first_timestamp,
            "first_point_minus_header_s": (
                first_timestamp - header_timestamp
            ),
            "last_point_minus_header_s": last_timestamp - header_timestamp,
        }
        print(json.dumps(summary, indent=2, sort_keys=True))

        if message.header.frame_id != "livox_frame":
            raise RuntimeError("raw point cloud frame is not livox_frame")
        if summary["imu_adapted_frame"] != "imu_link":
            raise RuntimeError("adapted IMU frame is not imu_link")
        expected_correction_s = -0.009782937
        if abs(
            summary["imu_adapted_minus_raw_stamp_median_s"]
            - expected_correction_s
        ) > 1.0e-7:
            raise RuntimeError("adapted IMU timestamp correction mismatch")
        if summary["imu_orientation_covariance_0"] != -1.0:
            raise RuntimeError("adapted IMU orientation is not unavailable")
        for stream, minimum, maximum in (
            ("raw", 8.0, 12.0),
            ("compact", 8.0, 12.0),
            ("imu", 150.0, 250.0),
        ):
            rate = summary[f"{stream}_rate_hz"]
            if not minimum <= rate <= maximum:
                raise RuntimeError(
                    f"{stream} rate {rate:.3f} Hz outside "
                    f"[{minimum}, {maximum}]"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
