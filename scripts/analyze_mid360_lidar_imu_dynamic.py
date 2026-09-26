#!/usr/bin/env python3
"""Estimate MID-360 LiDAR/IMU timing from manual wall-facing yaw motion."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_mid360_static_bag import stamp_seconds
from analyze_mid360_wall_bag import (
    fit_vertical_plane,
    refine_plane,
    rotation_yx,
    summary,
)


def xyzt_array(message):
    fields = {field.name: field for field in message.fields}
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z", "timestamp"],
            "formats": [
                endian + "f4",
                endian + "f4",
                endian + "f4",
                endian + "f8",
            ],
            "offsets": [
                fields[name].offset
                for name in ("x", "y", "z", "timestamp")
            ],
            "itemsize": message.point_step,
        }
    )
    records = np.frombuffer(
        message.data,
        dtype=dtype,
        count=message.width * message.height,
    )
    timestamps = records["timestamp"].astype(np.float64)
    if timestamps.size and np.nanmedian(np.abs(timestamps)) > 1.0e14:
        timestamps *= 1.0e-9
    points = np.column_stack((records["x"], records["y"], records["z"]))
    return points, timestamps


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def collect_imu(path, topic):
    reader = open_reader(path)
    imu_type = next(
        item.type
        for item in reader.get_all_topics_and_types()
        if item.name == topic
    )
    message_type = get_message(imu_type)
    stamps = []
    angular_velocity = []
    acceleration = []
    receive_latency = []
    while reader.has_next():
        current_topic, serialized, bag_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        stamp = stamp_seconds(message.header.stamp)
        stamps.append(stamp)
        angular_velocity.append(
            [
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ]
        )
        acceleration.append(
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
        )
        receive_latency.append(bag_ns * 1.0e-9 - stamp)
    return (
        np.asarray(stamps, dtype=np.float64),
        np.asarray(angular_velocity, dtype=np.float64),
        np.asarray(acceleration, dtype=np.float64),
        np.asarray(receive_latency, dtype=np.float64),
    )


def motion_bounds(stamps, gyro, threshold, padding):
    magnitude = np.linalg.norm(gyro, axis=1)
    active = np.flatnonzero(magnitude >= threshold)
    if not active.size:
        raise RuntimeError("no IMU motion above threshold")
    return stamps[active[0]] - padding, stamps[active[-1]] + padding


def collect_wall_bearings(
    path,
    start,
    end,
    roll,
    pitch,
    distance_min,
    distance_max,
    time_bin_s,
):
    reader = open_reader(path)
    cloud_type = next(
        item.type
        for item in reader.get_all_topics_and_types()
        if item.name == "/livox/lidar"
    )
    message_type = get_message(cloud_type)
    rng = np.random.default_rng(20260921)
    rotation = rotation_yx(roll, pitch)
    stamps = []
    bearings = []
    distances = []
    residuals = []
    inlier_counts = []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/livox/lidar":
            continue
        message = deserialize_message(serialized, message_type)
        header_stamp = stamp_seconds(message.header.stamp)
        if header_stamp > end:
            break
        if header_stamp + 0.12 < start:
            continue
        points, point_stamps = xyzt_array(message)
        radius = np.linalg.norm(points[:, :2], axis=1)
        selected_mask = (
            np.all(np.isfinite(points), axis=1)
            & np.isfinite(point_stamps)
            & (radius >= 0.25)
            & (radius <= 4.0)
            & (points[:, 2] >= -0.20)
            & (points[:, 2] <= 1.50)
        )
        selected = points[selected_mask]
        selected_stamps = point_stamps[selected_mask]
        plane = fit_vertical_plane(
            selected,
            rng,
            max_normal_z=math.sin(math.radians(12.0)),
            distance_min=distance_min,
            distance_max=distance_max,
        )
        if plane is None or plane["inlier_count"] < 300:
            continue
        first_stamp = float(np.min(selected_stamps))
        bin_ids = np.floor(
            (selected_stamps - first_stamp) / time_bin_s
        ).astype(np.int64)
        for bin_id in np.unique(bin_ids):
            in_bin = bin_ids == bin_id
            if np.count_nonzero(in_bin) < 300:
                continue
            bin_points = selected[in_bin]
            bin_stamps = selected_stamps[in_bin]
            refined = refine_plane(
                bin_points, plane["normal"], plane["distance"]
            )
            if refined is None or refined["inlier_count"] < 150:
                continue
            corrected = rotation @ refined["normal"]
            bearing = math.atan2(corrected[1], corrected[0])
            inlier_stamps = bin_stamps[refined["inliers"]]
            # For approximately linear motion, the least-squares plane normal
            # is centered at the mean time of its contributing points.
            stamps.append(float(np.mean(inlier_stamps)))
            bearings.append(bearing)
            distances.append(abs(float(refined["distance"])))
            residuals.append(float(refined["residual_std_m"]))
            inlier_counts.append(int(refined["inlier_count"]))
    if len(stamps) < 20:
        raise RuntimeError("too few wall fits for dynamic calibration")
    order = np.argsort(stamps)
    stamps = np.asarray(stamps)[order]
    bearings = np.asarray(bearings)[order]
    # A plane normal is axial: +n and -n represent the same wall. Unwrap on
    # doubled angles, then divide by two to remove pi jumps.
    bearings = np.unwrap(2.0 * bearings) / 2.0
    lidar_yaw = -(bearings - np.median(bearings[: min(20, len(bearings))]))
    return {
        "stamps": stamps,
        "yaw": lidar_yaw,
        "bearing": bearings,
        "distance": np.asarray(distances)[order],
        "residual": np.asarray(residuals)[order],
        "inlier_count": np.asarray(inlier_counts)[order],
    }


def integrate_gyro(stamps, values, bias):
    corrected = values - bias
    increments = 0.5 * (corrected[1:] + corrected[:-1]) * np.diff(stamps)
    return np.concatenate(([0.0], np.cumsum(increments)))


def fit_lag(lidar_stamps, lidar_yaw, imu_stamps, integrated, lag_grid):
    best = None
    results = []
    for lag in lag_grid:
        query = lidar_stamps + lag
        valid = (query >= imu_stamps[0]) & (query <= imu_stamps[-1])
        if np.count_nonzero(valid) < 20:
            continue
        predictor = np.interp(query[valid], imu_stamps, integrated)
        centered_time = lidar_stamps[valid] - np.mean(lidar_stamps[valid])
        design = np.column_stack(
            (np.ones(predictor.size), predictor, centered_time)
        )
        coefficients, _, _, _ = np.linalg.lstsq(
            design, lidar_yaw[valid], rcond=None
        )
        residual = lidar_yaw[valid] - design @ coefficients
        rms = float(np.sqrt(np.mean(residual**2)))
        total = float(np.sum((lidar_yaw[valid] - np.mean(lidar_yaw[valid])) ** 2))
        r_squared = 1.0 - float(np.sum(residual**2)) / total
        item = {
            "lag_s": float(lag),
            "rms_rad": rms,
            "scale": float(coefficients[1]),
            "intercept_rad": float(coefficients[0]),
            "residual_drift_rad_s": float(coefficients[2]),
            "r_squared": r_squared,
            "count": int(np.count_nonzero(valid)),
        }
        results.append(item)
        if best is None or rms < best["rms_rad"]:
            best = item
    if best is None:
        raise RuntimeError("no valid lag candidate")
    index = min(
        range(len(results)), key=lambda current: results[current]["rms_rad"]
    )
    if 0 < index < len(results) - 1:
        left, center, right = results[index - 1:index + 2]
        spacing = center["lag_s"] - left["lag_s"]
        denominator = (
            left["rms_rad"]
            - 2.0 * center["rms_rad"]
            + right["rms_rad"]
        )
        if denominator > 0.0:
            shift = 0.5 * (
                left["rms_rad"] - right["rms_rad"]
            ) / denominator
            best = dict(best)
            best["lag_s_parabolic"] = float(
                center["lag_s"] + np.clip(shift, -1.0, 1.0) * spacing
            )
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--imu-topic", default="/mid360/imu/data_raw")
    parser.add_argument("--motion-threshold-rad-s", type=float, default=0.05)
    parser.add_argument("--motion-padding-s", type=float, default=5.0)
    parser.add_argument("--roll-rad", type=float, default=-0.006135404)
    parser.add_argument("--pitch-rad", type=float, default=-0.004857536)
    parser.add_argument("--wall-distance-min", type=float, default=0.20)
    parser.add_argument("--wall-distance-max", type=float, default=0.85)
    parser.add_argument("--cloud-time-bin-s", type=float, default=0.020)
    parser.add_argument("--lag-min-s", type=float, default=-0.10)
    parser.add_argument("--lag-max-s", type=float, default=0.10)
    parser.add_argument("--lag-step-s", type=float, default=0.0005)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    imu_t, gyro, accel, latency = collect_imu(args.bag, args.imu_topic)
    motion_start, motion_end = motion_bounds(
        imu_t,
        gyro,
        args.motion_threshold_rad_s,
        args.motion_padding_s,
    )
    stationary = (imu_t < motion_start + 2.0) | (imu_t > motion_end - 2.0)
    if np.count_nonzero(stationary) < 100:
        stationary = np.linalg.norm(gyro, axis=1) < 0.02
    gyro_bias = np.median(gyro[stationary], axis=0)
    wall = collect_wall_bearings(
        args.bag,
        motion_start,
        motion_end,
        args.roll_rad,
        args.pitch_rad,
        args.wall_distance_min,
        args.wall_distance_max,
        args.cloud_time_bin_s,
    )
    integrated_z = integrate_gyro(imu_t, gyro[:, 2], gyro_bias[2])
    lag_grid = np.arange(
        args.lag_min_s,
        args.lag_max_s + 0.5 * args.lag_step_s,
        args.lag_step_s,
    )
    fit = fit_lag(
        wall["stamps"], wall["yaw"], imu_t, integrated_z, lag_grid
    )

    chunk_fits = []
    boundaries = np.linspace(motion_start, motion_end, 4)
    for lower, upper in zip(boundaries[:-1], boundaries[1:]):
        selected = (wall["stamps"] >= lower) & (wall["stamps"] <= upper)
        if np.count_nonzero(selected) < 20:
            continue
        chunk_fits.append(
            fit_lag(
                wall["stamps"][selected],
                wall["yaw"][selected],
                imu_t,
                integrated_z,
                lag_grid,
            )
        )

    result = {
        "bag": str(args.bag),
        "imu_samples": int(imu_t.size),
        "imu_duration_s": float(imu_t[-1] - imu_t[0]),
        "imu_period_s": summary(np.diff(imu_t).tolist()),
        "imu_receive_minus_header_s": summary(latency.tolist()),
        "gyro_bias_rad_s": [float(value) for value in gyro_bias],
        "gyro_axis_range_rad_s": {
            name: [float(np.min(gyro[:, index])), float(np.max(gyro[:, index]))]
            for index, name in enumerate(("x", "y", "z"))
        },
        "acceleration_axis_range_mps2": {
            name: [float(np.min(accel[:, index])), float(np.max(accel[:, index]))]
            for index, name in enumerate(("x", "y", "z"))
        },
        "motion_window_s": [float(motion_start), float(motion_end)],
        "wall_fits": int(wall["stamps"].size),
        "wall_fit_period_s": summary(np.diff(wall["stamps"]).tolist()),
        "wall_distance_m": summary(wall["distance"].tolist()),
        "wall_residual_std_m": summary(wall["residual"].tolist()),
        "wall_inlier_count": summary(wall["inlier_count"].tolist()),
        "lidar_yaw_range_deg": [
            math.degrees(float(np.min(wall["yaw"]))),
            math.degrees(float(np.max(wall["yaw"]))),
        ],
        "lag_convention": (
            "compare lidar_yaw(t) with integrated_gyro_z(t + lag); positive "
            "lag means the IMU stamp is later than LiDAR effective time"
        ),
        "full_fit": fit,
        "three_time_chunk_fits": chunk_fits,
        "manufacturer_transform": {
            "rotation_rpy_rad": [0.0, 0.0, 0.0],
            "translation_lidar_to_imu_m": [0.011, 0.02329, -0.04412],
        },
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
