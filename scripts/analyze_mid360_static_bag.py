#!/usr/bin/env python3
"""Analyze a stationary real MID-360 rosbag without replaying it."""

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.0e-9


def summary(values):
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def xyz_array(message):
    fields = {field.name: field for field in message.fields}
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": [fields[name].offset for name in ("x", "y", "z")],
            "itemsize": message.point_step,
        }
    )
    points = np.frombuffer(
        message.data,
        dtype=dtype,
        count=message.width * message.height,
    )
    return np.column_stack((points["x"], points["y"], points["z"]))


def point_time_bounds(message):
    field = next(
        field for field in message.fields if field.name == "timestamp"
    )
    endian = ">" if message.is_bigendian else "<"
    count = message.width * message.height
    first = struct.unpack_from(endian + "d", message.data, field.offset)[0]
    last = struct.unpack_from(
        endian + "d",
        message.data,
        (count - 1) * message.point_step + field.offset,
    )[0]
    scale = 1.0e-9 if abs(first) > 1.0e14 else 1.0
    return first * scale, last * scale


def floor_candidates(message, z_min, z_max, radius_min, radius_max):
    points = xyz_array(message)
    finite = np.all(np.isfinite(points), axis=1)
    radius = np.linalg.norm(points[:, :2], axis=1)
    return points[
        finite
        & (points[:, 2] >= z_min)
        & (points[:, 2] <= z_max)
        & (radius >= radius_min)
        & (radius <= radius_max)
    ]


def fit_floor_points(
    selected,
    rng,
    height_min=None,
    height_max=None,
    max_tilt_deg=20.0,
):
    if selected.shape[0] < 100:
        return None

    sample = selected
    if sample.shape[0] > 2500:
        sample = sample[rng.choice(sample.shape[0], 2500, replace=False)]

    best = None
    for _ in range(100):
        triple = sample[rng.choice(sample.shape[0], 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        norm = np.linalg.norm(normal)
        if norm < 1.0e-8:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < math.cos(math.radians(max_tilt_deg)):
            continue
        distance = -float(np.dot(normal, triple[0]))
        height = abs(distance)
        if height_min is not None and height < height_min:
            continue
        if height_max is not None and height > height_max:
            continue
        residual = np.abs(sample @ normal + distance)
        inliers = residual < 0.012
        score = int(np.count_nonzero(inliers))
        if best is None or score > best[0]:
            best = (score, normal, distance)
    if best is None:
        return None

    normal = best[1]
    distance = best[2]
    for _ in range(4):
        residual = selected @ normal + distance
        inliers = np.abs(residual) < 0.015
        if np.count_nonzero(inliers) < 100:
            return None
        cloud = selected[inliers]
        center = np.mean(cloud, axis=0)
        _, _, vh = np.linalg.svd(cloud - center, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal
        distance = -float(np.dot(normal, center))

    residual = selected @ normal + distance
    inliers = np.abs(residual) < 0.015
    inlier_residual = residual[inliers]
    roll = math.atan2(normal[1], normal[2])
    pitch = -math.asin(float(np.clip(normal[0], -1.0, 1.0)))
    return {
        "candidate_points": int(selected.shape[0]),
        "inlier_points": int(np.count_nonzero(inliers)),
        "inlier_fraction": float(np.mean(inliers)),
        "height_m": float(abs(distance)),
        "roll_deg": math.degrees(roll),
        "pitch_deg": math.degrees(pitch),
        "residual_std_m": float(np.std(inlier_residual)),
        "normal": [float(value) for value in normal],
    }


def fit_floor(
    message,
    rng,
    z_min,
    z_max,
    radius_min,
    radius_max,
    height_min,
    height_max,
    max_tilt_deg,
):
    selected = floor_candidates(
        message, z_min, z_max, radius_min, radius_max
    )
    return fit_floor_points(
        selected,
        rng,
        height_min=height_min,
        height_max=height_max,
        max_tilt_deg=max_tilt_deg,
    )


def vector_summary(vectors, names):
    if not vectors:
        return {name: {"count": 0} for name in names}
    array = np.asarray(vectors, dtype=np.float64)
    return {name: summary(array[:, index]) for index, name in enumerate(names)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--cloud-stride", type=int, default=10)
    parser.add_argument("--floor-z-min", type=float, default=-0.35)
    parser.add_argument("--floor-z-max", type=float, default=-0.10)
    parser.add_argument("--floor-radius-min", type=float, default=0.25)
    parser.add_argument("--floor-radius-max", type=float, default=3.0)
    parser.add_argument("--floor-height-min", type=float, default=0.18)
    parser.add_argument("--floor-height-max", type=float, default=0.23)
    parser.add_argument("--floor-max-tilt-deg", type=float, default=3.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
    }

    rng = np.random.default_rng(20260921)
    counts = {}
    first_bag_ns = None
    last_bag_ns = None
    cloud_index = 0
    planes = []
    rejected_planes = 0
    aggregate_floor_samples = []
    cloud_header_latency = []
    cloud_first_minus_header = []
    cloud_spans = []
    imu_si_accel = []
    imu_si_gyro = []
    imu_si_latency = []
    imu_vendor_accel = []
    wheel_latency = []
    wheel_first = None
    wheel_last = None
    odom_positions = []
    odom_yaws = []
    status_boot_ids = set()
    status_reconnects = []
    status_motion_blocked = []
    battery_voltage = []

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        first_bag_ns = bag_ns if first_bag_ns is None else first_bag_ns
        last_bag_ns = bag_ns
        message = deserialize_message(serialized, topic_types[topic])
        bag_time = bag_ns * 1.0e-9

        if topic == "/livox/lidar":
            header = stamp_seconds(message.header.stamp)
            cloud_header_latency.append(bag_time - header)
            first_point, last_point = point_time_bounds(message)
            cloud_first_minus_header.append(first_point - header)
            cloud_spans.append(last_point - first_point)
            if cloud_index % args.cloud_stride == 0:
                candidates = floor_candidates(
                    message,
                    args.floor_z_min,
                    args.floor_z_max,
                    args.floor_radius_min,
                    args.floor_radius_max,
                )
                if candidates.shape[0] > 3000:
                    candidates = candidates[
                        rng.choice(candidates.shape[0], 3000, replace=False)
                    ]
                aggregate_floor_samples.append(candidates)
                plane = fit_floor(
                    message,
                    rng,
                    args.floor_z_min,
                    args.floor_z_max,
                    args.floor_radius_min,
                    args.floor_radius_max,
                    args.floor_height_min,
                    args.floor_height_max,
                    args.floor_max_tilt_deg,
                )
                if plane is not None:
                    height_ok = (
                        args.floor_height_min
                        <= plane["height_m"]
                        <= args.floor_height_max
                    )
                    tilt_ok = (
                        abs(plane["roll_deg"]) <= args.floor_max_tilt_deg
                        and abs(plane["pitch_deg"])
                        <= args.floor_max_tilt_deg
                    )
                    if height_ok and tilt_ok:
                        planes.append(plane)
                    else:
                        rejected_planes += 1
            cloud_index += 1
        elif topic == "/mid360/imu/data_raw":
            imu_si_latency.append(
                bag_time - stamp_seconds(message.header.stamp)
            )
            imu_si_accel.append(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ]
            )
            imu_si_gyro.append(
                [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ]
            )
        elif topic == "/livox/imu":
            imu_vendor_accel.append(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ]
            )
        elif topic == "/wheel_ticks":
            wheel_latency.append(
                bag_time - stamp_seconds(message.header.stamp)
            )
            sample = {
                "left": int(message.left_ticks),
                "right": int(message.right_ticks),
                "sequence": int(message.sequence),
                "boot_id": int(message.boot_id),
            }
            wheel_first = sample if wheel_first is None else wheel_first
            wheel_last = sample
        elif topic == "/odom":
            position = message.pose.pose.position
            quaternion = message.pose.pose.orientation
            yaw = math.atan2(
                2.0
                * (
                    quaternion.w * quaternion.z
                    + quaternion.x * quaternion.y
                ),
                1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
            )
            odom_positions.append([position.x, position.y, position.z])
            odom_yaws.append(yaw)
        elif topic == "/carbot/status":
            status_boot_ids.add(int(message.boot_id))
            status_reconnects.append(int(message.reconnect_count))
            status_motion_blocked.append(bool(message.motion_blocked))
        elif topic == "/battery_state":
            battery_voltage.append(float(message.voltage))

    plane_keys = (
        "height_m",
        "roll_deg",
        "pitch_deg",
        "residual_std_m",
        "inlier_fraction",
    )
    plane_summary = {
        key: summary([plane[key] for plane in planes]) for key in plane_keys
    }
    aggregate_plane = None
    aggregate_plane_valid = False
    if aggregate_floor_samples:
        aggregate_plane = fit_floor_points(
            np.concatenate(aggregate_floor_samples, axis=0),
            rng,
            height_min=args.floor_height_min,
            height_max=args.floor_height_max,
            max_tilt_deg=args.floor_max_tilt_deg,
        )
        if aggregate_plane is not None:
            aggregate_plane_valid = (
                args.floor_height_min
                <= aggregate_plane["height_m"]
                <= args.floor_height_max
                and abs(aggregate_plane["roll_deg"])
                <= args.floor_max_tilt_deg
                and abs(aggregate_plane["pitch_deg"])
                <= args.floor_max_tilt_deg
            )
    gravity_alignment = None
    if imu_si_accel:
        mean_acceleration = np.mean(
            np.asarray(imu_si_accel, dtype=np.float64), axis=0
        )
        acceleration_norm = float(np.linalg.norm(mean_acceleration))
        unit_gravity = mean_acceleration / acceleration_norm
        gravity_alignment = {
            "mean_acceleration_mps2": [
                float(value) for value in mean_acceleration
            ],
            "mean_norm_mps2": acceleration_norm,
            "roll_deg": math.degrees(
                math.atan2(unit_gravity[1], unit_gravity[2])
            ),
            "pitch_deg": math.degrees(
                -math.asin(float(np.clip(unit_gravity[0], -1.0, 1.0)))
            ),
        }
    odom_delta = None
    if len(odom_positions) >= 2:
        start = np.asarray(odom_positions[0])
        end = np.asarray(odom_positions[-1])
        odom_delta = {
            "translation_m": [float(value) for value in end - start],
            "planar_distance_m": float(np.linalg.norm((end - start)[:2])),
            "yaw_deg": math.degrees(odom_yaws[-1] - odom_yaws[0]),
        }

    result = {
        "bag": str(args.bag),
        "duration_s": (last_bag_ns - first_bag_ns) * 1.0e-9,
        "topic_counts": counts,
        "floor_fit": {
            "sampled_frames": cloud_index // args.cloud_stride + 1,
            "successful_frames": len(planes),
            "rejected_candidate_frames": rejected_planes,
            "selection": {
                "z_m": [args.floor_z_min, args.floor_z_max],
                "radius_m": [args.floor_radius_min, args.floor_radius_max],
                "height_m": [
                    args.floor_height_min,
                    args.floor_height_max,
                ],
                "max_abs_roll_pitch_deg": args.floor_max_tilt_deg,
            },
            "statistics": plane_summary,
            "aggregate_plane": aggregate_plane,
            "aggregate_plane_valid": aggregate_plane_valid,
            "yaw_observable": False,
        },
        "pointcloud_timing_s": {
            "bag_receive_minus_header": summary(cloud_header_latency),
            "first_point_minus_header": summary(cloud_first_minus_header),
            "point_timestamp_span": summary(cloud_spans),
        },
        "mid360_imu_si": {
            "acceleration_mps2": vector_summary(imu_si_accel, ("x", "y", "z")),
            "angular_velocity_rad_s": vector_summary(
                imu_si_gyro, ("x", "y", "z")
            ),
            "bag_receive_minus_header_s": summary(imu_si_latency),
            "gravity_alignment": gravity_alignment,
        },
        "mid360_vendor_imu": {
            "acceleration_g": vector_summary(
                imu_vendor_accel, ("x", "y", "z")
            ),
        },
        "wheel_ticks": {
            "first": wheel_first,
            "last": wheel_last,
            "delta_left": (
                None
                if wheel_first is None
                else wheel_last["left"] - wheel_first["left"]
            ),
            "delta_right": (
                None
                if wheel_first is None
                else wheel_last["right"] - wheel_first["right"]
            ),
            "bag_receive_minus_header_s": summary(wheel_latency),
        },
        "odometry_static_delta": odom_delta,
        "status": {
            "boot_ids": sorted(status_boot_ids),
            "reconnect_count": summary(status_reconnects),
            "motion_blocked_true_count": int(sum(status_motion_blocked)),
        },
        "battery_voltage_v": summary(battery_voltage),
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
