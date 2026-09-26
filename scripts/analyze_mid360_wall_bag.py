#!/usr/bin/env python3
"""Fit vertical planes in a stationary MID-360 rosbag for yaw calibration."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_mid360_static_bag import xyz_array


def angle_wrap_deg(value):
    return (value + 180.0) % 360.0 - 180.0


def summary(values):
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "min": float(np.min(array)),
        "median": float(np.median(array)),
        "max": float(np.max(array)),
    }


def rotation_yx(roll, pitch):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rotation_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    )
    rotation_y = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    )
    return rotation_y @ rotation_x


def orient_plane(normal, distance):
    normal = normal / np.linalg.norm(normal)
    if distance > 0.0:
        normal = -normal
        distance = -distance
    return normal, float(distance)


def refine_plane(points, normal, distance):
    residual = points @ normal + distance
    inliers = np.abs(residual) < 0.02
    if np.count_nonzero(inliers) < 100:
        return None
    for _ in range(4):
        cloud = points[inliers]
        center = np.mean(cloud, axis=0)
        _, _, vh = np.linalg.svd(cloud - center, full_matrices=False)
        candidate = vh[-1]
        if np.dot(candidate, normal) < 0.0:
            candidate = -candidate
        normal = candidate / np.linalg.norm(candidate)
        distance = -float(np.dot(normal, center))
        normal, distance = orient_plane(normal, distance)
        residual = points @ normal + distance
        inliers = np.abs(residual) < 0.02
    cloud = points[inliers]
    residual = cloud @ normal + distance
    center = np.mean(cloud, axis=0)
    centered = cloud - center
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    return {
        "normal": normal,
        "distance": distance,
        "inliers": inliers,
        "inlier_count": int(cloud.shape[0]),
        "residual_std_m": float(np.std(residual)),
        "center_m": center,
        "principal_axes": vh,
        "singular_values": singular_values,
    }


def fit_vertical_plane(
    points,
    rng,
    max_normal_z,
    distance_min,
    distance_max,
):
    if points.shape[0] < 300:
        return None
    sample = points
    if sample.shape[0] > 8000:
        sample = sample[rng.choice(sample.shape[0], 8000, replace=False)]
    best = None
    for _ in range(500):
        triple = sample[rng.choice(sample.shape[0], 3, replace=False)]
        normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
        norm = np.linalg.norm(normal)
        if norm < 1.0e-8:
            continue
        normal /= norm
        if abs(normal[2]) > max_normal_z:
            continue
        distance = -float(np.dot(normal, triple[0]))
        normal, distance = orient_plane(normal, distance)
        plane_distance = abs(distance)
        if not distance_min <= plane_distance <= distance_max:
            continue
        residual = np.abs(sample @ normal + distance)
        score = int(np.count_nonzero(residual < 0.015))
        if best is None or score > best[0]:
            best = (score, normal, distance)
    if best is None:
        return None
    return refine_plane(points, best[1], best[2])


def plane_result(plane, roll_rad, pitch_rad, total_points):
    normal = plane["normal"]
    corrected = rotation_yx(roll_rad, pitch_rad) @ normal
    raw_bearing = math.degrees(math.atan2(normal[1], normal[0]))
    corrected_bearing = math.degrees(
        math.atan2(corrected[1], corrected[0])
    )
    expected_bearing = 90.0 if corrected_bearing >= 0.0 else -90.0
    yaw = angle_wrap_deg(expected_bearing - corrected_bearing)
    axes = plane["principal_axes"]
    singular_values = plane["singular_values"]
    return {
        "normal_livox": [float(value) for value in normal],
        "normal_after_roll_pitch": [float(value) for value in corrected],
        "distance_m": abs(float(plane["distance"])),
        "inlier_count": plane["inlier_count"],
        "inlier_fraction": plane["inlier_count"] / total_points,
        "residual_std_m": plane["residual_std_m"],
        "raw_normal_bearing_deg": raw_bearing,
        "roll_pitch_corrected_bearing_deg": corrected_bearing,
        "expected_parallel_wall_normal_bearing_deg": expected_bearing,
        "yaw_candidate_deg": yaw,
        "principal_axes": [
            [float(value) for value in row] for row in axes
        ],
        "singular_values": [float(value) for value in singular_values],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--cloud-stride", type=int, default=10)
    parser.add_argument("--radius-min", type=float, default=0.25)
    parser.add_argument("--radius-max", type=float, default=4.0)
    parser.add_argument("--z-min", type=float, default=-0.15)
    parser.add_argument("--z-max", type=float, default=1.5)
    parser.add_argument("--distance-min", type=float, default=0.20)
    parser.add_argument("--distance-max", type=float, default=3.5)
    parser.add_argument("--vertical-tolerance-deg", type=float, default=5.0)
    parser.add_argument("--roll-rad", type=float, default=-0.006135404)
    parser.add_argument("--pitch-rad", type=float, default=-0.004857536)
    parser.add_argument("--max-planes", type=int, default=4)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    lidar_type = next(
        item.type
        for item in reader.get_all_topics_and_types()
        if item.name == "/livox/lidar"
    )
    lidar_message = get_message(lidar_type)
    rng = np.random.default_rng(20260921)
    frames = []
    cloud_index = 0
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != "/livox/lidar":
            continue
        if cloud_index % args.cloud_stride == 0:
            message = deserialize_message(serialized, lidar_message)
            points = xyz_array(message)
            radius = np.linalg.norm(points[:, :2], axis=1)
            selected = points[
                np.all(np.isfinite(points), axis=1)
                & (radius >= args.radius_min)
                & (radius <= args.radius_max)
                & (points[:, 2] >= args.z_min)
                & (points[:, 2] <= args.z_max)
            ]
            if selected.shape[0] > 4000:
                selected = selected[
                    rng.choice(selected.shape[0], 4000, replace=False)
                ]
            frames.append(selected)
        cloud_index += 1

    aggregate = np.concatenate(frames, axis=0)
    remaining = aggregate
    planes = []
    max_normal_z = math.sin(math.radians(args.vertical_tolerance_deg))
    for _ in range(args.max_planes):
        plane = fit_vertical_plane(
            remaining,
            rng,
            max_normal_z,
            args.distance_min,
            args.distance_max,
        )
        if plane is None:
            break
        result = plane_result(
            plane, args.roll_rad, args.pitch_rad, remaining.shape[0]
        )
        frame_bearings = []
        frame_distances = []
        for frame in frames:
            residual = np.abs(
                frame @ plane["normal"] + plane["distance"]
            )
            near = frame[residual < 0.03]
            refined = refine_plane(
                near, plane["normal"], plane["distance"]
            )
            if refined is None:
                continue
            frame_result = plane_result(
                refined, args.roll_rad, args.pitch_rad, near.shape[0]
            )
            frame_bearings.append(
                frame_result["roll_pitch_corrected_bearing_deg"]
            )
            frame_distances.append(frame_result["distance_m"])
        result["frame_bearing_deg"] = summary(frame_bearings)
        result["frame_distance_m"] = summary(frame_distances)
        planes.append(result)
        keep = np.abs(
            remaining @ plane["normal"] + plane["distance"]
        ) >= 0.035
        remaining = remaining[keep]

    planes.sort(key=lambda item: item["inlier_count"], reverse=True)
    result = {
        "bag": str(args.bag),
        "sampled_frames": len(frames),
        "candidate_points": int(aggregate.shape[0]),
        "roll_rad": args.roll_rad,
        "pitch_rad": args.pitch_rad,
        "parallel_wall_assumption": True,
        "planes": planes,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
