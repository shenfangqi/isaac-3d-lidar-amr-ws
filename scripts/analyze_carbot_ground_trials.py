#!/usr/bin/env python3
"""Calculate tracked-base slip and effective geometry from measured trials."""

import argparse
import csv
import math
from pathlib import Path
import statistics


FIELDS = (
    "trial_id",
    "surface",
    "payload_kg",
    "motion",
    "measured_distance_m",
    "measured_yaw_rad",
    "left_tick_delta",
    "right_tick_delta",
    "battery_voltage_v",
    "notes",
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--counts-per-revolution", type=float, default=1560.0)
    parser.add_argument("--baseline-radius-m", type=float, default=0.02175)
    parser.add_argument("--baseline-track-m", type=float, default=0.254)
    parser.add_argument("--create-template", action="store_true")
    return parser.parse_args()


def optional_float(row, key):
    value = row[key].strip()
    return None if not value else float(value)


def create_template(path):
    if path.exists():
        raise FileExistsError(f"refusing to overwrite {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()


def signed_magnitude(value, reference):
    return math.copysign(abs(value), reference) if reference else abs(value)


def analyze_row(row, meters_per_tick, baseline_radius_m, baseline_track_m):
    left_ticks = int(row["left_tick_delta"])
    right_ticks = int(row["right_tick_delta"])
    left_m = left_ticks * meters_per_tick
    right_m = right_ticks * meters_per_tick
    encoder_distance_m = 0.5 * (left_m + right_m)
    encoder_yaw_rad = (right_m - left_m) / baseline_track_m
    measured_distance_m = optional_float(row, "measured_distance_m")
    measured_yaw_rad = optional_float(row, "measured_yaw_rad")

    result = {
        "trial_id": row["trial_id"],
        "surface": row["surface"],
        "payload_kg": row["payload_kg"],
        "motion": row["motion"],
        "encoder_distance_m": encoder_distance_m,
        "encoder_yaw_rad": encoder_yaw_rad,
    }
    if measured_distance_m is not None and abs(encoder_distance_m) > 1.0e-9:
        measured_signed = signed_magnitude(measured_distance_m, encoder_distance_m)
        result["longitudinal_slip_fraction"] = 1.0 - (
            measured_signed / encoder_distance_m
        )
        result["effective_radius_m"] = baseline_radius_m * (
            measured_signed / encoder_distance_m
        )
    if measured_yaw_rad is not None and abs(encoder_yaw_rad) > 1.0e-9:
        measured_signed = signed_magnitude(measured_yaw_rad, encoder_yaw_rad)
        result["yaw_gain"] = measured_signed / encoder_yaw_rad
        result["turn_slip_fraction"] = 1.0 - abs(result["yaw_gain"])
        result["effective_track_separation_m"] = abs(
            (right_m - left_m) / measured_signed
        )
    return result


def summarize(results, key):
    values = [result[key] for result in results if key in result]
    if not values:
        return None
    return min(values), statistics.mean(values), max(values)


def main():
    args = parse_args()
    if args.create_template:
        create_template(args.csv_path)
        print(f"created {args.csv_path}")
        return

    meters_per_tick = (
        2.0 * math.pi * args.baseline_radius_m / args.counts_per_revolution
    )
    with args.csv_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(FIELDS) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"missing columns: {sorted(missing)}")
        results = [
            analyze_row(
                row,
                meters_per_tick,
                args.baseline_radius_m,
                args.baseline_track_m,
            )
            for row in reader
        ]

    for result in results:
        metrics = " ".join(
            f"{key}={value:.6f}"
            for key, value in result.items()
            if isinstance(value, float)
        )
        print(
            f"TRIAL {result['trial_id']} surface={result['surface']} "
            f"payload_kg={result['payload_kg']} motion={result['motion']} {metrics}"
        )

    for key in (
        "longitudinal_slip_fraction",
        "effective_radius_m",
        "yaw_gain",
        "turn_slip_fraction",
        "effective_track_separation_m",
    ):
        summary = summarize(results, key)
        if summary is not None:
            print(
                f"SUMMARY {key} min={summary[0]:.6f} "
                f"mean={summary[1]:.6f} max={summary[2]:.6f}"
            )


if __name__ == "__main__":
    main()
