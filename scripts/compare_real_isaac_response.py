#!/usr/bin/env python3
"""Compare surveyed real Carbot trials with matched Isaac response profiles."""

import argparse
import csv
import json
from pathlib import Path
import statistics


def optional_float(row, key):
    value = row.get(key, "").strip()
    return None if not value else float(value)


def load_real_trials(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    by_id = {}
    for row in rows:
        distance = optional_float(row, "measured_distance_m")
        yaw = optional_float(row, "measured_yaw_rad")
        if distance is not None and row["motion"] == "reverse":
            distance = -abs(distance)
        by_id[row["trial_id"]] = {
            "distance_m": distance,
            "yaw_rad": yaw,
        }
    return by_id


def relative_error(simulated, real):
    if real is None or abs(real) < 1.0e-9:
        return None
    return (simulated - real) / real


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("real_trials", type=Path)
    parser.add_argument("isaac_profile", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    real = load_real_trials(args.real_trials)
    simulated = json.loads(args.isaac_profile.read_text(encoding="utf-8"))
    comparisons = []
    for trial in simulated["trials"]:
        trial_id = trial["id"]
        if trial_id not in real:
            raise KeyError(f"real trial {trial_id!r} is missing")
        truth = real[trial_id]
        if truth["distance_m"] is not None:
            sim_value = trial["final_pose_dx_m"]
            real_value = truth["distance_m"]
            metric = "distance_m"
        else:
            # The ideal-kinematic action writes the articulation pose directly,
            # so root_ang_vel_b does not represent its complete yaw response.
            sim_value = trial["final_pose_yaw_rad"]
            real_value = truth["yaw_rad"]
            metric = "yaw_rad"
        difference = sim_value - real_value
        comparisons.append(
            {
                "id": trial_id,
                "metric": metric,
                "real": real_value,
                "isaac": sim_value,
                "isaac_minus_real": difference,
                "relative_error_fraction": relative_error(
                    sim_value, real_value
                ),
                "absolute_error": abs(difference),
                "first_motion_latency_s": trial["first_motion_latency_s"],
                "stop_tail_s": trial["stop_tail_s"],
            }
        )
    linear_errors = [
        item["relative_error_fraction"]
        for item in comparisons
        if item["metric"] == "distance_m"
    ]
    yaw_errors = [
        item["relative_error_fraction"]
        for item in comparisons
        if item["metric"] == "yaw_rad"
    ]
    output = {
        "comparison": comparisons,
        "summary": {
            "mean_absolute_linear_relative_error_fraction": statistics.mean(
                abs(value) for value in linear_errors
            ),
            "mean_absolute_yaw_relative_error_fraction": statistics.mean(
                abs(value) for value in yaw_errors
            ),
            "real_first_motion_latency_s": 0.073,
            "real_ground_stop_tail_s": [0.57, 0.92],
            "decision": "navigation_contract_only_not_dynamic_twin",
        },
        "units_note": (
            "yaw values are radians; relative error retains direction"
        ),
    }
    rendered = json.dumps(output, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
