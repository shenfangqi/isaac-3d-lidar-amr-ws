#!/usr/bin/env python3
"""Summarize bounded Issue 9 ground-pulse JSONL evidence.

The summary deliberately keeps odometry-derived observations separate from the
external measurements required to complete physical acceptance.
"""

import argparse
import json
import math
from pathlib import Path


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def planar_delta(a, b) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def twist_nonzero(message: dict, epsilon: float = 1e-5) -> bool:
    return (
        abs(float(message.get("linear", {}).get("x", 0.0))) > epsilon
        or abs(float(message.get("angular", {}).get("z", 0.0))) > epsilon
    )


def summarize_file(path: Path) -> dict:
    rows = []
    malformed = 0
    for line in path.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            malformed += 1

    if not rows:
        return {"file": path.name, "classification": "empty", "malformed_lines": malformed}

    starts = [row for row in rows if row.get("kind") == "start"]
    stops = [row for row in rows if row.get("kind") == "stop_requested"]
    results = [row for row in rows if row.get("kind") == "result"]
    result = results[-1] if results else {}
    started = bool(result.get("started", starts))
    succeeded = result.get("result") == "PULSE_COMPLETE" and started

    summary = {
        "file": path.name,
        "classification": "success" if succeeded else "rejected_or_failed",
        "result": result.get("result", "MISSING_RESULT"),
        "reason": result.get("reason", ""),
        "started": started,
        "settled": bool(result.get("settled", False)),
        "malformed_lines": malformed,
    }
    if not succeeded or not starts or not stops:
        return summary

    start = starts[-1]
    stop = stops[-1]
    anchor = result.get("anchor") or start.get("pose")
    stop_pose = stop.get("pose")
    final_pose = result.get("final_pose")
    summary.update(
        {
            "command_linear_mps": float(start.get("linear", 0.0)),
            "command_angular_rps": float(start.get("angular", 0.0)),
            "command_seconds": float(start.get("seconds", 0.0)),
            "net_translation_m": planar_delta(anchor, final_pose),
            "net_yaw_rad": angle_delta(anchor[2], final_pose[2]),
            "post_stop_translation_m": planar_delta(stop_pose, final_pose),
            "post_stop_yaw_rad": angle_delta(stop_pose[2], final_pose[2]),
            "path_length_m": float(result.get("path_length", 0.0)),
            "absolute_yaw_travel_rad": float(result.get("absolute_yaw_travel", 0.0)),
        }
    )

    dropout = [row for row in rows if row.get("kind") == "watchdog_dropout_start"]
    reference_time = float(dropout[-1]["monotonic"] if dropout else stop["monotonic"])
    for topic in ("/cmd_vel_command", "/cmd_vel"):
        samples = [
            row
            for row in rows
            if row.get("kind") == "sample"
            and row.get("topic") == topic
            and float(row.get("monotonic", 0.0)) >= reference_time
        ]
        nonzero = [row for row in samples if twist_nonzero(row.get("message", {}))]
        key = topic.removeprefix("/").replace("/", "_")
        if nonzero:
            last_nonzero = nonzero[-1]
            summary[f"{key}_last_nonzero_after_reference_ms"] = 1000.0 * (
                float(last_nonzero["monotonic"]) - reference_time
            )
            zeros_after = [
                row
                for row in samples
                if float(row["monotonic"]) > float(last_nonzero["monotonic"])
                and not twist_nonzero(row.get("message", {}))
            ]
            if zeros_after:
                summary[f"{key}_first_zero_after_reference_ms"] = 1000.0 * (
                    float(zeros_after[0]["monotonic"]) - reference_time
                )
    if dropout:
        summary["timing_reference"] = "watchdog_dropout_start"
    else:
        summary["timing_reference"] = "stop_requested"
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    files = sorted(
        set(args.directory.glob("*pulse*.jsonl"))
        | set(args.directory.glob("*watchdog*.jsonl"))
    )
    trials = [summarize_file(path) for path in files]
    successes = [trial for trial in trials if trial["classification"] == "success"]
    payload = {
        "schema_version": 1,
        "measurement_source": "robot_odometry_and_command_topics",
        "external_physical_measurement_complete": False,
        "trial_count": len(trials),
        "successful_started_trials": len(successes),
        "empty_trials": sum(trial["classification"] == "empty" for trial in trials),
        "rejected_or_failed_trials": sum(
            trial["classification"] == "rejected_or_failed" for trial in trials
        ),
        "observed_maxima": {
            "post_stop_translation_m": max(
                (trial["post_stop_translation_m"] for trial in successes), default=None
            ),
            "post_stop_abs_yaw_rad": max(
                (abs(trial["post_stop_yaw_rad"]) for trial in successes), default=None
            ),
            "path_length_m": max((trial["path_length_m"] for trial in successes), default=None),
            "absolute_yaw_travel_rad": max(
                (trial["absolute_yaw_travel_rad"] for trial in successes), default=None
            ),
        },
        "trials": trials,
    }
    rendered = json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.write_text(rendered)
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
