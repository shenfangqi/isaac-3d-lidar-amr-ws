#!/usr/bin/env python3
"""Run deterministic offline regression of the evidence-constrained twin."""

import argparse
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaac_sim.carbot_control import (  # noqa: E402
    CarbotCommandLimiter,
    ControlLimits,
)
from isaac_sim.evidence_models import (  # noqa: E402
    EvidenceActuatorModel,
    load_evidence_profile,
)


def run_trial(parameters, evidence, linear, angular, duration_s, dt_s=0.02):
    model = EvidenceActuatorModel(evidence["actuator"])
    limiter = CarbotCommandLimiter(
        ControlLimits.from_parameters(parameters), response_model=model
    )
    elapsed = 0.0
    first_motion_s = None
    distance_m = 0.0
    yaw_rad = 0.0
    while elapsed < duration_s - 1.0e-12:
        command = limiter.update(linear, angular, 0.0, dt_s)
        elapsed += dt_s
        distance_m += command.applied_linear_mps * dt_s
        yaw_rad += command.applied_angular_rad_s * dt_s
        if first_motion_s is None and max(
            abs(command.applied_linear_mps),
            abs(command.applied_angular_rad_s),
        ) > evidence["actuator"]["stationary_threshold_mps"]:
            first_motion_s = elapsed
    command_end_distance_m = distance_m
    command_end_yaw_rad = yaw_rad
    stop_tail_s = None
    for index in range(round(2.0 / dt_s)):
        command = limiter.update(0.0, 0.0, 0.0, dt_s)
        distance_m += command.applied_linear_mps * dt_s
        yaw_rad += command.applied_angular_rad_s * dt_s
        if stop_tail_s is None and max(
            abs(command.applied_linear_mps),
            abs(command.applied_angular_rad_s),
        ) <= evidence["actuator"]["stationary_threshold_mps"]:
            stop_tail_s = (index + 1) * dt_s
    return {
        "command": {"linear_mps": linear, "angular_rad_s": angular},
        "duration_s": duration_s,
        "first_motion_latency_s": first_motion_s,
        "stop_tail_s": stop_tail_s,
        "command_distance_m": command_end_distance_m,
        "command_yaw_rad": command_end_yaw_rad,
        "final_distance_m": distance_m,
        "final_yaw_rad": yaw_rad,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    parameters = yaml.safe_load(
        (ROOT / "src/carbot_description/config/carbot_parameters.yaml")
        .read_text(encoding="utf-8")
    )
    evidence = load_evidence_profile(
        ROOT / parameters["simulation"]["evidence_profile"]
    )
    trials = {
        "forward": run_trial(parameters, evidence, 0.05, 0.0, 10.0),
        "reverse": run_trial(parameters, evidence, -0.05, 0.0, 10.0),
        "left": run_trial(parameters, evidence, 0.0, 0.5, 3.0),
        "right": run_trial(parameters, evidence, 0.0, -0.5, 3.0),
    }
    bounds = evidence["actuator"]["stop_tail_range_s"]
    checks = {
        "latency_matches_20ms_runtime_resolution": abs(
            trials["forward"]["first_motion_latency_s"]
            - evidence["actuator"]["command_latency_s"]
        ) <= 0.04,
        "forward_response_is_reduced": (
            0.80
            <= trials["forward"]["command_distance_m"] / 0.5
            <= 0.92
        ),
        "reverse_response_is_reduced": (
            0.80
            <= abs(trials["reverse"]["command_distance_m"]) / 0.5
            <= 0.92
        ),
        "stop_tail_in_measured_range": all(
            bounds[0] <= trials[name]["stop_tail_s"] <= bounds[1]
            for name in ("forward", "reverse")
        ),
        "right_turn_compensation_applied": (
            abs(trials["right"]["command_yaw_rad"])
            < trials["left"]["command_yaw_rad"]
        ),
    }
    output = {
        "profile_status": evidence["status"],
        "mode": "deterministic_offline_evidence_regression",
        "trials": trials,
        "checks": checks,
        "passed": all(checks.values()),
        "limitations": evidence["limitations"],
    }
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    if not output["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
