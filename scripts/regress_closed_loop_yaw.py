#!/usr/bin/env python3
"""Regress fused-yaw closed-loop targets through the Isaac response model."""

import argparse
import json
import math
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from isaac_sim.carbot_control import CarbotCommandLimiter, ControlLimits  # noqa: E402
from isaac_sim.evidence_models import EvidenceActuatorModel, load_evidence_profile  # noqa: E402


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    parser.add_argument("--target-deg", type=float, default=30.0)
    parser.add_argument("--pairs", type=int, default=4)
    parser.add_argument("--angular", type=float, default=0.40)
    parser.add_argument("--tolerance-deg", type=float, default=1.5)
    parser.add_argument("--brake-horizon", type=float, default=0.35)
    args = parser.parse_args()

    parameters = yaml.safe_load(
        (ROOT / "src/carbot_description/config/carbot_parameters.yaml").read_text(
            encoding="utf-8"
        )
    )
    evidence = load_evidence_profile(ROOT / parameters["simulation"]["evidence_profile"])
    model = EvidenceActuatorModel(evidence["actuator"])
    limiter = CarbotCommandLimiter(
        ControlLimits.from_parameters(parameters), response_model=model
    )
    dt = 0.02
    yaw = 0.0
    yaw_rate = 0.0
    tolerance = math.radians(args.tolerance_deg)

    def step(command, duration):
        nonlocal yaw, yaw_rate
        count = round(duration / dt)
        for _ in range(count):
            output = limiter.update(0.0, command, 0.0, dt)
            yaw_rate = output.applied_angular_rad_s
            yaw = wrap(yaw + yaw_rate * dt)

    def seek(target):
        attempts = []
        for attempt in range(1, 4):
            start = yaw
            elapsed = 0.0
            while elapsed < 8.0:
                error = wrap(target - yaw)
                braking_angle = max(tolerance, abs(yaw_rate) * args.brake_horizon)
                if abs(error) <= braking_angle:
                    break
                step(math.copysign(args.angular, error), dt)
                elapsed += dt
            else:
                raise RuntimeError("simulated yaw target timeout")
            step(0.0, 1.5)
            error = wrap(target - yaw)
            attempts.append(
                {
                    "attempt": attempt,
                    "motion_deg": math.degrees(wrap(yaw - start)),
                    "remaining_error_deg": math.degrees(error),
                }
            )
            if abs(error) <= tolerance:
                break
        return attempts

    results = []
    start_yaw = yaw
    for pair in range(1, args.pairs + 1):
        baseline = yaw
        direction = 1.0 if pair % 2 else -1.0
        away_target = wrap(baseline + direction * math.radians(args.target_deg))
        away_attempts = seek(away_target)
        return_attempts = seek(baseline)
        results.append(
            {
                "pair": pair,
                "direction": "left_then_right" if direction > 0 else "right_then_left",
                "away_attempts": away_attempts,
                "return_attempts": return_attempts,
                "return_yaw_error_deg": math.degrees(wrap(yaw - baseline)),
            }
        )

    errors = [abs(row["return_yaw_error_deg"]) for row in results]
    report = {
        "mode": "isaac_evidence_degraded_closed_loop_yaw_regression",
        "controller": {
            "angular_command_rad_s": args.angular,
            "target_deg": args.target_deg,
            "tolerance_deg": args.tolerance_deg,
            "brake_horizon_s": args.brake_horizon,
        },
        "pairs": results,
        "cumulative_yaw_error_deg": math.degrees(wrap(yaw - start_yaw)),
        "maximum_pair_return_error_deg": max(errors),
        "passed": max(errors) <= args.tolerance_deg,
    }
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
