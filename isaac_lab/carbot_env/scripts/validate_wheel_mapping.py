#!/usr/bin/env python3
"""Validate wheel direction, speed mapping, and coupled saturation."""

import argparse
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--hold-seconds", type=float, default=3600.0)
parser.add_argument("--render-stride", type=int, default=10)
parser.add_argument("--realtime", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay  # noqa: E402


def _run_actions(env, actions, seconds, label):
    steps = max(1, round(seconds / env.step_dt))
    started = time.perf_counter()
    reset_count = 0
    for _ in range(steps):
        step_started = time.perf_counter()
        _, _, terminated, time_out, _ = env.step(actions)
        reset_count += int(torch.logical_or(terminated, time_out).sum().item())
        if args.realtime:
            remaining = env.step_dt - (time.perf_counter() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
    env.sim.render()
    print(
        f"[G4][TIMING] {label}: {steps} control steps in "
        f"{time.perf_counter() - started:.3f} s wall; resets={reset_count}",
        flush=True,
    )
    return reset_count


def _targets(action_term):
    return (
        action_term._left_targets[0].clone(),
        action_term._right_targets[0].clone(),
    )


def _uniform(values, tolerance=1e-5):
    return torch.allclose(values, values[0].expand_as(values), atol=tolerance)


def main():
    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    cfg.viewer.eye = (1.5, -3.0, 2.4)
    cfg.viewer.lookat = (0.0, 0.0, 0.10)
    cfg.viewer.origin_type = "world"
    cfg.viewer.asset_name = None
    cfg.events.reset_base.params["pose_range"] = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }
    cfg.observations.policy.lidar_ranges = None
    cfg.observations.policy.height_scan = None
    cfg.scene.lidar = None
    cfg.scene.height_scanner = None
    cfg.terminations.goal_reached = None
    cfg.sim.render_interval = args.render_stride
    # The normal navigation profile is intentionally tighter than the wheel
    # envelope and cannot reach coupled saturation.  This validation-only
    # gate expands the input clamp to the canonical robot maxima so the same
    # production action term must exercise its saturation path.
    cfg.actions.twist.max_linear_velocity_mps = 0.50
    cfg.actions.twist.max_angular_velocity_rad_s = 3.50
    cfg.actions.twist.forward_track_deadband_mps = 0.0
    cfg.actions.twist.reverse_track_deadband_mps = 0.0

    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()
    robot = env.scene["robot"]
    action_term = env.action_manager.get_term("twist")
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    forward = zeros.clone()
    positive_yaw = zeros.clone()
    negative_yaw = zeros.clone()
    saturated_curve = zeros.clone()
    forward[:, 0] = 0.12
    positive_yaw[:, 1] = 0.30
    negative_yaw[:, 1] = -0.30
    saturated_curve[:, 0] = 0.50
    saturated_curve[:, 1] = 3.50

    print(
        "[G4] Sequence: forward -> positive yaw -> negative yaw -> "
        "coupled-saturation curve -> stop.",
        flush=True,
    )
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(f"[G4] Waiting for operator trigger: {trigger}", flush=True)
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(
        f"[G4] Operator ready; starting in {args.startup_delay:.0f} s.",
        flush=True,
    )
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    resets = []
    print("[G4] FORWARD 0.12 m/s for 4.0 s", flush=True)
    resets.append(_run_actions(env, forward, 4.0, "forward"))
    forward_left, forward_right = _targets(action_term)

    resets.append(_run_actions(env, zeros, 2.0, "first-stop"))
    print("[G4] POSITIVE YAW +0.30 rad/s for 3.0 s", flush=True)
    resets.append(_run_actions(env, positive_yaw, 3.0, "positive-yaw"))
    positive_left, positive_right = _targets(action_term)

    resets.append(_run_actions(env, zeros, 2.0, "second-stop"))
    print("[G4] NEGATIVE YAW -0.30 rad/s for 3.0 s", flush=True)
    resets.append(_run_actions(env, negative_yaw, 3.0, "negative-yaw"))
    negative_left, negative_right = _targets(action_term)

    resets.append(_run_actions(env, zeros, 2.0, "third-stop"))
    print(
        "[G4] SATURATION DIAGNOSTIC command [0.50 m/s, +3.50 rad/s] "
        "for 1.2 s",
        flush=True,
    )
    resets.append(
        _run_actions(env, saturated_curve, 1.2, "saturated-curve")
    )
    saturated_left, saturated_right = _targets(action_term)
    processed = action_term.processed_actions[0].clone()
    applied = action_term.applied_actions[0].clone()
    scale = action_term.wheel_scale[0].item()

    resets.append(_run_actions(env, zeros, 3.0, "final-stop"))
    final_linear_speed = torch.linalg.vector_norm(
        robot.data.root_lin_vel_b[0, :2]
    ).item()
    final_angular_speed = abs(robot.data.root_ang_vel_b[0, 2].item())

    sign = action_term.cfg.wheel_joint_coordinate_sign
    radius = action_term.cfg.effective_sprocket_radius_m
    half_separation = action_term.cfg.effective_track_separation_m / 2.0
    expected_forward_raw = sign * 0.12 / radius
    forward_ok = (
        _uniform(forward_left)
        and _uniform(forward_right)
        and torch.allclose(
            forward_left,
            torch.full_like(forward_left, expected_forward_raw),
            atol=1e-4,
        )
        and torch.allclose(forward_left, forward_right, atol=1e-5)
    )
    positive_ok = (
        _uniform(positive_left)
        and _uniform(positive_right)
        and positive_left[0].item() > 0.0
        and positive_right[0].item() < 0.0
        and torch.allclose(positive_left, -positive_right, atol=1e-4)
    )
    negative_ok = (
        _uniform(negative_left)
        and _uniform(negative_right)
        and negative_left[0].item() < 0.0
        and negative_right[0].item() > 0.0
        and torch.allclose(negative_left, -negative_right, atol=1e-4)
    )

    unsaturated_left = (processed[0] - processed[1] * half_separation) / radius
    unsaturated_right = (processed[0] + processed[1] * half_separation) / radius
    peak = max(abs(unsaturated_left.item()), abs(unsaturated_right.item()))
    expected_scale = action_term.cfg.max_wheel_velocity_rad_s / peak
    ratio_before = abs((unsaturated_left / unsaturated_right).item())
    ratio_after = abs(
        (saturated_left[0] / saturated_right[0]).item()
    )
    saturation_ok = (
        scale < 1.0
        and abs(scale - expected_scale) <= 1e-5
        and abs(ratio_after - ratio_before) <= 1e-5
        and abs(
            max(
                abs(saturated_left[0].item()),
                abs(saturated_right[0].item()),
            )
            - action_term.cfg.max_wheel_velocity_rad_s
        )
        <= 1e-4
        and torch.allclose(applied, processed * scale, atol=1e-5)
    )
    stop_ok = final_linear_speed <= 0.05 and final_angular_speed <= 0.05
    reset_ok = sum(resets) == 0

    print(
        "[G4][METRICS] "
        f"forward_raw=({forward_left[0].item():.3f}, "
        f"{forward_right[0].item():.3f}) rad/s, "
        f"positive_yaw_raw=({positive_left[0].item():.3f}, "
        f"{positive_right[0].item():.3f}), "
        f"negative_yaw_raw=({negative_left[0].item():.3f}, "
        f"{negative_right[0].item():.3f}), "
        f"saturated_raw=({saturated_left[0].item():.3f}, "
        f"{saturated_right[0].item():.3f}), scale={scale:.5f}, "
        f"ratio=({ratio_before:.5f}->{ratio_after:.5f}), "
        f"processed={processed.tolist()}, applied={applied.tolist()}, "
        f"final_speeds=({final_linear_speed:.4f} m/s, "
        f"{final_angular_speed:.4f} rad/s), resets={tuple(resets)}",
        flush=True,
    )
    if not (
        forward_ok
        and positive_ok
        and negative_ok
        and saturation_ok
        and stop_ok
        and reset_ok
    ):
        raise RuntimeError(
            "G4 failed: "
            f"forward_ok={forward_ok}, positive_ok={positive_ok}, "
            f"negative_ok={negative_ok}, saturation_ok={saturation_ok}, "
            f"stop_ok={stop_ok}, reset_ok={reset_ok}"
        )

    print(
        "[G4] PASS: all six joints per side agree; forward and yaw signs "
        f"are correct; coupled saturation scale={scale:.5f} preserves "
        f"wheel ratio={ratio_after:.5f}.",
        flush=True,
    )
    if args.hold_seconds > 0.0:
        print(
            "[G4] Holding the stopped robot for visual confirmation "
            f"({args.hold_seconds:.0f} s maximum).",
            flush=True,
        )
        deadline = time.monotonic() + args.hold_seconds
        while simulation_app.is_running() and time.monotonic() < deadline:
            env.sim.render()
            time.sleep(0.05)
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
