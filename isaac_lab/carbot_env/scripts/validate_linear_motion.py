#!/usr/bin/env python3
"""Visually and quantitatively validate low-speed forward/reverse motion."""

import argparse
import math
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--speed", type=float, default=0.10)
parser.add_argument("--motion-seconds", type=float, default=5.0)
parser.add_argument("--stop-seconds", type=float, default=3.0)
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


def _yaw_from_wxyz(quaternion):
    w, x, y, z = quaternion.tolist()
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose(robot):
    return robot.data.root_pos_w[0].clone(), _yaw_from_wxyz(
        robot.data.root_quat_w[0]
    )


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
        f"[G2][TIMING] {label}: {steps} control steps in "
        f"{time.perf_counter() - started:.3f} s wall; resets={reset_count}",
        flush=True,
    )
    return reset_count


def main():
    if not 0.0 < args.speed <= 0.25:
        raise ValueError("--speed must be in (0.0, 0.25] m/s for this gate")
    if args.render_stride < 1:
        raise ValueError("--render-stride must be at least 1")

    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    # A fixed world camera makes both direction changes visually unambiguous.
    cfg.viewer.eye = (0.0, -3.0, 1.0)
    cfg.viewer.lookat = (0.0, 0.0, 0.12)
    cfg.viewer.origin_type = "world"
    cfg.viewer.asset_name = None
    cfg.events.reset_base.params["pose_range"] = {
        "x": (0.0, 0.0),
        "y": (0.0, 0.0),
        "yaw": (0.0, 0.0),
    }
    # This motion gate does not consume scanner observations. Removing them
    # keeps the real-time control cadence independent of unused sensor work.
    cfg.observations.policy.lidar_ranges = None
    cfg.observations.policy.height_scan = None
    cfg.scene.lidar = None
    cfg.scene.height_scanner = None
    cfg.terminations.goal_reached = None
    cfg.sim.render_interval = args.render_stride

    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()
    robot = env.scene["robot"]
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    forward = zeros.clone()
    reverse = zeros.clone()
    forward[:, 0] = args.speed
    reverse[:, 0] = -args.speed

    print("[G2] Sequence: forward -> stop -> reverse -> stop.", flush=True)
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(
            f"[G2] Waiting for operator trigger: {trigger}", flush=True
        )
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(
        f"[G2] Operator ready; starting in {args.startup_delay:.0f} s.",
        flush=True,
    )
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    start, start_yaw = _pose(robot)
    print(
        f"[G2] FORWARD {args.speed:.2f} m/s for {args.motion_seconds:.1f} s",
        flush=True,
    )
    forward_resets = _run_actions(
        env, forward, args.motion_seconds, "forward"
    )
    forward_end, forward_yaw = _pose(robot)

    print(f"[G2] STOP for {args.stop_seconds:.1f} s", flush=True)
    first_stop_resets = _run_actions(
        env, zeros, args.stop_seconds, "first-stop"
    )
    first_stop, _ = _pose(robot)

    print(
        f"[G2] REVERSE {-args.speed:.2f} m/s for {args.motion_seconds:.1f} s",
        flush=True,
    )
    reverse_resets = _run_actions(
        env, reverse, args.motion_seconds, "reverse"
    )
    reverse_end, reverse_yaw = _pose(robot)

    print(f"[G2] FINAL STOP for {args.stop_seconds:.1f} s", flush=True)
    final_stop_resets = _run_actions(
        env, zeros, args.stop_seconds, "final-stop"
    )
    final, final_yaw = _pose(robot)

    forward_dx = (forward_end[0] - start[0]).item()
    stop_dx = (first_stop[0] - forward_end[0]).item()
    reverse_dx = (reverse_end[0] - first_stop[0]).item()
    final_stop_dx = (final[0] - reverse_end[0]).item()
    lateral_error = abs((final[1] - start[1]).item())
    return_error = abs((final[0] - start[0]).item())
    yaw_drift = max(
        abs(forward_yaw - start_yaw),
        abs(reverse_yaw - start_yaw),
        abs(final_yaw - start_yaw),
    )
    final_speed = torch.linalg.vector_norm(robot.data.root_lin_vel_b[0, :2]).item()
    resets = (
        forward_resets,
        first_stop_resets,
        reverse_resets,
        final_stop_resets,
    )
    expected_minimum = args.speed * args.motion_seconds * 0.65
    direction_ok = forward_dx > expected_minimum and reverse_dx < -expected_minimum
    straight_ok = lateral_error <= 0.05 and yaw_drift <= math.radians(3.0)
    # The stop phase includes acceleration-limited braking distance.  At this
    # gate's 0.15 m/s command that is about 3 cm, so bound it separately from
    # the final settled speed instead of treating all motion as static drift.
    stop_ok = (
        abs(stop_dx) <= 0.05
        and abs(final_stop_dx) <= 0.05
        and final_speed <= 0.05
    )
    reset_ok = sum(resets) == 0

    print(
        "[G2][METRICS] "
        f"forward_dx={forward_dx:.4f} m, stop_dx={stop_dx:.4f} m, "
        f"reverse_dx={reverse_dx:.4f} m, final_stop_dx={final_stop_dx:.4f} m, "
        f"return_error={return_error:.4f} m, lateral_error={lateral_error:.4f} m, "
        f"yaw_drift={math.degrees(yaw_drift):.2f} deg, "
        f"final_speed={final_speed:.4f} m/s, resets={resets}",
        flush=True,
    )
    if not (direction_ok and straight_ok and stop_ok and reset_ok):
        raise RuntimeError(
            "G2 failed: "
            f"direction_ok={direction_ok}, straight_ok={straight_ok}, "
            f"stop_ok={stop_ok}, reset_ok={reset_ok}"
        )

    print(
        f"[G2] PASS: forward={forward_dx:.3f} m, "
        f"reverse={reverse_dx:.3f} m, return error={return_error:.3f} m, "
        f"yaw drift={math.degrees(yaw_drift):.2f} deg.",
        flush=True,
    )
    if args.hold_seconds > 0.0:
        print(
            "[G2] Holding the stopped robot for visual confirmation "
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
