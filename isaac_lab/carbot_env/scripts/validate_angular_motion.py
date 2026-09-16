#!/usr/bin/env python3
"""Visually and quantitatively validate low-speed positive/negative yaw."""

import argparse
import math
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--robot-usd")
parser.add_argument("--angular-speed", type=float, default=0.35)
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


def _angle_delta(end, start):
    return math.atan2(math.sin(end - start), math.cos(end - start))


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
        f"[G3][TIMING] {label}: {steps} control steps in "
        f"{time.perf_counter() - started:.3f} s wall; resets={reset_count}",
        flush=True,
    )
    return reset_count


def main():
    if not 0.0 < args.angular_speed <= 0.6:
        raise ValueError(
            "--angular-speed must be in (0.0, 0.6] rad/s for this gate"
        )
    if args.render_stride < 1:
        raise ValueError("--render-stride must be at least 1")

    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    if args.robot_usd:
        cfg.scene.robot.spawn.usd_path = args.robot_usd
    cfg.episode_length_s = 3600.0
    # Fixed elevated view keeps the chassis heading visible without following
    # the robot during the positive and negative yaw phases.
    cfg.viewer.eye = (1.8, -2.8, 2.4)
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

    env = ManagerBasedRLEnv(cfg=cfg)
    env.reset()
    robot = env.scene["robot"]
    action_term = env.action_manager.get_term("twist")
    body_masses = robot.data.default_mass[0]
    print(
        f"[G3][MASS] total articulation mass={body_masses.sum().item():.3f} kg; "
        + ", ".join(
            f"{name}={body_masses[index].item():.3f}"
            for index, name in enumerate(robot.body_names)
        ),
        flush=True,
    )
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    positive = zeros.clone()
    negative = zeros.clone()
    positive[:, 1] = args.angular_speed
    negative[:, 1] = -args.angular_speed

    print("[G3] Sequence: positive yaw -> stop -> negative yaw -> stop.", flush=True)
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(f"[G3] Waiting for operator trigger: {trigger}", flush=True)
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(
        f"[G3] Operator ready; starting in {args.startup_delay:.0f} s.",
        flush=True,
    )
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    start, start_yaw = _pose(robot)
    print(
        f"[G3] POSITIVE YAW +{args.angular_speed:.2f} rad/s for "
        f"{args.motion_seconds:.1f} s",
        flush=True,
    )
    positive_resets = _run_actions(
        env, positive, args.motion_seconds, "positive-yaw"
    )
    positive_end, positive_yaw = _pose(robot)
    positive_left_velocity = robot.data.joint_vel[
        0, action_term._left_joint_ids
    ].mean().item()
    positive_right_velocity = robot.data.joint_vel[
        0, action_term._right_joint_ids
    ].mean().item()
    print(
        "[G3][WHEELS] positive targets/actual means: "
        f"left=({action_term._left_targets[0, 0].item():.3f}, "
        f"{positive_left_velocity:.3f}) rad/s, "
        f"right=({action_term._right_targets[0, 0].item():.3f}, "
        f"{positive_right_velocity:.3f}) rad/s",
        flush=True,
    )

    print(f"[G3] STOP for {args.stop_seconds:.1f} s", flush=True)
    first_stop_resets = _run_actions(
        env, zeros, args.stop_seconds, "first-stop"
    )
    first_stop, first_stop_yaw = _pose(robot)

    print(
        f"[G3] NEGATIVE YAW -{args.angular_speed:.2f} rad/s for "
        f"{args.motion_seconds:.1f} s",
        flush=True,
    )
    negative_resets = _run_actions(
        env, negative, args.motion_seconds, "negative-yaw"
    )
    negative_end, negative_yaw = _pose(robot)

    print(f"[G3] FINAL STOP for {args.stop_seconds:.1f} s", flush=True)
    final_stop_resets = _run_actions(
        env, zeros, args.stop_seconds, "final-stop"
    )
    final, final_yaw = _pose(robot)

    positive_delta = _angle_delta(positive_yaw, start_yaw)
    positive_stop_delta = _angle_delta(first_stop_yaw, positive_yaw)
    negative_delta = _angle_delta(negative_yaw, first_stop_yaw)
    negative_stop_delta = _angle_delta(final_yaw, negative_yaw)
    return_error = abs(_angle_delta(final_yaw, start_yaw))
    position_drift = torch.linalg.vector_norm(final[:2] - start[:2]).item()
    maximum_excursion = max(
        torch.linalg.vector_norm(positive_end[:2] - start[:2]).item(),
        torch.linalg.vector_norm(first_stop[:2] - start[:2]).item(),
        torch.linalg.vector_norm(negative_end[:2] - start[:2]).item(),
        position_drift,
    )
    final_linear_speed = torch.linalg.vector_norm(
        robot.data.root_lin_vel_b[0, :2]
    ).item()
    final_angular_speed = abs(robot.data.root_ang_vel_b[0, 2].item())
    resets = (
        positive_resets,
        first_stop_resets,
        negative_resets,
        final_stop_resets,
    )
    expected_angle = args.angular_speed * args.motion_seconds
    direction_ok = (
        expected_angle * 0.8 <= positive_delta <= expected_angle * 1.1
        and -expected_angle * 1.1 <= negative_delta <= -expected_angle * 0.8
    )
    symmetry_error = abs(abs(positive_delta) - abs(negative_delta))
    symmetry_ok = (
        symmetry_error <= math.radians(8.0)
        and return_error <= math.radians(5.0)
    )
    inplace_ok = maximum_excursion <= 0.10
    stop_ok = final_linear_speed <= 0.05 and final_angular_speed <= 0.05
    reset_ok = sum(resets) == 0

    print(
        "[G3][METRICS] "
        f"positive={math.degrees(positive_delta):.2f} deg, "
        f"positive_brake={math.degrees(positive_stop_delta):.2f} deg, "
        f"negative={math.degrees(negative_delta):.2f} deg, "
        f"negative_brake={math.degrees(negative_stop_delta):.2f} deg, "
        f"return_error={math.degrees(return_error):.2f} deg, "
        f"symmetry_error={math.degrees(symmetry_error):.2f} deg, "
        f"position_drift={position_drift:.4f} m, "
        f"maximum_excursion={maximum_excursion:.4f} m, "
        f"final_speeds=({final_linear_speed:.4f} m/s, "
        f"{final_angular_speed:.4f} rad/s), resets={resets}",
        flush=True,
    )
    if not (direction_ok and symmetry_ok and inplace_ok and stop_ok and reset_ok):
        raise RuntimeError(
            "G3 failed: "
            f"direction_ok={direction_ok}, symmetry_ok={symmetry_ok}, "
            f"inplace_ok={inplace_ok}, stop_ok={stop_ok}, reset_ok={reset_ok}"
        )

    print(
        f"[G3] PASS: positive={math.degrees(positive_delta):.1f} deg, "
        f"negative={math.degrees(negative_delta):.1f} deg, "
        f"return error={math.degrees(return_error):.1f} deg, "
        f"position drift={position_drift:.3f} m.",
        flush=True,
    )
    if args.hold_seconds > 0.0:
        print(
            "[G3] Holding the stopped robot for visual confirmation "
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
