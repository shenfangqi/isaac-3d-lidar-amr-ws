#!/usr/bin/env python3
"""Validate one-metre travel against the canonical effective wheel radius."""

import argparse
import math
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--speed", type=float, default=0.20)
parser.add_argument("--target-distance", type=float, default=1.0)
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


def _realtime_delay(env, started):
    if args.realtime:
        remaining = env.step_dt - (time.perf_counter() - started)
        if remaining > 0.0:
            time.sleep(remaining)


def _step(env, actions):
    started = time.perf_counter()
    _, _, terminated, time_out, _ = env.step(actions)
    _realtime_delay(env, started)
    return int(torch.logical_or(terminated, time_out).sum().item())


def main():
    if not 0.0 < args.speed <= 0.25:
        raise ValueError("--speed must be in (0.0, 0.25] m/s for this gate")
    if args.target_distance <= 0.0:
        raise ValueError("--target-distance must be positive")

    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    cfg.viewer.eye = (0.5, -4.0, 1.5)
    cfg.viewer.lookat = (0.5, 0.0, 0.10)
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
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    forward = zeros.clone()
    forward[:, 0] = args.speed

    print(
        f"[G5] Sequence: settle -> drive {args.target_distance:.2f} m at "
        f"{args.speed:.2f} m/s -> acceleration-limited stop.",
        flush=True,
    )
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(f"[G5] Waiting for operator trigger: {trigger}", flush=True)
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(
        f"[G5] Operator ready; starting in {args.startup_delay:.0f} s.",
        flush=True,
    )
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    reset_count = 0
    for _ in range(round(2.0 / env.step_dt)):
        reset_count += _step(env, zeros)

    start_position = robot.data.root_pos_w[0].clone()
    start_yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
    start_left_travel = action_term.left_wheel_travel_rad[0].clone()
    start_right_travel = action_term.right_wheel_travel_rad[0].clone()
    acceleration = action_term.cfg.max_linear_acceleration_mps2
    braking_distance = args.speed * args.speed / (2.0 * acceleration)
    brake_at = args.target_distance - braking_distance
    if brake_at <= 0.0:
        raise ValueError("Target distance is shorter than the braking distance")

    print(
        f"[G5] FORWARD; braking begins near {brake_at:.3f} m "
        f"(predicted braking distance {braking_distance:.3f} m)",
        flush=True,
    )
    maximum_drive_steps = round(2.0 * args.target_distance / args.speed / env.step_dt)
    drive_steps = 0
    while drive_steps < maximum_drive_steps:
        reset_count += _step(env, forward)
        drive_steps += 1
        distance = (robot.data.root_pos_w[0, 0] - start_position[0]).item()
        if distance >= brake_at:
            break
    else:
        raise RuntimeError("G5 failed to reach the planned braking point")

    print("[G5] BRAKE and settle for 3.0 s", flush=True)
    for _ in range(round(3.0 / env.step_dt)):
        reset_count += _step(env, zeros)
    env.sim.render()

    final_position = robot.data.root_pos_w[0].clone()
    final_yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
    left_delta = torch.abs(
        action_term.left_wheel_travel_rad[0] - start_left_travel
    )
    right_delta = torch.abs(
        action_term.right_wheel_travel_rad[0] - start_right_travel
    )
    radius = action_term.cfg.effective_sprocket_radius_m
    left_distance = left_delta * radius
    right_distance = right_delta * radius
    body_distance = (final_position[0] - start_position[0]).item()
    lateral_error = abs((final_position[1] - start_position[1]).item())
    yaw_error = abs(
        math.atan2(
            math.sin(final_yaw - start_yaw),
            math.cos(final_yaw - start_yaw),
        )
    )
    mean_wheel_distance = torch.cat((left_distance, right_distance)).mean().item()
    wheel_spread = (
        torch.cat((left_distance, right_distance)).max()
        - torch.cat((left_distance, right_distance)).min()
    ).item()
    mean_wheel_angle = torch.cat((left_delta, right_delta)).mean().item()
    wheel_revolutions = mean_wheel_angle / (2.0 * math.pi)
    final_linear_speed = torch.linalg.vector_norm(
        robot.data.root_lin_vel_b[0, :2]
    ).item()
    final_angular_speed = abs(robot.data.root_ang_vel_b[0, 2].item())

    distance_ok = abs(body_distance - args.target_distance) <= 0.02
    wheel_ok = (
        abs(mean_wheel_distance - body_distance) <= 0.03
        and wheel_spread <= 0.01
    )
    straight_ok = lateral_error <= 0.02 and yaw_error <= math.radians(1.0)
    stop_ok = final_linear_speed <= 0.05 and final_angular_speed <= 0.05
    reset_ok = reset_count == 0
    print(
        "[G5][METRICS] "
        f"body_distance={body_distance:.4f} m, "
        f"wheel_distance={mean_wheel_distance:.4f} m, "
        f"wheel_angle={mean_wheel_angle:.3f} rad "
        f"({wheel_revolutions:.3f} rev), radius={radius:.5f} m, "
        f"wheel_spread={wheel_spread:.4f} m, "
        f"lateral_error={lateral_error:.4f} m, "
        f"yaw_error={math.degrees(yaw_error):.2f} deg, "
        f"final_speeds=({final_linear_speed:.4f} m/s, "
        f"{final_angular_speed:.4f} rad/s), resets={reset_count}",
        flush=True,
    )
    if not (distance_ok and wheel_ok and straight_ok and stop_ok and reset_ok):
        raise RuntimeError(
            "G5 failed: "
            f"distance_ok={distance_ok}, wheel_ok={wheel_ok}, "
            f"straight_ok={straight_ok}, stop_ok={stop_ok}, "
            f"reset_ok={reset_ok}"
        )

    print(
        f"[G5] PASS: body={body_distance:.3f} m; effective-radius "
        f"wheel distance={mean_wheel_distance:.3f} m; "
        f"wheel rotation={wheel_revolutions:.3f} rev.",
        flush=True,
    )
    if args.hold_seconds > 0.0:
        print(
            "[G5] Holding the stopped robot for visual confirmation "
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
