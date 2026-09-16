#!/usr/bin/env python3
"""Validate 90/360-degree turns against the canonical effective track width."""

import argparse
import math
from pathlib import Path
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--startup-delay", type=float, default=12.0)
parser.add_argument("--start-trigger-file")
parser.add_argument("--angular-speed", type=float, default=0.35)
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


def _step(env, actions):
    started = time.perf_counter()
    _, _, terminated, time_out, _ = env.step(actions)
    if args.realtime:
        remaining = env.step_dt - (time.perf_counter() - started)
        if remaining > 0.0:
            time.sleep(remaining)
    return int(torch.logical_or(terminated, time_out).sum().item())


def _wheel_turn(action_term, left_start, right_start):
    sign = action_term.cfg.wheel_joint_coordinate_sign
    radius = action_term.cfg.effective_sprocket_radius_m
    separation = action_term.cfg.effective_track_separation_m
    left_distance = (
        sign * (action_term.left_wheel_travel_rad[0] - left_start) * radius
    )
    right_distance = (
        sign * (action_term.right_wheel_travel_rad[0] - right_start) * radius
    )
    per_wheel_angle = (right_distance - left_distance) / separation
    return per_wheel_angle, left_distance, right_distance


def _turn_and_stop(env, robot, action_term, command, zeros, target_angle, label):
    start_position = robot.data.root_pos_w[0, :2].clone()
    start_yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
    previous_yaw = start_yaw
    body_angle = 0.0
    left_start = action_term.left_wheel_travel_rad[0].clone()
    right_start = action_term.right_wheel_travel_rad[0].clone()
    acceleration = action_term.cfg.max_angular_acceleration_rad_s2
    braking_angle = args.angular_speed * args.angular_speed / (2.0 * acceleration)
    brake_at = target_angle - braking_angle
    maximum_steps = round(2.0 * target_angle / args.angular_speed / env.step_dt)
    reset_count = 0

    print(
        f"[G6] {label}: turn to {math.degrees(target_angle):.0f} deg at "
        f"{args.angular_speed:.2f} rad/s; brake near "
        f"{math.degrees(brake_at):.1f} deg",
        flush=True,
    )
    for _ in range(maximum_steps):
        reset_count += _step(env, command)
        yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
        body_angle += _angle_delta(yaw, previous_yaw)
        previous_yaw = yaw
        if body_angle >= brake_at:
            break
    else:
        raise RuntimeError(f"G6 {label} failed to reach its braking point")

    print(f"[G6] {label}: BRAKE and settle for {args.stop_seconds:.1f} s", flush=True)
    for _ in range(round(args.stop_seconds / env.step_dt)):
        reset_count += _step(env, zeros)
        yaw = _yaw_from_wxyz(robot.data.root_quat_w[0])
        body_angle += _angle_delta(yaw, previous_yaw)
        previous_yaw = yaw
    env.sim.render()

    wheel_angles, left_distance, right_distance = _wheel_turn(
        action_term, left_start, right_start
    )
    wheel_angle = wheel_angles.mean().item()
    wheel_spread = (wheel_angles.max() - wheel_angles.min()).item()
    position_drift = torch.linalg.vector_norm(
        robot.data.root_pos_w[0, :2] - start_position
    ).item()
    final_linear_speed = torch.linalg.vector_norm(
        robot.data.root_lin_vel_b[0, :2]
    ).item()
    final_angular_speed = abs(robot.data.root_ang_vel_b[0, 2].item())
    left_mean = left_distance.mean().item()
    right_mean = right_distance.mean().item()

    body_error = abs(body_angle - target_angle)
    wheel_error = abs(wheel_angle - body_angle)
    passed = (
        body_error <= math.radians(2.0)
        and wheel_error <= math.radians(2.0)
        and wheel_spread <= math.radians(0.5)
        and position_drift <= 0.02
        and final_linear_speed <= 0.05
        and final_angular_speed <= 0.05
        and reset_count == 0
    )
    print(
        f"[G6][{label}][METRICS] body={math.degrees(body_angle):.2f} deg, "
        f"wheel={math.degrees(wheel_angle):.2f} deg, "
        f"body_error={math.degrees(body_error):.2f} deg, "
        f"wheel_error={math.degrees(wheel_error):.2f} deg, "
        f"left/right=({left_mean:.4f}, {right_mean:.4f}) m, "
        f"wheel_spread={math.degrees(wheel_spread):.3f} deg, "
        f"drift={position_drift:.4f} m, "
        f"final_speeds=({final_linear_speed:.4f} m/s, "
        f"{final_angular_speed:.4f} rad/s), resets={reset_count}",
        flush=True,
    )
    if not passed:
        raise RuntimeError(f"G6 {label} failed its geometry or stop checks")
    return body_angle, wheel_angle


def main():
    if not 0.0 < args.angular_speed <= 0.6:
        raise ValueError("--angular-speed must be in (0.0, 0.6] rad/s")
    if args.stop_seconds <= 0.0:
        raise ValueError("--stop-seconds must be positive")

    cfg = CarbotEnvCfgPlay()
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
    cfg.viewer.eye = (1.8, -2.8, 2.8)
    cfg.viewer.lookat = (0.0, 0.0, 0.08)
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
    positive = zeros.clone()
    positive[:, 1] = args.angular_speed

    separation = action_term.cfg.effective_track_separation_m
    print(
        "[G6] Sequence: +90 deg -> stop -> +360 deg -> stop; "
        f"effective track separation={separation:.3f} m.",
        flush=True,
    )
    env.sim.render()
    if args.start_trigger_file:
        trigger = Path(args.start_trigger_file)
        print(f"[G6] Waiting for operator trigger: {trigger}", flush=True)
        while simulation_app.is_running() and not trigger.exists():
            env.sim.render()
            time.sleep(0.05)
        if not simulation_app.is_running():
            env.close()
            return
        trigger.unlink(missing_ok=True)
    print(f"[G6] Operator ready; starting in {args.startup_delay:.0f} s.", flush=True)
    deadline = time.monotonic() + args.startup_delay
    while simulation_app.is_running() and time.monotonic() < deadline:
        env.sim.render()
        time.sleep(0.05)

    reset_count = 0
    for _ in range(round(2.0 / env.step_dt)):
        reset_count += _step(env, zeros)
    if reset_count:
        raise RuntimeError("G6 reset during initial settling")

    body_90, wheel_90 = _turn_and_stop(
        env, robot, action_term, positive, zeros, math.pi / 2.0, "TURN-90"
    )
    body_360, wheel_360 = _turn_and_stop(
        env, robot, action_term, positive, zeros, 2.0 * math.pi, "TURN-360"
    )
    print(
        f"[G6] PASS: 90 deg body/wheel=({math.degrees(body_90):.2f}, "
        f"{math.degrees(wheel_90):.2f}) deg; 360 deg body/wheel="
        f"({math.degrees(body_360):.2f}, {math.degrees(wheel_360):.2f}) deg; "
        f"effective track separation={separation:.3f} m.",
        flush=True,
    )

    if args.hold_seconds > 0.0:
        print(
            "[G6] Holding the stopped robot for visual confirmation "
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
