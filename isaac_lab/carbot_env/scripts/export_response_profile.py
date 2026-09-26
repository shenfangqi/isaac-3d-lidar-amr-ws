#!/usr/bin/env python3
"""Run real-trial-matched Carbot commands in Isaac Lab and export metrics."""

import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay  # noqa: E402


TRIALS = (
    {
        "id": "forward_0p05_10s",
        "linear": 0.05,
        "angular": 0.0,
        "duration_s": 10.0,
    },
    {
        "id": "reverse_0p05_10s",
        "linear": -0.05,
        "angular": 0.0,
        "duration_s": 10.0,
    },
    {
        "id": "turn_left_composite_0p20",
        "linear": 0.0,
        "angular": 0.20,
        "duration_s": 12.0,
    },
    {
        "id": "turn_right_composite_0p20",
        "linear": 0.0,
        "angular": -0.20,
        "duration_s": 14.0,
    },
)


def yaw_from_wxyz(quaternion):
    w, x, y, z = quaternion.tolist()
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_delta(end, start):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def configure():
    cfg = CarbotEnvCfgPlay()
    cfg.seed = 42
    cfg.sim.device = args.device
    cfg.episode_length_s = 3600.0
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
    cfg.sim.render_interval = 20
    return cfg


def run_steps(env, actions, seconds):
    steps = max(1, round(seconds / env.step_dt))
    yaw_integral = 0.0
    distance_integral = 0.0
    first_motion_s = None
    for index in range(steps):
        env.step(actions)
        linear = float(env.scene["robot"].data.root_lin_vel_b[0, 0].item())
        angular = float(env.scene["robot"].data.root_ang_vel_b[0, 2].item())
        distance_integral += linear * env.step_dt
        yaw_integral += angular * env.step_dt
        if first_motion_s is None and max(abs(linear), abs(angular)) > 1.0e-3:
            first_motion_s = (index + 1) * env.step_dt
    return distance_integral, yaw_integral, first_motion_s


def run_trial(env, trial):
    env.reset()
    zeros = torch.zeros((env.num_envs, 2), device=env.device)
    for _ in range(round(1.0 / env.step_dt)):
        env.step(zeros)
    robot = env.scene["robot"]
    start_position = robot.data.root_pos_w[0].clone()
    start_yaw = yaw_from_wxyz(robot.data.root_quat_w[0])
    command = zeros.clone()
    command[:, 0] = trial["linear"]
    command[:, 1] = trial["angular"]
    distance_integral, yaw_integral, latency = run_steps(
        env, command, trial["duration_s"]
    )
    command_end_position = robot.data.root_pos_w[0].clone()
    command_end_yaw = yaw_from_wxyz(robot.data.root_quat_w[0])
    stop_steps = round(2.0 / env.step_dt)
    stop_tail_s = None
    for index in range(stop_steps):
        env.step(zeros)
        linear = abs(float(robot.data.root_lin_vel_b[0, 0].item()))
        angular = abs(float(robot.data.root_ang_vel_b[0, 2].item()))
        if stop_tail_s is None and max(linear, angular) <= 1.0e-3:
            stop_tail_s = (index + 1) * env.step_dt
    final_position = robot.data.root_pos_w[0].clone()
    final_yaw = yaw_from_wxyz(robot.data.root_quat_w[0])
    return {
        **trial,
        "step_dt_s": env.step_dt,
        "first_motion_latency_s": latency,
        "stop_tail_s": stop_tail_s,
        "command_distance_integral_m": distance_integral,
        "command_yaw_integral_rad": yaw_integral,
        "command_pose_dx_m": float(
            command_end_position[0] - start_position[0]
        ),
        "command_pose_dy_m": float(
            command_end_position[1] - start_position[1]
        ),
        "command_pose_yaw_rad": angle_delta(command_end_yaw, start_yaw),
        "final_pose_dx_m": float(final_position[0] - start_position[0]),
        "final_pose_dy_m": float(final_position[1] - start_position[1]),
        "final_pose_yaw_rad": angle_delta(final_yaw, start_yaw),
    }


def main():
    env = ManagerBasedRLEnv(cfg=configure())
    try:
        results = [run_trial(env, trial) for trial in TRIALS]
    finally:
        env.close()
    output = {
        "runtime": "Isaac Lab 4.5 CarbotEnvCfgPlay",
        "seed": 42,
        "trials": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(output, indent=2, sort_keys=True), flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
