#!/usr/bin/env python3
"""Start one Carbot Isaac Lab environment and apply bounded zero actions."""

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--steps", type=int, default=20)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import torch  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402

from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay  # noqa: E402
from isaac_lab.carbot_env.spec import (  # noqa: E402
    load_environment_spec,
    load_robot_parameters,
)


def main():
    cfg = CarbotEnvCfgPlay()
    robot = load_robot_parameters()
    expected_per_joint_effort = (
        robot["dynamics"]["per_side_effort_limit_nm"] / 6.0
    )
    actual_per_joint_effort = cfg.scene.robot.actuators[
        "tracks"
    ].effort_limit_sim
    if actual_per_joint_effort != expected_per_joint_effort:
        raise RuntimeError(
            "Per-side effort was not distributed over six wheel joints"
        )
    env = ManagerBasedRLEnv(cfg=cfg)
    observations, _ = env.reset()
    expected_action_shape = (env.num_envs, 2)
    actions = torch.zeros(expected_action_shape, device=env.device)
    for _ in range(args.steps):
        observations, _, _, _, _ = env.step(actions)
    policy = observations["policy"]
    spec = load_environment_spec()
    height_grid = spec["observations"]["height_scan"]["grid_size"]
    expected_observation_dim = (
        4
        + 2
        + 3
        + spec["observations"]["lidar"]["ray_count"]
        + height_grid[0] * height_grid[1]
        + 2
    )
    if policy.shape != (env.num_envs, expected_observation_dim):
        raise RuntimeError(f"Unexpected policy observation shape: {policy.shape}")
    action_term = env.action_manager.get_term("twist")
    if not torch.allclose(action_term.processed_actions, actions):
        raise RuntimeError("Zero action changed inside the safety boundary")
    print(
        "Carbot Isaac Lab smoke passed: "
        f"actions={expected_action_shape}, observations={tuple(policy.shape)}"
    )
    env.close()


try:
    main()
finally:
    simulation_app.close()
