"""Goal-navigation rewards independent from the Carbot USD."""

import torch


def position_tracking(env, command_name: str, standard_deviation_m: float):
    command = env.command_manager.get_command(command_name)
    distance = torch.linalg.vector_norm(command[:, :2], dim=1)
    return 1.0 - torch.tanh(distance / standard_deviation_m)


def heading_error(env, command_name: str):
    command = env.command_manager.get_command(command_name)
    return torch.abs(command[:, 3])


def planar_speed(env, asset_name: str = "robot"):
    asset = env.scene[asset_name]
    return torch.abs(asset.data.root_lin_vel_b[:, 0]) + torch.abs(
        asset.data.root_ang_vel_b[:, 2]
    )


def goal_reached_reward(
    env,
    command_name: str,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
):
    command = env.command_manager.get_command(command_name)
    position_ok = torch.linalg.vector_norm(command[:, :2], dim=1) < (
        position_tolerance_m
    )
    heading_ok = torch.abs(command[:, 3]) < heading_tolerance_rad
    return torch.logical_and(position_ok, heading_ok).float()
