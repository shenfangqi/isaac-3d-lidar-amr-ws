"""Carbot goal-navigation termination terms."""

import torch


def excessive_tilt(env, asset_name: str = "robot", minimum_up_z: float = 0.7):
    asset = env.scene[asset_name]
    return asset.data.projected_gravity_b[:, 2] > -minimum_up_z


def goal_reached_termination(
    env,
    command_name: str,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
):
    command = env.command_manager.get_command(command_name)
    return torch.logical_and(
        torch.linalg.vector_norm(command[:, :2], dim=1) < position_tolerance_m,
        torch.abs(command[:, 3]) < heading_tolerance_rad,
    )
