"""Deployable Carbot policy observations."""

from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCaster


def planar_velocity(
    env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
):
    """Return odometry-compatible body ``[linear.x, angular.z]``."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.stack(
        (asset.data.root_lin_vel_b[:, 0], asset.data.root_ang_vel_b[:, 2]),
        dim=1,
    )


def lidar_ranges(
    env,
    sensor_cfg: SceneEntityCfg,
    maximum_range_m: float,
):
    """Return finite, clipped ray ranges suitable for real LiDAR parity."""
    sensor: RayCaster = env.scene.sensors[sensor_cfg.name]
    origins = sensor.data.pos_w.unsqueeze(1)
    ranges = torch.linalg.vector_norm(sensor.data.ray_hits_w - origins, dim=-1)
    ranges = torch.nan_to_num(
        ranges,
        nan=maximum_range_m,
        posinf=maximum_range_m,
        neginf=0.0,
    )
    return torch.clamp(ranges, min=0.0, max=maximum_range_m)
