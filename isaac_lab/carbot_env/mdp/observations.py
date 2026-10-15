"""Deployable Carbot policy observations."""

from __future__ import annotations

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg


def planar_velocity(
    env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
):
    """Return odometry-compatible body ``[linear.x, angular.z]``."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.stack(
        (asset.data.root_lin_vel_b[:, 0], asset.data.root_ang_vel_b[:, 2]),
        dim=1,
    )


def ray_measurements(env, sensor_cfg, minimum_range_m, maximum_range_m, validity=False):
    """Distances from each optical origin, not from the base-link centre."""
    from isaaclab.utils.math import quat_apply
    from ..sensor_geometry import encode_distances
    sensor = env.scene.sensors[sensor_cfg.name]
    data = sensor.data
    origins = data.pos_w[:, None, :] + quat_apply(
        data.quat_w[:, None, :].expand(-1, sensor.num_rays, -1), sensor.ray_starts)
    ranges = torch.linalg.vector_norm(data.ray_hits_w - origins, dim=-1)
    return encode_distances(ranges, minimum_range_m, maximum_range_m)[int(validity)]


def camera_depth(env, minimum_range_m, maximum_range_m, validity=False):
    """Optical-axis depth (metres), row-major, with explicit missing-data mask."""
    from ..sensor_geometry import encode_distances
    raw = env.scene.sensors['d455'].data.output['distance_to_image_plane'].flatten(1)
    return encode_distances(raw, minimum_range_m, maximum_range_m)[int(validity)]
