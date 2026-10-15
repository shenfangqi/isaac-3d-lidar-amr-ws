#!/usr/bin/env python3
"""Validate actual GPU sensor readings against known walls in two clones."""
import argparse
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser(description=__doc__)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app = AppLauncher(args).app

import json
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.utils.math import quat_apply
from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay
from isaac_lab.carbot_env.sensor_geometry import encode_distances


WALLS = [((1., 0., 1.), (.1, 4., 2.)),
         ((-1.5, 0., 1.), (.1, 4., 2.)),
         ((0., 1.2, 1.), (4., .1, 2.)),
         ((0., -1.2, 1.), (4., .1, 2.))]


def analytic_hits(origins, directions, env_origins):
    """Slab intersections in each clone's local world coordinates."""
    origins = origins - env_origins[:, None, :]
    distance = torch.full(origins.shape[:-1], float('inf'), device=origins.device)
    safe_direction = torch.where(directions.abs() < 1e-10, torch.full_like(directions, 1e-10), directions)
    for center, size in WALLS:
        center = torch.tensor(center, device=origins.device)
        half = torch.tensor(size, device=origins.device)/2
        a = (center-half-origins)/safe_direction
        b = (center+half-origins)/safe_direction
        near = torch.minimum(a, b).amax(-1)
        far = torch.maximum(a, b).amin(-1)
        distance = torch.minimum(distance, torch.where((far >= near) & (near > 0), near, float('inf')))
    floor = -origins[..., 2]/safe_direction[..., 2]
    return torch.minimum(distance, torch.where((directions[..., 2] < 0) & (floor > 0), floor, float('inf')))


def check_rays(env, name):
    sensor = env.scene[name]
    data = sensor.data
    quat = data.quat_w[:, None, :].expand(-1, sensor.num_rays, -1)
    origins = data.pos_w[:, None, :] + quat_apply(quat, sensor.ray_starts)
    directions = quat_apply(quat, sensor.ray_directions)
    expected = analytic_hits(origins, directions, env.scene.env_origins)
    measured = (data.ray_hits_w-origins).norm(dim=-1)
    valid = (expected <= sensor.cfg.max_distance) & torch.isfinite(expected)
    assert valid.any(), name
    error = (expected[valid]-measured[valid]).abs().max().item()
    if error >= .003:
        bad = torch.argmax(torch.where(valid, (expected-measured).abs(), 0)).item()
        e, r = divmod(bad, sensor.num_rays)
        print('RAY_MISMATCH', name, e, r, 'origin', origins[e,r].tolist(),
              'direction', directions[e,r].tolist(), 'expected', expected[e,r].item(),
              'actual', measured[e,r].item(), 'hit', data.ray_hits_w[e,r].tolist(), flush=True)
    assert error < 0.003, (name, error)
    return {'rays_per_env': sensor.num_rays, 'max_error_m': error}


def main():
    cfg = CarbotEnvCfgPlay()
    cfg.scene.num_envs = 2
    cfg.scene.env_spacing = 8.
    cfg.events.reset_base = None
    cfg.events.reset_joints = None
    cfg.scene.d455.update_latest_camera_pose = True
    for i, (center, size) in enumerate(WALLS):
        setattr(cfg.scene, f'wall_{i}', AssetBaseCfg(
            prim_path=f'{{ENV_REGEX_NS}}/Wall{i}',
            spawn=sim_utils.CuboidCfg(size=size, collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=AssetBaseCfg.InitialStateCfg(pos=center)))
    env = ManagerBasedRLEnv(cfg)
    env.reset()
    actions = torch.zeros((2, 2), device=env.device)
    for _ in range(12):
        observations, *_ = env.step(actions)
    assert observations['policy'].shape == (2, 3723)
    assert torch.isfinite(observations['policy']).all()
    report = {name: check_rays(env, name) for name in ('lidar', 'tof')}
    camera = env.scene['d455']
    data = camera.data
    depth = data.output['distance_to_image_plane'].squeeze(-1)
    height, width = depth.shape[-2:]
    v, u = torch.meshgrid(torch.arange(height, device=env.device)+.5,
                          torch.arange(width, device=env.device)+.5, indexing='ij')
    k = data.intrinsic_matrices
    directions = torch.stack(((u[None]-k[:, None, None, 0, 2])/k[:, None, None, 0, 0],
                              (v[None]-k[:, None, None, 1, 2])/k[:, None, None, 1, 1],
                              torch.ones((2,height,width), device=env.device)), -1).flatten(1, 2)
    directions = quat_apply(data.quat_w_ros[:, None, :].expand(-1,height*width,-1), directions)
    origins = data.pos_w[:, None, :].expand_as(directions)
    expected = analytic_hits(origins, directions, env.scene.env_origins).reshape_as(depth)
    valid = torch.isfinite(expected) & (expected >= .6) & (expected <= 6)
    error = (depth[valid]-expected[valid]).abs().max().item()
    assert valid.any() and error < .03, ('d455', error)
    report['d455'] = {'shape': list(depth.shape), 'max_error_m': error,
                      'valid_pixels': int(valid.sum())}
    # Different pose for clone 1 proves full body rotation and per-clone origins.
    state = env.scene['robot'].data.default_root_state.clone()
    state[:, :3] += env.scene.env_origins
    state[1, 0] += .15
    state[1, 3:7] = torch.tensor([.70710678, 0., 0., .70710678], device=env.device)
    env.scene['robot'].write_root_state_to_sim(state)
    env.scene.reset()
    env.action_manager.reset()
    for _ in range(12):
        env.step(actions)
    for name in ('lidar', 'tof'):
        report[name]['after_pose_change'] = check_rays(env, name)
    assert not torch.allclose(env.scene['d455'].data.output['distance_to_image_plane'][0],
                              env.scene['d455'].data.output['distance_to_image_plane'][1])
    report['pose_change_verified'] = True
    print('CARBOT_SENSOR_VALIDATION ' + json.dumps(report), flush=True)
    env.close()


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    raise
finally:
    app.close()
