#!/usr/bin/env python3
"""Interactive WebRTC sensor dashboard for the Carbot training environment."""
import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--steps', type=int, default=0, help='0 runs until interrupted.')
parser.add_argument('--screenshot', type=str, help='Optional Kit window screenshot for visual QA.')
parser.add_argument('--self-test', action='store_true', help='Exercise move/turn/reset controls before exiting.')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
args.width, args.height = 1600, 1000
launcher = AppLauncher(args)
app = launcher.app

import json
import signal
import traceback
import torch
import omni.ui as ui
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaacsim.core.utils.extensions import enable_extension
from isaac_lab.carbot_env.carbot_env_cfg import CarbotEnvCfgPlay

enable_extension('isaacsim.util.debug_draw')
from isaac_lab.carbot_env.sensor_dashboard import SensorDashboard


# Static reference obstacles shared by rendered depth and geometry ray casting.
OBSTACLES = [
    ('FrontWall', (1., 0., .35), (.1, 2.5, .7), (.38,.44,.51)),
    ('RearWall', (-1.4, 0., .35), (.1, 2.5, .7), (.38,.44,.51)),
    ('LeftWall', (-.2, 1.2, .35), (2.5, .1, .7), (.38,.44,.51)),
    ('RightWall', (-.2, -1.2, .35), (2.5, .1, .7), (.38,.44,.51)),
    ('LowBlock', (-.6, .5, .04), (.18,.18,.08), (.75,.42,.15)),
    ('Post', (.45, -.45, .18), (.08,.08,.36), (.7,.65,.35)),
]


def main():
    cfg = CarbotEnvCfgPlay()
    cfg.seed = 42
    cfg.ui_window_class = None
    cfg.events.reset_base = None
    cfg.events.reset_joints = None
    cfg.terminations.time_out = None
    cfg.terminations.goal_reached = None
    cfg.scene.d455.update_latest_camera_pose = True
    for name, pos, size, color in OBSTACLES:
        setattr(cfg.scene, name, AssetBaseCfg(
            prim_path=f'{{ENV_REGEX_NS}}/{name}',
            spawn=sim_utils.CuboidCfg(
                size=size, collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color)),
            init_state=AssetBaseCfg.InitialStateCfg(pos=pos)))
    env = ManagerBasedRLEnv(cfg)
    dashboard = None
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        env.reset()
        dashboard = SensorDashboard(env)
        actions = torch.zeros((1, 2), device=env.device)
        baseline = None
        checks = {}
        step = 0
        while app.is_running() and running and (args.steps == 0 or step < args.steps):
            with torch.inference_mode():
                dashboard.apply_commands()
                observations, *_ = env.step(actions)
            if step % 5 == 0:
                dashboard.update()
            if step == 15:
                dashboard.arrange_layout()
                print('CARBOT_SENSOR_CHECK_READY: WebRTC 127.0.0.1; live D455, 8 ToF, MID-360', flush=True)
                baseline = env.scene['robot'].data.root_pos_w.clone()
                checks['observation_shape'] = list(observations['policy'].shape)
            if args.self_test:
                if step == 20:
                    dashboard.commands.append(('move', .1))
                if step == 35:
                    checks['forward_m'] = float(env.scene['robot'].data.root_pos_w[0,0]-baseline[0,0])
                    assert abs(checks['forward_m']-.1) < .005
                    dashboard.commands.append(('turn', .2617993877991494))
                if step == 50:
                    quat = env.scene['robot'].data.root_quat_w[0]
                    checks['yaw_quaternion_z'] = float(quat[3])
                    assert abs(checks['yaw_quaternion_z']-.130526) < .002
                    dashboard.commands.append(('reset', 0))
                if step == 65:
                    assert abs(float(env.scene['robot'].data.root_pos_w[0,0]-baseline[0,0])) < .005
                    checks['reset_verified'] = True
                    print('CARBOT_SENSOR_DASHBOARD_TEST ' + json.dumps(checks), flush=True)
            if args.screenshot and step == 75:
                enable_extension('omni.kit.renderer.capture')
                import omni.kit.renderer.capture
                Path(args.screenshot).parent.mkdir(parents=True, exist_ok=True)
                capture = omni.kit.renderer.capture.acquire_renderer_capture_interface()
                capture.capture_next_frame_swapchain(args.screenshot)
            step += 1
    finally:
        if dashboard:
            dashboard.close()
        env.close()


try:
    main()
except Exception:
    traceback.print_exc()
    raise
finally:
    app.close()
