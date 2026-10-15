"""Three sensor configurations for the installed Isaac Lab 2.2 runtime."""
import math
from isaaclab.sensors import RayCasterCfg, TiledCameraCfg, patterns
import isaaclab.sim as sim_utils
from .spec import load_environment_spec, load_robot_parameters
from .sensor_geometry import lidar_pattern, tof_pattern
from .static_raycast import StaticSceneRayCaster


def mid360_rays(cfg, device):
    return lidar_pattern(load_robot_parameters(), load_environment_spec()['observations']['lidar'], device)


def tof_rays(cfg, device):
    return tof_pattern(load_robot_parameters(), load_environment_spec()['observations']['tof'], device)


def ray_sensor(pattern, settings):
    return RayCasterCfg(
        class_type=StaticSceneRayCaster,
        prim_path='{ENV_REGEX_NS}/Robot/base_footprint/base_link',
        ray_alignment='base', pattern_cfg=patterns.PatternBaseCfg(func=pattern),
        max_distance=settings['maximum_range_m'], mesh_prim_paths=['/World/ground'],
        update_period=settings['update_period_s'], debug_vis=False,
    )


def d455_camera():
    settings = load_environment_spec()['observations']['d455']
    mount = load_robot_parameters()['sensors']['d455_sim_mount']
    center = mount['camera_center_from_pitch_xyz_m']
    front = center[0] + mount['camera_body_size_m'][0]/2 + 0.001
    # Optical origin remains provisional at the model's front centre. A separate
    # prim preserves the model camera and inherits both existing gimbal axes.
    return TiledCameraCfg(
        prim_path='{ENV_REGEX_NS}/Robot/base_footprint/base_link/d455_mount/arm1_yaw/arm2_pitch/training_depth',
        update_period=settings['update_period_s'],
        width=settings['width'], height=settings['height'],
        data_types=['distance_to_image_plane'], depth_clipping_behavior='none',
        offset=TiledCameraCfg.OffsetCfg(pos=(front, center[1], center[2]), convention='world'),
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=10.0,
            horizontal_aperture=20*math.tan(math.radians(settings['horizontal_fov_deg']/2)),
            vertical_aperture=20*math.tan(math.radians(settings['vertical_fov_deg']/2)),
            # Keep close occluders visible; range validity is applied AFTER rendering.
            clipping_range=(0.01, settings['maximum_range_m']),
        ),
    )
