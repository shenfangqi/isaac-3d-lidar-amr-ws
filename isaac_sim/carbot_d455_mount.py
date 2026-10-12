"""Visual two-axis D455 mount; no collision or rigid-body changes."""

import math


def clamp_gimbal_angles(config, yaw_rad, pitch_rad):
    """Bound the simulated pan and tilt angles in radians."""
    if not math.isfinite(yaw_rad) or not math.isfinite(pitch_rad):
        raise ValueError("D455 gimbal angles must be finite")
    yaw_limit = float(config["yaw_limit_rad"])
    pitch_limit = float(config["pitch_limit_rad"])
    return (
        max(-yaw_limit, min(yaw_limit, yaw_rad)),
        max(-pitch_limit, min(pitch_limit, pitch_rad)),
    )


def create_d455_mount(stage, robot_path, config):
    """Author a kinematic pan/tilt mount and a D455 camera prim."""
    from pxr import Gf, UsdGeom

    parent_path = f"{robot_path}/base_footprint/base_link"
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise RuntimeError(f"D455 parent is missing: {parent_path}")

    def xform(path, xyz):
        prim = UsdGeom.Xform.Define(stage, path)
        api = UsdGeom.XformCommonAPI(prim)
        api.SetTranslate(Gf.Vec3d(*xyz))
        return api

    def box(path, size, color, xyz=(0.0, 0.0, 0.0)):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        api = UsdGeom.XformCommonAPI(cube)
        api.SetTranslate(Gf.Vec3d(*xyz))
        api.SetScale(Gf.Vec3f(*size))
        return cube

    root = f"{parent_path}/d455_mount"
    yaw_path = f"{root}/arm1_yaw"
    pitch_path = f"{yaw_path}/arm2_pitch"
    xform(root, (0.0, 0.0, 0.0))
    pivot = config["yaw_pivot_xyz_m"]
    box(f"{root}/base", (0.030, 0.030, 0.008), (0.18, 0.19, 0.21),
        (pivot[0] - 0.012, pivot[1], pivot[2]))
    yaw_api = xform(yaw_path, config["yaw_pivot_xyz_m"])
    mast_height = float(config["mast_height_m"])
    box(f"{yaw_path}/mast", (0.012, 0.012, mast_height),
        (0.28, 0.29, 0.31), (0.0, 0.0, mast_height / 2))
    pitch_api = xform(pitch_path, (0.0, 0.0, mast_height))
    box(f"{pitch_path}/bracket", (0.032, 0.020, 0.006),
        (0.30, 0.31, 0.33))

    body_center = config["camera_center_from_pitch_xyz_m"]
    body_path = f"{pitch_path}/d455_body"
    box(body_path, config["camera_body_size_m"], (0.55, 0.57, 0.59),
        body_center)
    front_x = body_center[0] + config["camera_body_size_m"][0] / 2
    for index, y in enumerate((-0.040, 0.0, 0.040)):
        box(f"{pitch_path}/front_aperture_{index}",
            (0.001, 0.009, 0.009), (0.035, 0.045, 0.055),
            (front_x + 0.0005, y, body_center[2]))

    camera_path = f"{pitch_path}/d455_depth_camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera_api = UsdGeom.XformCommonAPI(camera)
    camera_api.SetTranslate(Gf.Vec3d(front_x + 0.001, 0.0, body_center[2]))
    # USD cameras look along -Z; this points the lens along robot +X.
    camera_api.SetRotate(Gf.Vec3f(90.0, 0.0, -90.0))
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 10.0))
    set_d455_mount_angles(
        yaw_api, pitch_api, config, 0.0, float(config["initial_pitch_rad"])
    )
    return yaw_api, pitch_api, camera_path


def set_d455_mount_angles(yaw_api, pitch_api, config, yaw_rad, pitch_rad):
    """Apply bounded pan/tilt without changing the robot articulation."""
    from pxr import Gf

    yaw, pitch = clamp_gimbal_angles(config, yaw_rad, pitch_rad)
    yaw_api.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
    # Robot +X is forward; a negative Y-axis rotation pitches it upward.
    pitch_api.SetRotate(Gf.Vec3f(0.0, -math.degrees(pitch), 0.0))
    return yaw, pitch
