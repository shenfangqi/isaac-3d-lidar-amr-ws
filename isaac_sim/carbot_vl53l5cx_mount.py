"""Eight low-perimeter visual VL53L5CX mounts."""

import math


def vl53l5cx_placements(geometry, config):
    """Return board-centre poses with optical centres near the footprint."""
    footprint = geometry["footprint_m"]
    front = max(point[0] for point in footprint)
    rear = min(point[0] for point in footprint)
    left = max(point[1] for point in footprint)
    right = min(point[1] for point in footprint)
    perimeter_inset = config["optical_center_footprint_inset_m"]
    lateral_inset = config["front_rear_lateral_inset_m"]
    longitudinal_inset = config["side_longitudinal_inset_m"]
    z = (
        config["optical_center_height_from_ground_m"]
        - geometry["base_link_height_m"]
    )
    optical_poses = (
        ("front_left", front - perimeter_inset, left - lateral_inset, z, 12.5),
        ("front_right", front - perimeter_inset, right + lateral_inset, z, -12.5),
        ("rear_left", rear + perimeter_inset, left - lateral_inset, z, 157.5),
        ("rear_right", rear + perimeter_inset, right + lateral_inset, z, -157.5),
        ("left_front", front - longitudinal_inset, left - perimeter_inset, z, 67.5),
        ("left_rear", rear + longitudinal_inset, left - perimeter_inset, z, 112.5),
        ("right_front", front - longitudinal_inset, right + perimeter_inset, z, -67.5),
        ("right_rear", rear + longitudinal_inset, right + perimeter_inset, z, -112.5),
    )
    optical_offset = config["board_size_m"][0] / 2
    return tuple(
        (
            name,
            optical_x - math.cos(math.radians(yaw_deg)) * optical_offset,
            optical_y - math.sin(math.radians(yaw_deg)) * optical_offset,
            optical_z,
            yaw_deg,
        )
        for name, optical_x, optical_y, optical_z, yaw_deg in optical_poses
    )


def create_vl53l5cx_mounts(stage, robot_path, geometry, config):
    """Author eight visual sensor frames and nonphysical L-brackets."""
    from pxr import Gf, Sdf, UsdGeom

    parent_path = f"{robot_path}/base_footprint/base_link"
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise RuntimeError(f"VL53L5CX parent is missing: {parent_path}")
    root = f"{parent_path}/vl53l5cx_array"
    UsdGeom.Xform.Define(stage, root)
    plate = geometry["body_visual_base_plate"]
    half_plate_x = plate["size_m"][0] / 2
    half_plate_y = plate["size_m"][1] / 2
    plate_bottom = plate["position_from_base_link_m"][2] - plate["size_m"][2] / 2

    def bracket_box(path, center, size, yaw_deg=0.0):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.16, 0.17, 0.19)])
        api = UsdGeom.XformCommonAPI(cube)
        api.SetTranslate(Gf.Vec3d(*center))
        api.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_deg))
        api.SetScale(Gf.Vec3f(*size))

    sensor_paths = []
    for name, x, y, z, yaw_deg in vl53l5cx_placements(geometry, config):
        sensor_path = f"{root}/{name}"
        sensor = UsdGeom.Xform.Define(stage, sensor_path)
        transform = UsdGeom.XformCommonAPI(sensor)
        transform.SetTranslate(Gf.Vec3d(x, y, z))
        transform.SetRotate(Gf.Vec3f(0.0, 0.0, yaw_deg))
        sensor.GetPrim().CreateAttribute(
            "carbot:sensorType", Sdf.ValueTypeNames.String
        ).Set("VL53L5CX")
        sensor.GetPrim().CreateAttribute(
            "carbot:detectionFovDeg", Sdf.ValueTypeNames.Float2
        ).Set(Gf.Vec2f(*config["detection_fov_deg"]))

        anchor_x = max(-half_plate_x, min(half_plate_x, x))
        anchor_y = max(-half_plate_y, min(half_plate_y, y))
        dx, dy = x - anchor_x, y - anchor_y
        arm_length = math.hypot(dx, dy)
        if arm_length > 1e-6:
            bracket_box(
                f"{root}/{name}_bracket_arm",
                ((x + anchor_x) / 2, (y + anchor_y) / 2, plate_bottom - 0.0025),
                (arm_length, 0.005, 0.005),
                math.degrees(math.atan2(dy, dx)),
            )
        board_top = z + config["board_size_m"][2] / 2
        drop_height = plate_bottom - board_top
        if drop_height > 1e-6:
            bracket_box(
                f"{root}/{name}_bracket_drop",
                (x, y, board_top + drop_height / 2),
                (0.005, 0.005, drop_height),
            )

        body = UsdGeom.Cube.Define(stage, f"{sensor_path}/board")
        body.CreateSizeAttr(1.0)
        body.CreateDisplayColorAttr([Gf.Vec3f(0.92, 0.56, 0.08)])
        UsdGeom.XformCommonAPI(body).SetScale(
            Gf.Vec3f(*config["board_size_m"])
        )
        window = UsdGeom.Cube.Define(stage, f"{sensor_path}/optical_window")
        window.CreateSizeAttr(1.0)
        window.CreateDisplayColorAttr([Gf.Vec3f(0.02, 0.03, 0.04)])
        window_api = UsdGeom.XformCommonAPI(window)
        window_api.SetTranslate(Gf.Vec3d(config["board_size_m"][0] / 2,
                                          0.0, 0.0))
        window_api.SetScale(Gf.Vec3f(*config["optical_window_size_m"]))
        optical = UsdGeom.Xform.Define(stage, f"{sensor_path}/sensor_origin")
        UsdGeom.XformCommonAPI(optical).SetTranslate(
            Gf.Vec3d(config["board_size_m"][0] / 2, 0.0, 0.0)
        )
        sensor_paths.append(sensor_path)
    return tuple(sensor_paths)
