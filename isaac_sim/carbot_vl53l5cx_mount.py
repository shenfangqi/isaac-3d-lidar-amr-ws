"""Eight visual VL53L5CX mounts at the green plate edges."""


def vl53l5cx_placements(plate, config):
    """Return two outward-facing sensor poses near each plate edge."""
    half_x = plate["size_m"][0] / 2
    half_y = plate["size_m"][1] / 2
    z = (plate["position_from_base_link_m"][2]
         + plate["size_m"][2] / 2
         + config["center_above_plate_m"])
    radial_x = half_x - config["edge_inset_m"]
    radial_y = half_y - config["edge_inset_m"]
    corner_x = half_x - config["left_right_corner_inset_m"]
    corner_y = half_y - config["front_rear_corner_inset_m"]
    return (
        ("front_left", radial_x, corner_y, z, 12.5),
        ("front_right", radial_x, -corner_y, z, -12.5),
        ("rear_left", -radial_x, corner_y, z, 157.5),
        ("rear_right", -radial_x, -corner_y, z, -157.5),
        ("left_front", corner_x, radial_y, z, 67.5),
        ("left_rear", -corner_x, radial_y, z, 112.5),
        ("right_front", corner_x, -radial_y, z, -67.5),
        ("right_rear", -corner_x, -radial_y, z, -112.5),
    )


def create_vl53l5cx_mounts(stage, robot_path, plate, config):
    """Author eight visual sensor frames without adding collisions or mass."""
    from pxr import Gf, Sdf, UsdGeom

    parent_path = f"{robot_path}/base_footprint/base_link"
    if not stage.GetPrimAtPath(parent_path).IsValid():
        raise RuntimeError(f"VL53L5CX parent is missing: {parent_path}")
    root = f"{parent_path}/vl53l5cx_array"
    UsdGeom.Xform.Define(stage, root)
    sensor_paths = []
    for name, x, y, z, yaw_deg in vl53l5cx_placements(plate, config):
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
