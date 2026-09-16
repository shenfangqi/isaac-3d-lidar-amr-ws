"""Create the Carbot Mid-360 RTX coverage proxy and ROS publisher."""

import math
import shutil
from pathlib import Path


def mid360_runtime_config(parameters):
    """Return the simulation fields consumed by the Isaac runtime."""
    mid360 = parameters["sensors"]["mid360"]
    control_period_s = parameters["control"]["differential_period_s"]
    pointcloud_rate_hz = mid360["pointcloud_rate_hz"]
    publish_step_count = 1.0 / (pointcloud_rate_hz * control_period_s)
    rounded_step_count = round(publish_step_count)
    if rounded_step_count < 1 or not math.isclose(
        publish_step_count, rounded_step_count, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(
            "MID360 pointcloud_rate_hz must divide the simulation control "
            "rate into an integer publish step count"
        )
    return {
        "profile_name": mid360["simulation_profile_name"],
        "pointcloud_topic": mid360["pointcloud_topic"],
        "pointcloud_rate_hz": pointcloud_rate_hz,
        "frame_skip_count": rounded_step_count - 1,
        "frame_id": mid360["simulation_compatibility_frame"],
        "origin_from_housing_bottom_m": tuple(
            mid360["simulation_rtx_origin_from_housing_bottom_m"]
        ),
    }


def find_unique_link_prim(stage, articulation_root_path, link_name):
    """Find one active link below the configured articulation only.

    Isaac stages can contain inactive legacy robots or runtime helper prims with
    the same terminal link name.  A global name-only lookup can therefore bind
    the sensor to a different robot tree.
    """
    root = str(articulation_root_path).rstrip("/")
    root_prim = stage.GetPrimAtPath(root)
    if not root_prim.IsValid() or not root_prim.IsActive():
        raise RuntimeError(f"Carbot articulation root is missing: {root}")
    matches = [
        prim for prim in stage.Traverse()
        if prim.GetName() == link_name
        and str(prim.GetPath()).startswith(f"{root}/")
    ]
    if len(matches) != 1:
        paths = [str(prim.GetPath()) for prim in matches]
        raise RuntimeError(
            f"Expected one active {link_name} prim, found {paths}"
        )
    return matches[0]


def validate_carbot_hierarchy(stage, articulation_root_path, sensor_path=None):
    """Reject duplicate active Carbot roots and a detached RTX sensor."""
    root = str(articulation_root_path).rstrip("/")
    asset_root = root.rsplit("/", 1)[0]
    carbot_roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName().lower() == "carbot"
    ]
    if len(carbot_roots) != 1 or carbot_roots[0] != asset_root:
        raise RuntimeError(
            f"Expected one active Carbot root at {asset_root}, found {carbot_roots}"
        )
    if sensor_path is not None:
        sensor = str(sensor_path)
        if not sensor.startswith(f"{root}/"):
            raise RuntimeError(
                f"Mid-360 must be below {root}, got {sensor}"
            )
    return asset_root


def install_profile(profile_path, get_extension_path_from_name):
    """Install the local profile where the RTX extension resolves configs."""
    extension_path = Path(
        get_extension_path_from_name("isaacsim.sensors.rtx")
    )
    installed_profile = (
        extension_path / "data/lidar_configs" / profile_path.name
    )
    shutil.copyfile(profile_path, installed_profile)
    return installed_profile


def create_mid360_pipeline(stage, parameters):
    """Attach an RTX lidar to Carbot and publish its PointCloud2 stream."""
    import omni.graph.core as og
    import omni.kit.commands
    import omni.replicator.core as rep
    from pxr import Gf

    config = mid360_runtime_config(parameters)
    articulation_root_path = parameters["simulation"][
        "articulation_root_prim"
    ]
    asset_root = validate_carbot_hierarchy(stage, articulation_root_path)
    parent = find_unique_link_prim(
        stage,
        articulation_root_path,
        parameters["sensors"]["mid360"]["parent_frame"],
    )
    sensor_name = "mid360_rtx"
    sensor_path = parent.GetPath().AppendChild(sensor_name)
    if stage.GetPrimAtPath(sensor_path).IsValid():
        raise RuntimeError(f"Mid-360 RTX prim already exists: {sensor_path}")

    created, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=sensor_name,
        parent=str(parent.GetPath()),
        config=config["profile_name"],
        translation=Gf.Vec3d(*config["origin_from_housing_bottom_m"]),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=False,
    )
    if not created or sensor is None or not sensor.IsValid():
        raise RuntimeError("Isaac Sim failed to create the Carbot Mid-360")
    validate_carbot_hierarchy(stage, articulation_root_path, sensor.GetPath())

    render_product = rep.create.render_product(
        sensor.GetPath(), [1, 1], name="CarbotMid360"
    )
    graph_path = f"{asset_root}/ROS2_Mid360"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                (
                    "PublishPointCloud",
                    "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "PublishPointCloud.inputs:renderProductPath",
                    render_product.path,
                ),
                (
                    "PublishPointCloud.inputs:topicName",
                    config["pointcloud_topic"],
                ),
                ("PublishPointCloud.inputs:frameId", config["frame_id"]),
                ("PublishPointCloud.inputs:type", "point_cloud"),
                ("PublishPointCloud.inputs:fullScan", False),
                (
                    "PublishPointCloud.inputs:frameSkipCount",
                    config["frame_skip_count"],
                ),
                (
                    "PublishPointCloud.inputs:resetSimulationTimeOnStop",
                    True,
                ),
            ],
            og.Controller.Keys.CONNECT: [
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishPointCloud.inputs:execIn",
                ),
            ],
        },
    )
    return sensor, render_product, graph_path
