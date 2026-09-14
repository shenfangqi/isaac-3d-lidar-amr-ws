"""Create the Carbot Mid-360 RTX coverage proxy and ROS publisher."""

import shutil
from pathlib import Path


def mid360_runtime_config(parameters):
    """Return the simulation fields consumed by the Isaac runtime."""
    mid360 = parameters["sensors"]["mid360"]
    return {
        "profile_name": mid360["simulation_profile_name"],
        "pointcloud_topic": mid360["pointcloud_topic"],
        "frame_id": mid360["simulation_compatibility_frame"],
        "origin_from_housing_bottom_m": tuple(
            mid360["simulation_rtx_origin_from_housing_bottom_m"]
        ),
    }


def find_unique_link_prim(stage, link_name):
    """Find one active imported URDF link by its terminal prim name."""
    matches = [
        prim for prim in stage.Traverse()
        if prim.GetName() == link_name
    ]
    if len(matches) != 1:
        paths = [str(prim.GetPath()) for prim in matches]
        raise RuntimeError(
            f"Expected one active {link_name} prim, found {paths}"
        )
    return matches[0]


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
    parent = find_unique_link_prim(stage, "lidar_link")
    sensor_path = parent.GetPath().AppendChild("mid360_rtx")
    if stage.GetPrimAtPath(sensor_path).IsValid():
        raise RuntimeError(f"Mid-360 RTX prim already exists: {sensor_path}")

    created, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=str(sensor_path),
        parent=None,
        config=config["profile_name"],
        translation=Gf.Vec3d(*config["origin_from_housing_bottom_m"]),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=False,
    )
    if not created or sensor is None or not sensor.IsValid():
        raise RuntimeError("Isaac Sim failed to create the Carbot Mid-360")

    render_product = rep.create.render_product(
        sensor.GetPath(), [1, 1], name="CarbotMid360"
    )
    graph_path = "/Carbot/ROS2_Mid360"
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
