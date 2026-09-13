"""Load the Mid-360 warehouse inside an existing streaming Kit app."""

import asyncio
import shutil
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from omni.isaac.core.utils.extensions import get_extension_path_from_name


USD_PATH = (
    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/usd/"
    "warehouse_3d_nav_origin_carter.usd"
)
WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
PROFILE_PATH = WORKSPACE / "isaac_sim/lidar_configs/Livox_Mid360_Approx.json"
# Legacy Carter asset paths are retained only to preserve the upstream mount
# and graph connections. The active sensor configuration is Mid-360.
SENSOR_PRIM = "/nova_carter_ROS111/chassis_link/XT_32/PandarXT_32_10hz"
PUBLISHER_PRIM = "/nova_carter_ROS111/ros_lidars/publish_front_3d_lidar_scan"


async def load_and_play() -> None:
    rtx_extension = Path(get_extension_path_from_name("isaacsim.sensors.rtx"))
    installed_profile = rtx_extension / "data/lidar_configs/Livox_Mid360_Approx.json"
    shutil.copyfile(PROFILE_PATH, installed_profile)
    print(f"Installed Mid-360 proxy profile: {installed_profile}", flush=True)

    print(f"Opening streaming USD: {USD_PATH}", flush=True)
    omni.usd.get_context().open_stage(USD_PATH)

    app = omni.kit.app.get_app()
    for _ in range(500):
        await app.next_update_async()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Streaming warehouse USD stage failed to load")

    sensor = stage.GetPrimAtPath(SENSOR_PRIM)
    publisher = stage.GetPrimAtPath(PUBLISHER_PRIM)
    if not sensor.IsValid() or not publisher.IsValid():
        raise RuntimeError("Expected Carter lidar or ROS publisher prim is missing")

    sensor.GetAttribute("sensorModelConfig").Set("Livox_Mid360_Approx")
    publisher.GetAttribute("inputs:topicName").Set("/livox/lidar")
    publisher.GetAttribute("inputs:frameId").Set("front_3d_lidar")
    print(
        "Sensor profile: Livox_Mid360_Approx; "
        "ROS point cloud: /livox/lidar [frame_id=front_3d_lidar]",
        flush=True,
    )

    print(f"Streaming stage loaded: {stage.GetRootLayer().realPath}", flush=True)
    omni.timeline.get_timeline_interface().play()

    for _ in range(30):
        await app.next_update_async()

    print("Streaming Mid-360 warehouse timeline started.", flush=True)


asyncio.ensure_future(load_and_play())
