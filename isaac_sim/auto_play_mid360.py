"""Run the warehouse with a Mid-360 coverage proxy.

The RTX lidar profile approximates the Mid-360 range, point rate, and
asymmetric vertical field of view. Isaac Sim 4.5 cannot reproduce Livox's
non-repetitive scan pattern, per-point timestamps, packet loss, or motion
distortion, so this script is intended for coverage and navigation testing.
"""

import os
import shutil
import time
from pathlib import Path

# Isaac Sim's ROS bridge ships an internal Humble runtime. These variables
# must be set before SimulationApp loads any bridge libraries.
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
_bridge_lib = "/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"
_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
if _bridge_lib not in _ld_library_path.split(":"):
    os.environ["LD_LIBRARY_PATH"] = ":".join(
        value for value in (_ld_library_path, _bridge_lib) if value
    )

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.isaac.core.utils.extensions import (  # noqa: E402
    enable_extension,
    get_extension_path_from_name,
)


WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
USD_PATH = WORKSPACE / "isaac_sim/usd/warehouse_3d_nav_origin_carter.usd"
PROFILE_PATH = WORKSPACE / "isaac_sim/lidar_configs/Livox_Mid360_Approx.json"
# The upstream Carter USD keeps this legacy prim path. We reuse its calibrated
# mounting transform and ROS graph connections, then replace the active RTX
# sensor profile and output topic in memory before the timeline starts.
SENSOR_PRIM = "/nova_carter_ROS111/chassis_link/XT_32/PandarXT_32_10hz"
PUBLISHER_PRIM = (
    "/nova_carter_ROS111/ros_lidars/publish_front_3d_lidar_scan"
)


def update_app(count, delay=0.01):
    for _ in range(count):
        simulation_app.update()
        time.sleep(delay)


print("Enabling ROS 2 bridge and RTX sensor extensions...")
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")
update_app(200)

rtx_extension = Path(get_extension_path_from_name("isaacsim.sensors.rtx"))
installed_profile = rtx_extension / "data/lidar_configs/Livox_Mid360_Approx.json"
shutil.copyfile(PROFILE_PATH, installed_profile)
print(f"Installed Mid-360 proxy profile: {installed_profile}")

print(f"Opening USD: {USD_PATH}")
omni.usd.get_context().open_stage(str(USD_PATH))
update_app(500)

stage = omni.usd.get_context().get_stage()
if stage is None:
    simulation_app.close()
    raise RuntimeError("Warehouse USD stage failed to load")

sensor = stage.GetPrimAtPath(SENSOR_PRIM)
publisher = stage.GetPrimAtPath(PUBLISHER_PRIM)
if not sensor.IsValid() or not publisher.IsValid():
    simulation_app.close()
    raise RuntimeError("Expected Carter lidar or ROS publisher prim is missing")

sensor.GetAttribute("sensorModelConfig").Set("Livox_Mid360_Approx")
publisher.GetAttribute("inputs:topicName").Set("/livox/lidar")
# Keep the existing simulated sensor frame. It already has the correct Carter
# mounting transform; a real Mid-360 launch should use its calibrated frame.
publisher.GetAttribute("inputs:frameId").Set("front_3d_lidar")

print("Sensor profile: Livox_Mid360_Approx")
print("ROS point cloud: /livox/lidar [frame_id=front_3d_lidar]")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
update_app(300)

print("Timeline started; entering the headless Isaac Sim loop.")
while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

simulation_app.close()
