"""Open the warehouse USD headlessly and start its timeline."""

import time

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": True})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402


print("Enabling ROS 2 bridge and RTX sensor extensions...")
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.sensors.rtx")

for _ in range(200):
    simulation_app.update()
    time.sleep(0.01)

USD_PATH = (
    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/usd/"
    "warehouse_3d_nav_origin_carter.usd"
)

print(f"Opening USD: {USD_PATH}")
omni.usd.get_context().open_stage(USD_PATH)

for _ in range(500):
    simulation_app.update()
    time.sleep(0.01)

stage = omni.usd.get_context().get_stage()
if stage is None:
    simulation_app.close()
    raise RuntimeError("Warehouse USD stage failed to load")

print(f"Current stage: {stage.GetRootLayer().realPath}")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

for _ in range(300):
    simulation_app.update()
    time.sleep(0.01)

print("Timeline started; entering the headless Isaac Sim loop.")
while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.01)

simulation_app.close()
