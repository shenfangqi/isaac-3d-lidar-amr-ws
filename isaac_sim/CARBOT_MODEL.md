# Carbot model build

The reviewable robot source is `src/carbot_description/urdf/carbot.urdf.xacro`.
It loads `carbot_parameters.yaml` directly; geometry and dynamics must not be
copied into the Xacro or USD build script.

Generate the intermediate URDF in the ROS 2 Humble container:

```bash
mkdir -p /workspace/ros-humble/isaac_3d_lidar_amr_ws/.codex_tmp/carbot_build
xacro \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/src/carbot_description/urdf/carbot.urdf.xacro \
  parameters_file:=/workspace/ros-humble/isaac_3d_lidar_amr_ws/src/carbot_description/config/carbot_parameters.yaml \
  -o /workspace/ros-humble/isaac_3d_lidar_amr_ws/.codex_tmp/carbot_build/carbot.urdf
```

Then run the importer in the Isaac Sim 4.5 container:

```bash
cd /isaac-sim
./python.sh \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/scripts/build_carbot_usd.py
```

The importer creates `isaac_sim/usd/carbot.usd`, its generated
`isaac_sim/usd/configuration/carbot_*.usd` layers, and the Carter-free
`isaac_sim/usd/warehouse_3d_nav_origin_carbot.usd`. The latter sublayers the
existing robot-free warehouse and places `/Carbot` at the former Carter origin.
Control and RTX LiDAR ROS graphs are added in later implementation stages.
