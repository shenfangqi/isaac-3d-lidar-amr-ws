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

## Differential-control runtime

Run the stage-C controller inside the Isaac Sim container:

```bash
/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/run_carbot.sh
```

It subscribes to `/cmd_vel`, applies the common body limits, acceleration
limits, 500 ms watchdog, and curvature-preserving coupled wheel saturation,
then commands all 12 wheel joints. The current `ideal_kinematic` mode enforces
the bounded planar articulation velocity while retaining wheel/contact physics;
torque/PID actuator response is intentionally deferred to a separately
calibrated high-fidelity mode. It publishes `/odom`,
`odom -> base_footprint`, `/joint_states`, and `/clock`. Start the description
launch separately when the fixed robot TFs are needed. The RTX Mid-360 graph
is intentionally deferred to stage D.

## WebRTC and RViz demo

Run the WebRTC variant inside the Isaac Sim container:

```bash
/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/run_carbot_webrtc.sh
```

Connect `isaacsim-webrtc-client.AppImage` to `127.0.0.1`. This entry point
loads and controls Carbot inside the same streaming Kit process and disables
the warehouse's legacy Carter ROS graph, preserving single publishers for
odometry and dynamic TF. For RViz, launch `description.launch.py` with
`use_sim_time:=true`, then open `configs/rviz/carbot_phase_c.rviz`.
