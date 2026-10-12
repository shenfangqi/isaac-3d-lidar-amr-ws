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
existing robot-free warehouse and places `/World/Carbot` at the former Carter
origin.
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

The front-center green plate carries a visual D455 prototype on a two-axis
mount. `/isaac_sim/d455_gimbal_cmd` accepts a `std_msgs/Float64MultiArray`
`[yaw_rad, pitch_rad]`; positive yaw turns left and positive pitch looks up.
The runtime clamps yaw to ±90° and pitch to ±15°.
The D455 body uses the manufacturer's nominal 124 × 29 × 26 mm dimensions.
The bracket dimensions and placement are provisional until the real mount is
measured. The nested `d455_depth_camera` USD Camera prim follows both axes,
but no D455 RGB/depth ROS publisher or collision/inertia is enabled yet.
Its center starts 5 cm above the green plate with a 15° upward tilt.

The green plate also has eight upright VL53L5CX visual mounts: two near the
corners of each front, rear, left, and right edge. Each has a distinct
`sensor_origin`; the front pair angles 12.5° outward and the other six angle
22.5° toward their adjacent corners. Their breakout-board dimensions are provisional;
the VL53L5CX chip size and detection FoV come from the ST datasheet. No range
data publisher is enabled yet.
