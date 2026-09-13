# Mid-360 simulation profile

This project contains an Isaac Sim **coverage proxy** for one Livox Mid-360.
It models the main parameters that affect geometric coverage and navigation:

- 360 degree horizontal field of view;
- -7 to +52 degree vertical field of view;
- 0.1 to 40 metre range;
- 10 Hz nominal revolution and 200,000 emitted points per second;
- one return per ray.

Isaac Sim 4.5 does not have a native Livox scan model. The proxy is therefore
a 40-emitter rotary sensor. It cannot validate the real non-repetitive scan
pattern, Livox packet layout and `tag`/`line` fields, per-point timestamps,
motion distortion, packet loss, vibration, reflectivity behaviour, or the
extrinsic calibration of the physical installation.

## Run

The normal full-stack launcher now uses Mid-360 exclusively:

```bash
cd /home/shenfq/projects/ros-humble
./start_nav_all.sh
```

For component debugging, start the containers first, then run each command in a separate terminal.

```bash
./isaac_sim/run_mid360.sh
```

```bash
docker exec -it -u admin isaac-ros-nvblox bash -lc '
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
  source /opt/ros/humble/setup.bash
  source /workspaces/isaac_ros-dev/install/setup.bash
  source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/setup.bash
  ros2 launch isaac_3d_lidar_bringup mid360_nvblox.launch.py
'
```

```bash
docker exec -it ros2-dev-humble bash -lc '
  export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
  source /opt/ros/humble/setup.bash
  ros2 launch /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
    pointcloud_topic:=/livox/lidar localization_mode:=ground_truth
'
```

The simulated point cloud is `/livox/lidar`; its frame remains
`front_3d_lidar` because the Carter USD already publishes the correct mounting
transform for that prim. Real hardware should use the calibrated Mid-360 frame
(commonly `livox_frame`) and must not reuse Carter's transform.

## nvblox adapter

The current nvblox CUDA conversion path consumes exactly
`lidar_width * lidar_height` entries. The launch file pads each variable-length
cloud with NaN points to 1000 x 40. This padding is safe because nvblox projects
each finite XYZ point into its spherical range image; it does not depend on the
input point order. A cloud larger than 40,000 points is rejected instead of
being silently truncated.

For the real Mid-360, measure the maximum points per ROS frame produced by the
chosen `livox_ros_driver2` transfer format before reusing these dimensions.
