# Mid-360 simulation profile

This project contains an Isaac Sim **coverage proxy** for one Livox Mid-360.
It models the main parameters that affect geometric coverage and navigation:

- 360 degree horizontal field of view;
- -7 to +52 degree vertical field of view;
- 0.1 to 40 metre range;
- 10 Hz nominal revolution and 200,000 emitted points per second;
- one return per ray.

The ROS RTX helper publishes a completed rotary revolution (`fullScan=true`).
Do not change it to instantaneous-slice output while the proxy rotation and
ROS publication rates are both 10 Hz: that phase-locks every message to the
same azimuth and can leave the vehicle-front `+X` hemisphere completely empty.

Isaac Sim 4.5 does not have a native Livox scan model. The proxy is therefore
a 40-emitter rotary sensor. It cannot validate the real non-repetitive scan
pattern, Livox packet layout and `tag`/`line` fields, per-point timestamps,
motion distortion, packet loss, vibration, reflectivity behaviour, or the
extrinsic calibration of the physical installation.

The canonical manufacturer limits and their source URLs live under
`sensors.mid360.manufacturer_specs` in
`src/carbot_description/config/carbot_parameters.yaml`. Consistency tests bind
the RTX profile and nvblox limits to that source. The RTX error review keeps
`rangeAccuracyM=0.03 m` as a conservative official 1-sigma limit, while
`rangeResolutionM=0.01 m` and the `0.05 deg` angular standard deviations are
explicit simulation assumptions rather than claimed MID-360 specifications.

The real driver preserves the complete
`x/y/z/intensity/tag/line/timestamp` cloud on `/livox/lidar`. The separate
`/mid360/points_xyz` stream is deliberately compacted and stride-reduced for
the Jetson-to-workstation Wi-Fi path; it must not replace the raw topic for
recording, calibration, return-quality analysis or timestamp analysis.

## Run

The Phase E saved-map full-stack launcher uses Carbot, loads `warehouse_v3`,
and defaults to deterministic ground-truth localization:

```bash
cd /home/shenfq/projects/ros-humble
./start_nav_all.sh
```

For the Carbot blank-mapping component demo, start the containers first, then
run each command in a separate terminal. Do not run it alongside
`start_nav_all.sh`.

```bash
./isaac_sim/run_carbot.sh
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

The simulated point cloud is `/livox/lidar`; its frame is the Carbot
compatibility frame `front_3d_lidar`, colocated with `livox_frame`. The official
mechanical drawing places coordinate origin O `0.047 m` above the housing
bottom. With the measured housing top at `0.222 m` and the official `0.060 m`
housing height, the modeled housing bottom is at `0.162 m` and the point-cloud
origin is at `0.209 m` above ground. An installed floor-plane fit measured
approximately `0.2066 m`, providing an independent check of the model.

A stationary A-B-A chassis-reversal calibration on 2026-09-21 separated the
fixed MID-360 installation tilt from the reversing floor/track-support term.
The accepted sensor rotation is roll `-0.3515327 deg` (`-0.006135404 rad`),
pitch `-0.2783163 deg` (`-0.004857536 rad`), and yaw `+1.1206755 deg`
(`+0.019559477 rad`). Roll/pitch come from the A-B-A chassis reversal; yaw
comes from the subsequent 64-frame parallel-wall capture. The value lives
only in the canonical parameter source and is imported into both URDF and USD.
The official MID-360 manual defines identical IMU and point-cloud axes and
locates the IMU chip at `[0.011, 0.02329, -0.04412] m` in `livox_frame`.
The robot description therefore models `livox_frame -> imu_link` with that
translation and identity rotation. A 2026-09-21 dynamic wall/gyro calibration
also resolved the adapted IMU timestamp correction to `-0.009782937 s` with
approximately `0.6 ms` retained uncertainty. This timing correction is a ROS
runtime contract; it does not alter Isaac Sim physics timestamps.

For translation, the measured right-wall-to-near-track gap was `0.289 m` and
the canonical track outer half-width is `0.133 m`, placing the base centerline
`0.422 m` from the wall. Comparing that with the fitted LiDAR wall distance
`0.42185533 m` gives the accepted lateral mount offset `Y=-0.00014467 m`
(right of centerline). The ruler measurement limits this result to roughly
`+/-0.0005 m`. For X, rear-wall gaps to the right/left rear-drive-wheel outer
extrema were `1.153/1.155 m`. Combining the `1.154 m` mean with the fitted
`1.30078363 m` rear plane, canonical `-0.13225 m` wheel extremum and the wall
plane's `0.71955 deg` vertical tilt gives `X=+0.01656608 m`. The `0.002 m`
side-to-side spread is retained as the approximate X uncertainty.

For RViz inspection, run the Carbot description and open the Phase D config:

```bash
ros2 launch carbot_description description.launch.py use_sim_time:=true
rviz2 -d configs/rviz/carbot_phase_d.rviz --ros-args -p use_sim_time:=true
```

## nvblox adapter

The current nvblox CUDA conversion path consumes exactly
`lidar_width * lidar_height` entries. The launch file pads each variable-length
cloud with NaN points to 1000 x 40. This padding is safe because nvblox projects
each finite XYZ point into its spherical range image; it does not depend on the
input point order. A cloud larger than 40,000 points is rejected instead of
being silently truncated.

For the real Mid-360, measure the maximum points per ROS frame produced by the
chosen `livox_ros_driver2` transfer format before reusing these dimensions.
