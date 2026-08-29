# Container and DDS environments

## Inspect containers

```bash
sudo docker ps --format '{{.Names}}\t{{.Status}}'
```

Use `docker` without sudo when the current user has access. Otherwise request the interactive sudo prompt; never store credentials.

## ROS 2 Humble / Nav2

Container: `ros2-dev-humble`

```bash
sudo docker exec -it ros2-dev-humble bash
```

Initialize every shell:

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
```

For automation, use noninteractive `docker exec ... bash -lc` and include the same setup.

## Isaac ROS / nvblox

Container: `isaac-ros-nvblox`; run as `admin`.

```bash
sudo docker exec -it -u admin isaac-ros-nvblox bash
```

Initialize in this order:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/setup.bash
```

## Isaac Sim

Container: `isaac-sim`.

The project launcher uses the workspace script `isaac_3d_lidar_amr_ws/isaac_sim/auto_play_warehouse.py`. It sets `headless=True`, opens `warehouse_3d_nav_origin_carter.usd`, enables the Humble ROS bridge, and starts the timeline without a native Isaac window.

Typical environment:

```bash
cd /isaac-sim
export isaac_sim_package_path=/isaac-sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0
./python.sh /workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/auto_play_warehouse.py
```
