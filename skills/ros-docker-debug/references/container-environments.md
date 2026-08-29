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

There are three supported routes. Inspect the container and Isaac processes before choosing one; never run more than one Isaac route at a time.

### Automated navigation modes

The default launcher uses `auto_play_warehouse.py`, opens the warehouse, starts the timeline, and runs the full saved-map navigation stack without a WebRTC stream:

```bash
cd /home/shenfq/projects/ros-humble
./start_nav_all.sh
```

The unified WebRTC mode uses `runheadless.sh` plus `streaming_auto_play.py` inside the same Isaac process, then starts nvblox, Nav2, AMCL, and RViz as usual:

```bash
cd /home/shenfq/projects/ros-humble
ISAAC_WEBRTC=1 ./start_nav_all.sh
/home/shenfq/桌面/myApps/isaacsim-webrtc-client.AppImage
```

Connect the client to `127.0.0.1`. Do not launch the client until the launcher reports `Isaac WebRTC : ready-at-127.0.0.1`. Success still requires `[ OK ] All startup health checks passed.` Use `ISAAC_WEBRTC=1 ./start_nav_all.sh --health-check` when rechecking this mode so the WebRTC gates are included.

Both automated modes load the same `warehouse_3d_nav_origin_carter.usd` and Play it automatically. Stop either mode with `./stop_nav_all.sh`.

### Direct non-streaming Isaac process

This low-overhead component command is the implementation used by the default automated mode; do not run it alongside either complete launcher mode.

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

### Standalone interactive WebRTC UI mode

This path was validated locally on 2026-08-29 with Isaac Sim 4.5 and `/home/shenfq/桌面/myApps/isaacsim-webrtc-client.AppImage`.

Use this only when editing the USD without nvblox, Nav2, or RViz. First stop the saved-map stack and reject any surviving Isaac process. On the host:

```bash
cd /home/shenfq/projects/ros-humble
./stop_nav_all.sh
xhost +local:root
docker start isaac-sim
```

Start the streaming experience in the container (an attached shell is fine; a noninteractive `docker exec` is easier to reproduce):

```bash
docker exec -it isaac-sim bash -lc '
export isaac_sim_package_path=/isaac-sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${isaac_sim_package_path}/exts/isaacsim.ros2.bridge/humble/lib"
export ROS_DOMAIN_ID=0
cd /isaac-sim
exec ./runheadless.sh
'
```

Do not continue until the server log contains all of:

```text
omni.kit.livestream.webrtc
Streaming server started.
Isaac Sim Full Streaming App is loaded.
```

On the same host, launch the client and connect to `127.0.0.1`:

```bash
/home/shenfq/桌面/myApps/isaacsim-webrtc-client.AppImage
```

In the streamed Isaac UI, open:

```text
/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/usd/warehouse_3d_nav_origin_carter.usd
```

Wait for `/nova_carter_ROS111` to appear in the Stage tree, then press Play. The toolbar control changing from Play to Pause is the visual confirmation that the timeline is running.

`runheadless.sh` already runs an Isaac Sim Kit application. Never execute `python.sh auto_play_warehouse.py` while it is active: that starts a second Isaac Sim instance rather than controlling the streamed one. Closing the AppImage also does not stop the server; interrupt the `runheadless.sh` shell and stop the `isaac-sim` container when finished.
