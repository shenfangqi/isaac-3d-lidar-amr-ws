# Ubuntu 24.04 installation and launch

This procedure targets an x86_64 desktop/workstation with a supported NVIDIA RTX GPU. The validated project uses Isaac Sim 4.5.0-era assets and ROS 2 Humble inside containers.

Official references, checked 2026-08-29:

- Docker Engine on Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- Isaac Sim 4.5 requirements: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html
- Isaac Sim 4.5 container setup: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_container.html
- Isaac ROS 3.2 system requirements: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_common/index.html

Use current official commands from those pages rather than copying stale repository bootstrap snippets. Isaac Sim 4.5 lists Linux driver `535.129.03` as its minimum/recommended baseline; newer GPUs may require a newer production driver. The validated Ubuntu 24.04 host used driver 570.169. Require an RTX-capable GPU, at least 8 GB VRAM and 32 GB system RAM; 16 GB VRAM and 64 GB RAM are preferable for this stack.

## 1. Host gate

Require:

- Ubuntu 24.04 x86_64, not WSL.
- A graphical X11 session when `START_RVIZ=1`; for headless-only operation use `START_RVIZ=0`.
- `nvidia-smi` succeeds after reboot.
- At least 120 GB free on the filesystem holding Docker data and the workspace.
- Reliable network for Ubuntu/Docker/NVIDIA packages and Isaac cloud assets.

Run:

```bash
skills/ubuntu24-isaac-amr-bootstrap/scripts/preflight.sh /absolute/path/to/ros-humble
```

It is expected to report missing Docker/images on the first pass, but hardware, OS, RAM, disk, and workspace failures must be resolved.

## 2. Install Docker Engine

Install Docker Engine from Docker's official Ubuntu `noble` repository, including `docker-ce`, `docker-ce-cli`, `containerd.io`, `docker-buildx-plugin`, and `docker-compose-plugin`. Remove conflicting distro packages only after inspecting them and confirming removal is acceptable.

Enable Docker, add the intended user to the `docker` group, then log out and back in. Verify without sudo:

```bash
docker info
docker run --rm hello-world
```

Do not continue while the project user needs an unexpected sudo path; the launch scripts select either direct Docker or a deliberate sudo fallback.

## 3. Install NVIDIA driver and container runtime

Install a production NVIDIA driver compatible with the actual GPU using NVIDIA/Ubuntu package-manager guidance, then reboot. Do not force the old 535 branch onto a newer GPU that it does not support.

Install the current stable NVIDIA Container Toolkit from its official apt repository, then configure Docker:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify GPU passthrough with a current CUDA base image appropriate for the installed driver:

```bash
docker run --rm --runtime=nvidia --gpus all nvidia/cuda:12.6.2-base-ubuntu24.04 nvidia-smi
```

## 4. Restore and verify artifacts

Read `artifact-contract.md`. Verify the user's workspace archive and image archives before extraction/loading:

```bash
sha256sum -c SHA256SUMS
docker load -i isaac-sim-backup-before-ipc-fix.tar.gz
docker load -i isaac-ros-nvblox-backup.tar.gz
docker load -i ros2-dev-humble-backup.tar.gz
```

`docker load` accepts gzip-compressed archives with current Docker Engine. Confirm all three exact tags with `docker image inspect`.

Extract the project so the supplied directory itself is `<ros-humble-root>`. Preserve executable bits on `start_nav_all.sh` and `stop_nav_all.sh`. Do not place the workspace under a root-owned directory.

Run the preflight again. Resolve every failure before creating containers.

## 5. Create the three persistent containers

Ensure no containers named `isaac-sim`, `isaac-ros-nvblox`, or `ros2-dev-humble` exist. If they do, inspect them and ask before replacement.

For an X11 desktop, allow the container root user for the current display:

```bash
xhost +si:localuser:root
```

Review the NVIDIA Omniverse/Isaac Sim license. If the user or their organization accepts it, create the containers with the explicit gate below. Do not set this variable on their behalf:

```bash
ACCEPT_NVIDIA_EULA=Y \
  skills/ubuntu24-isaac-amr-bootstrap/scripts/recreate_containers.sh \
  /absolute/path/to/ros-humble
```

The script maps the parent of `<ros-humble-root>` to `/workspace` in Isaac Sim and maps `<ros-humble-root>` to `/workspace/ros-humble` in the ROS containers. This preserves every hard-coded in-container path used by the launchers.

Privacy telemetry consent defaults to `N`. Only an authorized user may opt in by additionally setting `NVIDIA_PRIVACY_CONSENT=Y`.

## 6. Build the shared project overlay

Start only the two build containers:

```bash
docker start isaac-ros-nvblox ros2-dev-humble
```

Build the nvblox-dependent bringup package as `admin` in the Isaac ROS image:

```bash
docker exec -u admin isaac-ros-nvblox bash -lc '
set -e
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspaces/isaac_ros-dev/install/setup.bash
cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
colcon build --packages-select isaac_3d_lidar_bringup --symlink-install
'
```

Build the Nav2-side exploration package in the Humble image:

```bash
docker exec ros2-dev-humble bash -lc '
set -e
source /opt/ros/humble/setup.bash
cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
colcon build --packages-select isaac_3d_lidar_exploration --symlink-install
'
```

Saved-map navigation does not require KISS-ICP. Build it only for an explicitly requested odometry workflow and resolve its system dependencies separately.

Stop the containers after the build so the normal launcher begins from a known state:

```bash
/absolute/path/to/ros-humble/stop_nav_all.sh
```

## 7. Cold start and acceptance

From `<ros-humble-root>`:

```bash
./start_nav_all.sh
```

When the local Isaac Sim WebRTC UI is required alongside RViz, use the unified mode and connect the client to `127.0.0.1` after the launcher reports it ready:

```bash
ISAAC_WEBRTC=1 ./start_nav_all.sh
```

Allow several minutes for first-run shader and asset caches. Success requires the exact final line:

```text
[ OK ] All startup health checks passed.
```

The launcher validates the three containers, single instances of nvblox/padder/Nav2 nodes, AMCL initialization, RViz simulation time and `map` fixed frame, live 3D LiDAR and `/scan`, `warehouse_v3` dimensions `417 x 424` at `0.05 m`, the `0.5 m` nvblox LiDAR minimum, unknown-space rejection, lifecycle states, Actions, and `map -> base_link` TF.

Use RViz **2D Goal Pose**, not **Publish Point**. Send only a short goal in known free space for the first motion test.

## 8. Failure handling

On any failed startup:

```bash
./stop_nav_all.sh
```

Preserve and inspect:

```text
logs/start_nav_all/isaac_sim.log
logs/start_nav_all/nvblox.log
logs/start_nav_all/navigation.log
logs/start_nav_all/rviz.log
```

Use the sibling `ros-docker-debug` skill for DDS, topics, TF, lifecycle, Action, QoS, or duplicate-process diagnosis. Do not rerun startup over a partial stack.
