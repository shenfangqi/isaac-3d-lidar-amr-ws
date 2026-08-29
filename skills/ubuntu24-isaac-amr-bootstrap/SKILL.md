---
name: ubuntu24-isaac-amr-bootstrap
description: Provision or migrate this Isaac Sim 4.5, Isaac ROS nvblox, ROS 2 Humble AMR workspace onto a fresh x86_64 Ubuntu 24.04 NVIDIA workstation, restore its exact container images and warehouse_v3 artifacts, rebuild the project overlay, and launch validated saved-map navigation. Use for new-machine setup or disaster recovery; do not use for ordinary startup on an already provisioned host.
---

# Ubuntu 24.04 Isaac AMR Bootstrap

Reproduce the validated project, not a generic ROS installation. Ubuntu 24.04 is the host; ROS 2 Humble and the Isaac dependencies remain inside their existing Ubuntu 22.04-era containers.

## Required route

1. Read [references/artifact-contract.md](references/artifact-contract.md) before changing the target. Resolve how the user supplies the workspace and three image archives. Never invent a repository URL or silently replace a custom image with an upstream image.
2. Read [references/install-and-launch.md](references/install-and-launch.md) for host provisioning, image restore, overlay build, container creation, launch, and acceptance gates.
3. Run `scripts/preflight.sh <ros-humble-root>` before installation and again after artifacts are restored. Treat every failure as a blocker; warnings require an explicit risk decision.
4. Use `ACCEPT_NVIDIA_EULA=Y scripts/recreate_containers.sh <ros-humble-root>` only after the user has reviewed and accepted the NVIDIA terms and the required images are loaded. It refuses to overwrite containers.
5. Build the project overlay using the commands in the install reference, then start with `<ros-humble-root>/start_nav_all.sh`.
6. Require the literal final result `[ OK ] All startup health checks passed.` Do not claim success merely because RViz opened.

## Reproducibility boundary

The current repository has empty `docker/docker-compose.yml`, `docker/Dockerfile.humble`, `docker/Dockerfile.jetson`, and `scripts/build.sh`. The working runtime therefore depends on three custom image artifacts:

- `isaac-sim-backup-before-ipc-fix:latest`
- `isaac-ros-nvblox-backup:latest`
- `ros2-dev-humble-backup:latest`

Without those archives, or new reviewed Dockerfiles that reproduce them, stop and report that exact reconstruction is blocked. Pulling `nvcr.io/nvidia/isaac-sim:4.5.0` alone does not reconstruct the ROS bridge and project-specific changes in the saved image.

## Safety and state rules

- Inspect existing Docker objects before mutation. Never remove, rename, or replace an existing container or image without explicit authorization.
- Never set `ACCEPT_NVIDIA_EULA=Y` on the user's behalf. The person or organization using Isaac Sim must make that decision.
- Verify transfer checksums before `docker load` or extracting the workspace.
- Do not publish NVIDIA-derived image archives or project assets. Keep transfers private and subject to their licenses.
- Do not weaken `warehouse_v3` map thresholds, Nav2 footprint, inflation, unknown-space rejection, LiDAR padding, or `0.5 m` minimum range to make acceptance pass.
- On failure, preserve `logs/start_nav_all/`, stop with `stop_nav_all.sh`, and diagnose through the sibling `ros-docker-debug` skill.

After provisioning, use the sibling `isaac-amr-project-state` skill for normal startup/shutdown and validated navigation behavior.
