# Transfer artifact contract

The target host cannot be rebuilt exactly from the checked-in-looking files alone. Resolve a private project transfer source before provisioning.

## Supported acquisition modes

Use one of these, in preference order:

1. A user-supplied private archive containing the `ros-humble` tree plus three Docker/OCI image archives.
2. A user-supplied private repository URL plus separately supplied map/USD files and image archives.
3. Reviewed, non-empty Dockerfiles and a lock manifest that build images equivalent to the three custom images.

If none exists, stop. Do not guess a Git URL, regenerate `warehouse_v3`, or substitute upstream images.

## Minimum workspace layout

The target may use any absolute host path, but the directory passed as `<ros-humble-root>` must contain:

```text
ros-humble/
├── start_nav_all.sh
├── stop_nav_all.sh
├── cyclonedds_ros_local.xml
└── isaac_3d_lidar_amr_ws/
    ├── configs/
    ├── isaac_sim/
    │   ├── auto_play_warehouse.py
    │   └── usd/warehouse_3d_nav_origin_carter.usd
    ├── launch/
    ├── maps/
    │   ├── 2d/warehouse_v3.pgm
    │   ├── 2d/warehouse_v3.yaml
    │   └── nvblox/warehouse_v3.nvblx
    ├── skills/
    └── src/
```

Build and log directories are not authoritative transfer artifacts. Rebuild `build/`, `install/`, and `log/` on the target when possible.

## Known project-file fingerprints

These SHA-256 values identify the validated 2026-08-29 baseline:

```text
cae7664f29c8ced82c92efb17086cf3223646c102b7507866cb0f1e36eef47ca  maps/nvblox/warehouse_v3.nvblx
a9e1e47bfad1292c0a1536425d89057c0604611aad9c05e31663dd3bd22bbe47  maps/2d/warehouse_v3.pgm
5196d371292dd62781ef8c7433fcb4dc934a6b4d3b33e92d9198e942cbf524dd  maps/2d/warehouse_v3.yaml
4c7fed45c773862e75b347b0a2b6a33a695427c79482d3feba38ce13e81700aa  isaac_sim/usd/warehouse_3d_nav_origin_carter.usd
a1c0afd820a94bfa665d7cc2e62efc8e7a7224a9a62bbdbb63aa013d0c06a781  isaac_sim/auto_play_warehouse.py
63253a3662ccbbbeeac15c34f1f707378da1a5acb48b0538eff202c0dbeabe45  configs/nav2_params.yaml
1b3c8193ca5a41bf24cd2a9f151bfdfd81eca5da184fc1b5684094bdedf294f5  configs/amcl_params.yaml
6b3514ba7e24469ba4b676a72faa3b0c0e3a0f873eeb2d72f5c73cfc907a6e30  launch/nvblox_with_map.launch.py
b9765f9d3da5447d3e1c3037b74f964e449500996f5d50763befe74b20893e71  launch/nav_stack.launch.py
60beb9338e90f95d26bdf6f4c36fbfad29e9a0e4c45b2e82f5145538457f08a9  ../start_nav_all.sh
ff0191a323838f44f305ee9614b1c5b19e3902fa3bd47934692a9c1a231834f4  ../stop_nav_all.sh
6c21c9fb4942ff9e1c4b6930caf824c72eceef73bfce0ed2eadbff6cc5103b38  ../cyclonedds_ros_local.xml
```

The preflight script checks these. If the user intentionally supplies a newer project revision, do not overwrite it to match old hashes; obtain and verify that revision's own signed or user-approved manifest instead.

## Required image archives

After `docker load`, these tags must exist:

```text
isaac-sim-backup-before-ipc-fix:latest
isaac-ros-nvblox-backup:latest
ros2-dev-humble-backup:latest
```

On the validated source machine they occupied approximately 23.9 GB, 42.9 GB, and 4.4 GB respectively. Allow at least 120 GB free on the target for compressed archives, loaded layers, shader caches, builds, and logs.

Create archive checksums at export time and transfer the checksum file beside the archives. Verify with `sha256sum -c SHA256SUMS` before loading. A typical private export is:

```bash
docker image save isaac-sim-backup-before-ipc-fix:latest | gzip -1 > isaac-sim-backup-before-ipc-fix.tar.gz
docker image save isaac-ros-nvblox-backup:latest | gzip -1 > isaac-ros-nvblox-backup.tar.gz
docker image save ros2-dev-humble-backup:latest | gzip -1 > ros2-dev-humble-backup.tar.gz
sha256sum *.tar.gz > SHA256SUMS
```

Do not execute the export unless the user authorizes creation of these large files and supplies an explicit destination with enough free space.

## License boundary

The Isaac Sim and Isaac ROS images include NVIDIA software. Treat archives as private migration artifacts, retain applicable license/EULA material, and do not upload them to a public registry or repository.
