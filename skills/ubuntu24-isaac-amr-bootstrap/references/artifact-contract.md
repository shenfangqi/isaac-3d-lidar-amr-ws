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
    │   ├── auto_play_mid360.py
    │   ├── lidar_configs/Livox_Mid360_Approx.json
    │   ├── streaming_auto_play.py
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
a8db2832271f2ffc79ff090457220cfb7699a30375179328ff28c92459e5edf6  isaac_sim/auto_play_mid360.py
a3c094c89ad7fdf6aec8c98b6ee03b374ca3f006d9f1f8f05b675ce2bf801636  isaac_sim/streaming_auto_play.py
bb084c5ce44a4d2508867aabaa0f6eaca9e2be60cbb1efb6f11d1aec3ce8f485  isaac_sim/lidar_configs/Livox_Mid360_Approx.json
63253a3662ccbbbeeac15c34f1f707378da1a5acb48b0538eff202c0dbeabe45  configs/nav2_params.yaml
1b3c8193ca5a41bf24cd2a9f151bfdfd81eca5da184fc1b5684094bdedf294f5  configs/amcl_params.yaml
da22ae5f81db23aeefc0d4113a759152551c4085c4fceb5ca1ed25062e7ce5fb  launch/nvblox_with_map.launch.py
1a830cdd8d98f4cfd62e13bbe70cbbf8a230a9e562db6a0480ff774a1f3c5cc3  launch/nav_stack.launch.py
7beb163786f3c5b2a2192625529d315579fc83fe5e6caeaf809e3899452c5d85  ../start_nav_all.sh
3b2615a78959f3751efd409f7bc35522fcdea484086ace438b597ddf23f24005  ../stop_nav_all.sh
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
