# Physical Jetson target

This is the durable project runbook for the real Jetson used by `isaac_3d_lidar_amr_ws`. Read it before any remote inspection, deployment, configuration, build, launch, or diagnosis. Live state can change; re-check it instead of treating this inventory as proof that a process is still running.

## Connection

Last verified: 2026-09-27 (Asia/Tokyo).

| Field | Value |
|---|---|
| Preferred SSH alias | `isaac-jetson` |
| Alternate SSH alias | `jetson-amr` |
| mDNS hostname | `ubuntu.local` |
| Hostname reported by Linux | `ubuntu` |
| Login user | `shenfq` |
| Current Wi-Fi IPv4 | `192.168.1.109/24` (DHCP; diagnostic only) |
| Wi-Fi interface | `wlP1p1s0` |
| SSH service | Active on TCP 22 when verified |
| Authentication | Dedicated Ed25519 key; password fallback may be available |
| Physical robot ROS domain | `0` (matches completed ESP32 firmware) |
| micro-ROS Agent | User service `micro-ros-agent.service`, CP2102 USB serial at 921600 baud |
| Wheel odometry | User service `carbot-wheel-odometry.service` |
| Phone Web teleop | User service `carbot-web-teleop.service`, TCP 8080 |

The local host configuration is in `~/.ssh/config`. The project-specific private key is `~/.ssh/id_ed25519_isaac_jetson`; its public key is installed in the Jetson user's `~/.ssh/authorized_keys`. The private-key fingerprint is:

```text
SHA256:Do75vMcxYvH62BGiBnuT64frxQYlJprtenz/RYzwefs
```

The Jetson ED25519 host-key fingerprint observed on 2026-09-06 is:

```text
SHA256:0qwKUGGlo+mzXH3UAl1bgm6whn1sCIM3kx0HZcvDBhg
```

Never store the Jetson password in this repository, a Skill, shell history, an environment file, or an SSH command. Request an interactive password prompt if key authentication or sudo requires it.

## Verified platform inventory

| Component | Verified value |
|---|---|
| Hardware model | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super |
| Architecture | `aarch64` |
| OS | Ubuntu 22.04.5 LTS (Jammy) |
| Kernel | `5.15.185-tegra` |
| JetPack | `6.2.2+b24` |
| Jetson Linux / L4T | R36.5.0 (`nvidia-l4t-core 36.5.0-20260115194252`) |
| CUDA compiler | CUDA 12.6 (`nvcc` at `/usr/local/cuda-12.6/bin/nvcc`) |
| ROS | ROS 2 Humble (`ros2` at `/opt/ros/humble/bin/ros2`) |
| Docker client | Docker 29.4.1 |
| Root filesystem | `/dev/nvme0n1p1`, 467 GB total, 32 GB used, 412 GB available when verified |

The user belongs to `sudo`, `video`, `render`, `i2c`, `gpio`, `dialout`, and
`docker`. Passwordless sudo was not available when verified, so privileged
administration normally needs an interactive `sudo` prompt. Do not attempt to
persist or automate the sudo password.

## Standard access and checks

Open an interactive shell:

```bash
ssh isaac-jetson
```

Run a noninteractive read-only check:

```bash
ssh -o BatchMode=yes isaac-jetson 'hostname; whoami; uname -m; cat /etc/nv_tegra_release'
```

Before any mutation, confirm that the alias still resolves to the intended Jetson and inspect relevant live state:

```bash
getent ahostsv4 ubuntu.local
ssh -o BatchMode=yes isaac-jetson 'hostname; whoami; uptime; ip -brief address'
```

Prefer `isaac-jetson` over a literal IP because the Wi-Fi address is assigned by DHCP. If mDNS resolution fails, discover the current address on the same LAN and diagnose mDNS; do not permanently replace the hostname with an unverified address.

For privileged interactive work:

```bash
ssh -t isaac-jetson 'sudo --prompt="[sudo] Jetson password: " COMMAND'
```

Replace `COMMAND` with the reviewed command. Never embed a password in the command line or pipe it through `sudo -S`.

## Known project locations on the Jetson

The following ROS-related directories existed under `/home/shenfq/Projects` when inventoried:

```text
/home/shenfq/Projects/micro_ros_agent_ws
/home/shenfq/Projects/carbot-ros2
/home/shenfq/Projects/lidar-mid360/ws_livox
```

The deployed Isaac ROS workspace is
`/home/shenfq/Projects/isaac_ros-dev`; its project package is
`/home/shenfq/Projects/isaac_ros-dev/src/isaac_3d_lidar_bringup`, and the
official ROS 2 FAST-LIO2 source is
`/home/shenfq/Projects/isaac_ros-dev/src/FAST_LIO`. Inspect the live source,
install overlay, container and hardware before replacing a deployment.

The tracked-base host integration is maintained in `/home/shenfq/Projects/carbot-ros2`. Its Agent and the completed ESP32 firmware use `ROS_DOMAIN_ID=0`. Isaac simulation also uses Domain 0 but remains isolated by the workstation's loopback-only DDS profile; physical operation requires explicitly loading the LAN DDS profile. The ESP32 performs the differential-track conversion and subscribes to standard `/cmd_vel`. The deployed `carbot_msgs` and `carbot_hardware` packages support `carbot-wheel-odometry.service`, which publishes raw `/wheel/odom` without TF. During mapping/navigation, the `carbot-nvblox` container's FAST-LIO base adapter is the sole `/odom` and `odom -> base_footprint` owner; the legacy `carbot-state-estimation.service` must remain disabled and inactive. Do not start the legacy `carbot_driver` alongside Nav2 because it publishes its own nonzero `/cmd_vel` stream.

The real mapping container uses image `carbot-isaac-ros-nvblox:3.3-lio` and is
managed with:

```bash
cd /home/shenfq/Projects/isaac_ros-dev
scripts/jetson_nvblox_container.sh {start|recreate|stop|status|logs} mapping
```

In real mapping mode its authoritative TF ownership is the FAST-LIO base
adapter for `odom -> base_footprint` and `odom -> fast_lio_imu`, and
robot_state_publisher for the body/joint tree. nvblox builds directly in
`odom`; FAST-LIO2 itself publishes no TF, and no SLAM Toolbox, KISS-ICP, EKF,
or `map -> odom` publisher is present.

The enabled `carbot-web-teleop.service` serves the phone controller on TCP
8080 and publishes only to `/cmd_vel_command`. It starts disarmed, requires the
calibrated command compensator to be its unique subscriber, refuses to arm if
another upstream command publisher exists, and uses a 300 ms browser-command
lease. The 2026-09-24 deployment was verified from the workstation and on the
Jetson with only zero Twist messages; no nonzero physical command was sent.
Use it only for supervised mapping, and do not enable Nav2 or another teleop
publisher at the same time.

### ESP32 USB serial transport

The live ESP32-to-Jetson micro-ROS transport uses the board's CP2102
USB-UART. The device enumerates as CP2102 `10c4:ea60`, USB serial `0001`, with
stable path
`/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`.
User `shenfq` is in `dialout`, and the device is readable/writable in the
current login session. Always use the stable `by-id` path when moving the cable
between Jetson ports; `/dev/ttyUSB0` and `by-path` are not stable deployment
inputs.

The live user service runs the Agent at `921600 8N1` and waits for the stable
device path, so reconnecting the cable does not require a Jetson restart. The
legacy UDP listener on port 8888 must remain absent. Stop the Agent before
flashing because the bootloader and micro-ROS share the CP2102 port. Service
`active` alone is not proof of a live ESP32 session: also verify fresh
`/carbot/status` and `/wheel_ticks` data before sending motion commands.

## Remote-operation rules

- Treat the Jetson as physical robot hardware, not as the validated Isaac simulation host. Do not run motion-producing commands unless the user explicitly requests them and the surrounding area is safe.
- Resolve and print `hostname`, `whoami`, and the intended target path before destructive or privileged commands.
- Use key authentication for ordinary SSH. Use interactive prompts for sudo and any fallback login; never record passwords.
- Re-check power mode, thermals, storage, device nodes, network interfaces, ROS domain and middleware, running nodes, containers, and connected sensors when they matter to the requested operation.
- Stop and report if the live platform identity or SSH host key unexpectedly differs from this record.
- Keep host-specific secrets and credentials outside Git. Store only non-secret operational facts and reproducible commands here.
- Update this file after verified changes to the Jetson's hostname, account, network route, JetPack/L4T, CUDA, ROS, Docker, workspace location, launch procedure, or hardware wiring.
