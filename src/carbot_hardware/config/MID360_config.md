# Carbot MID-360 driver configuration

`MID360_config.json` is the repository-owned configuration loaded by
`mid360_stack.launch.py`. It is based on the upstream Livox ROS Driver 2
`config/MID360_config.json` structure:

- upstream source: <https://github.com/Livox-SDK/livox_ros_driver2/blob/master/config/MID360_config.json>
- MID-360 Ethernet protocol: <https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md>

Project-specific values are the recorded wired-LAN addresses: Jetson host
`192.168.2.100` and MID-360 `192.168.2.202`. Official MID-360 ports are kept.
`pcl_data_type=1` requests 32-bit Cartesian packets and `pattern_mode=0`
selects the non-repetitive pattern. The ROS launch uses `xfer_format=0`, so the
authoritative `/livox/lidar` `PointCloud2` retains
`x/y/z/intensity/tag/line/timestamp`.

The JSON extrinsic values intentionally remain zero: the ROS TF tree owns the
`base_link` to `livox_frame` transform, avoiding double application.

## Online acceptance

On 2026-09-21 this file was deployed to the Jetson and loaded from the installed
`carbot_hardware` package. Source, install and workstation SHA-256 checksums
matched. Live inspection verified `livox_frame`, all seven raw fields, about
10 Hz raw and compact point clouds, about 200 Hz adapted IMU, and approximately
20k points per raw frame. Per-point timestamps use epoch nanoseconds; the
sampled first point matched the ROS header within approximately 0.24 us and the
frame spanned approximately 100.01 ms.

The system `ptp4l` master was running on `enP8p1s0`, but its management socket
is restricted to root. Exact PTP offset remains a privileged follow-up; the
coherent live timestamps are not a substitute for recording that offset.
