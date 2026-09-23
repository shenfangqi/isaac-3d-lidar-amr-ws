# Powered static durability acceptance (2026-09-23)

This acceptance was performed with the Jetson, ESP32 and MID-360 powered while
the vehicle remained stationary. No nonzero velocity command was sent. The
source bag remains on the Jetson at:

```text
/home/shenfq/Projects/carbot-ros2/calibration_data/2026-09-23_static_durability/static_30min_01
```

The SQLite bag is 11.0 GiB and is intentionally not copied into Git. It spans
1965.579 s and contains 1,061,689 messages across the raw/compact point clouds,
raw/adapted IMU, wheel ticks, wheel odometry, fused odometry, status,
diagnostics and TF topics.

## ESP32 time and communication

- 31 minute-boundary samples retained one boot ID (`4005221903`).
- `agent_connected` and `time_synchronized` remained true.
- `reconnect_count`, `consecutive_ping_failures` and `invalid_cmd_count`
  remained zero.
- Both tick counters remained exactly zero for the complete stationary run.
- All `/wheel_ticks` headers were strictly monotonic; non-monotonic count was
  zero. The observed bag rate was 49.951 Hz and the maximum header gap was
  119.976 ms.

## MID-360 and EKF

- `/mid360/imu/data_raw`: 199.906 Hz, zero non-monotonic headers.
- `/livox/lidar`: 9.969 Hz, zero non-monotonic headers.
- `/mid360/points_xyz`: 9.887 Hz, zero non-monotonic headers.
- Online post-bag checks measured both raw/adapted IMU streams at about
  200.2 Hz and both point-cloud streams at about 10.0 Hz.
- The bag contains only 116.4 Hz of `/livox/imu` and isolated larger receive
  gaps while simultaneously writing the full raw point cloud. Because the
  live publisher rates are nominal, these are recorder I/O losses/backlog, not
  a MID-360 driver outage. Future long-duration timing bags should omit the
  full raw point-cloud payload or use a higher-throughput storage profile.
- Wheel odometry remained exactly at zero pose and zero twist.
- Fused `/odom` position remained exactly fixed. MID-360 gyro integration
  produced a net yaw drift of 0.905 deg and a maximum excursion of 1.180 deg
  over 32.76 minutes (about 0.036 deg/min maximum-excursion rate).

## Jetson durability and PTP

- All six user services remained `active/running` at all 31 samples with
  `NRestarts=0`: micro-ROS agent, description, wheel odometry, MID-360,
  state estimation and command compensation.
- Available memory stayed between 6247 and 6359 MiB; one-minute load average
  stayed between 1.21 and 4.07.
- `/cmd_vel` remained silent and the command source remained zero.
- The system `ptp4l` process remained active on `enP8p1s0`. After the boot-time
  link transition it entered `MASTER`; the Ethernet link remained
  `UP/LOWER_UP`. The Jetson is configured as the isolated-link master and the
  MID-360 as slave.
- Exact `pmc` slave offset and mean path delay remain unrecorded because the
  management socket is `root:root 0660` and the non-interactive SSH session
  cannot satisfy the Jetson sudo password prompt. No permissions or PTP
  configuration were changed to bypass this boundary.

Conclusion: ESP communication/time-header stability and Jetson service/ROS
topology durability passed. MID-360 live publication is healthy. The two
remaining observations are the bounded static EKF yaw drift and the privileged
exact PTP offset query.
