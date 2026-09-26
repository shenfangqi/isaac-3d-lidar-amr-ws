# 2026-09-22 alignment closure

> Superseded later on 2026-09-22 by the controlled driven acceptance in
> `../2026-09-22_ekf_dynamic/README.md`. `robot_localization` is now installed,
> the ESP time-resynchronization firmware is flashed and live-verified, and the
> MID-360/wheel EKF is enabled on the Jetson. The sections below preserve the
> earlier audit state that defined those release gates.

## IMU policy

The estimator contract uses `/mid360/imu/data_raw` as its only IMU input.
ESP32 remains responsible for `/wheel_ticks` and base control only. Its legacy
`/imu/data_raw` publisher may still be visible, but it is not an estimator input;
the 2026-09-22 live graph reported zero subscribers on that topic.

The proposed `robot_localization` contract is
`src/carbot_hardware/config/mid360_wheel_ekf.yaml`. It consumes MID-360 yaw rate
and wheel-odometry planar velocity. It is deliberately not launched on the
Jetson: `robot_localization` is not installed there, and no existing bag contains
MID-360 IMU, wheel motion and `/cmd_vel` together for dynamic A/B acceptance.

`mid360_wheel_fusion_readiness.json` is the automated gate result for the best
available manual-yaw bag. It correctly rejects that bag: wheel tick deltas are
zero, `/cmd_vel` is absent, and its adapted IMU frame predates the later
`imu_link` correction. The current live output was separately verified as
`imu_link` at approximately 200 Hz. Live fusion must remain disabled until one
controlled driven bag passes the gate and wheel-only versus fused localization
is compared against external pose truth.

The inert configuration was copied to the Jetson source and installed package;
both copies have SHA-256
`9764b828b1188f2c5734aff8e2b2617577371129f5e4add2c85b239bf3fb0a46`.
No EKF process or service was started.

## Real versus Isaac motion response

`isaac_response_profile.json` and `real_isaac_response_comparison.json` are
generated artifacts. With matched low-speed commands, mean absolute relative
error is 12.26% for linear travel and 16.74% for yaw. Isaac stop tails are
0.02-0.16 s, versus measured real ground stop tails of 0.57-0.92 s. The current
model is therefore suitable for interface/navigation-contract work, not a
high-fidelity dynamics twin.

Regenerate the simulation profile with
`isaac_lab/carbot_env/scripts/export_response_profile.py`, then compare it with
`scripts/compare_real_isaac_response.py`.

## ESP32 wheel-clock synchronization

The firmware worktree `/home/shenfq/Code/carbot` contains a periodic micro-ROS
epoch resynchronization change on branch `codex/periodic-ros-time-resync`.
Connected sessions resample every 60 s, retain the last valid offset after a
failed sample, and limit each successful correction to 2 ms. Host tests and a
full ESP-IDF 5.4.4 build pass at commit `ce613ed`; the generated binary is
preserved under `.codex_tmp/esp32_periodic_resync_build/build/`.
The change cannot be flashed from either workstation or Jetson until the ESP32
is attached through a data-capable USB cable; no serial device or OTA endpoint
is currently available. Until it is flashed and passes a 30-minute timestamp
drift capture, wheel/IMU fusion remains a release gate.

Final live safety check: `/cmd_vel` had zero publishers and one base subscriber;
wheel ticks stayed exactly `11747/2693` across a two-second observation. The
ESP status reported a connected, time-synchronized session with zero reconnects.
`active_command_source=1` denotes the web source, but the unchanged ticks prove
that its current command was neutral. Workstation ROS and Isaac containers were
stopped after validation.
