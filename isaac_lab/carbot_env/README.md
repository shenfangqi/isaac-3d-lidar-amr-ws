# Carbot Isaac Lab Phase F environment

This is the initial manager-based goal-navigation environment for Carbot. Its
policy action is always physical-unit `[linear.x, angular.z]`; the action term
applies simulation Nav2 velocity limits, canonical acceleration limits, coupled
wheel saturation, the imported joint sign, and the canonical 500 ms watchdog.
It commands all twelve wheel joints and, while the canonical simulation mode is
`ideal_kinematic_tracked_differential`, enforces the same bounded planar body
velocity as the Isaac Sim runtime. PWM and motor torque are not policy outputs.

The policy observation group contains a relative goal, odometry-compatible
planar velocity, projected gravity, MID-360 geometry ranges (8 elevations × 72
azimuths), eight VL53L5CX arrays (8 × 8 zones each), a rendered D455 depth image
(32 × 24), validity masks for all three sensors, and the previous action. The
flat policy tensor has 3723 entries. The old simulator-only 5 × 5 height scan
has been removed; existing policies using that layout must be retrained.

Sensor values are metres, followed by separate 0/1 validity terms in the order
specified by `environment.yaml`. Invalid values (including too-close returns)
are encoded as zero with validity zero, never as clear free space. MID-360
ranges are elevation-major/azimuth-minor from -180° (exclusive +180°); the
ToF arrays follow `vl53l5cx_placements` order, then image row top-to-bottom and
column left-to-right. D455 uses optical-axis depth, not radial distance.
Optical origins and orientations come from the canonical robot parameters.
Update periods are 100 ms (LiDAR), 66.7 ms (ToF), and 33.3 ms (camera); actual
updates are quantized by simulation/render steps, and observations hold the
last measurement between updates.

The D455 camera inherits the existing mount (yaw 0°, pitch +15°). Mount motion
is not a policy action. Its nominal 87° × 58° field of view follows the
[manufacturer specification](https://www.realsenseai.com/compare-depth-cameras/).
The 0.6–6 m software window is a conservative prototype choice, not a hardware
minimum-distance calibration. The synthetic image is a pinhole depth render;
stereo matching, material failures, noise, exposure, and latency are not yet
calibrated. Optical placement at the body front centre is provisional.

MID-360 and ToF use ideal geometric ray casting against static scene geometry.
`StaticSceneRayCaster` triangulates ground and static Mesh/Cube geometry under
each environment, isolating each clone's rays from other clones. Robot geometry
is excluded; self-occlusion must be measured before deployment. Dynamic rigid
obstacles are explicitly rejected, and static geometry must not be moved after
sensor initialization. This backend is for initial static-obstacle training;
it does not reproduce Livox's scan sequence or ToF zone integration/optics.
D455 rendering sees all rendered geometry, so keep clones sufficiently far apart
or enclose each environment to avoid seeing adjacent environments.

These sensors feed Isaac Lab tensors directly. The separate Isaac Sim WebRTC/ROS
launcher retains its MID-360 `/livox/lidar` publisher; this change does not add
D455/ToF ROS publishers or hardware drivers. Deployment must construct the same
observation order, validity masks, units and extrinsics from actual sensor data.

## Validation

Run CPU-only contract tests from the repository root:

```bash
python3 -m pytest -q isaac_lab/carbot_env/tests
```

Run a one-environment Kit smoke test in the existing Isaac Sim container:

```bash
docker start isaac-sim
docker exec isaac-sim bash -lc '
  cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
  /workspace/IsaacLab/isaaclab.sh -p \
    isaac_lab/carbot_env/scripts/smoke_env.py --headless --steps 20
'
docker stop isaac-sim
```

## WebRTC interactive launch

Use the following persistent launch sequence when inspecting the Phase F scene
with the desktop Isaac Sim WebRTC Streaming Client. Do not run another Isaac Sim
or Isaac Lab process in the same container at the same time.

```bash
docker start isaac-sim
docker exec -it \
  -e TERM=xterm \
  -e PUBLIC_IP=127.0.0.1 \
  isaac-sim bash -lc '
    cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
    exec /workspace/IsaacLab/isaaclab.sh -p \
      isaac_lab/carbot_env/scripts/smoke_env.py \
      --livestream 1 --steps 100000000
  '
```

Wait for both `Streaming server started` and
`Completed setting up the environment`. Then double-click
`~/桌面/myApps/isaacsim-webrtc-client.AppImage`, enter `127.0.0.1`, and select
**Connect**. The AppImage may be started normally from the desktop; when it is
started by automation, it must be kept in a persistent terminal/session because
a short-lived background command can terminate the Electron client when that
command exits.

The Simulation Output `USD`/Fabric selection affects how simulation state is
synchronized to the viewport; it does not establish the WebRTC connection. For
a clean shutdown, close the client, press Ctrl+C in the persistent server
terminal, and then run `docker stop isaac-sim`.

Both sensor scripts enable cameras automatically. To verify actual ranges and
depth against known walls in two cloned environments, run:

```bash
docker exec -e TERM=xterm -e PYTHONUNBUFFERED=1 isaac-sim bash -lc '
  cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
  /workspace/IsaacLab/isaaclab.sh -p \
    isaac_lab/carbot_env/scripts/validate_sensors.py --headless
'
```

Validated on 2026-10-13 with Isaac Sim 4.5 / Isaac Lab 2.2: 13 related unit
and contract tests passed; the two-environment wall test passed for all three
sensors and after a pose change; the default smoke test returned `(1, 3723)`.
The sub-millimetre errors in this ideal geometry fixture describe numerical
agreement only, not real sensor accuracy.

Require the `CARBOT_SENSOR_VALIDATION` result, not just an exit code. The test
compares GPU measurements with analytic ray/box intersections and checks readings
after translation/rotation and reset. Physics-only calibration scripts disable
all three sensors and do not require camera rendering.

The smoke scene is flat and is intended to validate loading, interfaces, tensor
dimensions, reset, and zero-action safety. The ideal planar execution mode is a
temporary baseline, not evidence of calibrated tracked contact dynamics or
successful policy training.

## Sim-to-Real gate

[`hardware_calibration_backlog.yaml`](hardware_calibration_backlog.yaml) is a
persistent release gate. Effective radius `0.02175 m` and effective track
separation `0.254 m` are confirmed baselines; their real-vehicle trials are
acceptance checks rather than missing-calibration blockers. Jetson wheel-tick
decoding and odometry ownership, slip, latency, deadband, braking, inertial
parameters, friction, IMU/Mid-360 extrinsics, physical watchdog/e-stop, and
real observation parity remain blocking. Simulation-only results must never
close blocking items. Measured values must flow back to the canonical Carbot
parameter source before domain-randomization ranges or a policy are frozen.

## Interactive sensor dashboard (WebRTC)

From the workstation, run:

```bash
cd /home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws
./isaac_sim/run_sensor_check_webrtc.sh
```

Wait for `CARBOT_SENSOR_CHECK_READY`, then open the Isaac Sim WebRTC client and
connect to `127.0.0.1`. Keep the terminal open. The launcher checks for an existing
Isaac process and refuses to start a second one. This inspection scene uses
Isaac Lab directly and publishes no ROS motion commands.

The **Carbot | Sensor Check** panel shows D455 depth, eight named ToF 8×8 grids,
and MID-360 range statistics. Orange means near, cyan means far; dark cells are
invalid, not confirmed free space. D455 and ToF use different fixed colour scales,
shown under the images. ToF names F/R mean front/rear; L/R on the front/rear
modules mean left/right. Click a ToF name to display that module's valid rays.
Cyan 3D points are MID-360 hits, orange points are ToF hits, and the green arrow
indicates the vehicle's +X/front direction. These markers are viewport overlays.

Use **Forward/Back 10 cm**, **Turn left/right 15 deg**, and **Reset pose** to
compare measurements. These are direct pose changes for inspection, not driving
or collision avoidance. Four fixed reference walls, a low block and a narrow
post provide targets. Do not drag the obstacles: the ray-casting mesh is static.
The forward wall's near face starts at x=0.95 m; range is measured from each
sensor's optical origin, so it differs from the vehicle-centre distance.

The dashboard updates at roughly 10 Hz of simulation time; each sensor retains
its own measurement cadence. **Overview** restores the initial viewport.

To close only the sensor inspection app, run in another terminal:

```bash
./isaac_sim/stop_sensor_check.sh
```

The container remains available. Closing the WebRTC client alone does not stop
the simulation server.
