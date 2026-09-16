# Carbot Isaac Lab Phase F environment

This is the initial manager-based goal-navigation environment for Carbot. Its
policy action is always physical-unit `[linear.x, angular.z]`; the action term
applies simulation Nav2 velocity limits, canonical acceleration limits, coupled
wheel saturation, the imported joint sign, and the canonical 500 ms watchdog.
It commands all twelve wheel joints and, while the canonical simulation mode is
`ideal_kinematic_tracked_differential`, enforces the same bounded planar body
velocity as the Isaac Sim runtime. PWM and motor torque are not policy outputs.

The policy observation group contains a relative goal, odometry-compatible
planar velocity, projected gravity, 72 horizontal ray ranges, a 5 x 5 height
scan, and the previous action. It deliberately excludes Isaac ground-truth pose
and other simulator-only state.

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

The smoke scene is flat and is intended to validate loading, interfaces, tensor
dimensions, reset, and zero-action safety. The ideal planar execution mode is a
temporary baseline, not evidence of calibrated tracked contact dynamics or
successful policy training.

## Sim-to-Real gate

[`hardware_calibration_backlog.yaml`](hardware_calibration_backlog.yaml) is a
persistent release gate. Jetson wheel-tick decoding and odometry ownership,
effective radius/track separation, slip, latency, deadband, braking, inertial
parameters, friction, IMU/Mid-360 extrinsics, physical watchdog/e-stop, and
real observation parity are still pending. Simulation-only results must never
close those items. Measured values must flow back to the canonical Carbot
parameter source before domain-randomization ranges or a policy are frozen.
