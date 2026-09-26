# Current authoritative project state

Session handoff and next-step procedure:
[`docs/carbot_2026-09-21_handoff.md`](../../../docs/carbot_2026-09-21_handoff.md).

Newest checkpoint: [real MID-360 FAST-LIO2 mapping: 2026-09-26](#real-mid-360-fast-lio2-mapping-2026-09-26).

## Real MID-360 FAST-LIO2 mapping: 2026-09-26

- Real mapping now runs the official FAST-LIO2 ROS 2 implementation, vendored
  from `hku-mars/FAST_LIO` ROS2 commit
  `a4743b095409588842a5b30ddfa27e29d2f99164` with ikd-Tree commit
  `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`. The Jetson image installs the
  required ROS PCL packages and the runtime mounts/sources the Livox workspace.
- The MID-360 driver publishes `livox_ros_driver2/msg/CustomMsg` directly on
  `/livox/lidar` at about 10 Hz. A live frame contained 19,968 points whose
  `offset_time` span was 99.830 ms, preserving the per-point timing needed for
  tightly coupled IMU deskew. FAST-LIO2 also consumes the corrected SI-unit
  `/mid360/imu/data_raw`; its configured LiDAR-to-IMU translation is
  `[-0.011, -0.02329, 0.04412] m`, time offset is zero because the adapter
  already applies the measured `-9.782937 ms`, and online extrinsic estimation
  is disabled.
- FAST-LIO2 publishes raw full 3D odometry on `/fast_lio/imu_odom` and
  deskewed clouds under `/fast_lio/*`, but publishes no TF. The
  `fast_lio_base_adapter` is the single `/odom` publisher and owns only
  `odom -> base_footprint` plus the auxiliary full-attitude
  `odom -> fast_lio_imu`; robot_state_publisher owns the body/joint edges.
  The legacy host `carbot-state-estimation.service` is disabled and inactive,
  and no KISS-ICP, robot_localization, or SLAM Toolbox node runs in mapping.
- The authoritative real mapping chain is FAST-LIO2 pose plus nvblox:
  nvblox integrates a fixed `1000 x 40` NaN-padded body cloud directly in
  `odom`, publishes its `0.05 m` 2.5D occupancy grid, and supplies the 3D
  voxel display. RViz mapping uses Fixed Frame `odom`; no second scan matcher
  or `map -> odom` correction is present. The low-bandwidth accumulated-cloud
  voxel marker is published in `odom` at up to 0.5 Hz and 6,000 points.
- Static live acceptance found one raw and one adapted odometry publisher at
  about 10 Hz, body cloud and `/scan` near 9-10 Hz, and no continuing pointcloud
  queue drops after increasing transform-filter headroom. One stationary
  20-second sample moved about `0.2 mm` in X, `3.25 mm` in Y and `0.066 deg`
  in yaw. A stationary nvblox grid was `200 x 144` at `0.05 m`, with 1,716
  occupied, 6,384 free and 20,700 unknown cells. Container load was about two
  CPU cores and 602 MiB.
- The mapping container uses `carbot-isaac-ros-nvblox:3.3-lio`. Static
  acceptance did not move the vehicle and did not save this disposable map;
  Web remained `armed=false`. A supervised low-speed motion test is still
  required to validate direction, collision/slip recovery and map consistency
  under motion before this estimator is accepted for map production.

## Physical mass and center-of-mass baseline: 2026-09-23

- The operator reported the assembled vehicle mass as `3.4 kg` and its center
  of mass at the vehicle geometric center. The canonical model now uses
  `3.28 kg` for the merged base body plus twelve `0.01 kg` wheel links.
- The geometric center remains `[0.0125, 0.0, 0.035] m` in `base_link`
  coordinates; the `+0.0125 m` X value reflects the asymmetric footprint and
  is not a forward COM offset relative to the vehicle geometry.
- Inertias were scaled with base-body mass only to preserve the previous
  provisional radii of gyration. They remain estimates, so the combined
  mass/COM/inertia release item stays blocked pending CAD or pendulum evidence.

## Offline evidence-constrained twin: 2026-09-23

- `ideal_navigation` remains the deterministic navigation baseline. A separate
  `evidence_degraded` response mode now consumes the generated, versioned
  `configs/carbot/evidence_degraded.yaml`; it models measured command latency,
  directional finite-duration gains, braking tail, encoder quantization and
  sparse faults, MID-360 gyro residuals, and point-cloud delay/noise/dropout.
- `CARBOT_SIM_ESTIMATOR_MODE=evidence_ekf` publishes truth only on
  `/ground_truth/odom`, publishes the real `/wheel_ticks`, `/carbot/status` and
  `/mid360/imu/data_raw` contracts, and reserves `/odom` plus odom TF for the
  same wheel-odometry and robot_localization configuration used on Jetson.
- The deterministic offline response regression passes for the 20 ms runtime:
  first observable response `0.10 s`, forward/reverse ten-second distances
  `0.4308/-0.4274 m`, and stop tails `0.84 s`. These are evidence-model checks,
  not new physical measurements or proof of a contact-dynamics twin.
- An isolated full-process forward/turn test produced 500 truth and 404 EKF
  samples; final EKF errors were `3.96 mm` and `0.044 deg`. The default
  `ideal_navigation` stack then passed all startup health gates plus 0.4 m
  straight and turning goals with `SUCCEEDED`, zero recoveries and zero final
  velocity. Evidence is in `calibration_data/2026-09-23_offline_twin/`.
- Domain-randomization timing, actuator, encoder and IMU ranges now include the
  measured wood-floor zero-payload evidence while mass, COM, inertia, friction,
  other surfaces and payloads remain uncalibrated release blockers.
- This work was performed with the vehicle powered off. No Jetson, MID-360 or
  ESP32 runtime was contacted, and no ESP32 source was changed.

## Right-turn command compensation acceptance: 2026-09-23

- The canonical right-turn command scale is `0.896`. The Jetson formal path is
  `/cmd_vel_command -> cmd_vel_compensator -> /cmd_vel -> ESP32`; the enabled
  `carbot-command-compensation.service` is the sole normal `/cmd_vel` publisher
  and remains silent until an upstream command arrives. Nav2's real-robot
  velocity smoother defaults to `/cmd_vel_command`. No ESP32 source or firmware
  was changed.
- Isaac Sim uses the same scale plus the measured uncompensated right-turn
  response gain `1/0.896`. A final isolated sequence returned within `16.0 mm`,
  with `+51.5662/-51.5662 deg` turns and zero yaw residual.
- The final supervised real sequence measured `0.427 mm` forward/reverse return
  error, `0.874 mm` full-sequence position error and `0.095 deg` final yaw error.
  The individual turns were `+30.0185/-28.6048 deg` (pair residual `1.4137 deg`),
  and the observed actuator-side right command was exactly `-0.2688 rad/s` for
  a requested `-0.3000 rad/s`.
- ESP boot ID stayed `268099638`; final ticks `8/14` remained unchanged during
  the final audit. All micro-ROS, wheel, MID-360, EKF and compensation services
  were active, and `/cmd_vel_command` returned to zero publishers. Evidence is
  under `calibration_data/2026-09-23_cmd_comp_final/`.
- Jetson still has `Linger=no`. A fresh SSH login can restart user services and
  the ESP micro-ROS session may need roughly 25--60 seconds to reconnect. Motion
  gates must wait for fresh ticks and the live ESP `/cmd_vel` subscription.

## MID-360/wheel online EKF acceptance: 2026-09-22

- Jetson now has `robot_localization` installed and enabled through
  `carbot-state-estimation.service`. Raw encoder odometry is `/wheel/odom` with
  no TF; the EKF is the sole `/odom` and dynamic `odom -> base_footprint`
  publisher. The service is enabled and all micro-ROS, wheel, MID-360 and EKF
  services were active at final acceptance.
- The only estimator IMU remains `/mid360/imu/data_raw`; ESP32 `/imu/data_raw`
  is excluded. The MID-360 adapter estimates temperature-dependent gyro-z bias
  only while wheel odometry proves the chassis stationary. It publishes an
  explicit conservative gyro-z covariance of `4e-6 (rad/s)^2` after 400
  stationary samples.
- The final EKF fuses wheel integrated yaw as a low-frequency anchor, wheel X/Y
  velocity including the non-holonomic lateral-zero constraint, and MID-360
  yaw rate as the high-frequency response. Dynamic process noise is enabled.
- A `53.735 s` controlled bag contains `/cmd_vel`, real wheel motion, MID-360
  IMU, compact point clouds, TF and one uninterrupted ESP boot. The readiness
  gate passed. In the exact common replay window, point-cloud ICP measured
  `0.18938698 rad`, wheel yaw measured `0.19008596 rad`, and EKF yaw measured
  `0.18940368 rad`; EKF error was about `0.001 deg`.
- Final 20 s online static acceptance measured zero X/Y drift, about
  `0.00176 rad` yaw range, ending yaw covariance `5.1e-6 rad^2`, zero
  `/cmd_vel` publishers, and unchanged wheel ticks. Evidence is under
  `calibration_data/2026-09-22_ekf_dynamic/`.

## MID-360-only fusion and dynamics audit: 2026-09-22

This section is a historical pre-acceptance checkpoint. Its statements that
robot_localization was absent, EKF was disabled, or firmware flashing was
pending are superseded by the newer EKF acceptance, operator-confirmed flash,
and offline-twin checkpoints above.

- The only permitted estimator IMU is `/mid360/imu/data_raw`. ESP32 remains the
  wheel-tick and base-control source; its legacy `/imu/data_raw` publisher had
  zero subscribers in the live graph and is excluded from the EKF contract.
- Live MID-360 output reported frame `imu_link`. The best existing combined bag
  is not a valid driven fusion A/B: `/cmd_vel` is absent, wheel deltas are zero,
  and the bag predates the `imu_link` frame correction. The automated readiness
  gate rejects it, so live fusion remains disabled pending one controlled driven
  MID-360/wheel bag and external-pose comparison. Jetson also does not currently
  have `robot_localization` installed.
- Matched-command Isaac profiling found 12.26% mean absolute linear-response
  error and 16.74% yaw-response error relative to surveyed real trials. Isaac
  stop tails were 0.02-0.16 s versus 0.57-0.92 s on the real floor. The model is
  a navigation/interface contract, not yet a dynamics twin.
- ESP32 firmware source now implements 60 s periodic epoch resampling, retains
  the last valid offset after a failed sample, and limits offset changes to 2 ms.
  Host unit tests and a full ESP-IDF 5.4.4 build pass at firmware commit
  `ce613ed`. Flashing and the required 30-minute drift validation are pending
  because neither workstation nor Jetson exposes an ESP32 USB serial device and
  the firmware has no OTA endpoint.
- Evidence and reusable scripts are under
  `calibration_data/2026-09-22_alignment/`. The inert EKF configuration was
  deployed to Jetson source/install with matching SHA-256, but no EKF process
  was started. Final `/cmd_vel` publisher count was zero and wheel ticks were
  unchanged over two seconds. No nonzero motion command was sent; workstation
  ROS and Isaac containers were stopped.

## MID-360 IMU extrinsic/time calibration: 2026-09-21

- A `243.046 s` lifted/manual yaw-excitation bag produced `3595` 20-ms wall
  poses and `48160` raw IMU samples. No `/cmd_vel` publisher was present.
- Official MID-360 geometry gives `livox_frame -> imu_link` translation
  `[0.011, 0.02329, -0.04412] m`; IMU and point-cloud axes are identical, so
  rotation is identity. Dynamic yaw independently confirmed polarity and a
  gyro scale of `0.996542`.
- Wall yaw versus integrated gyro resolved a `+9.782937 ms` lag (IMU timestamp
  later than effective LiDAR time), so `/mid360/imu/data_raw` now applies
  `-0.009782937 s`. Three chunks span `9.1673-10.2468 ms`; retained uncertainty
  is approximately `0.6 ms`. Full-fit `R^2=0.9999084`, RMS `0.119 deg`.
- Raw `/livox/imu` is unchanged. Adapted output uses frame `imu_link`, SI
  acceleration and unavailable orientation. The observed session gyro bias is
  not hard-coded because it is temperature-dependent.
- Evidence is in `calibration_data/2026-09-21_static_mid360/`; canonical
  parameter SHA is
  `b079ed22e68807d53896cd86e1738ec72a535b4ead613c4f2dc7d416da75567b`.
- Jetson source/install hashes match. Online pairing of `747` raw/adapted IMU
  samples measured `-9.783030 ms` median, output frame `imu_link`, and an
  unavailable-orientation marker. Live `livox_frame -> imu_link` exactly
  reports `[0.011, 0.02329, -0.04412] m` and identity rotation. IMU/raw/compact
  rates were `200.11/10.04/10.01 Hz`; `/cmd_vel` publisher count remained zero
  and wheel ticks remained `2/0`. The ESP32 was not restarted.

## MID-360 parallel-wall yaw calibration: 2026-09-21

- A `64.230 s` stationary bag was recorded after placing the chassis centerline
  parallel to a nearby wall. No motion command was published and the wheel
  ticks remained stationary.
- The dominant wall was `0.421855 m` from the LiDAR. Its 64 per-frame normal
  fits averaged `-91.12080 deg` after applying the accepted roll/pitch, with
  `0.01011 deg` standard deviation. The accepted yaw is therefore
  `+1.1206755 deg` (`+0.019559477 rad`).
- The opposite wall implied `+1.3218736 deg`, so about `0.2012 deg` is retained
  as wall/placement systematic uncertainty.
- The measured wall-to-near-track outer-edge gap was `0.289 m`. Adding the
  canonical `0.133 m` track outer half-width places the base centerline
  `0.422 m` from the right wall. Against the fitted `0.42185533 m` LiDAR wall
  distance, mount Y is `-0.00014467 m` (right of centerline), with about
  `+/-0.0005 m` ruler uncertainty.
- Rear-wall gaps to the right/left rear-drive-wheel outer extrema were
  `1.153/1.155 m`. Their mean, the canonical `-0.13225 m` rear wheel extremum,
  fitted `1.30078363 m` rear plane, accepted Y/Z and full tilted plane equation
  resolve mount X to `+0.01656608 m`. The `2 mm` left/right spread is retained
  as approximately `+/-2 mm` X uncertainty.
- The canonical YAML, rebuilt URDF/Isaac USD and Jetson source/install now
  consume the same RPY. The parameter SHA is
  `b079ed22e68807d53896cd86e1738ec72a535b4ead613c4f2dc7d416da75567b`;
  live `base_link -> lidar_link` reports translation
  `[0.01656608, -0.00014467, 0.072] m`, and `base_link -> livox_frame` reports
  `-0.352/-0.278/+1.121 deg` with point origin Z `0.119 m`.
- `carbot_description` passed `52/52`, Isaac Lab CPU contracts passed `8/8`,
  and all workstation ROS/Isaac containers were stopped after verification.
  Analysis and the raw Pose D bag are stored with the A-B-A evidence.

## MID-360 A-B-A roll/pitch calibration: 2026-09-21

- Three stationary full-cloud bags were recorded on the Jetson with no Nav2
  process and zero `/cmd_vel` publishers. All captures had zero wheel/odometry
  drift. Pose C returned to Pose A without rebooting the ESP32 and reproduced
  the fitted floor normal within `0.49 mm` height, `0.127 deg` roll and
  `0.019 deg` pitch.
- The chassis was manually reversed by approximately 180 degrees for Pose B.
  Combining Pose B with the Pose A/C mean separates the reversing floor/support
  component from the fixed sensor installation. The accepted MID-360 rotation
  is roll `-0.3515327 deg` (`-0.006135404 rad`), pitch `-0.2783163 deg`
  (`-0.004857536 rad`); the later parallel-wall result supplies yaw. The
  canonical parameter source now contains this RPY, so both the robot
  description and rebuilt Isaac USD
  consume the same transform.
- Mechanical height remains authoritative; the fitted floor distance is not
  used as Z because track support and replacement changed it across poses.
  XY was resolved by the later orthogonal-wall dimensional measurements. A
  static IMU half-sum suggests a small gravity-direction offset but remains
  confounded by accelerometer bias; it was not used in place of the later
  official/dynamic IMU calibration.
- Raw bags, analysis JSON, SHA-256 values and the reusable offline analyzer are
  under `calibration_data/2026-09-21_static_mid360`. The large `.db3` files are
  ignored by Git.
- `carbot_description` passed its isolated `52/52` result and Isaac Lab CPU
  contracts passed `8/8`. The regenerated USD records the canonical parameter
  SHA and contains the calibrated lidar-link quaternion. Jetson source and
  installed YAML hashes match the workstation; after a full Jetson reboot the
  description, MID-360, wheel odometry, micro-ROS and PTP services all returned
  active, and live `base_link -> livox_frame` remained `-0.352/-0.278/0 deg`
  before the later parallel-wall yaw update.

## MID-360 online deployment and acceptance: 2026-09-21

- Jetson identity and both links were reverified: Wi-Fi `192.168.1.109/24`,
  LiDAR Ethernet `192.168.2.100/24`, and MID-360 `192.168.2.202`. Three sensor
  pings had zero loss; the Ethernet interface reported zero RX/TX errors,
  drops, missed packets, carrier errors or collisions.
- The repository-owned `MID360_config.json`, launch, full-field relay and live
  validator were deployed to `/home/shenfq/Projects/carbot-ros2` and
  `carbot_hardware` rebuilt successfully. Workstation, Jetson source and Jetson
  install SHA-256 values match. The active driver reports the installed project
  JSON path, `xfer_format=0`, and `publish_freq=10.0`.
- Online validation confirmed exactly one LiDAR publisher and the raw fields
  `x/y/z/intensity/tag/line/timestamp` in `livox_frame`. Direct CLI samples were
  about `10.00 Hz` raw, `10.00 Hz` compact and `199.95 Hz` adapted IMU. A
  separate 8-second contract run observed `19968` raw points, `9.75/9.99 Hz`
  raw/compact and `191.74 Hz` IMU while processing all three streams in one
  Python executor.
- Per-point timestamps are epoch nanoseconds. In the sampled frame, the first
  point differed from the ROS header by about `0.24 us`, the point span was
  `100.01 ms`, and header-to-validator wall age was about `120.75 ms`.
- Live acceptance exposed a ROS 2 Humble YAML ambiguity where field name `y`
  in a string-array parameter became boolean. The contract parameter was
  changed to a CSV string; local tests passed `10/10`, Jetson relay tests passed
  `3/3`, and the restarted stack contained the driver, IMU adapter and relay
  with no launch warnings/errors or service restarts.
- System `ptp4l` is running as the isolated-link master. A privileged read-only
  query confirmed the Jetson port is `MASTER`; its own `offsetFromMaster=0`
  must not be reported as the LiDAR offset. Direct SDK2 reads of the MID-360
  `time_offset` key produced a five-sample mean of `-26.014 us`, median
  `-25.888 us`, range `-28.612..-23.034 us` and standard deviation
  `2.063 us`; all samples reported PTP `time_sync_type=1`. `Linger=no` remains
  unchanged, so user services depend on a live login/session.

## MID-360 offline configuration closure: 2026-09-21

- `carbot_parameters.yaml` is now the canonical project source for official
  MID-360 range, FoV, precision, point-rate, IMU, synchronization, mechanical,
  power and environment limits, with direct Livox product/protocol URLs and
  machine-checked provenance.
- The Isaac RTX profile remains an approximate coverage proxy. Its `0.03 m`
  range accuracy is tied to the conservative official 1-sigma limit;
  `0.01 m` range resolution and `0.05 deg` angular standard deviations are
  explicitly documented simulation assumptions, not manufacturer claims.
- `carbot_hardware` now owns and installs `MID360_config.json`, configured for
  the recorded Jetson `192.168.2.100` and sensor `192.168.2.202` addresses.
  The launch no longer silently loads the Livox package's generic example.
- `/livox/lidar` remains the authoritative full cloud with expected fields
  `x/y/z/intensity/tag/line/timestamp`; the relay refuses to derive the
  bandwidth-reduced `/mid360/points_xyz` stream if any expected source field is
  absent. The compact stream is not suitable for timestamp, return-quality or
  calibration analysis.
- Offline targeted tests passed `25/25`; the isolated `carbot_description`
  package test result passed `52/52`, and `carbot_description` plus
  `carbot_hardware` built successfully. The installed package contains the new
  JSON and its provenance note. Jetson deployment/config comparison and live
  topic/rate/timestamp inspection remain pending until the Jetson and MID-360
  can be powered. No physical hardware was contacted for this checkpoint.

## Overhead-clearance simulation fixtures: 2026-09-20

- Both the default headless and WebRTC Carbot launch paths now create two
  persistent runtime collision beams without modifying the saved
  `warehouse_v3` map: a green `0.40 m`-clearance beam at map `(2.0, 0.0)` and
  a red `0.28 m`-clearance beam at map `(-2.0, 0.0)`. Both beams are
  `0.40 x 1.00 x 0.10 m` and are recreated on every Isaac Sim startup.
- RViz now subscribes to `/overhead_clearance_markers` and displays the same
  fixtures as translucent cubes with `PASS: 0.40 m clearance` and
  `BLOCKED: 0.28 m clearance` labels. Isaac collision geometry and the ROS
  markers share `configs/carbot/common.yaml`; markers are visual only and do
  not change the scan slice or navigation costs.
- A full cold-start navigation acceptance passed all launcher health checks.
  The `0.40 m` beam produced zero `/scan` hits and the one-plan route passed
  directly underneath with `0.000 m` lateral offset. The `0.28 m` beam
  produced up to 32 `/scan` hits and the one-plan route detoured with
  `1.205 m` minimum lateral offset. Both goals and the intervening return-home
  goal finished `SUCCEEDED`, and the final commanded speed was zero.
- Repeat the acceptance with `python3 scripts/validate_overhead_clearance.py`
  after starting the complete simulation stack. Marker-inclusive scoped
  regression passed `41` tests with `8` optional-dependency skips. The latest
  default headless-Isaac plus RViz start passed all health checks, including
  the marker node/topic checks, and is currently running for user inspection.

## Physical Carbot software closure: 2026-09-19

- The supervised mapping/return acceptance covered about `1.435 m`. Saved-map
  navigation returned `SUCCEEDED`; final map position/yaw error was about
  `5.7 cm/9.6 deg`, and a separate Spin restored the route-start heading to
  within about `0.2 deg`. No longer artificial distance trial is required.
- The canonical parameter source now records partial physical evidence for
  directional track deadband (`0.05 m/s` forward, `0.02 m/s` reverse), reliable
  turn command (`0.40--0.50 rad/s`), first-motion latency, stop tail, and
  longitudinal/yaw gain. Isaac Lab consumes the real deployment envelope
  (`0.10 m/s`, `0.50 rad/s`, `0.20 m/s^2`, `2.00 rad/s^2`) and applies the
  measured directional deadband per track.
- Jetson saved-map startup is consolidated in
  `scripts/jetson_navigation_start.sh`: inactive launch and read-only preflight,
  localization activation, initial-pose plus `map -> odom` confirmation, then
  navigation activation and final silent velocity-topology checks. Diagnostic
  mode remains isolated on `/cmd_vel_diagnostic`.
- Isaac regression passed 43 tests with 8 optional-dependency skips. Headless
  checks passed at `0.9935 m` for the one-metre run, symmetric `+/-138.69 deg`
  turns, and a `0.510 s` watchdog trigger. The Isaac container was stopped.
- ESP32 was powered down by the operator after a battery alarm. Do not request
  further physical motion until the battery and safety conditions are restored.
  The standard hardware e-stop, multi-surface/payload calibration, precision
  MID-360 extrinsics, and periodic ESP32 time resynchronization still block
  unattended autonomy and policy release.

## Shutdown checkpoint: 2026-09-15 — Carbot Phase F foundation complete

The project now contains a runnable Isaac Lab manager-based Carbot foundation
environment under `isaac_lab/carbot_env`. CPU contract tests passed 8/8, and an
Isaac Lab 4.5 headless smoke run loaded the local `carbot.usd`, resolved 20
bodies and all 12 wheel joints, exposed a two-dimensional action and a
108-dimensional policy observation, reset successfully, and stepped zero actions.

- Policy actions are physical-unit `[linear.x, angular.z]`, not normalized PWM,
  torque, or direct track commands. The action term applies the simulation Nav2
  limits, canonical acceleration limits, coupled wheel saturation, imported
  joint sign, and canonical 500 ms watchdog before commanding wheel joints.
- The policy observation is a relative goal, planar odometry-compatible speed,
  projected gravity, exactly 72 finite-clipped LiDAR rays, a 5 x 5 height scan,
  and previous action. Isaac ground-truth pose and privileged simulator state are
  explicitly forbidden policy observations.
- Task goals, rewards, success criteria, and the disabled curriculum live outside
  the robot/USD configuration. Direct left/right track velocity is reserved and
  disabled; PWM and motor torque are prohibited as policy outputs.
- Dynamics and domain-randomization ranges remain
  `TEMP_ESTIMATE_NOT_CALIBRATED`. They are training assumptions, not physical
  confidence intervals.
- `isaac_lab/carbot_env/hardware_calibration_backlog.yaml` is the persistent
  Sim-to-Real release gate. Effective radius `0.02175 m` and effective track
  separation `0.254 m` are confirmed baselines with real-vehicle acceptance
  checks pending; they are not missing-calibration blockers. Jetson odometry,
  slip, actuator dynamics, inertial/friction values, time/extrinsics, physical
  watchdog/e-stop, observation parity, and real navigation remain blocking.
- After validation, the dedicated project containers were stopped. No Phase F
  policy has been trained, frozen, exported, or deployed.

## Shutdown checkpoint: 2026-09-14 — Carbot Phase E complete

Carbot Phase E separates the simulation and physical-robot ROS configurations.
The simulation path passed both ground-truth and AMCL cold-start health checks;
the ground-truth run also completed a short `NavigateToPose` goal with
`SUCCEEDED` and zero recoveries. The real-hardware path has passed configuration,
launch-argument, lint, and package tests only; it has not been motion-tested on
the physical robot.

After validation, `./stop_nav_all.sh` stopped `ros2-dev-humble`,
`isaac-ros-nvblox`, and `isaac-sim` cleanly. No project RViz, Isaac Kit,
nvblox, pointcloud padder, or Carbot launch process remains, and TCP port 49100
is not listening. The next session starts from a clean stopped state.

- `launch/carbot_sim.launch.py` defaults to ground truth and may select AMCL;
  `launch/carbot_real.launch.py` is AMCL-only and defaults to manual Initial Pose.
  `launch/nav_stack.launch.py` remains only as a simulation compatibility wrapper.
- `configs/carbot/common.yaml` owns interface names and points to the canonical
  robot geometry. The sim and real profiles explicitly separate clock, odometry,
  localization, limits, and parameter files.
- Both costmaps use the measured Carbot polygon
  `[[0.155,0.133],[0.155,-0.133],[-0.130,-0.133],[-0.130,0.133]]`, with
  `base_footprint` as the navigation base and `inflation_radius=0.45 m`.
- Simulation consumes Isaac's `/odom` directly. No `/chassis/odom` relay exists.
  The real profile requires external `/odom` and `odom -> base_footprint` from
  the Jetson-side odometry pipeline.
- The preserved `warehouse_v3` map remains the default for nvblox and Nav2.
  The Carbot cold-start checks confirmed `417 x 424` at `0.05 m/cell`, live raw
  and padded Mid-360 point clouds, `/scan`, costmaps, and `map -> base_link`.
- The local parent `start_nav_all.sh` and `stop_nav_all.sh` were updated for the
  Carbot scene/scripts, direct `/odom`, Carbot health invariants, and the new
  RViz configuration. They live one directory above this Git repository.

- The generated Carbot composition disables `/World/Robot/Shen_Carter`,
  `/World/ROS2_Carter_Graph`, and the legacy `/World/ROS2_LidarRTX`; only the
  Carbot articulation and its independent ROS 2 RTX graph are active.
- At this historical pre-calibration checkpoint, the Mid-360 housing bottom
  was modeled at `z=0.157 m`, its top at `z=0.222 m`, and its XY offset as
  `[-0.003, 0]`. These values are superseded by the 2026-09-21 manufacturer
  origin and physical XYZ/RPY calibration recorded at the top of this file.
- The raw `/livox/lidar` cloud is published exactly once with frame
  `front_3d_lidar`. The adapter pads it to `/livox/lidar_nvblox` at `1000 x 40`,
  and exactly one nvblox node consumes that topic with simulation time enabled
  and minimum valid range `0.5 m`.
- At that checkpoint live TF reported
  `odom -> front_3d_lidar = [-0.003, 0, 0.157]`. A blank-map
  run produced a live `0.05 m` OccupancyGrid, confirming the Carbot cloud-to-map
  path. This is a new live map, not the saved `warehouse_v3` map.
- The old Carter prims remain disabled in the generated composition and must not
  be re-enabled alongside the Carbot articulation or ROS graph.

## Physical Jetson target: recorded 2026-09-06

The project's real-hardware target is reachable through the local SSH alias `isaac-jetson` as user `shenfq`. Connection details, verified platform inventory, authentication rules, and safe remote-operation conventions are maintained in [jetson-target.md](jetson-target.md). Read that reference before every Jetson operation; do not copy passwords into project files or commands.

- The Jetson `carbot-ros2` workspace retains the micro-ROS Agent and now runs it as the enabled user service `micro-ros-agent.service`, listening on UDP 8888 in `ROS_DOMAIN_ID=0` to match the completed ESP32 firmware. The service does not publish `/cmd_vel`.
- Isaac simulation also uses Domain 0 but remains isolated with loopback-only `cyclonedds_ros_local.xml`. The opt-in workstation LAN configuration is `configs/cyclonedds_ros_jetson.xml`, loaded by `scripts/real_robot_ros_env.sh` inside a host-networked project container.
- A non-motion `std_msgs/msg/String` probe passed bidirectionally between workstation CycloneDDS and Jetson Fast DDS on Domain 0. The ESP32 was intentionally off during this check, so repeat the `/cmd_vel` endpoint check after it is started.
- The ESP32 static-resource issue reported as `publisher init failed` was fixed in its firmware. After clearing stale Agent sessions, `carbot_base` maintained one `/cmd_vel` subscription and four publishers: `/wheel_ticks`, `/imu/data_raw`, `/battery_state`, and `/carbot/status`. With the chassis lifted, 2026-09-06 ROS 2 smoke tests verified all differential-drive directions through standard `/cmd_vel`: forward at `linear.x=0.08 m/s`, reverse at `linear.x=-0.08 m/s`, in-place left at `angular.z=+0.25 rad/s`, and in-place right at `angular.z=-0.25 rad/s`. The user visually confirmed the track directions were correct. Every motion was followed by repeated zero Twist messages, and the final `/cmd_vel` subscription count remained one. The Jetson currently lacks matching Python type support for the ESP32's `carbot_msgs`, so the custom wheel/status payloads were not decoded during those tests.
- Do not let the existing Isaac navigation launcher load the physical LAN DDS profile. Real hardware still requires verified `/odom`, `odom -> base_footprint`, Mid-360 data/extrinsics, `use_sim_time=false`, AMCL initialization, command timeout, and physical emergency stop before any nonzero command is allowed.

## Physical Carbot checkpoint: 2026-09-16

- Workstation simulation, nvblox, and Nav2 were stopped before joining the physical LAN. Only the host-networked Humble container remains available for physical diagnostics.
- The authoritative ESP32 `carbot_msgs` definitions from firmware commit `63dd45c` were added to this workspace and deployed to Jetson. Live `/wheel_ticks` decoded at about 50 Hz; a 15 s lifted static sample had zero left/right drift and stable boot ID `549633487`.
- `carbot_hardware` was deployed to `/home/shenfq/Projects/carbot-ros2`. Enabled user service `carbot-wheel-odometry.service` is active alongside `micro-ros-agent.service`. The workstation observed exactly one `/odom` publisher and a live `odom -> base_footprint` TF.
- With both tracks lifted, one-shot forward/reverse and positive/negative turn tests passed tick-sign checks. A one-shot `0.05 m/s` command first changed ticks at about 73 ms and stopped changing at about 586 ms; explicit zero commands followed. The vehicle has no standard physical e-stop. The phone web UI red stop did not override a continuous ROS command and is not an independent e-stop. In a later lifted trial, manually cutting ESP32 power about 1.3 s after motion began caused telemetry loss and the operator visually confirmed that the tracks stopped. Repowering caused no motion; ROS returned with new boot ID `2491872445`, command source 0, and zero ticks. This cutoff is accepted only for supervised low-speed calibration with a dedicated operator able to reach it immediately. Final autonomous-navigation acceptance still requires direct drive-power or hardware-enable interruption.
- Directional deadband is strongly asymmetric. In forward vehicle motion, the M3/left track did not sustain motion at `0.01-0.03 m/s` and became stable near `0.05 m/s`; in reverse it sustained motion near `0.02 m/s`. Ground-load braking and slip remain pending.
- ESP32 IMU is currently invalid for fusion. 447 stationary samples had a median acceleration norm of `96.590896 m/s^2`; the driver already scales acceleration to m/s² and the ROS publisher multiplies by standard gravity again. Dynamic gyro is also effectively unresponsive: during externally measured approximately 86° left and 92° right ground turns, bias-corrected integration of all three ROS angular-velocity axes remained near zero and `gyro.z` peaked at only about `0.004 rad/s`. Diagnose sensor refresh/scale, fix, and reflash before IMU/extrinsic fusion work.
- Raw bags, ESP32 telemetry, metadata, and interpretation are under `calibration_data/2026-09-16_lifted`. The ground-trial procedure is `docs/carbot_ground_calibration_protocol.md`.

## Live checkpoint: 2026-08-29

This is the newest authoritative state after converting and validating the project as a Mid-360-only runtime. `ISAAC_WEBRTC=1 ./start_nav_all.sh` is currently running with RViz, and the Isaac WebRTC AppImage is connected/available at `127.0.0.1` for user inspection.

- The current live start completed with `[ OK ] All startup health checks passed.` Keep `isaac-sim`, `isaac-ros-nvblox`, and `ros2-dev-humble` running until the user finishes visual confirmation; stop them with `./stop_nav_all.sh` afterward.

- The normal and WebRTC Isaac launchers now install `Livox_Mid360_Approx`, publish `/livox/lidar`, and retain the Carter asset's legacy sensor prim path only as an internal mounting/graph connection.
- nvblox uses `mid360_nvblox.launch.py`, `/livox/lidar_nvblox`, and a `1000 x 40` padded spherical cloud. The independent XT32 launch and configuration files were removed.
- Nav2, Frontier Exploration, RViz, startup health checks, shutdown patterns, and user documentation now default to Mid-360.
- The post-conversion `./start_nav_all.sh` regression completed with `[ OK ] All startup health checks passed.` It verified `/livox/lidar`, `/livox/lidar_nvblox`, nvblox `1000 x 40`, live occupancy and Scan, AMCL/Nav2 lifecycle, map dimensions, RViz, and `map -> base_link`.
- The post-conversion `ISAAC_WEBRTC=1 START_RVIZ=0 ./start_nav_all.sh` regression also completed with all health checks passed, including the streamed Mid-360 stage and WebRTC endpoint. Both launch modes were stopped cleanly afterward.

- `ISAAC_WEBRTC=1 ./start_nav_all.sh` completed with `[ OK ] All startup health checks passed.` The same Isaac instance provided WebRTC, loaded and played the warehouse automatically, and supplied the full nvblox/Nav2/RViz stack.
- The AppImage connected to `127.0.0.1`; the native Isaac UI showed `/nova_carter_ROS111` and the Pause control while RViz remained open behind it.
- A separate default `./start_nav_all.sh` cold-start regression also completed with all health checks passed and `Isaac WebRTC : disabled`, preserving the original low-overhead behavior.
- The standalone `/isaac-sim/runheadless.sh` route and AppImage connection to `127.0.0.1` were also validated for Isaac-only UI access.
- The latest `./stop_nav_all.sh` run stopped `isaac-sim`, `isaac-ros-nvblox`, and `ros2-dev-humble`; the standalone WebRTC Client and server processes were also closed. The next session starts from a clean stopped state.

## Cold-start baseline: 2026-07-19 end of day

Use this baseline after shutdown or whenever inspection confirms that no project process survives.

- The host was shut down after documentation synchronization. `ros2-dev-humble`, `isaac-ros-nvblox`, and `isaac-sim` were verified stopped; no ROS process survives that shutdown.
- `start_nav_all.sh` and `stop_nav_all.sh` passed Bash syntax validation. The workspace Isaac launcher passed Python syntax validation.
- For the next saved-map navigation session, run `./start_nav_all.sh` from `/home/shenfq/projects/ros-humble`.
- Use `ISAAC_WEBRTC=1 ./start_nav_all.sh` when RViz and the Isaac WebRTC UI are both required.
- The launcher starts Isaac Sim headlessly, loads `warehouse_v3` through nvblox, starts AMCL/Nav2, and opens only RViz2. Require the final line `[ OK ] All startup health checks passed.`
- Aggregate logs are `logs/start_nav_all/{isaac_sim,nvblox,navigation,rviz}.log`; the immediately preceding run is retained as `.previous`.
- Use `./start_nav_all.sh --health-check` for a read-only recheck of an already running stack. Use `./stop_nav_all.sh` for the project shutdown path.

The current engineering objective after validated simulation navigation is real-hardware migration: choose manual or surveyed fixed Initial Pose, measure the real base-to-LiDAR transform and odometry noise, then retune AMCL without changing the validated map or costmap safety settings.

## Validated artifacts and configuration

- `maps/nvblox/warehouse_v3.nvblx` is about 37 MB.
- `maps/nvblox/warehouse_v3.ply` is about 14 MB.
- `maps/2d/warehouse_v3.pgm` is `417 x 424` at `0.05 m/pixel`.
- `maps/2d/warehouse_v3.yaml` has origin `[-14.4, -7.6, 0]` and must use `free_thresh: 0.196` so gray-205 unknown cells remain unknown.
- Reloaded `/map` statistics were 117,570 unknown, 52,618 free, and 6,620 occupied cells.
- `launch/nvblox_with_map.launch.py`, `launch/carbot_sim.launch.py`, and `launch/carbot_real.launch.py` default to warehouse_v3.
- `configs/nav2_params_{sim,real}.yaml` use `GridBased.allow_unknown=false`, `global_costmap.track_unknown_space=true`, the measured Carbot polygon, `inflation_radius=0.45`, and `xy_goal_tolerance=0.10`.
- The projected `/scan` is configured for `base_footprint`, height
  `0.10..0.35 m`, range minimum `0.5 m`, 361 rays, and Best Effort/Volatile
  QoS. The `0.35 m` ceiling leaves `0.11 m` over the `0.24 m` Carbot and its
  top-mounted MID-360; this clearance change still requires live regression.
- The only simulated raw 3D LiDAR topic is `/livox/lidar`; nvblox consumes `/livox/lidar_nvblox` padded to `1000 x 40`.
- Rotation is limited to about `0.35 rad/s`; relevant behavior plugin limits require a Navigation restart after configuration changes.

## Saved-map runtime design

The normal simulation launcher defaults are:

```text
LOCALIZATION_MODE=ground_truth
AMCL_INITIAL_POSE_MODE=odom_identity
START_RVIZ=1
```

`odom_identity` is valid only for the odom-aligned warehouse_v3 Isaac simulation. `carbot_sim.launch.py` supports:

```text
localization_mode:=ground_truth
localization_mode:=amcl
amcl_initial_pose_mode:=odom_identity
amcl_initial_pose_mode:=fixed amcl_initial_x:=X amcl_initial_y:=Y amcl_initial_yaw:=YAW
amcl_initial_pose_mode:=manual
```

- Use `ground_truth` for simulation verification; it publishes identity `map -> odom` and does not require `2D Pose Estimate`.
- Use AMCL `manual` for an arbitrary real-robot start and set RViz `2D Pose Estimate`.
- Use AMCL `fixed` only for a surveyed docking or start pose.
- The one-shot `/amcl_pose_initializer` waits for active AMCL and stationary odometry, publishes Initial Pose three times, verifies `/amcl_pose`, and exits. It must not remain running after success.

## Required saved-map navigation health

Before sending a Goal, require:

- Exactly one nvblox node, container, pointcloud padder, projected scan node, robot-state publisher, map server, and each Nav2 server; no odometry relay.
- No ground-truth `map -> odom` publisher while using AMCL, and no leftover one-shot initializer.
- `map_server`, `amcl`, `controller_server`, `planner_server`, `behavior_server`, and `bt_navigator` active.
- `/navigate_to_pose`, `/spin`, and `/backup` available.
- RViz Fixed Frame `map` and `use_sim_time=true`.
- Warehouse map dimensions `417 x 424`, resolution `0.05 m`, live `/scan`, live nvblox occupancy, and `map -> base_link` TF.
- The robot center free in `/map`, global costmap, and local costmap.

Use `2D Goal Pose` in RViz and wait for the current action to reach a terminal state before sending another Goal. Near map boundaries or obstacles, choose a nearby safer replacement rather than weakening the validated footprint or inflation geometry.

## Mapping boundary

Do not resume mapping into warehouse_v3 unless intentionally creating a newer version. For any mapping or map-save task, stop saved-map Navigation and old nvblox launches, then follow [the map-generation tutorial](../../../docs/map_gen/README.md) completely. Never load v1/v2 while creating a new version.
