# 2026-09-21 MID-360 stationary calibration capture

## Safety and runtime state

- No Nav2 process was started and `/cmd_vel` had zero publishers.
- Wheel ticks were unchanged for the complete capture.
- The workstation process table contained no Isaac Sim, nvblox, or Nav2 process.
- Jetson identity, address, and ED25519 host fingerprint matched the project record.
- `carbot-mid360.service`, `micro-ros-agent.service`,
  `carbot-wheel-odometry.service`, and `carbot-mid360-ptp.service` were active.
- The MID-360 Ethernet link had zero RX/TX errors and drops. Three sensor pings
  had zero loss.

## Bag

The raw bag was recorded on the Jetson so the full PointXYZRTLT cloud did not
cross Wi-Fi during capture:

```text
mid360_static_full_65s/
```

- Duration: `63.915750035 s`
- Size: `367.8 MiB`
- Start: `2026-09-21 16:42:39 JST`
- Raw point clouds: `639`
- MID-360 SI IMU samples: `12785`
- Wheel tick and odometry samples: `3197` each
- Database SHA-256:
  `8aeb4297f5f74d3b61fab1be8b14a311cd802fb05ae54117b215a00acb3c6e8f`

The `.db3` is ignored by Git. `metadata.yaml` and
`mid360_static_analysis.json` retain the reproducible inventory and results.

## Results

`scripts/analyze_mid360_static_bag.py` sampled 64 raw clouds. It rejected ten
single-frame plane candidates that disagreed with the mechanical height or
maximum tilt gate. An aggregate RANSAC/SVD fit used 31,076 candidate points and
9,934 inliers:

| Quantity | Result |
|---|---:|
| LiDAR-origin height over observed floor | `0.20117 m` |
| Apparent roll | `-0.7207 deg` |
| Apparent pitch | `-0.6579 deg` |
| Plane inlier residual standard deviation | `6.58 mm` |
| Plane inlier fraction | `31.97%` |

The SI IMU mean acceleration was
`[0.15832, -0.12534, 9.70107] m/s^2`, norm `9.70317 m/s^2`. Its gravity
direction gave apparent roll `-0.7402 deg` and pitch `-0.9349 deg`. Roll agrees
with the point-cloud floor normal within about `0.02 deg`; pitch differs by
about `0.28 deg`.

These are combined floor, track-support, chassis, LiDAR, and internal IMU
angles. A single stationary pose cannot separate those terms or observe yaw.
The first pose alone therefore does **not** replace the nominal fixed transform.

### 180-degree reversal pair

The chassis was manually rotated by approximately 180 degrees and a second
`64.335 s`, `371.1 MiB` bag was recorded as `mid360_static_pose_b_65s`.
Its database SHA-256 is
`4ae9d25885560641074d5bc0d84019ec5f02b338c3c07bdcdafd49ad78353c4b`.
The vehicle remained stationary: wheel tick and odometry deltas were zero.

An initial unconstrained fit selected another horizontal surface at `0.167 m`.
Both bags' height histograms instead contained the same strong floor peak near
`z=-0.196 m`. The analyzer therefore applies the independently measured
mechanical-height gate during RANSAC model selection, not merely after fitting.
The corrected aggregate results are:

| Quantity | Pose A | Pose B |
|---|---:|---:|
| Floor height | `0.20117 m` | `0.19197 m` |
| LiDAR apparent roll | `-0.7207 deg` | `-0.0459 deg` |
| LiDAR apparent pitch | `-0.6579 deg` | `+0.0917 deg` |
| IMU gravity roll | `-0.7402 deg` | `-0.3802 deg` |
| IMU gravity pitch | `-0.9349 deg` | `-0.3515 deg` |

Under the 180-degree reversal assumptions, the half-sum gives the fixed
installation candidate and the half-difference gives the floor/support term:

| Component | Roll | Pitch |
|---|---:|---:|
| LiDAR fixed candidate | `-0.3833 deg` | `-0.2831 deg` |
| LiDAR reversing floor/support | `-0.3374 deg` | `-0.3748 deg` |
| IMU fixed candidate | `-0.5602 deg` | `-0.6432 deg` |
| IMU reversing floor/support | `-0.1800 deg` | `-0.2917 deg` |

### Pose C repeat and accepted LiDAR roll/pitch

After manually returning the chassis to the original orientation without
rebooting the ESP32, a third `63.537 s` bag was recorded. Pose C reproduced
Pose A within `0.49 mm` height, `0.127 deg` roll, and `0.019 deg` pitch. The
IMU gravity estimates repeated within `0.064 deg` roll and `0.001 deg` pitch.
The Pose C database SHA-256 is
`9e8076b8940ec5d26bfb57c8365ed1add5c3ad95db01977143017a812c503c99`.

Using the Pose A/C mean as the repeated first orientation and Pose B as the
reversed orientation gives the accepted LiDAR installation result:

```text
roll  = -0.3515327 deg = -0.006135404 rad
pitch = -0.2783163 deg = -0.004857536 rad
yaw   = +1.1206755 deg = +0.019559477 rad (parallel-wall refinement below)
```

This RPY is written to the canonical `rotation_rpy_rad`. Mechanical Z
remains authoritative because the three floor distances include track support
and placement changes. The IMU half-sum gives a candidate gravity direction,
but accelerometer bias is not separated by this static experiment. It was
therefore not used as an extrinsic rotation measurement; the official MID-360
axis definition and the dynamic validation below are authoritative.

Timing and stability evidence:

- Point timestamp span: median `99.950 ms`, mean `99.990 ms`.
- First point minus ROS header: median `0`, bounded by about `+/-0.24 us`.
- Bag receive minus point-cloud header: median `111.292 ms`, p95 `112.473 ms`.
- Bag receive minus MID-360 IMU header: median `1.418 ms`, p95 `1.937 ms`.
- Bag receive minus wheel header: median `3.097 ms`, p95 `11.662 ms`, maximum
  `39.378 ms`.
- Wheel tick delta: left `0`, right `0`; odometry translation/yaw delta: zero.
- ESP32 boot ID remained `2670297074`; reconnect count remained exactly `3`.
- Battery median was `8.1486 V`; `battery_low=false` during the live precheck.

The system PTP service was continuously in the master role, but both the root
management socket and network management port required elevated privileges.
No permissions or ptp4l configuration were changed. An exact MID-360 slave
servo offset is therefore still not claimed from `pmc`; the device timestamps,
monotonicity, and receive-latency evidence remain the available time proof.

## Dynamic MID-360 IMU extrinsic and time calibration

`mid360_imu_time_yaw_excitation_01` records `243.046 s` of raw point clouds,
raw/adapted IMU and wheel evidence while the lifted vehicle was manually yawed
back and forth. No `/cmd_vel` publisher was present. The useful motion window
contained `3595` independently fitted 20 ms wall poses and `48160` IMU samples.

The official MID-360 user manual states that the IMU axes are identical to the
point-cloud coordinate axes and gives the IMU-chip position in point-cloud
coordinates as:

```text
livox_frame -> imu_link
translation = [0.011, 0.02329, -0.04412] m
rotation RPY = [0, 0, 0] rad
```

Dynamic wall yaw was compared with bias-corrected integrated gyro Z while
jointly fitting yaw intercept, gyro scale and residual drift. The best lag was
`+0.009782937 s` under the convention
`lidar_yaw(t) ~ integrated_gyro_z(t + lag)`. Thus the correction added to the
outgoing adapted IMU timestamp is `-0.009782937 s`. Three independent time
chunks gave `9.1673`, `10.1891` and `10.2468 ms`; approximately `0.6 ms` is
retained as timing uncertainty. The full fit had `R^2=0.9999084`, gyro scale
`0.996542`, RMS yaw residual `0.002075 rad` (`0.119 deg`) and residual drift
`0.000390 rad/s`.

The adapter now publishes `/mid360/imu/data_raw` in `imu_link`, subtracts the
calibrated `9.782937 ms`, converts acceleration to SI units and continues to
mark orientation unavailable. The raw `/livox/imu` stream remains unchanged.
The session gyro bias `[0.003177, -0.003918, 0.005394] rad/s` is evidence only;
it is temperature-dependent and is not hard-coded.

Analysis output:
`mid360_imu_time_yaw_excitation_01_analysis.json` (SHA-256
`ca36f5fcd00f3b58d88dc667ee6a3758a64e9dcebb8686c379d92bef4015def4`).

## Reproduce

On a machine with ROS 2 Humble, `rosbag2_py`, the project custom messages, and
NumPy available:

```bash
python3 scripts/analyze_mid360_static_bag.py \
  calibration_data/2026-09-21_static_mid360/mid360_static_full_65s \
  --output calibration_data/2026-09-21_static_mid360/mid360_static_analysis.json

python3 scripts/analyze_mid360_lidar_imu_dynamic.py \
  calibration_data/2026-09-21_static_mid360/mid360_imu_time_yaw_excitation_01 \
  --output calibration_data/2026-09-21_static_mid360/mid360_imu_time_yaw_excitation_01_analysis.json
```

## Parallel-wall yaw refinement

With the chassis centerline placed parallel to a nearby wall, Pose D recorded
`631` raw clouds over `64.230 s`. The dominant near wall was `0.421855 m` from
the LiDAR origin. Across 64 sampled frames its corrected normal bearing was
`-91.12080 deg` with standard deviation `0.01011 deg`, giving the accepted
mount yaw `+1.1206755 deg` (`+0.019559477 rad`). An opposite wall independently
gave `+1.3218736 deg`; the `0.2012 deg` difference is retained as a conservative
wall/placement systematic rather than hidden in the frame-fit precision.
The Pose D database SHA-256 is
`cae4d3ea56b08591dfcf0643d3b0c11994c7fb2b0496ff26a1a267e5d6d88578`.

```bash
python3 scripts/analyze_mid360_wall_bag.py \
  calibration_data/2026-09-21_static_mid360/mid360_wall_parallel_pose_d_65s \
  --output calibration_data/2026-09-21_static_mid360/mid360_wall_parallel_pose_d_analysis.json
```

The measured right-wall-to-near-track outer-edge gap was `0.289 m`. The
canonical physical track centerline/width make the track outer half-width
`0.225/2 + 0.041/2 = 0.133 m`, so the base centerline was `0.422 m` from the
wall. Comparing this with the fitted `0.42185533 m` LiDAR wall distance gives
the lateral mount offset `Y=-0.00014467 m` (right of centerline). The millimeter
ruler reading limits this result to approximately `+/-0.0005 m`.

The same capture contains an orthogonal rear wall at `1.30078363 m`. Measured
wall gaps to the right/left rear-drive-wheel outer extrema were `1.153 m` and
`1.155 m`. Using their `1.154 m` mean, the canonical rear wheel extremum
`x=-0.13225 m`, the calibrated Y/Z, and the fitted rear-wall normal
`[-0.99992103, -0.00047577, 0.01255827]` resolves mount
`X=+0.01656608 m`. The full plane equation matters: omitting the wall's
`0.71955 deg` vertical tilt would give `+0.01453363 m`. The `2 mm` left/right
measurement spread is retained as an approximate `+/-2 mm` X uncertainty.

## Next physical fixture

Place the chassis on a surveyed level plate, record one pose, rotate the whole
chassis by 180 degrees without changing the sensor mount, and record a second
pose. The half-sum/half-difference of the two fitted tilts separates fixed
sensor mounting error from floor/chassis reference error. Add two surveyed,
orthogonal vertical planes with measured base references to make XY fully
observable and independently verify the wall-derived yaw.
