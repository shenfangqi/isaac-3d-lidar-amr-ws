# MID-360 and wheel EKF dynamic acceptance

`controlled_mid360_wheel_cmd_01` is the first controlled driven bag containing
the command, cumulative wheel ticks, raw wheel odometry, MID-360 IMU, compact
point cloud, TF, and ESP status in one uninterrupted boot. The sequence used
low-speed forward, reverse, left-turn, and right-turn commands; every segment
ended with an explicit zero Twist.

The readiness gate passed all required checks. The ESP32 IMU topic was not
recorded and is not an estimator input. Stationary periods after motion measured
a MID-360 gyro-z bias near `+0.00414 rad/s` and noise standard deviation near
`0.00105 rad/s`. The deployed design estimates this temperature-dependent bias
online only while `/wheel/odom` proves the chassis stationary, and uses a
conservative yaw-rate covariance floor of `4e-6 (rad/s)^2`.

A 2D point-cloud ICP trajectory supplied independent yaw truth. It accepted
528 of 528 adjacent frame pairs with `0.0178 m` median residual. In the exact
common output window of the final isolated ROS Domain 42 replay, ICP measured
`0.18938698 rad`, wheel odometry measured `0.19008596 rad`, and the EKF measured
`0.18940368 rad`. The EKF error is therefore about `0.001 deg`, versus about
`0.040 deg` for wheel-only yaw. EKF translation remained within `3.4 mm` of the
wheel solution. Dynamic process noise kept the ending X/Y/yaw covariance near
`0.0214/0.0454/0.0000346`. The offline A/B therefore passes.

The corresponding replay output is
`controlled_mid360_wheel_cmd_01_ekf_ab`. Large rosbag database files are kept as
local calibration evidence and may be ignored by Git.

## Five-pair directional turn repeatability

`turn_repeat_5pairs_03` contains five left/right pairs at `+/-0.30 rad/s` for
3 seconds with 2-second zero-command settling periods. EKF yaw averaged
`30.18 deg` left with `0.68 deg` sample standard deviation and `35.44 deg`
right magnitude with `0.20 deg` sample standard deviation. Right-turn response
was therefore `1.1743` times the left-turn response. A first-order open-loop
symmetry compensation is right command multiplier `0.8516` (or left multiplier
`1.1743`), pending a compensated validation run. The five uncompensated pairs
accumulated `-26.21 deg` net yaw. The robot was stationary with zero
`/cmd_vel` publishers after the capture.

`compensated_visible_motion_04` applied the first-pass `0.8516` multiplier to a
right-turn command after a matched forward/reverse run. The robot travelled
about `0.308 m`, returned within `7.5 mm`, turned `32.08 deg` left and
`30.49 deg` right magnitude, and ended the complete sequence within about
`9 mm` and `-0.28 deg` of its starting EKF pose. Turn-pair residual fell from
about `5.9 deg` uncompensated to `1.60 deg`. The first multiplier is slightly
too strong; this run estimates a refined right multiplier near `0.896` before
global deployment.
