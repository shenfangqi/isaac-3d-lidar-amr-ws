# Right-turn command compensation final acceptance (2026-09-23)

The canonical right-turn command scale is `0.896`. The formal physical command
path is now:

```text
/cmd_vel_command -> cmd_vel_compensator -> /cmd_vel -> ESP32
```

The Jetson service `carbot-command-compensation.service` is enabled. It remains
silent until an upstream command arrives, preserves zero and left-turn commands,
and maps a requested `-0.3000 rad/s` right turn to `-0.2688 rad/s`. Nav2's
velocity smoother now targets `/cmd_vel_command`. No ESP32 firmware or source
was modified.

Isaac Sim consumes the same canonical scale and the measured uncompensated
right/left yaw response gain (`1 / 0.896`). Its final forward/reverse and
left/right sequence returned within `16.0 mm`; the turns were exactly
`+51.5662/-51.5662 deg`, with zero yaw residual.

`physical_final_04` is the accepted real-vehicle run. It commanded `0.08 m/s`
forward/reverse for three seconds and `+/-0.30 rad/s` turns for three seconds,
with explicit zero commands between every segment and at shutdown. Results:

- linear return error: `0.427 mm`;
- full-sequence position error: `0.874 mm`;
- left/right turns: `+30.0185/-28.6048 deg`;
- isolated turn-pair residual: `+1.4137 deg`;
- full-sequence final yaw error: `+0.0948 deg`;
- measured right actuator command mean: `-0.268800 rad/s`;
- one uninterrupted ESP boot ID (`268099638`);
- final ticks `8/14`, unchanged during the final three-second audit;
- all micro-ROS, wheel odometry, MID-360, EKF and command-compensation services
  remained active; `/cmd_vel_command` returned to zero publishers.

`physical_final_01..03` are non-motion preflight captures. Their strict gates
rejected execution while the post-reboot micro-ROS session was reconnecting or
while the read-only bag recorder added expected subscriptions. They contain no
nonzero command. Large rosbag database files are ignored by Git; metadata,
logs, the accepted JSON analysis and this summary remain tracked.

## Detailed bag analysis

`physical_final_04_detailed_analysis.json` adds header-time-based dynamics and
cross-sensor checks:

- steady forward/reverse velocity was `+0.05482/-0.05373 m/s`;
- smoothed peak linear acceleration was about `0.215 m/s^2` in both directions;
- steady left/right yaw rate was `+0.17243/-0.17251 rad/s`;
- smoothed peak left/right angular acceleration was `0.702/0.780 rad/s^2`;
- first-motion latency was `0.327..0.483 s` and stop tail was
  `0.388..0.465 s` for these low-speed segments;
- all tracked EKF pose/twist covariance values were finite.

The isolated turn-pair residual is independently reproduced by wheel odometry
(`1.423 deg`), bias-corrected MID-360 gyro integration (`1.438 deg`) and EKF
(`1.403 deg`). It is therefore a physical finite-run response variation, not
an EKF-only estimation error. The accepted right turn produced 95.33% of the
left angle in this run. A single-run equal-angle estimate would be about
`0.940`, but it does not supersede the five-pair calibration and visible final
acceptance that selected the canonical `0.896`.
