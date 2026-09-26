# Closed-loop yaw acceptance (2026-09-23)

The fixed-duration turn tests proved that one right-turn scale cannot cancel
the Carbot track/ground state variation.  With the canonical `0.896` scale,
four `0.40 rad/s` open-loop pairs accumulated `+4.24 deg`; an effective
`0.940` candidate accumulated `-7.97 deg`, and a subsequent `0.911` candidate
accumulated `-13.01 deg`.  The non-monotonic result rejects further scalar
search as an accurate heading-control strategy.

`closed_loop_30deg_4pairs_01` instead used the deployed MID-360/wheel EKF
`/odom` yaw as feedback.  It alternated four `30 deg` excursions and returns,
used a requested `0.40 rad/s` command in the measured reliable turn range,
predicted braking from current yaw rate, and allowed at most two corrective
pulses after the initial attempt.

Results:

- pair return yaw errors: `+0.767`, `-0.994`, `+0.722`, `+0.702 deg`;
- per-pair position errors: `0.81`, `1.33`, `3.29`, `2.00 mm`;
- cumulative yaw error after all four pairs: `+1.225 deg`;
- cumulative position error: `3.285 mm`;
- one uninterrupted ESP boot ID (`4005221903`), with stationary final ticks;
- actuator output retained the canonical mapping: left `+0.4000 rad/s`, right
  `-0.3584 rad/s`.

The real Nav2 profile now uses a `0.03 rad` yaw goal tolerance, a
`0.50 rad/s` rotate-to-heading request, a measured-reliable
`0.40..0.50 rad/s` Spin range, and the existing `2.00 rad/s^2` final smoother
ramp.  The formerly divergent top-level real profile was synchronized to the
same values.  The canonical `0.896` right-turn scale remains a nominal actuator
correction; final heading accuracy is owned by feedback, not timed motion.

The same four-pair target sequence was regressed through the
`evidence_degraded` actuator layer used by Isaac Sim.  Its pair return errors
were `+0.626`, `-0.671`, `+0.626`, and `-0.671 deg`; maximum absolute error was
`0.671 deg` and cumulative error was `-0.090 deg`.  The existing open-loop
evidence response regression also remained fully passing.  This closes the
same-target feedback contract at the Isaac command/response layer; it does not
claim that uncalibrated PhysX mass, friction, or flexible-track contact is now
a high-fidelity dynamics twin.

Jetson deployment also installed the ROS Humble Nav2, pointcloud-to-laserscan,
laser-filter, and SLAM Toolbox runtime dependencies and installed the
`isaac_3d_lidar_bringup` package.  Package regressions passed (`3 passed,
1 skipped`), focused cross-profile regressions passed (`2 passed`), and a
non-actuating launch with `autostart=false` and output redirected to
`/cmd_vel_diagnostic` loaded and shut down all Nav2 processes cleanly.  The
final `/cmd_vel_command` topology returned to zero publishers and one
compensator subscriber, and wheel ticks stayed unchanged for five seconds.
