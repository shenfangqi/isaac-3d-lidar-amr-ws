# Validation and diagnostic history

Use this file for regression comparison or parameter provenance. It is historical evidence, not current process state.

## Unified WebRTC and RViz acceptance

- `ISAAC_WEBRTC=1 ./start_nav_all.sh` used one `runheadless.sh` Kit process plus `streaming_auto_play.py`; the server, streaming app, warehouse load, timeline Play, `/clock`, odometry, and 3D LiDAR gates all passed.
- The WebRTC AppImage connected to `127.0.0.1` and displayed the playing warehouse while RViz remained live. The full nvblox/Nav2/AMCL health check passed without duplicate Isaac or ROS nodes.
- The default non-streaming `./start_nav_all.sh` was cold-started separately after the change and also passed every health check.
- `./stop_nav_all.sh` cleanly stopped all three containers after both modes.

## Final saved-map startup acceptance

- A full cold start with all three containers stopped completed end to end and passed every health check.
- Isaac Sim ran headlessly with no native Isaac window or WebRTC AppImage; exactly one RViz window appeared.
- RViz used simulation time and Fixed Frame `map`; warehouse_v3 dimensions, live topics, TF, lifecycle nodes, and NavigateToPose/Spin/BackUp Actions were correct.
- Exactly one AMCL and one `/chassis/odom -> /odom` relay ran; the one-shot initializer exited and no ground-truth publisher remained.
- A later stale ROS CLI daemon failure was avoided by using `--no-daemon` for supported discovery commands and hidden Action services for Action readiness.

## AMCL validation

- Generic AMCL parameters were unstable during turns. The final projected-3D-scan tuning uses a likelihood field, beam skipping, 180 beams, `update_min_d/a=0.10`, 800..3000 particles, motion noise `alpha1/2/3/5=0.01`, and `alpha4=0.005`.
- A slow 360-degree spin completed with maximum sampled correction `0.084 m` and about `0.014 m` net translation change.
- Short turning and return goals succeeded without recoveries; the final return error was about `0.077 m` and `0.236 rad`, with maximum sampled TF correction `0.078 m`.
- Cold-start navigation acceptance completed a `0.709 m` path with `SUCCEEDED`, zero recoveries, about `0.098 m / 0.155 rad` final error, and maximum TF correction `0.029 m`.
- A diagnostic five-goal batch passed 3/5; the two failures used borderline targets with only about `0.50..0.57 m` static-obstacle clearance and large terminal rotations. Logs showed collision prediction and progress failure, not a time or TF-frame regression.
- A strict-safe batch selected targets with at least `0.80 m` obstacle and `0.60 m` unknown clearance and passed 5/5. Those clearances were test-selection criteria only.

## RViz Goal defect history

- RViz initially used wall time, causing future TF extrapolation.
- After simulation time was fixed, Fixed Frame `odom` caused periodic replanning to reuse an old odom-stamped Goal and produce past extrapolation.
- The validated configuration therefore enforces RViz `use_sim_time=true` and Fixed Frame `map`.
- `Publish Point` is not a Nav2 Goal by itself. It publishes `geometry_msgs/PointStamped` on `/clicked_point`; movement requires a subscriber bridge. Standard navigation uses `2D Goal Pose`.

## Ground-truth validation

- Multiple five-goal ground-truth regressions passed 5/5 with zero recoveries.
- Representative paths ranged from about `1.060 m` to `6.875 m`; final XY errors were `0.090..0.096 m`, yaw errors `0.121..0.144 rad`, and no meaningful TF jump occurred.

## Mapping and exploration evidence

- Isaac Sim produced sparse variable-length clouds of about 22k–25k returns. Padding them with NaN rays to `1800 x 31` fixed nvblox spherical range-image integration.
- Before padding, the grid stayed near `64 x 24` with zero occupied cells. After padding, a stationary scan produced about `256 x 249`, 30,189 free cells, and 2,237 occupied cells.
- Frontier Explorer used `0.55/0.40/0.60 m` obstacle/unknown/boundary clearances and never published `/cmd_vel` directly.
- Two targeted gap viewpoints, `(-3.725, 0.425)` and `(-3.775, -6.225)`, were preplanned and reached successfully. The raw user clicks were rejected because one was too close to an obstacle and the other too close to unknown space.
- The completed v3 grid reached about `256 x 424` live before export. The final PGM was visually accepted: outer walls closed, internal structures clear, and no dense salt-and-pepper floor noise.

## Superseded 2026-07-12 checkpoint

An earlier shutdown left only a small unsaved partial v3 in memory. That state was superseded by the completed and saved warehouse_v3. Preserve it only as historical context; never resume from it.
