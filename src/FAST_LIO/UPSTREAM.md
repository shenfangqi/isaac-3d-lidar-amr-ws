# Upstream provenance

This source tree is vendored from the official `hku-mars/FAST_LIO` repository:

- Branch: `ROS2`
- Commit: `a4743b095409588842a5b30ddfa27e29d2f99164`
- ikd-Tree commit: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- Retrieved: 2026-09-25

The upstream `doc/` media bundle is intentionally not mirrored here because it
is not required to build or run FAST-LIO2. The paper and demonstration media
remain available from the upstream repository identified above.

Local integration changes are intentionally small and auditable:

- parameterized output frame names;
- optional TF broadcasting so the Carbot adapter is the sole TF authority;
- sensor-data QoS for the MID-360 IMU;
- populated odometry twist before publishing.
