#!/usr/bin/env bash
# One command after the Jetson boots: RViz opens, the robot localizes itself
# (in place first; it may turn in place or creep forward 0.2-0.4 m when the
# view is ambiguous), then Nav2 activates and waits for an RViz 2D Goal Pose.
# Nothing moves before the operator confirms below.

set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "自动定位时小车可能会原地转动（45–90°）或往前走 0.2–0.4 m。"
echo "请确认：有人在车旁，可以随时拿起车或断电；车周围至少 0.4 m 没有障碍物。"
read -r -p "确认后输入 y 开始（其他任意键取消）： " answer
if [[ "${answer}" != "y" && "${answer}" != "Y" ]]; then
  echo "已取消，车没有动。"
  exit 1
fi

# A previous run (or the stack the Jetson left behind) must not be running.
"${workspace}/scripts/stop_real_robot_navigation_rviz.sh" >/dev/null 2>&1 || true

exec "${workspace}/scripts/start_real_robot_navigation_rviz.sh" --automatic-activate \
  --localization-strategy segmented_rotation \
  --localization-motion guarded \
  --motion-profile "${workspace}/docs/evidence/issue13_motion_profile_accepted_2026-10-06.json" \
  --operator-rotation-clear --operator-present \
  --localization-translation guarded \
  --linear-profile "${workspace}/docs/evidence/issue53_linear_profile_accepted_2026-10-13.json" \
  --surface-recheck decide "$@"
