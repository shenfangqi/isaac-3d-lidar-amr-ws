#!/usr/bin/env bash

set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${workspace}/scripts/stop_real_robot_navigation_rviz.sh" "$@"
