#!/usr/bin/env bash
set -euo pipefail
if [[ "$(docker inspect -f '{{.State.Running}}' isaac-sim 2>/dev/null || true)" == true ]]; then
  docker exec isaac-sim pkill -TERM -f '^/isaac-sim/kit/python/bin/python3 isaac_lab/carbot_env/scripts/inspect_sensors.py' || [[ $? == 1 ]]
fi
