#!/usr/bin/env bash

set -euo pipefail

container="carbot-real-mapping-rviz"

if ! docker container inspect "${container}" >/dev/null 2>&1; then
  echo "RViz container is already stopped."
  exit 0
fi

# RViz/Qt can wait indefinitely for a graphics or DDS thread during teardown.
# Bound the graceful stop, then remove only this dedicated container.
docker stop --time 3 "${container}" >/dev/null 2>&1 || true

for _ in 1 2 3 4 5; do
  if ! docker container inspect "${container}" >/dev/null 2>&1; then
    echo "RViz container stopped."
    exit 0
  fi
  sleep 0.2
done

docker rm -f "${container}" >/dev/null 2>&1 || true
if docker container inspect "${container}" >/dev/null 2>&1; then
  echo "Failed to remove RViz container ${container}." >&2
  exit 1
fi

echo "RViz container force-removed after bounded shutdown."
