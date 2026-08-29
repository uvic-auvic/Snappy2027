#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This script must be run on the Jetson (aarch64)." >&2
  exit 1
fi

docker compose -f docker/docker-compose.yml --profile jetson build snappy-jetson
docker compose -f docker/docker-compose.yml --profile jetson up -d snappy-jetson
docker compose -f docker/docker-compose.yml --profile jetson exec \
  -e SNAPPY_WITH_CUDA=ON \
  snappy-jetson \
  bash /ros2_ws/scripts/container_setup.sh
docker compose -f docker/docker-compose.yml --profile jetson exec snappy-jetson bash
