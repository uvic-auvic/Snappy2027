#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" == "aarch64" ]]; then
  echo "This script is for an x86_64 Linux development machine." >&2
  echo "On the Jetson, run scripts/setup_docker_jetson.sh." >&2
  exit 1
fi

docker compose -f docker/docker-compose.yml up -d snappy-dev
docker compose -f docker/docker-compose.yml exec snappy-dev bash /ros2_ws/scripts/container_setup.sh
docker compose -f docker/docker-compose.yml exec snappy-dev bash
