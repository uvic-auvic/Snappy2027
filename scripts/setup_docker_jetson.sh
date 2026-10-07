#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${script_dir}/.."

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This script must be run on the Jetson (aarch64)." >&2
  exit 1
fi

if ! runtimes="$(docker info --format '{{json .Runtimes}}')"; then
  echo "Cannot reach Docker. Check docker context ls and DOCKER_HOST/DOCKER_CONTEXT." >&2
  exit 1
fi
if [[ "${runtimes}" != *'"nvidia":'* ]]; then
  echo "The selected Docker daemon does not have the NVIDIA runtime." >&2
  echo "On Jetson, use the host Docker Engine, not Docker Desktop." >&2
  echo "See docs/docker.md for context selection and runtime configuration." >&2
  exit 1
fi

docker compose -f docker/docker-compose.yml --profile jetson build snappy-jetson
docker compose -f docker/docker-compose.yml --profile jetson up -d snappy-jetson
docker compose -f docker/docker-compose.yml --profile jetson exec \
  -e SNAPPY_WITH_CUDA=ON \
  snappy-jetson \
  bash /ros2_ws/scripts/container_setup.sh
docker compose -f docker/docker-compose.yml --profile jetson exec snappy-jetson bash
