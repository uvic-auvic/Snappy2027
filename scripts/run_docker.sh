#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="${repo_root}/docker/docker-compose.yml"

docker compose -f "${compose_file}" up -d snappy-dev
docker compose -f "${compose_file}" exec snappy-dev bash
