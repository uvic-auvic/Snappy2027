#!/usr/bin/env bash
# Run the viewer in the initialized development container.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
service="${1:-snappy-jetson}"
if [[ $# -gt 0 ]]; then shift; fi
case "$service" in
  snappy-dev|snappy-jetson) ;;
  *) echo "Usage: $0 [snappy-jetson|snappy-dev] [ROS arguments...]" >&2; exit 2 ;;
esac
: "${DISPLAY:?Run this from a Linux desktop terminal with DISPLAY set.}"
command -v xauth >/dev/null || { echo 'Install xauth on the host first.' >&2; exit 1; }
xauth_file="$(mktemp /tmp/snappy-vision-xauth.XXXXXX)"
trap 'rm -f -- "$xauth_file"' EXIT
authority_source="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ ! -r "$authority_source" ]]; then
  echo "Cannot read Xauthority file: $authority_source" >&2
  exit 1
fi
# Read a copy so xauth never needs to lock or modify the desktop authority file.
cp -- "$authority_source" "$xauth_file"
cookies="$(xauth -f "$xauth_file" nlist "$DISPLAY")"
if [[ -z "$cookies" ]]; then
  echo "No X11 authorization cookie for DISPLAY=$DISPLAY. Run from your desktop session." >&2
  exit 1
fi
# FamilyWild permits the cookie to work with the container's hostname.
printf '%s\n' "$cookies" | sed 's/^..../ffff/' | xauth -f "$xauth_file" nmerge -
compose=(docker compose -f "$repo_root/docker/docker-compose.yml" --profile jetson)
container_id="$("${compose[@]}" ps -q "$service")"
if [[ -z "$container_id" ]]; then
  echo "Start and initialize $service before running the viewer." >&2
  exit 1
fi
container_auth="/tmp/$(basename -- "$xauth_file")"
cleanup() {
  docker exec "$container_id" rm -f -- "$container_auth" >/dev/null 2>&1 || true
  rm -f -- "$xauth_file"
}
trap cleanup EXIT
docker cp "$xauth_file" "$container_id:$container_auth"
"${compose[@]}" exec \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY="$container_auth" \
  -e QT_X11_NO_MITSHM=1 "$service" /bin/bash -lc \
  'source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; exec ros2 run snappy_computer_vision displayFeed "$@"' \
  bash "$@"
