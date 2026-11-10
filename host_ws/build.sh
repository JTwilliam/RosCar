#!/usr/bin/env bash
set -eo pipefail
HOST_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HOST_WS/../scripts/common.sh"

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: bash host_ws/build.sh [additional colcon build arguments...]\n'
  printf 'Run on Ubuntu 22.04 Desktop + ROS 2 Humble; builds display package only.\n'
  exit 0
fi
# Build configuration is optional; startup deliberately requires the local host.env.
config="${ROSCAR_CONFIG:-$HOST_WS/host.env}"
if [[ -r "$config" ]]; then source "$config"; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOST_WS/log/ros}"
load_ros
command -v colcon >/dev/null 2>&1 || fail 'colcon is missing; install python3-colcon-common-extensions'
cd -- "$HOST_WS"
exec colcon --log-base log/host_runtime build \
  --base-paths src \
  --build-base build/host_runtime \
  --install-base install/host_runtime \
  --packages-select roscar_visualization "$@"
