#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$workspace/../scripts/common.sh"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  printf 'Usage: bash pi_ws/build.sh [additional CMake arguments...]\n'
  exit 0
fi
load_ros
cd "$workspace"
# New output tree: never reuse old CMake caches after moving sources.
exec colcon --log-base log/pi_runtime build \
  --base-paths src --build-base build/pi_runtime --install-base install/pi_runtime \
  --symlink-install --packages-select learm_bridge learm_bridge_cpp icm20948_ros2 ldlidar \
  --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 "$@"
