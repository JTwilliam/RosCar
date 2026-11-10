#!/usr/bin/env bash
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# All machine-specific data now belongs to this workspace.
export ROSCAR_CONFIG="${ROSCAR_CONFIG:-$workspace/pi.env}"
export PI_INSTALL="${PI_INSTALL:-$workspace/install/pi_runtime}"
export LIDAR_INSTALL="${LIDAR_INSTALL:-$workspace/install/pi_runtime}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace/log/ros}"
exec bash "$workspace/../scripts/raspberry_pi.sh" "$@"
