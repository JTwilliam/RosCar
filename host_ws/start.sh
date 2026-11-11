#!/usr/bin/env bash
set -eo pipefail
HOST_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$HOST_WS/../scripts/common.sh"
usage() {
  cat <<'HELP'
Usage: bash host_ws/start.sh [rviz|imu-view|health] [--dry-run] [launch arguments...]
Run in a local Ubuntu 22.04 Desktop + ROS 2 Humble terminal, not on the Pi.
  rviz      Main RViz: /scan, /odom, /odometry/filtered and TF (default)
  imu-view  Optional IMU display node and independent attitude RViz
  health    Read-only terminal health monitor; no display or RViz required
--dry-run checks configuration/packages and prints command; starts no GUI/node.
Config: host_ws/host.env, or ROSCAR_CONFIG=/absolute/path/to/host.env
HELP
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
mode=rviz
if [[ $# -gt 0 && "$1" != --dry-run ]]; then mode="$1"; shift; fi
dry_run=false
extra=()
for arg in "$@"; do
  if [[ "$arg" == --dry-run ]]; then dry_run=true; else extra+=("$arg"); fi
done
case "$mode" in rviz|imu-view|health) ;; *) usage; fail "Unknown mode: $mode" ;; esac
export ROSCAR_CONFIG="${ROSCAR_CONFIG:-$HOST_WS/host.env}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOST_WS/log/ros}"
load_config
load_ros
load_overlay "$HOST_WS/install/host_runtime"
require_package roscar_visualization
if [[ "$mode" == health ]]; then
  require_package diagnostic_msgs
  require_package rclpy
  run_command ros2 run roscar_visualization health_monitor "${extra[@]}"
  exit 0
fi
require_package rviz2
if [[ "$dry_run" == false && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  fail 'No desktop display found. Run in a local Ubuntu Desktop terminal.'
fi
case "$mode" in
  rviz) launch_file=viewer.launch.py ;;
  imu-view) launch_file=imu_view.launch.py ;;
esac
run_command ros2 launch roscar_visualization "$launch_file" "${extra[@]}"
