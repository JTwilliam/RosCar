#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
usage() {
  cat <<'HELP'
Usage: bash scripts/desktop.sh [rviz|imu-view] [--dry-run] [ROS arguments...]
Run on Ubuntu 22.04 Desktop + ROS 2 Humble, not in an SSH shell on the Pi.
  rviz      Default: TF + /scan + /odom + /odometry/filtered; no driver build needed
  imu-view  Optional standalone IMU attitude view; requires built Python viewer package
--dry-run checks ROS packages and prints command without opening a GUI.
Configuration: scripts/robot.env or ROSCAR_CONFIG=/absolute/path/to/robot.env
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
case "$mode" in rviz|imu-view) ;; *) usage; fail "Unknown mode: $mode" ;; esac
load_config
load_ros
require_package rviz2
if [[ "$dry_run" == false && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  fail 'No desktop display found. Run in a local Ubuntu Desktop terminal.'
fi
case "$mode" in
  rviz)
    config="${RVIZ_CONFIG:-$ROSCAR_ROOT/config/desktop.rviz}"
    [[ -r "$config" ]] || fail "RViz config unavailable: $config"
    run_command ros2 run rviz2 rviz2 -d "$config" "${extra[@]}"
    ;;
  imu-view)
    load_overlay "${DESKTOP_INSTALL:-$ROSCAR_ROOT/pi_ws/install}"
    require_package icm20948_ros2
    run_command ros2 launch icm20948_ros2 viewer.launch.py "${extra[@]}"
    ;;
esac
