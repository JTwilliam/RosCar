#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"

usage() {
  cat <<'HELP'
Usage: bash scripts/raspberry_pi.sh MODE [--dry-run] [launch arguments...]
Run on Raspberry Pi 4B, Ubuntu 22.04 + ROS 2 Humble.
  all       IMU + STM32 bridge + EKF + optional lidar + measured mounting TFs
  imu       IMU only (keep stationary during calibration)
  chassis   STM32 bridge + EKF (IMU and mounting TF run separately)
  bridge    STM32 bridge only: encoder/odom/health; no IMU/EKF/lidar needed
  lidar     STL06N only; mounting TF runs separately
  imu-tf    measured base_link -> imu_link static TF
  lidar-tf  measured base_link -> laser static TF
--dry-run validates config/ROS packages and prints command; starts no nodes.
Configuration: scripts/robot.env or ROSCAR_CONFIG=/absolute/path/to/robot.env
Do not run 'all' together with individual acquisition/TF launchers.
HELP
}
if [[ $# == 0 || "$1" == --help || "$1" == -h ]]; then usage; exit 0; fi
mode="$1"; shift
dry_run=false
extra=()
for arg in "$@"; do
  if [[ "$arg" == --dry-run ]]; then dry_run=true; else extra+=("$arg"); fi
done
case "$mode" in all|imu|chassis|bridge|lidar|imu-tf|lidar-tf) ;; *) usage; fail "Unknown mode: $mode" ;; esac
load_config
port="${STM32_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
baud="${STM32_BAUD:-115200}"
bridge="${BRIDGE_IMPL:-python}"
case "$bridge" in
  python) bridge_package=icm20948_ros2; bridge_node_package=learm_bridge ;;
  cpp) bridge_package=learm_bridge_cpp; bridge_node_package=learm_bridge_cpp ;;
  *) fail 'BRIDGE_IMPL must be python or cpp' ;;
esac
with_lidar="${ENABLE_LIDAR:-true}"
external_tf="${USE_EXTERNAL_MOUNT_TF:-false}"
[[ "$with_lidar" == true || "$with_lidar" == false ]] || fail 'ENABLE_LIDAR must be true or false'
[[ "$external_tf" == true || "$external_tf" == false ]] || fail 'USE_EXTERNAL_MOUNT_TF must be true or false'

# Reject missing measurements before loading ROS or opening any devices.
if [[ "$mode" == imu-tf || ( "$mode" == all && "$external_tf" == false ) ]]; then
  mount_arguments "${IMU_MOUNT:-}" IMU_MOUNT
fi
if [[ "$mode" == lidar-tf || ( "$mode" == all && "$external_tf" == false && "$with_lidar" == true ) ]]; then
  mount_arguments "${LIDAR_MOUNT:-}" LIDAR_MOUNT
fi
load_ros
if [[ "$mode" == all || "$mode" == imu || "$mode" == chassis ]]; then
  load_overlay "${PI_INSTALL:-$ROSCAR_ROOT/pi_ws/install/pi_runtime}"
  require_package icm20948_ros2
fi
if [[ "$mode" == all || "$mode" == chassis ]]; then
  require_package "$bridge_node_package"
  require_package robot_localization
fi
if [[ "$mode" == lidar || ( "$mode" == all && "$with_lidar" == true ) ]]; then
  load_overlay "${LIDAR_INSTALL:-$ROSCAR_ROOT/pi_ws/install/pi_runtime}"
  require_package ldlidar
fi
if [[ "$mode" == bridge ]]; then
  load_overlay "${PI_INSTALL:-$ROSCAR_ROOT/pi_ws/install/pi_runtime}"
  require_package "$bridge_node_package"
fi
case "$mode" in
  bridge)
    run_command ros2 launch "$bridge_node_package" serial_bridge.launch.py \
      "port:=$port" "baud:=$baud" "publish_tf:=false" "${extra[@]}"
    ;;
  all)
    require_package tf2_ros
    run_command ros2 launch "$ROSCAR_ROOT/pi_ws/scripts/pi_system.launch.py" \
      "port:=$port" "baud:=$baud" "bus:=${IMU_BUS:-1}" "address:=${IMU_ADDRESS:-104}" \
      "enable_lidar:=$with_lidar" "external_mount_tf:=$external_tf" "bridge:=$bridge" \
      "imu_mount:=${IMU_MOUNT:-}" "lidar_mount:=${LIDAR_MOUNT:-}" "${extra[@]}"
    ;;
  imu)
    run_command ros2 launch icm20948_ros2 imu.launch.py \
      "bus:=${IMU_BUS:-1}" "address:=${IMU_ADDRESS:-104}" "${extra[@]}"
    ;;
  chassis)
    run_command ros2 launch "$bridge_package" chassis_ekf.launch.py \
      "port:=$port" "baud:=$baud" "${extra[@]}"
    ;;
  lidar) run_command ros2 launch ldlidar stl06n.launch.py "${extra[@]}" ;;
  imu-tf|lidar-tf)
    [[ ${#extra[@]} == 0 ]] || fail 'TF modes do not accept extra launch arguments'
    require_package tf2_ros
    if [[ "$mode" == imu-tf ]]; then
      mount_arguments "${IMU_MOUNT:-}" IMU_MOUNT; child=imu_link
    else
      mount_arguments "${LIDAR_MOUNT:-}" LIDAR_MOUNT; child=laser
    fi
    run_command ros2 run tf2_ros static_transform_publisher "${mount_args[@]}" \
      --frame-id base_link --child-frame-id "$child"
    ;;
esac
