#!/usr/bin/env bash
# Shared by the entry scripts; not a standalone launcher.
ROSCAR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

load_config() {
  local config="${ROSCAR_CONFIG:-$ROSCAR_ROOT/scripts/robot.env}"
  if [[ -f "$config" ]]; then
    # Trusted local shell configuration, not downloaded input.
    source "$config"
  elif [[ -n "${ROSCAR_CONFIG:-}" ]]; then
    fail "Configuration not found: $config"
  fi
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROSCAR_ROOT/log/ros}"
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || fail 'ROS_DOMAIN_ID must be an integer'
  [[ "$ROS_LOCALHOST_ONLY" == 0 || "$ROS_LOCALHOST_ONLY" == 1 ]] || fail 'ROS_LOCALHOST_ONLY must be 0 or 1'
}

load_ros() {
  [[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble not found at /opt/ros/humble'
  # ROS setup scripts are not guaranteed to work under nounset.
  source /opt/ros/humble/setup.bash
  printf 'ROS_DOMAIN_ID=%s ROS_LOCALHOST_ONLY=%s RMW=%s\n' \
    "${ROS_DOMAIN_ID:-30}" "${ROS_LOCALHOST_ONLY:-0}" "${RMW_IMPLEMENTATION:-default}"
}

load_overlay() {
  local setup="$1/local_setup.bash"
  [[ -r "$setup" ]] || fail "Build this workspace first; missing $setup"
  # local_setup avoids reintroducing underlays captured on another build host.
  source "$setup"
}

require_package() {
  ros2 pkg prefix "$1" >/dev/null 2>&1 || fail "ROS package unavailable: $1"
}

run_command() {
  printf 'Command:'
  printf ' %q' "$@"
  printf '\n'
  if [[ "$dry_run" == true ]]; then return 0; fi
  exec "$@"
}

# Expand an explicitly measured x y z roll pitch yaw tuple into named arguments.
mount_arguments() {
  local value="$1" label="$2" item
  local -a fields
  read -r -a fields <<< "$value"
  [[ ${#fields[@]} == 6 ]] || fail "$label requires six values: x y z roll pitch yaw (m/rad)"
  for item in "${fields[@]}"; do
    [[ "$item" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]] || fail "$label contains invalid number: $item"
  done
  mount_args=(--x "${fields[0]}" --y "${fields[1]}" --z "${fields[2]}"
              --roll "${fields[3]}" --pitch "${fields[4]}" --yaw "${fields[5]}")
}
