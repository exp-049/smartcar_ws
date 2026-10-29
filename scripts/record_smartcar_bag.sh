#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

usage() {
  cat <<'EOF'
Usage: record_smartcar_bag.sh [bag_path]

Records the Smartcar sensor and state topics. If bag_path is omitted, the bag
is written to bags/smartcar_YYYYMMDD_HHMMSS under the workspace.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "$#" -gt 1 ]]; then
  usage >&2
  exit 2
fi

if [[ ! -r "$ROS_SETUP" ]]; then
  printf 'ROS setup file not found: %s\n' "$ROS_SETUP" >&2
  exit 1
fi
if [[ ! -r "$WS_ROOT/install/setup.bash" ]]; then
  printf 'Workspace overlay not found. Build the workspace first: %s\n' "$WS_ROOT" >&2
  exit 1
fi

source "$ROS_SETUP"
source "$WS_ROOT/install/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${1:-${WS_ROOT}/bags/smartcar_${STAMP}}"
mkdir -p "$(dirname -- "$BAG_PATH")"
if [[ -e "$BAG_PATH" ]]; then
  printf 'Bag path already exists: %s\n' "$BAG_PATH" >&2
  exit 1
fi

TOPICS=(
  /car/scan
  /car/imu/data_raw
  /car/chassis/odom
  /tf
  /tf_static
  /joint_states
  /diagnostics
  /map
)

printf 'Recording Smartcar topics to: %s\n' "$BAG_PATH"
printf 'Velocity command topics are intentionally excluded. Press Ctrl-C to stop.\n'
exec ros2 bag record --output "$BAG_PATH" --topics "${TOPICS[@]}"
