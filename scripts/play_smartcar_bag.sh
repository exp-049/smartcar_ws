#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

usage() {
  cat <<'EOF'
Usage: play_smartcar_bag.sh [--live] bag_path

Default playback remaps recorded topics under /rosbag_replay so it stays
separate from live robot nodes. Use --live to replay onto original topic names.
EOF
}

MODE="isolated"
if [[ "${1:-}" == "--live" ]]; then
  MODE="live"
  shift
fi
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "$#" -ne 1 ]]; then
  usage >&2
  exit 2
fi

BAG_PATH="$1"
if [[ ! -d "$BAG_PATH" || ! -f "$BAG_PATH/metadata.yaml" ]]; then
  printf 'ROS bag directory or metadata.yaml not found: %s\n' "$BAG_PATH" >&2
  exit 1
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

PLAY_ARGS=(--clock)
if [[ "$MODE" == "isolated" ]]; then
  PLAY_ARGS+=(--remap
    '/car/scan:=/rosbag_replay/car/scan'
    '/car/imu/data_raw:=/rosbag_replay/car/imu/data_raw'
    '/car/chassis/odom:=/rosbag_replay/car/chassis/odom'
    '/tf:=/rosbag_replay/tf'
    '/tf_static:=/rosbag_replay/tf_static'
    '/joint_states:=/rosbag_replay/joint_states'
    '/diagnostics:=/rosbag_replay/diagnostics'
    '/map:=/rosbag_replay/map'
    '/clock:=/rosbag_replay/clock')
  printf 'Isolated playback: topics are remapped under /rosbag_replay.\n'
else
  printf 'LIVE playback publishes recorded sensor/state topics on their original names.\n'
  printf 'Stop robot control/navigation first; do not replay command topics to a moving robot.\n'
fi

exec ros2 bag play "$BAG_PATH" "${PLAY_ARGS[@]}"
