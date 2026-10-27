#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source "$PWD/ros2/install/setup.bash"
exec ros2 launch smartcar_sensor_hal sensors.launch.py "$@"
