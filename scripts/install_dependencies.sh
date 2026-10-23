#!/usr/bin/env bash
set -euo pipefail

ros_distro="${ROS_DISTRO:-jazzy}"
if [[ "$ros_distro" != "jazzy" ]]; then
  echo "This integrated configuration is validated on ROS 2 Jazzy; ROS_DISTRO=$ros_distro" >&2
  exit 2
fi

sudo apt update
sudo apt install -y \
  ros-${ros_distro}-ros-gz \
  ros-${ros_distro}-xacro \
  ros-${ros_distro}-robot-state-publisher \
  ros-${ros_distro}-rviz2 \
  ros-${ros_distro}-cartographer \
  ros-${ros_distro}-cartographer-ros \
  ros-${ros_distro}-navigation2 \
  ros-${ros_distro}-nav2-bringup \
  ros-${ros_distro}-nav2-smac-planner \
  ros-${ros_distro}-nav2-mppi-controller \
  ros-${ros_distro}-nav2-rotation-shim-controller
