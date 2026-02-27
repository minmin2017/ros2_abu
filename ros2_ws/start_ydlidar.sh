#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/vboxuser/ros2_ws/install/setup.bash

existing_driver_pid="$(pgrep -f '/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node' || true)"
if [[ -n "${existing_driver_pid}" ]]; then
  echo "ydlidar_ros2_driver_node is already running (pid: ${existing_driver_pid})." >&2
  echo "Stop it first to avoid serial conflicts on /dev/ttyUSB0." >&2
  exit 1
fi

exec ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/vboxuser/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
