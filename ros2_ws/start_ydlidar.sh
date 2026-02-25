#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/vboxuser/ros2_ws/install/setup.bash

exec ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=/home/vboxuser/ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
