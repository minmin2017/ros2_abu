# ROS2 Simulation Aliases
alias sim='~/sim'
alias simfull='~/simfull'
alias simnav='~/simnav'
alias teleop='~/teleop'
alias teleop_motor='ros2 launch mecanum4_description teleop_motor.launch.py'
alias armdock='export PYTHONPATH=$PYTHONPATH:~/roboarm_ws/install/my_vision_system/lib/python3.10/site-packages && ros2 run my_vision_system yolo_docking_node'
alias lidar='~/ros2_ws/start_ydlidar.sh'
alias lidarfilter='python3 ~/ros2_ws/scan_filter.py'
alias lidarview='rviz2 -d ~/ros2_ws/lidar_compare.rviz'

# Workspace Sourcing
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
if [ -f ~/roboarm_ws/install/setup.bash ]; then
    source ~/roboarm_ws/install/setup.bash
fi
