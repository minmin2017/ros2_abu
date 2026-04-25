# ROS2 Simulation Aliases
alias sim='~/sim'
alias simfull='~/simfull'
alias simnav='~/simnav'
alias teleop='~/teleop'
alias teleop_motor='ros2 launch mecanum4_description teleop_motor.launch.py'
alias armdock='export PYTHONPATH=$PYTHONPATH:~/roboarm_ws/install/my_vision_system/lib/python3.10/site-packages && ros2 run my_vision_system yolo_docking_node'
alias armyolo='pkill -9 -f "[m]y_vision_system.*yolo_node" 2>/dev/null; sleep 0.5; export PYTHONPATH=$PYTHONPATH:~/roboarm_ws/install/my_vision_system/lib/python3.10/site-packages && ros2 run my_vision_system yolo_node'
alias mapping='ros2 launch mecanum4_description real_mapping.launch.py'
alias btrun='python3 /home/minmin/ros2_ws/src/mecanum4_description/scripts/bt_sequence_node.py'
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
alias yolostart='~/yolostart'
alias yoloselect='source ~/roboarm_ws/install/setup.bash && ros2 run my_vision_system yolo_select_node'
