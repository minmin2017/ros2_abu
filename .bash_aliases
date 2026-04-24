# ROS2 Simulation Aliases
alias sim='~/sim'
alias simfull='~/simfull'
alias simnav='~/simnav'
alias teleop='~/teleop'
alias armdock='ros2 run my_vision_system yolo_docking_node'

# Workspace Sourcing
source /opt/ros/humble/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi
if [ -f ~/roboarm_ws/install/setup.bash ]; then
    source ~/roboarm_ws/install/setup.bash
fi
