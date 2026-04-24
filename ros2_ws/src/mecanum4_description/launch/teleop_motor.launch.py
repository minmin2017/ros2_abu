import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory("mecanum4_description")
    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = ' '.join(f.read().split())

    # 1. Robot State Publisher (เพื่อให้เห็นโมเดลหุ่นใน RViz ถ้าต้องการ)
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description},
        ],
    )

    # 2. Arduino Serial Bridge (สื่อสารกับ Mega)
    arduino_node = Node(
        package='mecanum4_description',
        executable='cmd_vel_to_arduino.py',
        name='cmd_vel_to_arduino',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint'
        }]
    )

    return LaunchDescription([
        state_publisher,
        arduino_node,
    ])
