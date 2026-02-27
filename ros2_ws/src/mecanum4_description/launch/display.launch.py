from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum4_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mecanum4_lidar.urdf')
    with open(urdf_file, encoding='utf-8') as urdf:
        robot_description = urdf.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
