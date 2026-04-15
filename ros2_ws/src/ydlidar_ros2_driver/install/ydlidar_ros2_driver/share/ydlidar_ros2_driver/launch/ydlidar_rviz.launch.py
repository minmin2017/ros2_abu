#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/home/vboxuser/.rviz2/default.rviz',
        description='Path to RViz2 configuration file.',
    )

    return LaunchDescription([
        rviz_config_arg,
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
        ),
    ])
