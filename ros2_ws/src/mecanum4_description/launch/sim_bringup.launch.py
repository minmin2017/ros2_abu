import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("mecanum4_description")
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    default_world = os.path.join(gazebo_pkg, "worlds", "empty.world")

    with open(urdf_file, encoding="utf-8") as urdf:
        robot_description = urdf.read()

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file path",
    )

    gazebo_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        "/opt/ros/humble/lib",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

    spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
        "-entity", "mecanum4",
        "-topic", "robot_description",
        "-x", "1.0",
        "-y", "-3",
        "-z", "0.2",
        "-Y", "1.57",
        "-timeout", "120",
    ],
    output="screen",
)
    delayed_spawn_robot = TimerAction(period=3.0, actions=[spawn_robot])

    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "120",
                ],
                output="screen",
            )
        ],
    )

    wheel_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "wheel_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "120",
                ],
                output="screen",
            )
        ],
    )

    cmd_vel_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="mecanum4_description",
                executable="cmd_vel_to_wheels.py",
                name="cmd_vel_to_wheels",
                output="screen",
                parameters=[{"use_sim_time": True}, {"use_stamped_cmd": False}],
            )
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            gazebo_plugin_path,
            gazebo,
            state_publisher,
            delayed_spawn_robot,
            joint_state_broadcaster,
            wheel_controller,
            cmd_vel_bridge,
        ]
    )
