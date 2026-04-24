import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("mecanum4_description")
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    world_file = "/home/poomjai/gazabo/worlds/abu_stadium.world"

    with open(urdf_file, encoding="utf-8") as urdf:
        robot_description = ' '.join(urdf.read().split())

    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        "/home/poomjai/gazabo/models:/usr/share/gazebo-11/models",
    )
    gazebo_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        "/opt/ros/humble/lib",
    )
    gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        "/home/poomjai/gazabo:/usr/share/gazebo-11",
    )

    gazebo_no_online_models = SetEnvironmentVariable(
        "GAZEBO_MODEL_DATABASE_URI",
        "",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_file}.items(),
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

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "mecanum4",
                    "-topic",
                    "robot_description",
                    "-x",
                    "1.0",
                    "-y",
                    "-0.5",
                    "-z",
                    "0.2",
                    "-timeout",
                    "120",
                ],
                output="screen",
            )
        ],
    )

    spawn_joint_state_broadcaster = TimerAction(
        period=6.0,
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

    spawn_wheel_controller = TimerAction(
        period=8.0,
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

    return LaunchDescription(
        [
            gazebo_model_path,
            gazebo_plugin_path,
            gazebo_resource_path,
            gazebo_no_online_models,
            gazebo,
            state_publisher,
            spawn_robot,
            spawn_joint_state_broadcaster,
            spawn_wheel_controller,
        ]
    )
