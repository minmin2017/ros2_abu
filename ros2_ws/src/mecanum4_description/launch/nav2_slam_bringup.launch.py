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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    default_world = "/home/minmin/gazabo/worlds/abu_stadium.world"
    nav2_params = os.path.join(pkg_path, "config", "nav2_params.yaml")
    slam_params = os.path.join(pkg_path, "config", "slam_params.yaml")
    rviz_config = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = f.read()

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file path",
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        "/home/minmin/gazabo/models:/usr/share/gazebo-11/models",
    )
    gazebo_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        "/opt/ros/humble/lib",
    )
    gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        "/home/minmin/gazabo:/usr/share/gazebo-11",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
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

    # ── Spawn robot at t+3 s ─────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "mecanum4",
                    "-topic", "robot_description",
                    "-x", "1.0",
                    "-y", "-0.5",
                    "-z", "0.2",
                    "-timeout", "120",
                ],
                output="screen",
            )
        ],
    )

    # ── Controllers at t+5-7 s ────────────────────────────────────────────────
    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "120",
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
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "120",
                ],
                output="screen",
            )
        ],
    )

    # ── cmd_vel → wheel visuals bridge at t+8 s ───────────────────────────────
    # use_stamped_cmd=False so this node receives geometry_msgs/Twist from nav2
    cmd_vel_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="mecanum4_description",
                executable="cmd_vel_to_wheels.py",
                name="cmd_vel_to_wheels",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"use_stamped_cmd": False},
                ],
            )
        ],
    )

    # ── SLAM Toolbox at t+10 s ────────────────────────────────────────────────
    # Provides /map topic and map→odom TF required by nav2
    slam_toolbox = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": True}],
            )
        ],
    )

    # ── Nav2 stack at t+12 s ──────────────────────────────────────────────────
    # Includes: controller, planner, BT navigator, behaviors, velocity smoother
    nav2 = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "params_file": nav2_params,
                    "autostart": "True",
                }.items(),
            )
        ],
    )

    # ── RViz2 with Nav2 default view at t+5 s ────────────────────────────────
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        world_arg,
        gazebo_model_path,
        gazebo_plugin_path,
        gazebo_resource_path,
        gazebo,
        state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        wheel_controller,
        cmd_vel_bridge,
        slam_toolbox,
        nav2,
        rviz,
    ])
