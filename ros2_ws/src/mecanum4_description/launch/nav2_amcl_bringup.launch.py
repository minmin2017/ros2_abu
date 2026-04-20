import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
    map_file = "/home/minmin/map.yaml"
    rviz_config = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = ' '.join(f.read().split())

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

    nav2_common_args = ["--ros-args", "--log-level", "info"]

    # ── Localization at t+10 s (map_server + AMCL, own lifecycle_manager) ─────
    # Separate lifecycle_manager so AMCL has time to publish map→odom TF
    # before Nav2 navigation nodes start requesting it.
    localization = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": True},
                    {"yaml_filename": map_file},
                ],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
                arguments=nav2_common_args,
            ),
        ],
    )

    # ── Publish initial pose to AMCL at t+16 s ───────────────────────────────
    # AMCL requires /initialpose to start publishing map→odom TF.
    # Publish 5 times at 1 Hz to ensure AMCL receives at least one message
    # (transient_local QoS isn't set by ros2 topic pub, so retry is needed).
    publish_initial_pose = TimerAction(
        period=16.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "-t", "5", "-r", "1", "/initialpose",
                    "geometry_msgs/msg/PoseWithCovarianceStamped",
                    '{"header": {"frame_id": "map"}, '
                    '"pose": {"pose": {"position": {"x": 1.0, "y": -0.5, "z": 0.0}, '
                    '"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}, '
                    '"covariance": [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0685]}}',
                ],
                output="screen",
            )
        ],
    )

    # ── Nav2 navigation stack at t+18 s ──────────────────────────────────────
    # Starts 8 s after localization — gives AMCL time to publish map→odom TF.
    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    nav2 = TimerAction(
        period=18.0,
        actions=[
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
                arguments=nav2_common_args,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": nav2_lifecycle_nodes},
                ],
                arguments=nav2_common_args,
            ),
        ],
    )

    # ── RViz2 at t+22 s ──────────────────────────────────────────────────────
    # Started AFTER Nav2 is up (t+18 s) so RViz connects to ready topics
    # instead of being flooded during the Nav2 spin-up burst — prevents the
    # Qt event loop from stalling on WSLg.
    rviz = TimerAction(
        period=22.0,
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
        localization,
        publish_initial_pose,
        nav2,
        rviz,
    ])
