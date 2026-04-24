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

    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    default_world = "/home/poomjai/gazabo/worlds/abu_stadium.world"
    default_map = os.path.join(
        os.path.expanduser("~"), "ros2_ws", "maps", "abu_stadium_map.yaml"
    )
    nav2_params = os.path.join(pkg_path, "config", "nav2_params.yaml")
    rviz_config = os.path.join(pkg_path, "config", "nav2_view.rviz")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = ' '.join(f.read().split())

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map yaml file",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file path",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch gzclient (Gazebo GUI). Default false on WSLg — "
                    "gzclient competes with RViz for the d3d12 GL device.",
    )

    # ── Environment ───────────────────────────────────────────────────────────
    gazebo_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        "/home/poomjai/gazabo/models:/usr/share/gazebo-11/models",
    )
    gazebo_plugin_path = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH", "/opt/ros/humble/lib"
    )
    gazebo_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        "/home/poomjai/gazabo:/usr/share/gazebo-11",
    )
    # GPU rendering pins (WSLg d3d12 → NVIDIA). Lidar stays on CPU ray.
    libgl_hw = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "0")
    gallium_d3d12 = SetEnvironmentVariable("GALLIUM_DRIVER", "d3d12")
    ogre_rtt = SetEnvironmentVariable("OGRE_RTT_MODE", "FBO")
    mesa_adapter = SetEnvironmentVariable("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")

    # ── Gazebo ────────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, {"robot_description": robot_description}],
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

    # ── Map server — serves the saved static map ──────────────────────────────
    map_server = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    nav2_params,
                    {"use_sim_time": True},
                    {"yaml_filename": LaunchConfiguration("map")},
                ],
            )
        ],
    )

    # ── AMCL — particle-filter localization on the static map ─────────────────
    amcl = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            )
        ],
    )

    # ── Lifecycle manager for map_server + amcl ───────────────────────────────
    lifecycle_localization = TimerAction(
        period=6.0,
        actions=[
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
            )
        ],
    )

    # ── Nav2 navigation stack at t+10 s (same as nav2_slam_bringup, no velocity_smoother) ──
    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]
    nav2_common_args = ["--ros-args", "--log-level", "info"]

    nav2 = TimerAction(
        period=10.0,
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

    # ── RViz2 ─────────────────────────────────────────────────────────────────
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

    # ── Camera viewers at t+12 s ─────────────────────────────────────────────
    # Separate rqt_image_view processes (own Qt event loop each) so that
    # image display doesn't starve RViz's main thread on WSLg d3d12.
    camera_views = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_image_view", "rqt_image_view",
                     "/camera/image_raw"],
                name="rqt_image_raw",
                output="log",
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_image_view", "rqt_image_view",
                     "/camera/debug_image"],
                name="rqt_image_debug",
                output="log",
            ),
        ],
    )

    return LaunchDescription([
        map_arg,
        world_arg,
        gui_arg,
        libgl_hw,
        gallium_d3d12,
        ogre_rtt,
        mesa_adapter,
        gazebo_model_path,
        gazebo_plugin_path,
        gazebo_resource_path,
        gazebo,
        state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        wheel_controller,
        cmd_vel_bridge,
        map_server,
        amcl,
        lifecycle_localization,
        nav2,
        rviz,
        camera_views,
    ])
