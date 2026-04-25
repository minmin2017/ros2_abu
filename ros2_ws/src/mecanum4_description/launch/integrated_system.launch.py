import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory("mecanum4_description")
    ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
    
    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    slam_params = os.path.join(pkg_path, "config", "slam_params_real.yaml")
    nav2_params = os.path.join(pkg_path, "config", "nav2_params_real.yaml")
    rviz_config = os.path.join(pkg_path, "config", "nav2_view.rviz")

    with open(urdf_file, encoding="utf-8") as f:
        robot_description = ' '.join(f.read().split())

    # 1. Robot State Publisher
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

    # 2. Static TF (base_footprint -> base_link)
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # 3. YDLidar Driver
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': '/dev/ydlidar',
            'frame_id': 'laser_frame',
            'baudrate': 128000,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 5,
            'range_max': 12.0,
            'range_min': 0.1, # Min range 0.1m but we will filter in scan_filter
            'use_sim_time': False
        }],
    )

    # 4. Scan Filter Node
    scan_filter_node = Node(
        package='mecanum4_description',
        executable='scan_filter.py',
        name='scan_filter_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 5. Arduino Serial Bridge
    arduino_node = Node(
        package='mecanum4_description',
        executable='cmd_vel_to_arduino.py',
        name='cmd_vel_to_arduino',
        parameters=[{
            'port': '/dev/arduino',
            'baudrate': 250000
        }]
    )

    # 6. SLAM Toolbox (Using filtered scan)
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": False}],
    )

    # 7. Nav2 Stack (Using filtered scan via nav2_params_real.yaml)
    nav2_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {"node_names": nav2_lifecycle_nodes},
            ],
        ),
    ]

    # 8. RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": False}],
        output="screen",
    )

    return LaunchDescription([
        state_publisher,
        static_tf_base,
        ydlidar_node,
        scan_filter_node,
        arduino_node,
        slam_toolbox,
        *nav2_nodes,
        rviz,
    ])
