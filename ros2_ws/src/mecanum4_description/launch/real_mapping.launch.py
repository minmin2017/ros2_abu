import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory("mecanum4_description")
    ydlidar_pkg = get_package_share_directory("ydlidar_ros2_driver")
    
    urdf_file = os.path.join(pkg_path, "urdf", "mecanum4_lidar.urdf")
    slam_params = os.path.join(pkg_path, "config", "slam_params_real.yaml")
    lidar_params = os.path.join(ydlidar_pkg, "params", "ydlidar.yaml")
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

    # 1.5 Static TF (Fix for base not found)
    # เชื่อม base_footprint (จุดบนพื้น) เข้ากับ base_link (จุดศูนย์กลางหุ่น)
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # 2. YDLidar Driver — S2PRO is single-channel; load full params file then override port
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params, {
            'port': '/dev/ydlidar',
            'frame_id': 'laser_frame',
            'use_sim_time': False,
        }],
    )

    # 3. Arduino Serial Bridge
    arduino_node = Node(
        package='mecanum4_description',
        executable='cmd_vel_to_arduino.py',
        name='cmd_vel_to_arduino',
        parameters=[{
            'port': '/dev/ttyUSB1', # /dev/arduino symlink หาย ใช้ raw tty ไปก่อน
            'baudrate': 250000
        }]
    )

    # 3.5 Scan Filter (/scan -> /scan_filtered) — slam_params_real.yaml subscribes to /scan_filtered
    scan_filter_node = Node(
        package='mecanum4_description',
        executable='scan_filter.py',
        name='scan_filter_node',
        output='screen',
    )

    # 4. SLAM Toolbox
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": False}],
    )

    # 5. RViz2
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
        arduino_node,
        scan_filter_node,
        slam_toolbox,
        rviz,
    ])
