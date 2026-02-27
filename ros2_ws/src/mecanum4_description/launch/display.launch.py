from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('mecanum4_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mecanum4_lidar.urdf')
    with open(urdf_file, encoding='utf-8') as urdf:
        robot_description = urdf.read()

    odom_port_arg = DeclareLaunchArgument('odom_port', default_value='/dev/ttyACM0')
    odom_baudrate_arg = DeclareLaunchArgument('odom_baudrate', default_value='2000000')
    odom_wheel_radius_arg = DeclareLaunchArgument('odom_wheel_radius', default_value='0.06')
    odom_lx_arg = DeclareLaunchArgument('odom_lx', default_value='0.30')
    odom_ly_arg = DeclareLaunchArgument('odom_ly', default_value='0.17')

    return LaunchDescription([
        odom_port_arg,
        odom_baudrate_arg,
        odom_wheel_radius_arg,
        odom_lx_arg,
        odom_ly_arg,

        Node(
            package='robocon_tutorial',
            executable='mecanum_serial_odometry',
            name='mecanum_serial_odometry',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('odom_port'),
                'baudrate': LaunchConfiguration('odom_baudrate'),
                'wheel_radius': LaunchConfiguration('odom_wheel_radius'),
                'lx': LaunchConfiguration('odom_lx'),
                'ly': LaunchConfiguration('odom_ly'),
            }],
        ),

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
