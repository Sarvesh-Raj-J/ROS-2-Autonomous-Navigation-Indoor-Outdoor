from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gps_imu_topic = LaunchConfiguration('gps_imu_topic')
    base_imu_topic = LaunchConfiguration('base_imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    report_period = LaunchConfiguration('report_period')
    log_to_console = LaunchConfiguration('log_to_console')

    return LaunchDescription([
        DeclareLaunchArgument('gps_imu_topic', default_value='/gps/imu'),
        DeclareLaunchArgument('base_imu_topic', default_value='/imu'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('report_period', default_value='1.0'),
        DeclareLaunchArgument('log_to_console', default_value='true'),
        Node(
            package='navigation',
            executable='yaw_debug_monitor',
            name='yaw_debug_monitor',
            output='screen',
            parameters=[{
                'gps_imu_topic': gps_imu_topic,
                'base_imu_topic': base_imu_topic,
                'odom_topic': odom_topic,
                'report_period': report_period,
                'log_to_console': log_to_console,
            }],
        ),
    ])
