from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.1.100'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('scan_period_sec', default_value='2.0'),
        DeclareLaunchArgument('rate_window_sec', default_value='5.0'),
        DeclareLaunchArgument('subscribe_all', default_value='true'),
        Node(
            package='ros_web_monitor',
            executable='monitor_node',
            name='ros_web_monitor',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'scan_period_sec': LaunchConfiguration('scan_period_sec'),
                'rate_window_sec': LaunchConfiguration('rate_window_sec'),
                'subscribe_all': LaunchConfiguration('subscribe_all'),
            }],
        ),
    ])
