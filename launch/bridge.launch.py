"""ROS 2 launch file for the Refringence bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('token', description='Refringence connector token'),
        DeclareLaunchArgument('server_url', default_value='https://api.refringence.com',
                              description='Refringence server URL'),
        DeclareLaunchArgument('rate_hz', default_value='30',
                              description='Telemetry publishing rate (Hz)'),

        Node(
            package='refringence_bridge',
            executable='refringence-bridge',
            name='refringence_bridge',
            output='screen',
            parameters=[{
                'token': LaunchConfiguration('token'),
                'server_url': LaunchConfiguration('server_url'),
                'rate_hz': LaunchConfiguration('rate_hz'),
            }],
        ),
    ])
