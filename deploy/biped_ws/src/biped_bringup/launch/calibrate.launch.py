"""Calibration launch — interactive calibration tool.

Calibrate_node manages its own CAN bus directly (no can_bus_node needed).

Usage:
    ros2 launch biped_bringup calibrate.launch.py
    ros2 launch biped_bringup calibrate.launch.py output_file:=calibration.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_channel', default_value='can0'),
        DeclareLaunchArgument('output_file', default_value='calibration.yaml'),

        # Interactive calibration tool (manages CAN directly)
        Node(
            package='biped_tools', executable='calibrate_node',
            name='calibrate_node', output='screen',
            parameters=[{
                'can_channel': LaunchConfiguration('can_channel'),
                'output_file': LaunchConfiguration('output_file'),
            }],
        ),
    ])
