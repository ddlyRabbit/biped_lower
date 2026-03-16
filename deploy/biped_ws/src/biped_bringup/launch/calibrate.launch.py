"""Calibration launch — interactive calibration tool.

Calibrate_node manages its own CAN bus directly (no can_bus_node needed).
Motor config (IDs, buses) comes from robot.yaml.

Usage:
    ros2 launch biped_bringup calibrate.launch.py
    ros2 launch biped_bringup calibrate.launch.py joint:=R_knee
    ros2 launch biped_bringup calibrate.launch.py output_file:=calibration.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')
    default_robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=default_robot_config),
        DeclareLaunchArgument('output_file', default_value='calibration.yaml'),
        DeclareLaunchArgument('joint', default_value='',
                              description='Single joint to recalibrate (merges into existing file). Empty = all.'),

        Node(
            package='biped_tools', executable='calibrate_node',
            name='calibrate_node', output='screen',
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config'),
                'output_file': LaunchConfiguration('output_file'),
                'joint': LaunchConfiguration('joint'),
            }],
        ),
    ])
