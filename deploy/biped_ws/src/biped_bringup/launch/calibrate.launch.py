"""Calibration launch — CAN buses + interactive calibration tool.

Single node manages both buses; calibrate_node walks through all 12 joints.

Usage:
    ros2 launch biped_bringup calibrate.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')
    default_robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=default_robot_config),
        DeclareLaunchArgument('output_file', default_value='calibration.yaml'),

        # CAN bus node — no calibration loaded (raw encoder values)
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_bus_node', output='screen',
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config'),
                'calibration_file': '',  # no offsets during calibration
                'loop_rate': 50.0,
                'motor_config_can0': CAN0_MOTORS,
                'motor_config_can1': CAN1_MOTORS,
            }],
        ),

        # Interactive calibration tool
        Node(
            package='biped_tools', executable='calibrate_node',
            name='calibrate_node', output='screen',
            parameters=[{
                'output_file': LaunchConfiguration('output_file'),
            }],
        ),
    ])
