"""Full robot bringup — all nodes.

Usage:
    ros2 launch biped_bringup bringup.launch.py
    ros2 launch biped_bringup bringup.launch.py gain_scale:=0.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Motor config strings (fallback if no robot_config YAML)
CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')
    default_robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('onnx_model', default_value='student_flat.onnx'),
        DeclareLaunchArgument('gain_scale', default_value='1.0'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=default_robot_config),

        # IMU
        Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 100.0, 'i2c_address': 75, 'reset_pin': 4}],
        ),

        # CAN bus — single node managing both buses (can0=right, can1=left)
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_bus_node', output='screen',
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config'),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'loop_rate': 50.0,
                # Fallback if no robot_config:
                'motor_config_can0': CAN0_MOTORS,
                'motor_config_can1': CAN1_MOTORS,
            }],
        ),

        # Safety monitor
        Node(
            package='biped_control', executable='safety_node',
            name='safety_node', output='screen',
        ),

        # State machine
        Node(
            package='biped_control', executable='state_machine_node',
            name='state_machine_node', output='screen',
        ),

        # Policy inference
        Node(
            package='biped_control', executable='policy_node',
            name='policy_node', output='screen',
            parameters=[{
                'onnx_model': LaunchConfiguration('onnx_model'),
                'gain_scale': LaunchConfiguration('gain_scale'),
                'loop_rate': 50.0,
            }],
        ),
    ])
