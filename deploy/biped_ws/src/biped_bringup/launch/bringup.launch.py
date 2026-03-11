"""Full robot bringup — all nodes.

Usage:
    ros2 launch biped_bringup bringup.launch.py
    ros2 launch biped_bringup bringup.launch.py gain_scale:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Motor config strings (locked)
CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('onnx_model', default_value='student_flat.onnx'),
        DeclareLaunchArgument('gain_scale', default_value='1.0'),
        DeclareLaunchArgument('calibration_file', default_value=''),

        # IMU
        Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 100.0, 'i2c_address': 75, 'reset_pin': 4}],
        ),

        # CAN bus — Right leg (can0, ID 1-6)
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_right', output='screen',
            parameters=[{
                'can_interface': 'can0',
                'motor_config': CAN0_MOTORS,
                'loop_rate': 50.0,
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),

        # CAN bus — Left leg (can1, ID 7-12)
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_left', output='screen',
            parameters=[{
                'can_interface': 'can1',
                'motor_config': CAN1_MOTORS,
                'loop_rate': 50.0,
                'calibration_file': LaunchConfiguration('calibration_file'),
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
