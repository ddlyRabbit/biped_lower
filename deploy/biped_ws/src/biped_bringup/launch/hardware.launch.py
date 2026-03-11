"""Hardware-only launch — IMU + CAN, no policy.

For testing hardware before policy integration.
    ros2 launch biped_bringup hardware.launch.py
    ros2 topic echo /joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),

        Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 50.0, 'i2c_address': 75, 'reset_pin': 4}],
        ),
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_right', output='screen',
            parameters=[{
                'can_interface': 'can0', 'motor_config': CAN0_MOTORS,
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_left', output='screen',
            parameters=[{
                'can_interface': 'can1', 'motor_config': CAN1_MOTORS,
                'calibration_file': LaunchConfiguration('calibration_file'),
            }],
        ),
        Node(
            package='biped_control', executable='safety_node',
            name='safety_node', output='screen',
        ),
    ])
