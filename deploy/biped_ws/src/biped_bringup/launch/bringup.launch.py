"""Full robot bringup — all nodes.

Usage:
    ros2 launch biped_bringup bringup.launch.py
    ros2 launch biped_bringup bringup.launch.py gain_scale:=0.3
    ros2 launch biped_bringup bringup.launch.py calibration_file:=calibration.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CAN0_MOTORS = (  # Right leg
    "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,"
    "R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
)
CAN1_MOTORS = (  # Left leg
    "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,"
    "L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"
)


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')
    description_dir = get_package_share_directory('biped_description')
    default_robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')
    urdf_path = os.path.join(description_dir, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()
    robot_description = ParameterValue(urdf_xml, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=default_robot_config),
        DeclareLaunchArgument('onnx_model', default_value='student_flat.onnx'),
        DeclareLaunchArgument('gain_scale', default_value='1.0'),
        DeclareLaunchArgument('max_pitch_deg', default_value='85.0'),
        DeclareLaunchArgument('max_roll_deg', default_value='85.0'),

        # Robot description
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # IMU
        Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 50.0, 'i2c_address': 75, 'reset_pin': 4}],
        ),

        # CAN bus — right leg on can0, left leg on can1
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_bus_node', output='screen',
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config'),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'loop_rate': 50.0,
                'motor_config_can0': '',
            }],
        ),

        # Safety
        Node(
            package='biped_control', executable='safety_node',
            name='safety_node', output='screen',
            parameters=[{
                'max_pitch_deg': LaunchConfiguration('max_pitch_deg'),
                'max_roll_deg': LaunchConfiguration('max_roll_deg'),
                
            }],
        ),

        # State machine
        Node(
            package='biped_control', executable='state_machine_node',
            name='state_machine_node', output='screen',
        ),

        # Policy
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
