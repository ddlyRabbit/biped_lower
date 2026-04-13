"""Hardware-only launch — IMU + CAN + safety + robot_state_publisher, no policy.

For testing hardware before policy integration.

Usage:
    ros2 launch biped_bringup hardware.launch.py
    ros2 launch biped_bringup hardware.launch.py imu_type:=im10a
    ros2 launch biped_bringup hardware.launch.py calibration_file:=calibration.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_imu_node(context):
    """Select IMU driver based on imu_type arg."""
    imu_type = LaunchConfiguration('imu_type').perform(context)
    if imu_type == 'im10a':
        return [Node(
            package='biped_driver', executable='im10a_imu_node',
            name='imu_node', output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baudrate': 460800, 'rate_hz': 300.0}],
        )]
    elif imu_type == 'bno085_cpp':
        return [Node(
            package='biped_driver_cpp', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 200.0, 'i2c_address': 74, 'reset_pin': 4}],
        )]
    else:  # bno085 (default)
        return [Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 50.0, 'i2c_address': 74, 'reset_pin': 4}],
        )]


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
        DeclareLaunchArgument('imu_type', default_value='bno085_cpp',
                              description='IMU driver: bno085_cpp (I2C) | bno085 (Python) | im10a (USB serial)'),
        DeclareLaunchArgument('max_pitch_deg', default_value='85.0'),
        DeclareLaunchArgument('max_roll_deg', default_value='85.0'),

        # Robot description
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # IMU — selected by imu_type arg (bno085 default, im10a for USB IMU)
        OpaqueFunction(function=_make_imu_node),

        # CAN bus
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_bus_node', output='screen',
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config'),
                'calibration_file': LaunchConfiguration('calibration_file'),
                'loop_rate': 50.0,
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
    ])
