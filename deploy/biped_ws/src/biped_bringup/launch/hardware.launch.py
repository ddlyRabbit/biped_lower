"""Hardware-only launch — IMU + CAN, no policy.

For testing hardware before policy integration.

Dual bus (MCP2515 HAT):
    ros2 launch biped_bringup hardware.launch.py

Single bus (USB-CAN adapter):
    ros2 launch biped_bringup hardware.launch.py bus_mode:=single
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fallback motor config strings
CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"
ALL_MOTORS = CAN0_MOTORS + "," + CAN1_MOTORS


def launch_setup(context):
    bringup_dir = get_package_share_directory('biped_bringup')
    bus_mode = LaunchConfiguration('bus_mode').perform(context)
    cal_file = LaunchConfiguration('calibration_file').perform(context)

    if bus_mode == 'single':
        robot_config = os.path.join(bringup_dir, 'config', 'robot_single_bus.yaml')
        motor_params = {
            'robot_config': robot_config,
            'calibration_file': cal_file,
            'loop_rate': 50.0,
            'motor_config_can0': ALL_MOTORS,
        }
    else:
        robot_config_override = LaunchConfiguration('robot_config').perform(context)
        if robot_config_override:
            robot_config = robot_config_override
        else:
            robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')
        motor_params = {
            'robot_config': robot_config,
            'calibration_file': cal_file,
            'loop_rate': 50.0,
            'motor_config_can0': CAN0_MOTORS,
            'motor_config_can1': CAN1_MOTORS,
        }

    return [
        Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 50.0, 'i2c_address': 75, 'reset_pin': 4}],
        ),
        Node(
            package='biped_driver', executable='can_bus_node',
            name='can_bus_node', output='screen',
            parameters=[motor_params],
        ),
        Node(
            package='biped_control', executable='safety_node',
            name='safety_node', output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bus_mode', default_value='dual',
                              description='CAN bus mode: dual (MCP2515 HAT) or single (USB-CAN)'),
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
