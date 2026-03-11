"""Calibration launch — CAN buses + interactive calibration tool.

Usage:
    ros2 launch biped_bringup calibrate.launch.py side:=right   # calibrate right leg (can0, ID 1-6)
    ros2 launch biped_bringup calibrate.launch.py side:=left    # calibrate left leg (can1, ID 7-12)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAN0_MOTORS = "R_hip_pitch:1:RS04,R_hip_roll:2:RS03,R_hip_yaw:3:RS03,R_knee:4:RS04,R_foot_pitch:5:RS02,R_foot_roll:6:RS02"
CAN1_MOTORS = "L_hip_pitch:7:RS04,L_hip_roll:8:RS03,L_hip_yaw:9:RS03,L_knee:10:RS04,L_foot_pitch:11:RS02,L_foot_roll:12:RS02"


def launch_setup(context):
    side = LaunchConfiguration('side').perform(context)

    if side == 'left':
        iface = 'can1'
        motors = CAN1_MOTORS
        output = 'calibration_left.yaml'
    else:
        iface = 'can0'
        motors = CAN0_MOTORS
        output = 'calibration_right.yaml'

    return [
        Node(
            package='biped_tools',
            executable='calibrate_node',
            name='calibrate_node',
            output='screen',
            prefix='xterm -e' if False else '',
            parameters=[{
                'can_interface': iface,
                'motor_config': motors,
                'output_file': output,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right',
                              description='Which leg to calibrate: right (can0) or left (can1)'),
        OpaqueFunction(function=launch_setup),
    ])
