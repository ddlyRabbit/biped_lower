"""Teleop launch — keyboard or gamepad.

Usage:
    ros2 launch biped_bringup teleop.launch.py                    # keyboard (default)
    ros2 launch biped_bringup teleop.launch.py mode:=gamepad      # BT gamepad
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    nodes = []

    if mode == 'gamepad':
        nodes.append(Node(
            package='joy', executable='joy_node',
            name='joy_node', output='screen',
        ))
        # TODO: gamepad_teleop node when implemented
    else:
        nodes.append(Node(
            package='biped_teleop', executable='keyboard_teleop',
            name='keyboard_teleop', output='screen',
            prefix='xterm -e' if mode == 'xterm' else '',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='keyboard',
                              description='Teleop mode: keyboard, gamepad, xterm'),
        OpaqueFunction(function=launch_setup),
    ])
