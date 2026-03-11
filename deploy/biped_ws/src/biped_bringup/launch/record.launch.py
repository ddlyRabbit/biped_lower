"""Foxglove bridge + rosbag recording.

Usage:
    ros2 launch biped_bringup record.launch.py
    
Connect Foxglove Studio on desktop to: ws://<pi-ip>:8765
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Foxglove WebSocket bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000,
            }],
        ),

        # Rosbag recording (all topics)
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a',
                 '-o', 'biped_recording'],
            output='screen',
        ),
    ])
