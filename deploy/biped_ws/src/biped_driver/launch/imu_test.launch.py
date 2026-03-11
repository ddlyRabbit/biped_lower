"""Launch BNO085 IMU node for standalone testing.

Usage:
  ros2 launch biped_driver imu_test.launch.py
  ros2 launch biped_driver imu_test.launch.py rate_hz:=200.0 use_game_quaternion:=true

Verify output:
  ros2 topic echo /imu/data
  ros2 topic echo /imu/gravity
  ros2 topic hz /imu/data
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rate_hz', default_value='100.0',
                              description='IMU publish rate in Hz'),
        DeclareLaunchArgument('i2c_address', default_value='75',
                              description='BNO085 I2C address (decimal, default 75 = 0x4B)'),
        DeclareLaunchArgument('use_game_quaternion', default_value='false',
                              description='Use game quaternion (no mag correction)'),

        Node(
            package='biped_driver',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'rate_hz': LaunchConfiguration('rate_hz'),
                'i2c_address': LaunchConfiguration('i2c_address'),
                'use_game_quaternion': LaunchConfiguration('use_game_quaternion'),
                'frame_id': 'imu_link',
            }],
        ),
    ])
