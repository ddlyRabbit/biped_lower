"""Policy test — full bringup with zero velocity (suspended robot test).

⚠️ Robot must be hanging / feet off ground!

Usage:
    ros2 launch biped_bringup policy_test.launch.py onnx_model:=student_flat.onnx gain_scale:=0.3
    
Then in another terminal:
    ros2 topic pub /state_command std_msgs/String "data: START"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('onnx_model', default_value='student_flat.onnx'),
        DeclareLaunchArgument('gain_scale', default_value='0.3'),
        DeclareLaunchArgument('calibration_file', default_value=''),

        # Full bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup.launch.py')
            ),
            launch_arguments={
                'onnx_model': LaunchConfiguration('onnx_model'),
                'gain_scale': LaunchConfiguration('gain_scale'),
                'calibration_file': LaunchConfiguration('calibration_file'),
            }.items(),
        ),

        # Foxglove bridge for monitoring
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': 8765, 'address': '0.0.0.0'}],
        ),
    ])
