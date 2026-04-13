"""Full robot bringup — all nodes + automatic rosbag recording.

Records MCAP bags to ~/biped_lower/bags/<timestamp>/ on every run.
Open .mcap files directly in Foxglove Studio for playback.

Usage:
    ros2 launch biped_bringup bringup.launch.py
    ros2 launch biped_bringup bringup.launch.py gain_scale:=0.3
    ros2 launch biped_bringup bringup.launch.py calibration_file:=calibration.yaml
    ros2 launch biped_bringup bringup.launch.py record:=false  # disable recording
    ros2 launch biped_bringup bringup.launch.py can_driver:=can_bus_node_cpp  # C++ driver
"""

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Topics to record (high-value, skip static/verbose topics)
RECORD_TOPICS = [
    '/joint_states',
    '/motor_states',
    '/joint_commands',
    '/imu/data',
    '/imu/gravity',
    '/cmd_vel',
    '/state_machine',
    '/robot_state',
    '/safety/status',
    '/safety/fault',
    '/tf',
]


def _make_imu_node(context):
    """Select IMU driver based on imu_type arg."""
    imu_type = LaunchConfiguration('imu_type').perform(context)
    if imu_type == 'im10a':
        return [Node(
            package='biped_driver', executable='im10a_imu_node',
            name='imu_node', output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'baudrate': 460800, 'rate_hz': 300.0}],
        )]
    else:  # bno085 (default)
        return [Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 100.0, 'i2c_address': 75, 'reset_pin': 4}],
        )]


def _make_can_driver_node(context):
    """Select CAN driver package based on can_driver arg."""
    driver = LaunchConfiguration('can_driver').perform(context)
    if driver == 'can_bus_node_cpp':
        pkg = 'biped_driver_cpp'
        exe = 'can_bus_node_cpp'
    else:
        pkg = 'biped_driver'
        exe = driver
    return [Node(
        package=pkg,
        executable=exe,
        name=exe, output='screen',
        prefix=['taskset -c 1'],
        parameters=[{
            'robot_config': LaunchConfiguration('robot_config').perform(context),
            'calibration_file': LaunchConfiguration('calibration_file').perform(context),
            'loop_rate': 200.0,
            'publish_rate': 200.0,
        }],
    )]


def generate_launch_description():
    bringup_dir = get_package_share_directory('biped_bringup')
    description_dir = get_package_share_directory('biped_description')
    default_robot_config = os.path.join(bringup_dir, 'config', 'robot.yaml')
    urdf_path = os.path.join(description_dir, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        urdf_xml = f.read()
    robot_description = ParameterValue(urdf_xml, value_type=str)

    bag_dir = os.path.expanduser(
        f'~/biped_lower/bags/{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')

    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        DeclareLaunchArgument('robot_config', default_value=default_robot_config),
        DeclareLaunchArgument('can_driver', default_value='can_bus_node',
                              description='CAN driver: can_bus_node | can_bus_node_async | can_bus_node_cpp'),
        DeclareLaunchArgument('onnx_model', default_value='student_flat.onnx'),
        DeclareLaunchArgument('gain_scale', default_value='1.0'),
        DeclareLaunchArgument('max_pitch_deg', default_value='85.0'),
        DeclareLaunchArgument('max_roll_deg', default_value='85.0'),
        DeclareLaunchArgument('imu_type', default_value='bno085',
                              description='IMU driver: bno085 | im10a'),
        DeclareLaunchArgument('record', default_value='true',
                              description='Enable rosbag recording (MCAP format)'),

        # Robot description (URDF → /tf, /tf_static, /robot_description)
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Policy ghost — second URDF model driven by policy output (SIM_WALK viz)
        # In Foxglove 3D panel: add second URDF display with frame_prefix "policy/"
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='policy_state_publisher', output='log',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': 'policy/',
            }],
            remappings=[
                ('joint_states', '/policy_viz_joints'),
            ],
        ),

        # IMU — selected by imu_type launch arg
        OpaqueFunction(function=_make_imu_node),

        # CAN bus — motor config from robot.yaml
        # Package depends on driver: can_bus_node_cpp uses biped_driver_cpp
        OpaqueFunction(function=_make_can_driver_node),

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
            parameters=[{
                'gain_scale': LaunchConfiguration('gain_scale'),
                'wiggle_config': os.path.join(os.path.expanduser('~'), 'biped_lower', 'deploy', 'biped_ws', 'src', 'biped_bringup', 'config', 'wiggle.yaml'),
                'trajectory_file': os.path.join(os.path.expanduser('~'), 'biped_lower', 'deploy', 'biped_ws', 'src', 'biped_bringup', 'config', 'trajectory.csv'),
            }],
        ),

        # Policy
        Node(
            package='biped_control', executable='policy_node',
            prefix=['taskset -c 2'],
            name='policy_node', output='screen',
            parameters=[{
                'onnx_model': LaunchConfiguration('onnx_model'),
                'gain_scale': LaunchConfiguration('gain_scale'),
                'loop_rate': 50.0,
            }],
        ),

        # Rosbag recording (MCAP format — open directly in Foxglove)
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record')),
            cmd=['ros2', 'bag', 'record',
                 '--storage', 'mcap',
                 '--max-bag-duration', '300',  # split every 5 min
                 '-o', bag_dir,
                 ] + RECORD_TOPICS,
            output='log',
        ),
    ])
