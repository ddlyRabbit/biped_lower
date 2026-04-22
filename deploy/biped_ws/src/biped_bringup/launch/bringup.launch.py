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
    '/policy_viz',
    '/policy_viz_joints',
    '/tf',
]


def _make_imu_node(context):
    """Select IMU driver based on imu_type arg."""
    unified = LaunchConfiguration('unified').perform(context)
    if unified == 'true':
        return []  # IMU handled by unified node
    
    imu_type = LaunchConfiguration('imu_type').perform(context)
    
    if imu_type == 'im10a':
        return [Node(
            package='biped_driver', executable='im10a_imu_node',
            name='imu_node', output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('im10a_port'), 
                'baudrate': LaunchConfiguration('im10a_baud'), 
                'rate_hz': 200.0
            }],
        )]
    elif imu_type == 'im10a_cpp':
        return [Node(
            package='biped_driver_cpp', executable='im10a_node',
            name='imu_node', output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('im10a_port'), 
                'baud_rate': LaunchConfiguration('im10a_baud'), 
                'rate_hz': 200.0
            }],
        )]
    elif imu_type == 'bno085_cpp':
        return [Node(
            package='biped_driver_cpp', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 200.0, 'i2c_bus': 1, 'i2c_address': 75, 'reset_pin': 4}],
        )]
    else:  # bno085 (default)
        return [Node(
            package='biped_driver', executable='imu_node',
            name='imu_node', output='screen',
            parameters=[{'rate_hz': 100.0, 'i2c_address': 75, 'reset_pin': 4}],
        )]


def _make_can_driver_node(context):
    """Select CAN driver package based on can_driver arg."""
    unified = LaunchConfiguration('unified').perform(context)
    if unified == 'true':
        return []  # CAN handled by unified node
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
            'loop_rate': 50.0,
            'publish_rate': 50.0,
        }],
    )]


def _make_control_nodes(context):
    """Select control package based on control_driver arg."""
    unified = LaunchConfiguration('unified').perform(context)
    driver = LaunchConfiguration('control_driver').perform(context)

    if driver == 'biped_control_cpp':
        pkg = 'biped_control_cpp'
        sm_exe = 'state_machine_node_cpp'
        sf_exe = 'safety_node_cpp'
        pl_exe = 'policy_node_cpp'
    else:
        pkg = 'biped_control'
        sm_exe = 'state_machine_node'
        sf_exe = 'safety_node'
        pl_exe = 'policy_node'

    nodes = [
        Node(
            package=pkg, executable=sf_exe,
            name='safety_node', output='screen',
            parameters=[{
                'max_pitch_deg': LaunchConfiguration('max_pitch_deg'),
                'max_roll_deg': LaunchConfiguration('max_roll_deg'),
            }],
        ),
        Node(
            package=pkg, executable=sm_exe,
            name='state_machine_node', output='screen',
            parameters=[{
                'wiggle_config': os.path.join(get_package_share_directory('biped_bringup'), 'config', 'wiggle.yaml'),
                'gain_scale': LaunchConfiguration('gain_scale'),
            }],
        ),
    ]

    # Unified node replaces IMU + CAN + policy; otherwise use separate policy node
    if unified == 'true':
        nodes.append(Node(
            package='biped_unified_cpp', executable='unified_node',
            name='unified_node', output='screen',
            prefix=['taskset -c 1'],
            parameters=[{
                'robot_config': LaunchConfiguration('robot_config').perform(context),
                'calibration_file': LaunchConfiguration('calibration_file').perform(context),
                'onnx_model': LaunchConfiguration('onnx_model'),
                'gain_scale': LaunchConfiguration('gain_scale'),
                'loop_rate': 50.0,
                'imu_type': LaunchConfiguration('imu_type'),
                'im10a_port': LaunchConfiguration('im10a_port'),
                'im10a_baud': LaunchConfiguration('im10a_baud'),
                'i2c_bus': 1,
                'i2c_address': 75,
                'imu_rate_hz': 200.0,
                'imu_reset_pin': 4,
            }],
        ))
    else:
        nodes.append(Node(
            package=pkg, executable=pl_exe,
            name='policy_node', output='screen',
            prefix=['taskset -c 2'],
            parameters=[{
                'onnx_model': LaunchConfiguration('onnx_model'),
                'gain_scale': LaunchConfiguration('gain_scale'),
            }],
            remappings=[
                ('joint_states', '/policy_viz_joints'),
            ],
        ))

    return nodes


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
                              description='CAN node: can_bus_node | can_bus_node_cpp'),
        DeclareLaunchArgument('imu_type', default_value='bno085',
                              description='IMU: bno085 | bno085_cpp | im10a | im10a_cpp'),
        DeclareLaunchArgument('im10a_port', default_value='/dev/ttyUSB0',
                              description='Serial port for IM10A'),
        DeclareLaunchArgument('im10a_baud', default_value='460800',
                              description='Baud rate for IM10A'),
                              description='CAN driver: can_bus_node | can_bus_node_async | can_bus_node_cpp'),
        DeclareLaunchArgument('control_driver', default_value='biped_control',
                              description='Control package: biped_control | biped_control_cpp'),
        DeclareLaunchArgument('unified', default_value='false',
                              description='Use unified sense+control node (replaces imu+can+policy)'),
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

        # Control Nodes (Safety, State Machine, Policy)
        OpaqueFunction(function=_make_control_nodes),

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
