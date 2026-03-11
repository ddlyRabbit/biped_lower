"""ROS2 CAN bus node for RobStride RS02/RS03/RS04 motors.

Manages multiple motors on one CAN interface. Publishes joint states,
subscribes to MIT commands. One instance per CAN bus (left/right leg).

Usage:
    ros2 run biped_driver can_bus_node --ros-args \
        -p can_interface:=can0 \
        -p motor_config:="L_hip_pitch:1:RS04,L_hip_roll:2:RS03,..."
"""

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommand, MITCommandArray, MotorState, MotorStateArray
from biped_driver.robstride_can import RobStrideMotor, MotorFeedback, MODE_MIT


class CanBusNode(Node):
    """ROS2 node for multi-motor CAN communication.

    Publishes:
        /joint_states    sensor_msgs/JointState     @ loop_rate
        /motor_states    biped_msgs/MotorStateArray  @ loop_rate

    Subscribes:
        /joint_commands  biped_msgs/MITCommandArray
    """

    def __init__(self):
        super().__init__('can_bus_node')

        # Parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('master_id', 253)
        self.declare_parameter('calibration_file', '')
        # Motor config: "joint_name:can_id:type,joint_name:can_id:type,..."
        self.declare_parameter('motor_config', '')

        self._iface = str(self.get_parameter('can_interface').value)
        self._rate = float(self.get_parameter('loop_rate').value)
        self._master_id = int(self.get_parameter('master_id').value)
        self._cal_file = str(self.get_parameter('calibration_file').value)
        motor_config_str = str(self.get_parameter('motor_config').value)

        # Parse motor config
        self._motors = {}       # {joint_name: RobStrideMotor}
        self._joint_names = []  # ordered list
        self._motor_ids = {}    # {joint_name: can_id}
        self._motor_types = {}  # {joint_name: actuator_type}
        self._offsets = {}      # {joint_name: calibration offset}
        self._last_commands = {}  # {joint_name: MITCommand}

        if motor_config_str:
            self._parse_motor_config(motor_config_str)

        # Load calibration offsets
        self._load_calibration()

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self._pub_joints = self.create_publisher(JointState, '/joint_states', sensor_qos)
        self._pub_motors = self.create_publisher(MotorStateArray, '/motor_states', sensor_qos)

        # Subscriber
        self._sub_commands = self.create_subscription(
            MITCommandArray, '/joint_commands', self._cmd_callback, 10)

        # Initialize motors
        self._init_motors()

        # Timer
        self._timer = self.create_timer(1.0 / self._rate, self._loop)

        self.get_logger().info(
            f'CAN bus node started — {self._iface}, {len(self._motors)} motors, '
            f'{self._rate}Hz'
        )

    def _parse_motor_config(self, config_str: str):
        """Parse motor config string: "name:id:type,name:id:type,..." """
        for entry in config_str.split(','):
            entry = entry.strip()
            if not entry:
                continue
            parts = entry.split(':')
            if len(parts) != 3:
                self.get_logger().error(f'Invalid motor config entry: {entry}')
                continue
            name, mid, mtype = parts[0].strip(), int(parts[1].strip()), parts[2].strip()
            self._joint_names.append(name)
            self._motor_ids[name] = mid
            self._motor_types[name] = mtype

    def _load_calibration(self):
        """Load encoder offsets from calibration YAML."""
        if not self._cal_file:
            self.get_logger().info('No calibration file — using zero offsets')
            for name in self._joint_names:
                self._offsets[name] = 0.0
            return

        try:
            with open(self._cal_file) as f:
                cal = yaml.safe_load(f) or {}
            for name in self._joint_names:
                self._offsets[name] = cal.get(name, {}).get('offset', 0.0)
            self.get_logger().info(f'Loaded calibration from {self._cal_file}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}, using zero offsets')
            for name in self._joint_names:
                self._offsets[name] = 0.0

    def _init_motors(self):
        """Create motor objects and enable them."""
        for name in self._joint_names:
            try:
                motor = RobStrideMotor(
                    self._iface,
                    self._motor_ids[name],
                    self._motor_types[name],
                    self._master_id,
                )
                self._motors[name] = motor
                self.get_logger().info(
                    f'  {name}: ID={self._motor_ids[name]}, '
                    f'type={self._motor_types[name]}, offset={self._offsets[name]:.3f}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to create motor {name}: {e}')

    def enable_all(self):
        """Enable all motors and set MIT mode."""
        for name, motor in self._motors.items():
            try:
                motor.set_mode(MODE_MIT)
                fb = motor.enable()
                if fb:
                    self.get_logger().info(f'  {name}: enabled, pos={fb.position:.3f}')
                else:
                    self.get_logger().warn(f'  {name}: enable — no response')
            except Exception as e:
                self.get_logger().error(f'  {name}: enable failed: {e}')

    def disable_all(self):
        """Disable all motors."""
        for name, motor in self._motors.items():
            try:
                motor.disable()
            except Exception:
                pass

    def _cmd_callback(self, msg: MITCommandArray):
        """Store latest MIT commands for each joint."""
        for cmd in msg.commands:
            if cmd.joint_name in self._motors:
                self._last_commands[cmd.joint_name] = cmd

    def _loop(self):
        """Main control loop: send commands, read feedback, publish."""
        now = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = now
        motor_msg = MotorStateArray()
        motor_msg.header.stamp = now

        for name in self._joint_names:
            motor = self._motors.get(name)
            if motor is None:
                continue

            # Send MIT command if available, otherwise just read
            fb = None
            try:
                if name in self._last_commands:
                    cmd = self._last_commands[name]
                    fb = motor.send_mit_command(
                        position=cmd.position,
                        velocity=cmd.velocity,
                        kp=cmd.kp,
                        kd=cmd.kd,
                        torque_ff=cmd.torque_ff,
                    )
                else:
                    # No command yet — send zero-torque read
                    fb = motor.send_mit_command(kp=0.0, kd=0.0, torque_ff=0.0)
            except Exception as e:
                self.get_logger().warn(
                    f'{name}: CAN error: {e}', throttle_duration_sec=1.0)

            if fb is not None:
                # Apply calibration offset
                calibrated_pos = fb.position - self._offsets[name]

                # JointState
                joint_msg.name.append(name)
                joint_msg.position.append(calibrated_pos)
                joint_msg.velocity.append(fb.velocity)
                joint_msg.effort.append(fb.torque)

                # MotorState
                ms = MotorState()
                ms.joint_name = name
                ms.can_id = self._motor_ids[name]
                ms.position = calibrated_pos
                ms.velocity = fb.velocity
                ms.torque = fb.torque
                ms.temperature = fb.temperature
                ms.fault_code = fb.fault_code
                ms.mode_status = fb.mode_status
                motor_msg.motors.append(ms)

        self._pub_joints.publish(joint_msg)
        self._pub_motors.publish(motor_msg)

    def destroy_node(self):
        """Cleanup: disable motors and close sockets."""
        self.get_logger().info('Shutting down — disabling motors')
        self.disable_all()
        for motor in self._motors.values():
            motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBusNode()
    try:
        # Enable motors on startup
        node.enable_all()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
