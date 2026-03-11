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


"""Ankle parallel linkage mapping.

Two RS02 motors per ankle with mirrored crank arms, asymmetric rod lengths,
connecting through a U-joint (cross-bearing) to the foot.

Geometry (from Onshape CAD, confirmed):
    Crank radius:     32.249 mm (both motors)
    Upper rod:        192 mm (knee-side motor)
    Lower rod:        98 mm (foot-side motor)
    Motor separation: 88.5 mm vertical
    Foot attachment:  ±31.398 mm lateral, 41.14 mm forward from U-joint

Linearized mapping (verified <9% error at max pitch, <1% at typical walking):
    Commands (foot → motors):
        θ_upper = 1.276 × pitch + 0.974 × roll
        θ_lower = 1.276 × pitch - 0.974 × roll

    Feedback (motors → foot):
        pitch = 0.392 × (θ_upper + θ_lower)
        roll  = 0.514 × (θ_upper - θ_lower)

Motor IDs: foot_pitch ID → upper motor (knee-side), foot_roll ID → lower motor (foot-side).
"""

# Ankle linkage gains (from CAD geometry)
_PITCH_GAIN = 41.14 / 32.249   # 1.2757 — motor rad per foot pitch rad
_ROLL_GAIN = 31.398 / 32.249   # 0.9736 — motor rad per foot roll rad
_INV_PITCH = 32.249 / (2 * 41.14)   # 0.3919 — foot pitch per motor sum
_INV_ROLL = 32.249 / (2 * 31.398)   # 0.5136 — foot roll per motor diff

# Ankle parallel linkage pairs: (pitch_joint=upper_motor, roll_joint=lower_motor)
ANKLE_PAIRS = [
    ("L_foot_pitch", "L_foot_roll"),
    ("R_foot_pitch", "R_foot_roll"),
]


def ankle_command_to_motors(pitch_cmd: float, roll_cmd: float):
    """Convert foot pitch/roll targets to upper/lower motor positions.

    θ_upper = 1.276 × pitch + 0.974 × roll
    θ_lower = 1.276 × pitch - 0.974 × roll
    """
    motor_upper = _PITCH_GAIN * pitch_cmd + _ROLL_GAIN * roll_cmd
    motor_lower = _PITCH_GAIN * pitch_cmd - _ROLL_GAIN * roll_cmd
    return motor_upper, motor_lower


def ankle_motors_to_feedback(motor_upper_pos: float, motor_lower_pos: float):
    """Convert upper/lower motor feedback to foot pitch/roll.

    pitch = 0.392 × (θ_upper + θ_lower)
    roll  = 0.514 × (θ_upper - θ_lower)
    """
    foot_pitch = _INV_PITCH * (motor_upper_pos + motor_lower_pos)
    foot_roll = _INV_ROLL * (motor_upper_pos - motor_lower_pos)
    return foot_pitch, foot_roll


class CanBusNode(Node):
    """ROS2 node for multi-motor CAN communication.

    Handles ankle parallel linkage: policy sees foot_pitch/foot_roll,
    CAN sends to motor_upper/motor_lower with appropriate coupling.

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

    def _get_ankle_pair(self, name: str):
        """Check if this joint is part of an ankle pair. Returns (pitch_name, roll_name) or None."""
        for pitch, roll in ANKLE_PAIRS:
            if name == pitch or name == roll:
                if pitch in self._motors and roll in self._motors:
                    return (pitch, roll)
        return None

    def _loop(self):
        """Main control loop: send commands, read feedback, publish.

        Ankle joints get parallel linkage transform:
        - Commands: foot_pitch/roll → motor_high/low
        - Feedback: motor_high/low → foot_pitch/roll
        """
        now = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = now
        motor_msg = MotorStateArray()
        motor_msg.header.stamp = now

        # Track ankle motors processed (avoid double-processing)
        ankle_processed = set()
        # Raw motor feedback for ankle reconstruction
        raw_feedback = {}

        for name in self._joint_names:
            motor = self._motors.get(name)
            if motor is None:
                continue

            # --- Ankle parallel linkage: transform commands ---
            ankle_pair = self._get_ankle_pair(name)
            if ankle_pair and name in ankle_processed:
                continue  # already handled as part of pair

            if ankle_pair:
                pitch_name, roll_name = ankle_pair
                ankle_processed.add(pitch_name)
                ankle_processed.add(roll_name)

                # Get pitch/roll commands from policy
                pitch_cmd = self._last_commands.get(pitch_name)
                roll_cmd = self._last_commands.get(roll_name)

                if pitch_cmd and roll_cmd:
                    # Transform to motor positions
                    motor_upper_pos, motor_lower_pos = ankle_command_to_motors(
                        pitch_cmd.position, roll_cmd.position)
                    # Average gains (both ankles should have same gains)
                    kp = (pitch_cmd.kp + roll_cmd.kp) / 2.0
                    kd = (pitch_cmd.kd + roll_cmd.kd) / 2.0

                    # Send to high motor (pitch ID) and low motor (roll ID)
                    try:
                        fb_upper = self._motors[pitch_name].send_mit_command(
                            position=motor_upper_pos, velocity=0.0,
                            kp=kp, kd=kd, torque_ff=0.0)
                        fb_lower = self._motors[roll_name].send_mit_command(
                            position=motor_lower_pos, velocity=0.0,
                            kp=kp, kd=kd, torque_ff=0.0)
                    except Exception as e:
                        self.get_logger().warn(
                            f'Ankle {pitch_name}/{roll_name}: CAN error: {e}',
                            throttle_duration_sec=1.0)
                        fb_upper = fb_lower = None
                else:
                    # No commands — zero torque read
                    try:
                        fb_upper = self._motors[pitch_name].send_mit_command(
                            kp=0.0, kd=0.0, torque_ff=0.0)
                        fb_lower = self._motors[roll_name].send_mit_command(
                            kp=0.0, kd=0.0, torque_ff=0.0)
                    except Exception as e:
                        fb_upper = fb_lower = None

                # Reconstruct pitch/roll from motor feedback
                if fb_upper is not None and fb_lower is not None:
                    upper_pos = fb_upper.position - self._offsets.get(pitch_name, 0.0)
                    lower_pos = fb_lower.position - self._offsets.get(roll_name, 0.0)

                    foot_pitch, foot_roll = ankle_motors_to_feedback(upper_pos, lower_pos)
                    foot_pitch_vel, foot_roll_vel = ankle_motors_to_feedback(
                        fb_upper.velocity, fb_lower.velocity)

                    # Publish as foot_pitch and foot_roll (what policy expects)
                    for jname, pos, vel, fb in [
                        (pitch_name, foot_pitch, foot_pitch_vel, fb_upper),
                        (roll_name, foot_roll, foot_roll_vel, fb_lower),
                    ]:
                        joint_msg.name.append(jname)
                        joint_msg.position.append(pos)
                        joint_msg.velocity.append(vel)
                        joint_msg.effort.append(fb.torque)

                        ms = MotorState()
                        ms.joint_name = jname
                        ms.can_id = self._motor_ids[jname]
                        ms.position = pos
                        ms.velocity = vel
                        ms.torque = fb.torque
                        ms.temperature = fb.temperature
                        ms.fault_code = fb.fault_code
                        ms.mode_status = fb.mode_status
                        motor_msg.motors.append(ms)
                continue

            # --- Normal joint (non-ankle) ---
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
                    fb = motor.send_mit_command(kp=0.0, kd=0.0, torque_ff=0.0)
            except Exception as e:
                self.get_logger().warn(
                    f'{name}: CAN error: {e}', throttle_duration_sec=1.0)

            if fb is not None:
                calibrated_pos = fb.position - self._offsets[name]

                joint_msg.name.append(name)
                joint_msg.position.append(calibrated_pos)
                joint_msg.velocity.append(fb.velocity)
                joint_msg.effort.append(fb.torque)

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
