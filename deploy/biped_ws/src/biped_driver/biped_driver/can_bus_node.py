"""ROS2 CAN bus node — single node managing all 12 motors across both buses.

Uses robstride_dynamics (Seeed shared library) via BipedMotorManager.
Handles ankle parallel linkage transparently.

Subscribes:
    /joint_commands   biped_msgs/MITCommandArray

Publishes:
    /joint_states     sensor_msgs/JointState       @ loop_rate
    /motor_states     biped_msgs/MotorStateArray    @ loop_rate
"""

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommand, MITCommandArray, MotorState, MotorStateArray

from biped_driver.robstride_can import (
    BipedMotorManager,
    JointConfig,
    MotorFeedback,
    ankle_command_to_motors,
    ankle_motors_to_feedback,
    ANKLE_PAIRS,
)


class CanBusNode(Node):
    """Single ROS2 node controlling all RobStride motors on can0 + can1.

    Motor config comes from robot.yaml. Ankle joints are transparently
    mapped through the parallel linkage transform so the policy sees
    foot_pitch/foot_roll while CAN talks to motor_upper/motor_lower.
    """

    def __init__(self):
        super().__init__("can_bus_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("robot_config", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("loop_rate", 50.0)

        # Motor config string fallback (comma-separated "name:id:type:bus")
        self.declare_parameter("motor_config_can0", "")
        self.declare_parameter("motor_config_can1", "")

        robot_config_path = str(self.get_parameter("robot_config").value)
        cal_file = str(self.get_parameter("calibration_file").value)
        self._rate = float(self.get_parameter("loop_rate").value)

        # ── Build motor manager ─────────────────────────────────────
        offsets = self._load_calibration(cal_file)

        if robot_config_path:
            config = self._load_robot_yaml(robot_config_path)
            self._mgr = BipedMotorManager.from_robot_yaml(config, offsets)
        else:
            # Fallback: build from param strings
            joints = self._parse_param_strings(offsets)
            self._mgr = BipedMotorManager(joints)

        self._joint_names = sorted(self._mgr.joints.keys())
        self._last_commands: dict[str, MITCommand] = {}

        # ── QoS ─────────────────────────────────────────────────────
        fast_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── Pub/Sub ─────────────────────────────────────────────────
        self._pub_joints = self.create_publisher(
            JointState, "/joint_states", fast_qos)
        self._pub_motors = self.create_publisher(
            MotorStateArray, "/motor_states", fast_qos)
        self._sub_commands = self.create_subscription(
            MITCommandArray, "/joint_commands", self._cmd_callback, 10)

        # ── Connect and start ───────────────────────────────────────
        self.get_logger().info(
            f"Connecting {len(self._mgr.joints)} motors on "
            f"{list(self._mgr._buses.keys())} @ {self._rate} Hz"
        )
        self._mgr.connect_all()
        self._mgr.flush_all()

        self._timer = self.create_timer(1.0 / self._rate, self._loop)
        self.get_logger().info("CAN bus node ready.")

    # ── Config loaders ──────────────────────────────────────────────

    def _load_robot_yaml(self, path: str) -> dict:
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to load robot config: {e}")
            return {}

    def _load_calibration(self, path: str) -> dict:
        if not path:
            return {}
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"No calibration loaded: {e}")
            return {}

    def _parse_param_strings(self, offsets: dict) -> list[JointConfig]:
        """Fallback: parse motor_config_canX params ("name:id:type,...")."""
        joints = []
        for bus in ("can0", "can1"):
            param_name = f"motor_config_{bus}"
            config_str = str(self.get_parameter(param_name).value)
            if not config_str:
                continue
            for entry in config_str.split(","):
                entry = entry.strip()
                if not entry:
                    continue
                parts = entry.split(":")
                if len(parts) != 3:
                    self.get_logger().error(f"Invalid motor entry: {entry}")
                    continue
                name, mid, mtype = parts[0].strip(), int(parts[1]), parts[2].strip()
                offset = offsets.get(name, {}).get("offset", 0.0) if offsets else 0.0
                joints.append(JointConfig(
                    name=name, can_bus=bus, can_id=mid,
                    model=f"rs-{mtype[2:].zfill(2)}" if mtype.startswith("RS") else mtype.lower(),
                    offset=offset,
                ))
        return joints

    # ── Command callback ────────────────────────────────────────────

    def _cmd_callback(self, msg: MITCommandArray):
        for cmd in msg.commands:
            if cmd.joint_name in self._mgr.joints:
                self._last_commands[cmd.joint_name] = cmd

    # ── Enable / Disable (called externally via service or state machine) ──

    def enable_all(self):
        self.get_logger().info("Enabling all motors (MIT mode)...")
        self._mgr.enable_all()
        self.get_logger().info("All motors enabled.")

    def disable_all(self):
        self.get_logger().info("Disabling all motors...")
        self._mgr.disable_all()
        self.get_logger().info("All motors disabled.")

    # ── Main control loop ───────────────────────────────────────────

    def _loop(self):
        """50Hz: send MIT commands, read feedback, publish."""
        now = self.get_clock().now().to_msg()
        joint_msg = JointState()
        joint_msg.header.stamp = now
        motor_msg = MotorStateArray()
        motor_msg.header.stamp = now

        # Track processed ankle joints to avoid double-handling
        processed = set()

        for name in self._joint_names:
            if name in processed:
                continue

            ankle_pair = self._mgr.get_ankle_pair(name)
            if ankle_pair:
                self._handle_ankle(ankle_pair, joint_msg, motor_msg)
                processed.add(ankle_pair[0])
                processed.add(ankle_pair[1])
            else:
                self._handle_normal(name, joint_msg, motor_msg)

        self._pub_joints.publish(joint_msg)
        self._pub_motors.publish(motor_msg)

    def _handle_normal(self, name: str, joint_msg: JointState, motor_msg: MotorStateArray):
        """Send/receive for a normal (non-ankle) joint."""
        cmd = self._last_commands.get(name)
        fb = None

        try:
            if cmd:
                pos = self._mgr.clamp(name, cmd.position)
                self._mgr.send_mit_command(
                    name, pos, cmd.kp, cmd.kd, cmd.velocity, cmd.torque_ff)
            else:
                # No command yet — zero stiffness, just read
                self._mgr.send_mit_command(name, 0.0, 0.0, 0.0)
            fb = self._mgr.read_feedback(name)
        except Exception as e:
            self.get_logger().warn(
                f"{name}: CAN error: {e}", throttle_duration_sec=1.0)

        if fb:
            joint_msg.name.append(name)
            joint_msg.position.append(fb.position)
            joint_msg.velocity.append(fb.velocity)
            joint_msg.effort.append(fb.torque)

            ms = MotorState()
            ms.joint_name = name
            ms.can_id = self._mgr.joints[name].can_id
            ms.position = fb.position
            ms.velocity = fb.velocity
            ms.torque = fb.torque
            ms.temperature = fb.temperature
            ms.fault_code = fb.fault_code
            ms.mode_status = fb.mode_status
            motor_msg.motors.append(ms)

    def _handle_ankle(
        self,
        pair: tuple[str, str],
        joint_msg: JointState,
        motor_msg: MotorStateArray,
    ):
        """Send/receive for an ankle pair with parallel linkage transform.

        Policy sees foot_pitch/foot_roll. CAN talks to upper/lower motors.
        pitch_name = upper motor (knee-side), roll_name = lower motor (foot-side).
        """
        pitch_name, roll_name = pair
        pitch_cmd = self._last_commands.get(pitch_name)
        roll_cmd = self._last_commands.get(roll_name)

        fb_upper = None
        fb_lower = None

        try:
            if pitch_cmd and roll_cmd:
                # Clamp in joint-space BEFORE linkage transform
                clamped_pitch = self._mgr.clamp(pitch_name, pitch_cmd.position)
                clamped_roll = self._mgr.clamp(roll_name, roll_cmd.position)
                motor_upper, motor_lower = ankle_command_to_motors(
                    clamped_pitch, clamped_roll)
                kp = (pitch_cmd.kp + roll_cmd.kp) / 2.0
                kd = (pitch_cmd.kd + roll_cmd.kd) / 2.0

                self._mgr.send_mit_command(
                    pitch_name, motor_upper, kp, kd, 0.0, 0.0)
                fb_upper = self._mgr.read_feedback(pitch_name)
                self._mgr.send_mit_command(
                    roll_name, motor_lower, kp, kd, 0.0, 0.0)
                fb_lower = self._mgr.read_feedback(roll_name)
            else:
                # No commands — zero stiffness
                self._mgr.send_mit_command(pitch_name, 0.0, 0.0, 0.0)
                fb_upper = self._mgr.read_feedback(pitch_name)
                self._mgr.send_mit_command(roll_name, 0.0, 0.0, 0.0)
                fb_lower = self._mgr.read_feedback(roll_name)
        except Exception as e:
            self.get_logger().warn(
                f"Ankle {pitch_name}/{roll_name}: CAN error: {e}",
                throttle_duration_sec=1.0)

        if fb_upper is not None and fb_lower is not None:
            # Inverse linkage: reconstruct foot pitch/roll from motor positions
            foot_pitch, foot_roll = ankle_motors_to_feedback(
                fb_upper.position, fb_lower.position)
            foot_pitch_vel, foot_roll_vel = ankle_motors_to_feedback(
                fb_upper.velocity, fb_lower.velocity)

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
                ms.can_id = self._mgr.joints[jname].can_id
                ms.position = pos
                ms.velocity = vel
                ms.torque = fb.torque
                ms.temperature = fb.temperature
                ms.fault_code = fb.fault_code
                ms.mode_status = fb.mode_status
                motor_msg.motors.append(ms)

    # ── Shutdown ────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down — disabling motors")
        try:
            self._mgr.disable_all()
        except Exception:
            pass
        self._mgr.disconnect_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CanBusNode()
    try:
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


if __name__ == "__main__":
    main()
