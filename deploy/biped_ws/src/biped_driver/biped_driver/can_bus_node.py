"""ROS2 CAN bus node — single node managing all 12 motors on can0.

Uses robstride_dynamics (Seeed shared library) via BipedMotorManager.
Handles ankle parallel linkage transparently.

Hardware layout:
    Dual CAN: can0 (right leg) + can1 (left leg) via Waveshare 2-CH CAN HAT

Subscribes:
    /joint_commands   biped_msgs/MITCommandArray

Publishes:
    /joint_states     sensor_msgs/JointState       @ loop_rate
    /motor_states     biped_msgs/MotorStateArray    @ loop_rate
"""

import yaml
from collections import defaultdict

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
    ANKLE_MOTOR_TO_JOINT,
)


class CanBusNode(Node):
    """Single ROS2 node controlling all RobStride motors on can0.

    Motor config comes from robot.yaml (or param strings as fallback).
    Ankle joints are transparently mapped through the parallel linkage
    transform: policy sees foot_pitch/foot_roll, CAN uses foot_top/foot_bottom motors.

    All soft-stop and clamping logic operates in motor command-space
    using calibration-derived limits.

    ESTOP behaviour: state_machine_node sends kp=0/kd=0 commands via
    /joint_commands — motors go free. This node always runs the loop
    and forwards whatever commands it receives.
    """

    def __init__(self):
        super().__init__("can_bus_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("robot_config", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("loop_rate", 50.0)

        # Motor config string fallback (comma-separated "name:id:type")
        self.declare_parameter("motor_config_can0", "")

        robot_config_path = str(self.get_parameter("robot_config").value)
        cal_file = str(self.get_parameter("calibration_file").value)
        self._rate = float(self.get_parameter("loop_rate").value)

        # ── Build motor manager ─────────────────────────────────────
        offsets = self._load_calibration(cal_file)

        if robot_config_path:
            config = self._load_robot_yaml(robot_config_path)
            self._mgr = BipedMotorManager.from_robot_yaml(config, offsets)
        else:
            joints = self._parse_param_strings(offsets)
            self._mgr = BipedMotorManager(joints)

        self._last_commands: dict[str, MITCommand] = {}
        # Last known motor-space positions (used for soft-stop torque).
        # For normal joints motor command-space = joint-space.
        # For ankle motors this is the raw post-linkage motor position.
        self._last_positions: dict[str, float] = {}

        # Pre-compute bus-grouped joint ordering for the control loop.
        # Within each bus, ankle pairs are grouped together.
        self._bus_joint_order = self._build_bus_joint_order()

        # ── QoS ─────────────────────────────────────────────────────
        fast_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ── Pub/Sub ─────────────────────────────────────────────────
        self._pub_joints = self.create_publisher(
            JointState, "/joint_states", fast_qos)
        self._pub_motors = self.create_publisher(
            MotorStateArray, "/motor_states", fast_qos)
        self._sub_commands = self.create_subscription(
            MITCommandArray, "/joint_commands", self._cmd_callback, 10)

        # ── Connect and enable ──────────────────────────────────────
        self._log_motor_map()
        self._mgr.connect_all()
        self._mgr.flush_all()
        self._mgr.enable_all()
        self.get_logger().info("All motors enabled in MIT mode.")

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
        """Fallback: parse motor_config_can0 param ("name:id:type,...")."""
        joints = []
        config_str = str(self.get_parameter("motor_config_can0").value)
        if not config_str:
            return joints
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
            model = f"rs-{mtype[2:].zfill(2)}" if mtype.startswith("RS") else mtype.lower()
            joints.append(JointConfig(
                name=name, can_bus="can0", can_id=mid, model=model, offset=offset,
            ))
        return joints

    def _log_motor_map(self):
        """Log per-bus motor assignments at startup for diagnostics."""
        buses = defaultdict(list)
        for name, jcfg in self._mgr.joints.items():
            buses[jcfg.can_bus].append(
                f"{name}(id={jcfg.can_id}, {jcfg.model})")
        for bus_name in sorted(buses):
            motors_str = ", ".join(sorted(buses[bus_name]))
            self.get_logger().info(f"  {bus_name}: {motors_str}")
        self.get_logger().info(
            f"Total: {len(self._mgr.joints)} motors on "
            f"{len(self._mgr._buses)} bus(es) @ {self._rate} Hz")

    def _build_bus_joint_order(self) -> list[list[str]]:
        """Group joints by bus. Returns [[joints_on_bus0], ...].

        Within each bus group, ankle pairs are kept adjacent (pitch before roll)
        and non-ankle joints are sorted alphabetically.
        """
        bus_groups = defaultdict(list)
        for name, jcfg in self._mgr.joints.items():
            bus_groups[jcfg.can_bus].append(name)

        result = []
        for bus_name in sorted(bus_groups):
            names = bus_groups[bus_name]
            # Separate ankle rolls (they'll be pulled in via their pitch pair)
            ankle_rolls = {r for r in names if self._mgr.is_ankle_bottom(r)}
            ordered = []
            seen = set()
            for name in sorted(names):
                if name in seen or name in ankle_rolls:
                    continue
                pair = self._mgr.get_ankle_pair(name)
                if pair:
                    ordered.append(pair[0])  # pitch
                    ordered.append(pair[1])  # roll
                    seen.add(pair[0])
                    seen.add(pair[1])
                else:
                    ordered.append(name)
                    seen.add(name)
            result.append(ordered)
        return result

    # ── Command callback ────────────────────────────────────────────

    def _cmd_callback(self, msg: MITCommandArray):
        for cmd in msg.commands:
            if cmd.joint_name in self._mgr.joints:
                self._last_commands[cmd.joint_name] = cmd

    # ── Main control loop ───────────────────────────────────────────

    def _loop(self):
        """50Hz: send MIT commands, read feedback, publish.

        Always runs. ESTOP = state_machine sends kp=0 → motors free.
        """
        now = self.get_clock().now().to_msg()
        joint_msg = JointState()
        joint_msg.header.stamp = now
        motor_msg = MotorStateArray()
        motor_msg.header.stamp = now

        processed = set()

        for bus_joints in self._bus_joint_order:
            for name in bus_joints:
                if name in processed:
                    continue

                pair = self._mgr.get_ankle_pair(name)
                if pair:
                    self._handle_ankle(pair, joint_msg, motor_msg)
                    processed.add(pair[0])
                    processed.add(pair[1])
                else:
                    self._handle_normal(name, joint_msg, motor_msg)
                    processed.add(name)

        self._pub_joints.publish(joint_msg)
        self._pub_motors.publish(motor_msg)

    def _handle_normal(self, name: str, joint_msg: JointState, motor_msg: MotorStateArray):
        """Send/receive for a normal (non-ankle) joint.

        send_mit_command handles clamping + soft-stop in motor
        command-space (= joint-space for normal joints).
        """
        cmd = self._last_commands.get(name)
        actual_pos = self._last_positions.get(name)
        fb = None

        try:
            if cmd:
                self._mgr.send_mit_command(
                    name, cmd.position, cmd.kp, cmd.kd, cmd.velocity, cmd.torque_ff,
                    actual_pos=actual_pos)
            else:
                # No command yet — truly zero torque (no soft stops)
                self._mgr.send_mit_command(name, 0.0, 0.0, 0.0)
            fb = self._mgr.read_feedback(name)
        except Exception as e:
            self.get_logger().warn(
                f"{name}: CAN error: {e}", throttle_duration_sec=1.0)

        if fb is not None:
            self._last_positions[name] = fb.position

        self._append_feedback(name, fb, joint_msg, motor_msg)

    def _handle_ankle(
        self,
        pair: tuple[str, str],
        joint_msg: JointState,
        motor_msg: MotorStateArray,
    ):
        """Send/receive for an ankle pair with parallel linkage transform.

        Flow: policy joint-space → Asimov forward → motor-space soft-stop → CAN
        Feedback: CAN → Asimov inverse → joint-space → /joint_states

        All clamping and soft-stop happens in motor command-space using
        calibration limits.  No joint-space clamping.

        top_name   → upper motor (knee-side CAN ID, motor A)
        bottom_name → lower motor (foot-side CAN ID, motor B)

        Publishes joint-space names (foot_pitch/foot_roll) on /joint_states.
        """
        top_name, bottom_name = pair
        # Joint-space names for /joint_states (policy compatibility)
        pitch_joint = ANKLE_MOTOR_TO_JOINT[top_name]
        roll_joint = ANKLE_MOTOR_TO_JOINT[bottom_name]
        pitch_cmd = self._last_commands.get(pitch_joint)
        roll_cmd = self._last_commands.get(roll_joint)

        # L ankle has mirrored pitch axis
        pitch_sign = -1 if top_name.startswith("L") else 1

        fb_upper = None
        fb_lower = None

        try:
            if pitch_cmd and roll_cmd:
                # 1. Asimov forward: joint-space → motor-space
                motor_upper, motor_lower = ankle_command_to_motors(
                    pitch_cmd.position, roll_cmd.position, pitch_sign)

                # Average gains (both are RS02)
                kp = (pitch_cmd.kp + roll_cmd.kp) / 2.0
                kd = (pitch_cmd.kd + roll_cmd.kd) / 2.0
                tff = (pitch_cmd.torque_ff + roll_cmd.torque_ff) / 2.0

                # 2. Send with per-motor soft-stop in motor command-space
                self._mgr.send_ankle_mit_command(
                    top_name, motor_upper, kp, kd, 0.0, tff,
                    actual_pos=self._last_positions.get(top_name))
                fb_upper = self._mgr.read_feedback(top_name)

                self._mgr.send_ankle_mit_command(
                    bottom_name, motor_lower, kp, kd, 0.0, tff,
                    actual_pos=self._last_positions.get(bottom_name))
                fb_lower = self._mgr.read_feedback(bottom_name)
            else:
                # No commands — zero stiffness, just read
                self._mgr.send_ankle_mit_command(top_name, 0.0, 0.0, 0.0)
                fb_upper = self._mgr.read_feedback(top_name)
                self._mgr.send_ankle_mit_command(bottom_name, 0.0, 0.0, 0.0)
                fb_lower = self._mgr.read_feedback(bottom_name)
        except Exception as e:
            self.get_logger().warn(
                f"Ankle {pitch_name}/{roll_name}: CAN error: {e}",
                throttle_duration_sec=1.0)

        # Store raw motor positions for next cycle's soft-stop torque
        if fb_upper is not None:
            self._last_positions[top_name] = fb_upper.position
        if fb_lower is not None:
            self._last_positions[bottom_name] = fb_lower.position

        if fb_upper is not None and fb_lower is not None:
            # Asimov inverse: motor-space → joint-space for /joint_states
            foot_pitch, foot_roll = ankle_motors_to_feedback(
                fb_upper.position, fb_lower.position, pitch_sign)
            foot_pitch_vel, foot_roll_vel = ankle_motors_to_feedback(
                fb_upper.velocity, fb_lower.velocity, pitch_sign)

            for jname, motor_name, pos, vel, fb in [
                (pitch_joint, top_name, foot_pitch, foot_pitch_vel, fb_upper),
                (roll_joint, bottom_name, foot_roll, foot_roll_vel, fb_lower),
            ]:
                self._append_feedback(jname, fb, joint_msg, motor_msg,
                                      override_pos=pos, override_vel=vel,
                                      motor_name=motor_name)

    def _append_feedback(
        self,
        name: str,
        fb,
        joint_msg: JointState,
        motor_msg: MotorStateArray,
        override_pos: float = None,
        override_vel: float = None,
        motor_name: str = None,
    ):
        """Append motor feedback to both JointState and MotorStateArray messages.

        name: joint-space name (published on /joint_states)
        motor_name: CAN motor name (for can_id lookup, defaults to name)
        """
        if fb is None:
            return

        pos = override_pos if override_pos is not None else fb.position
        vel = override_vel if override_vel is not None else fb.velocity

        joint_msg.name.append(name)
        joint_msg.position.append(pos)
        joint_msg.velocity.append(vel)
        joint_msg.effort.append(fb.torque)

        ms = MotorState()
        ms.joint_name = name
        mn = motor_name or name
        ms.can_id = self._mgr.joints[mn].can_id
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
