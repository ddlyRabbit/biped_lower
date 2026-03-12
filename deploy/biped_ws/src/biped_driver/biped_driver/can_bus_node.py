"""ROS2 CAN bus node — single node managing all 12 motors across 2 CAN buses.

Uses robstride_dynamics (Seeed shared library) via BipedMotorManager.
Handles ankle parallel linkage transparently.

Hardware layout:
    can0 → Right leg (6 motors: R_hip_pitch, R_hip_roll, R_hip_yaw, R_knee, R_foot_pitch, R_foot_roll)
    can1 → Left leg  (6 motors: L_hip_pitch, L_hip_roll, L_hip_yaw, L_knee, L_foot_pitch, L_foot_roll)

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
)


class CanBusNode(Node):
    """Single ROS2 node controlling all RobStride motors on can0 + can1.

    Motor config comes from robot.yaml (or param strings as fallback).
    Ankle joints are transparently mapped through the parallel linkage
    transform: policy sees foot_pitch/foot_roll, CAN sends motor_upper/lower.

    The control loop groups joints by CAN bus so each bus's write+read
    cycles are batched together, avoiding unnecessary bus switching.
    """

    def __init__(self):
        super().__init__("can_bus_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("robot_config", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("loop_rate", 50.0)
        self.declare_parameter("can_backend", "socketcan")  # "socketcan" or "waveshare"

        # Motor config string fallback (comma-separated "name:id:type")
        self.declare_parameter("motor_config_can0", "")
        self.declare_parameter("motor_config_can1", "")

        robot_config_path = str(self.get_parameter("robot_config").value)
        cal_file = str(self.get_parameter("calibration_file").value)
        self._rate = float(self.get_parameter("loop_rate").value)
        self._backend = str(self.get_parameter("can_backend").value)

        # ── Build motor manager ─────────────────────────────────────
        offsets = self._load_calibration(cal_file)

        if robot_config_path:
            config = self._load_robot_yaml(robot_config_path)
            self._mgr = BipedMotorManager.from_robot_yaml(config, offsets, backend=self._backend)
        else:
            joints = self._parse_param_strings(offsets)
            self._mgr = BipedMotorManager(joints, backend=self._backend)

        self._last_commands: dict[str, MITCommand] = {}
        self._last_positions: dict[str, float] = {}  # last known motor positions (normal joints)
        self._last_joint_positions: dict[str, float] = {}  # joint-space positions (for ankle soft stops)

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

        # ── Connect and start ───────────────────────────────────────
        self._log_motor_map()
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
                model = f"rs-{mtype[2:].zfill(2)}" if mtype.startswith("RS") else mtype.lower()
                joints.append(JointConfig(
                    name=name, can_bus=bus, can_id=mid, model=model, offset=offset,
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
            f"{len(self._mgr._buses)} buses @ {self._rate} Hz")

    def _build_bus_joint_order(self) -> list[list[str]]:
        """Group joints by bus. Returns [[joints_on_bus0], [joints_on_bus1], ...].

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
            ankle_rolls = {r for r in names if self._mgr.is_ankle_roll(r)}
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

    # ── Enable / Disable ────────────────────────────────────────────

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
        """50Hz: send MIT commands, read feedback, publish.

        Before motors are enabled: sends zero-torque MIT frames to get position feedback.
        After enabled: sends actual commands from /joint_commands.
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
        """Send/receive for a normal (non-ankle) joint."""
        cmd = self._last_commands.get(name)
        actual_pos = self._last_positions.get(name)
        fb = None

        try:
            if cmd:
                pos = self._mgr.clamp(name, cmd.position)
                self._mgr.send_mit_command(
                    name, pos, cmd.kp, cmd.kd, cmd.velocity, cmd.torque_ff,
                    actual_pos=actual_pos)
            else:
                # No command yet — zero stiffness, just read (soft stop still active)
                self._mgr.send_mit_command(name, 0.0, 0.0, 0.0,
                                           actual_pos=actual_pos)
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

        All clamping and soft stops happen in JOINT-SPACE (foot_pitch/roll)
        BEFORE the linkage forward transform to motor-space.

        Flow: policy joint-space → clamp → soft stop → linkage forward → motor CAN

        pitch_name → upper motor (knee-side CAN ID)
        roll_name  → lower motor (foot-side CAN ID)
        """
        pitch_name, roll_name = pair
        pitch_cmd = self._last_commands.get(pitch_name)
        roll_cmd = self._last_commands.get(roll_name)

        # Get last known joint-space positions for soft stop torque
        # (stored from previous cycle's linkage inverse)
        actual_pitch = self._last_joint_positions.get(pitch_name)
        actual_roll = self._last_joint_positions.get(roll_name)

        fb_upper = None
        fb_lower = None

        try:
            if pitch_cmd and roll_cmd:
                # 1. Clamp in joint-space (URDF limits)
                pitch_pos = self._mgr.clamp_joint(pitch_name, pitch_cmd.position)
                roll_pos = self._mgr.clamp_joint(roll_name, roll_cmd.position)

                # 2. Soft stop clamp in joint-space (2° inside limits)
                pitch_pos = self._mgr.softstop_clamp(pitch_name, pitch_pos)
                roll_pos = self._mgr.softstop_clamp(roll_name, roll_pos)

                # 3. Compute soft stop restoring torque in joint-space
                tff = (pitch_cmd.torque_ff + roll_cmd.torque_ff) / 2.0
                if actual_pitch is not None:
                    tff += self._mgr.compute_softstop_torque(pitch_name, actual_pitch)
                if actual_roll is not None:
                    tff += self._mgr.compute_softstop_torque(roll_name, actual_roll)

                # 4. Linkage forward: joint-space → motor-space
                motor_upper, motor_lower = ankle_command_to_motors(pitch_pos, roll_pos)

                # Average gains
                kp = (pitch_cmd.kp + roll_cmd.kp) / 2.0
                kd = (pitch_cmd.kd + roll_cmd.kd) / 2.0

                # 5. Send motor commands (no additional soft stops in send_ankle_mit_command)
                self._mgr.send_ankle_mit_command(
                    pitch_name, motor_upper, kp, kd, 0.0, tff)
                fb_upper = self._mgr.read_feedback(pitch_name)
                self._mgr.send_ankle_mit_command(
                    roll_name, motor_lower, kp, kd, 0.0, tff)
                fb_lower = self._mgr.read_feedback(roll_name)
            else:
                # No commands — zero stiffness, just read
                self._mgr.send_ankle_mit_command(pitch_name, 0.0, 0.0, 0.0)
                fb_upper = self._mgr.read_feedback(pitch_name)
                self._mgr.send_ankle_mit_command(roll_name, 0.0, 0.0, 0.0)
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

            # Store joint-space positions for next cycle's soft stop computation
            self._last_joint_positions[pitch_name] = foot_pitch
            self._last_joint_positions[roll_name] = foot_roll

            for jname, pos, vel, fb in [
                (pitch_name, foot_pitch, foot_pitch_vel, fb_upper),
                (roll_name, foot_roll, foot_roll_vel, fb_lower),
            ]:
                self._append_feedback(jname, fb, joint_msg, motor_msg,
                                      override_pos=pos, override_vel=vel)

    def _append_feedback(
        self,
        name: str,
        fb,
        joint_msg: JointState,
        motor_msg: MotorStateArray,
        override_pos: float = None,
        override_vel: float = None,
    ):
        """Append motor feedback to both JointState and MotorStateArray messages."""
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
        ms.can_id = self._mgr.joints[name].can_id
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
        # Enable motors in MIT mode (zero-torque) — they'll be free to move
        # but we get position feedback for the state_machine's soft start.
        # Actual stiffness only applied when /joint_commands has kp > 0.
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
