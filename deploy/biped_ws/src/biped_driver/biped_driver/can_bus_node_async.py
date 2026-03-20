"""ROS2 CAN bus node — async non-blocking version.

Two dedicated CAN worker threads (one per bus) run tight send/recv loops.
ROS2 main thread only reads/writes shared buffers — zero CAN I/O on ROS thread.

Target: 100-200Hz motor update rate (vs 50Hz blocking original).

Hardware layout:
    Dual CAN: can0 (right leg) + can1 (left leg) via Waveshare 2-CH CAN HAT

Subscribes:
    /joint_commands   biped_msgs/MITCommandArray

Publishes:
    /joint_states     sensor_msgs/JointState       @ publish_rate
    /motor_states     biped_msgs/MotorStateArray    @ publish_rate
"""

import time
import threading
import yaml
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
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

ANKLE_JOINT_LIMITS = {
    "foot_pitch": (-0.87267, 0.52360),
    "foot_roll":  (-0.26180, 0.26180),
}


# ─── Shared buffers ────────────────────────────────────────────────────────

@dataclass
class MotorCommand:
    """Command for a single motor (joint-space)."""
    position: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    velocity: float = 0.0
    torque_ff: float = 0.0
    valid: bool = False  # True once first command received


@dataclass
class MotorFeedbackEntry:
    """Feedback from a single motor."""
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    fault_code: int = 0
    mode_status: int = 0
    timestamp: float = 0.0  # time.monotonic()
    valid: bool = False


class SharedBuffer:
    """Thread-safe command/feedback buffer for one CAN bus."""

    def __init__(self, motor_names: list[str]):
        self._lock = threading.Lock()
        self._commands: dict[str, MotorCommand] = {n: MotorCommand() for n in motor_names}
        self._feedback: dict[str, MotorFeedbackEntry] = {n: MotorFeedbackEntry() for n in motor_names}
        self.loop_dt: float = 0.0  # last worker loop time
        self.loop_count: int = 0

    def write_command(self, name: str, cmd: MotorCommand):
        with self._lock:
            self._commands[name] = cmd

    def read_commands(self) -> dict[str, MotorCommand]:
        with self._lock:
            return {k: MotorCommand(v.position, v.kp, v.kd, v.velocity, v.torque_ff, v.valid)
                    for k, v in self._commands.items()}

    def write_feedback(self, name: str, fb: MotorFeedbackEntry):
        with self._lock:
            self._feedback[name] = fb

    def read_all_feedback(self) -> dict[str, MotorFeedbackEntry]:
        with self._lock:
            return dict(self._feedback)

    def update_stats(self, dt: float):
        with self._lock:
            self.loop_dt = dt
            self.loop_count += 1


# ─── CAN Worker Thread ─────────────────────────────────────────────────────

class CanBusWorker(threading.Thread):
    """Dedicated thread for one CAN bus — tight send/recv loop."""

    def __init__(
        self,
        bus_name: str,
        mgr: BipedMotorManager,
        motor_names: list[str],
        buffer: SharedBuffer,
        logger,
    ):
        super().__init__(daemon=True, name=f"can_worker_{bus_name}")
        self.bus_name = bus_name
        self._mgr = mgr
        self._motor_names = motor_names
        self._buffer = buffer
        self._logger = logger
        self._running = True
        self._last_positions: dict[str, float] = {}

        # Pre-compute ankle pairs on this bus
        self._ankle_pairs: list[tuple[str, str]] = []
        self._normal_motors: list[str] = []
        self._build_motor_groups()

    def _build_motor_groups(self):
        seen = set()
        for name in self._motor_names:
            if name in seen:
                continue
            pair = self._mgr.get_ankle_pair(name)
            if pair and pair[0] in self._motor_names and pair[1] in self._motor_names:
                self._ankle_pairs.append(pair)
                seen.add(pair[0])
                seen.add(pair[1])
            elif name not in seen:
                self._normal_motors.append(name)
                seen.add(name)

    def stop(self):
        self._running = False

    def run(self):
        """Tight loop: read commands → send all → read all → write feedback."""
        self._logger.info(f"[{self.bus_name}] Worker started: "
                          f"{len(self._normal_motors)} normal + "
                          f"{len(self._ankle_pairs)} ankle pairs")

        while self._running:
            t0 = time.monotonic()

            commands = self._buffer.read_commands()

            # ── Phase 1: Send all commands ──────────────────────────
            for name in self._normal_motors:
                cmd = commands.get(name)
                try:
                    if cmd and cmd.valid:
                        self._mgr.send_mit_command(
                            name, cmd.position, cmd.kp, cmd.kd,
                            cmd.velocity, cmd.torque_ff,
                            actual_pos=self._last_positions.get(name))
                    else:
                        self._mgr.send_mit_command(name, 0.0, 0.0, 0.0)
                except Exception as e:
                    self._logger.warn(
                        f"[{self.bus_name}] {name} send error: {e}",
                        throttle_duration_sec=1.0)

            for top_name, bottom_name in self._ankle_pairs:
                pitch_joint = ANKLE_MOTOR_TO_JOINT[top_name]
                roll_joint = ANKLE_MOTOR_TO_JOINT[bottom_name]
                pitch_cmd = commands.get(pitch_joint)
                roll_cmd = commands.get(roll_joint)
                pitch_sign = -1 if top_name.startswith("L") else 1

                try:
                    if pitch_cmd and pitch_cmd.valid and roll_cmd and roll_cmd.valid:
                        pitch_lo, pitch_hi = ANKLE_JOINT_LIMITS["foot_pitch"]
                        roll_lo, roll_hi = ANKLE_JOINT_LIMITS["foot_roll"]
                        pitch_pos = max(pitch_lo, min(pitch_hi, pitch_cmd.position))
                        roll_pos = max(roll_lo, min(roll_hi, roll_cmd.position))

                        motor_upper, motor_lower = ankle_command_to_motors(
                            pitch_pos, roll_pos, pitch_sign)
                        kp = (pitch_cmd.kp + roll_cmd.kp) / 2.0
                        kd = (pitch_cmd.kd + roll_cmd.kd) / 2.0
                        tff = (pitch_cmd.torque_ff + roll_cmd.torque_ff) / 2.0

                        self._mgr.send_ankle_mit_command(
                            top_name, motor_upper, kp, kd, 0.0, tff,
                            actual_pos=self._last_positions.get(top_name))
                        self._mgr.send_ankle_mit_command(
                            bottom_name, motor_lower, kp, kd, 0.0, tff,
                            actual_pos=self._last_positions.get(bottom_name))
                    else:
                        self._mgr.send_ankle_mit_command(top_name, 0.0, 0.0, 0.0)
                        self._mgr.send_ankle_mit_command(bottom_name, 0.0, 0.0, 0.0)
                except Exception as e:
                    self._logger.warn(
                        f"[{self.bus_name}] ankle {top_name}/{bottom_name} send error: {e}",
                        throttle_duration_sec=1.0)

            # ── Phase 2: Read all feedback ──────────────────────────
            now = time.monotonic()

            for name in self._normal_motors:
                try:
                    fb = self._mgr.read_feedback(name)
                    if fb is not None:
                        self._last_positions[name] = fb.position
                        self._buffer.write_feedback(name, MotorFeedbackEntry(
                            position=fb.position, velocity=fb.velocity,
                            torque=fb.torque, temperature=fb.temperature,
                            fault_code=fb.fault_code, mode_status=fb.mode_status,
                            timestamp=now, valid=True,
                        ))
                except Exception as e:
                    self._logger.warn(
                        f"[{self.bus_name}] {name} read error: {e}",
                        throttle_duration_sec=1.0)

            for top_name, bottom_name in self._ankle_pairs:
                for motor_name in (top_name, bottom_name):
                    try:
                        fb = self._mgr.read_feedback(motor_name)
                        if fb is not None:
                            self._last_positions[motor_name] = fb.position
                            self._buffer.write_feedback(motor_name, MotorFeedbackEntry(
                                position=fb.position, velocity=fb.velocity,
                                torque=fb.torque, temperature=fb.temperature,
                                fault_code=fb.fault_code, mode_status=fb.mode_status,
                                timestamp=now, valid=True,
                            ))
                    except Exception as e:
                        self._logger.warn(
                            f"[{self.bus_name}] {motor_name} read error: {e}",
                            throttle_duration_sec=1.0)

            dt = time.monotonic() - t0
            self._buffer.update_stats(dt)


# ─── ROS2 Node ──────────────────────────────────────────────────────────────

class AsyncCanBusNode(Node):
    """Non-blocking CAN bus node — worker threads handle all CAN I/O."""

    def __init__(self):
        super().__init__("can_bus_node")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("robot_config", "")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("publish_rate", 50.0)  # ROS publish rate (Hz)

        robot_config_path = str(self.get_parameter("robot_config").value)
        cal_file = str(self.get_parameter("calibration_file").value)
        self._pub_rate = float(self.get_parameter("publish_rate").value)

        # ── Build motor manager ─────────────────────────────────────
        offsets = self._load_calibration(cal_file)
        if robot_config_path:
            config = self._load_robot_yaml(robot_config_path)
            self._mgr = BipedMotorManager.from_robot_yaml(config, offsets)
        else:
            self.get_logger().error("robot_config required for async node")
            return

        self._joint_to_motor = {v: k for k, v in ANKLE_MOTOR_TO_JOINT.items()}

        # ── Group motors by bus ─────────────────────────────────────
        bus_motors: dict[str, list[str]] = defaultdict(list)
        for name, jcfg in self._mgr.joints.items():
            bus_motors[jcfg.can_bus].append(name)

        # ── Create shared buffers + workers ─────────────────────────
        # Buffer keys use joint-space names for ankle joints
        self._buffers: dict[str, SharedBuffer] = {}
        self._workers: list[CanBusWorker] = []

        for bus_name in sorted(bus_motors):
            motor_names = bus_motors[bus_name]
            # For ankle motors, buffer uses joint-space names
            buffer_names = []
            for name in motor_names:
                if name in ANKLE_MOTOR_TO_JOINT:
                    buffer_names.append(ANKLE_MOTOR_TO_JOINT[name])
                else:
                    buffer_names.append(name)
            # But worker needs motor names
            buf = SharedBuffer(buffer_names)
            self._buffers[bus_name] = buf
            worker = CanBusWorker(
                bus_name, self._mgr, motor_names, buf, self.get_logger())
            self._workers.append(worker)

        # ── QoS + Pub/Sub ───────────────────────────────────────────
        fast_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub_joints = self.create_publisher(JointState, "/joint_states", fast_qos)
        self._pub_motors = self.create_publisher(MotorStateArray, "/motor_states", fast_qos)
        self._sub_commands = self.create_subscription(
            MITCommandArray, "/joint_commands", self._cmd_callback, 10)

        # ── Connect, enable, start workers ──────────────────────────
        self._log_motor_map(bus_motors)
        self._mgr.connect_all()
        self._mgr.flush_all()
        self._mgr.enable_all()
        self.get_logger().info("All motors enabled in MIT mode.")

        for worker in self._workers:
            worker.start()
        self.get_logger().info(f"Started {len(self._workers)} CAN worker threads.")

        # ── Publish timer (ROS thread only reads buffers) ───────────
        self._timer = self.create_timer(1.0 / self._pub_rate, self._publish_loop)

        # ── Stats timer ─────────────────────────────────────────────
        self._stats_timer = self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"Async CAN bus node ready — publish @ {self._pub_rate}Hz")

    # ── Config loaders (same as original) ───────────────────────────

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

    def _log_motor_map(self, bus_motors: dict):
        for bus_name in sorted(bus_motors):
            names = ", ".join(sorted(bus_motors[bus_name]))
            self.get_logger().info(f"  {bus_name}: {names}")
        total = sum(len(v) for v in bus_motors.values())
        self.get_logger().info(f"Total: {total} motors on {len(bus_motors)} bus(es)")

    # ── Command callback (writes to shared buffer) ──────────────────

    def _cmd_callback(self, msg: MITCommandArray):
        for cmd in msg.commands:
            name = cmd.joint_name
            mc = MotorCommand(
                position=cmd.position, kp=cmd.kp, kd=cmd.kd,
                velocity=cmd.velocity, torque_ff=cmd.torque_ff,
                valid=True,
            )
            # Write to the correct bus buffer
            for buf in self._buffers.values():
                if name in buf._commands:
                    buf.write_command(name, mc)
                    break

    # ── Publish loop (reads shared buffers, no CAN I/O) ─────────────

    def _publish_loop(self):
        now = self.get_clock().now().to_msg()
        joint_msg = JointState()
        joint_msg.header.stamp = now
        motor_msg = MotorStateArray()
        motor_msg.header.stamp = now

        for bus_name, buf in self._buffers.items():
            feedback = buf.read_all_feedback()
            bus_motors = [n for n, jcfg in self._mgr.joints.items()
                          if jcfg.can_bus == bus_name]

            # Process ankle pairs
            processed = set()
            for name in bus_motors:
                if name in processed:
                    continue
                pair = self._mgr.get_ankle_pair(name)
                if pair and pair[0] in bus_motors and pair[1] in bus_motors:
                    top_name, bottom_name = pair
                    pitch_joint = ANKLE_MOTOR_TO_JOINT[top_name]
                    roll_joint = ANKLE_MOTOR_TO_JOINT[bottom_name]
                    pitch_sign = -1 if top_name.startswith("L") else 1

                    fb_top = feedback.get(top_name)
                    fb_bot = feedback.get(bottom_name)
                    if fb_top and fb_top.valid and fb_bot and fb_bot.valid:
                        foot_pitch, foot_roll = ankle_motors_to_feedback(
                            fb_top.position, fb_bot.position, pitch_sign)
                        foot_pitch_vel, foot_roll_vel = ankle_motors_to_feedback(
                            fb_top.velocity, fb_bot.velocity, pitch_sign)

                        for jname, pos, vel, fb in [
                            (pitch_joint, foot_pitch, foot_pitch_vel, fb_top),
                            (roll_joint, foot_roll, foot_roll_vel, fb_bot),
                        ]:
                            self._append_feedback(jname, fb, joint_msg, motor_msg,
                                                  override_pos=pos, override_vel=vel)

                    processed.add(pair[0])
                    processed.add(pair[1])
                else:
                    fb = feedback.get(name)
                    if fb and fb.valid:
                        self._append_feedback(name, fb, joint_msg, motor_msg)
                    processed.add(name)

        self._pub_joints.publish(joint_msg)
        self._pub_motors.publish(motor_msg)

    def _append_feedback(
        self, name: str, fb: MotorFeedbackEntry,
        joint_msg: JointState, motor_msg: MotorStateArray,
        override_pos: float = None, override_vel: float = None,
    ):
        pos = override_pos if override_pos is not None else fb.position
        vel = override_vel if override_vel is not None else fb.velocity

        joint_msg.name.append(name)
        joint_msg.position.append(pos)
        joint_msg.velocity.append(vel)
        joint_msg.effort.append(fb.torque)

        ms = MotorState()
        ms.joint_name = name
        ms.position = fb.position
        ms.velocity = fb.velocity
        ms.torque = fb.torque
        ms.temperature = fb.temperature
        ms.fault_code = fb.fault_code
        ms.mode_status = fb.mode_status
        motor_msg.motors.append(ms)

    # ── Stats logging ───────────────────────────────────────────────

    def _log_stats(self):
        for bus_name, buf in self._buffers.items():
            with buf._lock:
                dt = buf.loop_dt
                count = buf.loop_count
            if dt > 0:
                hz = 1.0 / dt
                self.get_logger().info(
                    f"[{bus_name}] CAN loop: {hz:.0f}Hz (dt={dt*1000:.1f}ms, "
                    f"total={count})")

    # ── Shutdown ────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info("Shutting down async CAN node")
        for worker in self._workers:
            worker.stop()
        for worker in self._workers:
            worker.join(timeout=2.0)
        try:
            self._mgr.disable_all()
        except Exception:
            pass
        self._mgr.disconnect_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsyncCanBusNode()
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
