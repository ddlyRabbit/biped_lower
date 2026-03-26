"""State machine node — IDLE → STAND → WALK / SIM_WALK / WALK_ZMP / WALK_SIM_ZMP / WIGGLE_SEQ / WIGGLE_ALL → ESTOP.

Manages robot lifecycle transitions. Publishes current state.
Reads safety status and gamepad buttons for transitions.

States:
    IDLE        — motors disabled, waiting for START command
    STAND       — soft start, then policy with zero velocity
    WALK        — policy with cmd_vel input
    WIGGLE_SEQ  — sequential sine sweep, one joint at a time
    SIM_WALK    — motors hold STAND, policy runs for viz only (/policy_viz)
    WIGGLE_ALL  — all joints wiggle simultaneously
    ESTOP       — zero torque, requires manual reset
"""

import math
import time
import yaml
import rcl_interfaces.msg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommand, MITCommandArray, RobotState
from biped_control.obs_builder import JOINT_ORDER, DEFAULT_POSITIONS, DEFAULT_GAINS

# URDF joint limits (rad) — safety clamp for wiggle
JOINT_LIMITS = {
    "L_hip_pitch": (-1.047, 2.217),  "R_hip_pitch": (-2.217, 1.047),
    "L_hip_roll":  (-0.209, 2.269),  "R_hip_roll":  (-2.269, 0.209),
    "L_hip_yaw":   (-1.571, 1.571),  "R_hip_yaw":   (-1.571, 1.571),
    "L_knee":      (0.0, 2.705),     "R_knee":      (0.0, 2.705),
    "L_foot_pitch":(-0.873, 0.524),  "R_foot_pitch":(-0.873, 0.524),
    "L_foot_roll": (-0.262, 0.262),  "R_foot_roll": (-0.262, 0.262),
}


class StateMachineNode(Node):
    """Robot state machine.

    Publishes:
        /state_machine  String   @ 10Hz
        /robot_state    RobotState @ 10Hz
        /joint_commands MITCommandArray (during STAND soft start)
        /cmd_vel        Twist (zero cmd during STAND)
    """

    def __init__(self):
        super().__init__('state_machine_node')

        self.declare_parameter('stand_ramp_time', 2.0)      # seconds to ramp to default pose
        self.declare_parameter('stand_gain_ramp_time', 1.0)  # seconds to ramp gains
        self.declare_parameter('stand_stable_time', 2.0)     # seconds stable before WALK allowed
        self.declare_parameter('wiggle_config', '')
        self.declare_parameter('gain_scale', 1.0,
            rcl_interfaces.msg.ParameterDescriptor(dynamic_typing=True))

        self._ramp_time = float(self.get_parameter('stand_ramp_time').value)
        self._gain_ramp_time = float(self.get_parameter('stand_gain_ramp_time').value)
        self._stable_time = float(self.get_parameter('stand_stable_time').value)

        # Wiggle state (config loaded on first use)
        self._wiggle_cfg = None
        self._wiggle_start = 0.0
        self._wiggle_joint_idx = 0

        # State
        self._state = "IDLE"
        self._state_start_time = time.time()
        self._safety_ok = True
        self._current_positions = {n: 0.0 for n in JOINT_ORDER}
        self._stand_start_positions = {}  # captured on IDLE→STAND
        self._transition_requested = None

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Bool, '/safety/status', self._safety_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)

        # Publishers
        self._pub_state = self.create_publisher(String, '/state_machine', 10)
        self._pub_robot = self.create_publisher(RobotState, '/robot_state', 10)
        self._pub_cmd = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Transition commands via topic
        self.create_subscription(String, '/state_command', self._command_cb, 10)

        # Timers
        self.create_timer(0.02, self._loop)    # 50Hz control
        self.create_timer(0.1, self._publish_state)  # 10Hz status

        self.get_logger().info(f'State machine started — state: {self._state}')

    def _load_wiggle_config(self) -> dict:
        """Load wiggle.yaml config. Returns per-joint wiggle params."""
        path = str(self.get_parameter('wiggle_config').value)
        self.get_logger().info(f'Wiggle config path: {path}')
        cfg = {
            'frequency': 1.0,
            'duration_per_joint': 3.0,
            'joints': {},
        }
        if path:
            try:
                with open(path) as f:
                    raw = yaml.safe_load(f)
                if raw and 'wiggle' in raw:
                    w = raw['wiggle']
                    
                    cfg['duration_per_joint'] = w.get('duration_per_joint', 3.0)
                    for name, params in w.get('joints', {}).items():
                        pos = params.get('pos', 0.087)
                        neg = params.get('neg', -0.087)
                        freq = params.get('freq', 1.0)
                        # Validate against joint limits
                        if name in JOINT_LIMITS:
                            lo, hi = JOINT_LIMITS[name]
                            default = DEFAULT_POSITIONS.get(name, 0.0)
                            if default + pos > hi:
                                pos = hi - default
                                self.get_logger().warn(f'{name}: pos clamped to {pos:.3f} (limit {hi:.3f})')
                            if default + neg < lo:
                                neg = lo - default
                                self.get_logger().warn(f'{name}: neg clamped to {neg:.3f} (limit {lo:.3f})')
                        cfg['joints'][name] = {'pos': pos, 'neg': neg, 'freq': freq}
                self.get_logger().info(f'Wiggle config loaded: {len(cfg["joints"])} joints')
                for n, p in cfg['joints'].items():
                    self.get_logger().info(f'  {n}: pos={p["pos"]:.4f} neg={p["neg"]:.4f} freq={p["freq"]}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load wiggle config: {e}')
        # Fill missing joints with defaults
        for name in JOINT_ORDER:
            if name not in cfg['joints']:
                cfg['joints'][name] = {'pos': 0.087, 'neg': -0.087, 'freq': cfg['frequency']}
        return cfg

    def _safety_cb(self, msg: Bool):
        self._safety_ok = msg.data
        if not msg.data and self._state not in ("IDLE", "ESTOP"):
            self._transition("ESTOP")

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self._current_positions and i < len(msg.position):
                self._current_positions[name] = msg.position[i]

    def _command_cb(self, msg: String):
        """Accept state transition commands: START, WALK, STOP, ESTOP, RESET."""
        cmd = msg.data.upper().strip()
        if cmd == "START" and self._state == "IDLE":
            self._transition("STAND")
        elif cmd == "WALK" and self._state == "STAND":
            self._transition("WALK")
        elif cmd == "SIM_WALK" and self._state == "STAND":
            self._transition("SIM_WALK")
        elif cmd == "WALK_ZMP" and self._state == "STAND":
            self._transition("WALK_ZMP")
        elif cmd == "WALK_SIM_ZMP" and self._state == "STAND":
            self._transition("WALK_SIM_ZMP")
        elif cmd == "STOP" and self._state in ("WALK", "SIM_WALK", "WALK_ZMP", "WALK_SIM_ZMP", "WIGGLE_SEQ", "WIGGLE_ALL"):
            self._transition("STAND")
        elif cmd == "WIGGLE_SEQ" and self._state == "STAND":
            self._transition("WIGGLE_SEQ")
        elif cmd == "WIGGLE_ALL" and self._state == "STAND":
            self._transition("WIGGLE_ALL")
        elif cmd == "ESTOP":
            self._transition("ESTOP")
        elif cmd == "RESET" and self._state == "ESTOP":
            self._transition("IDLE")

    def _transition(self, new_state: str):
        """Execute state transition."""
        old = self._state
        self._state = new_state
        self._state_start_time = time.time()

        if new_state == "STAND":
            # Capture current positions for soft start interpolation
            self._stand_start_positions = dict(self._current_positions)

        elif new_state in ("WIGGLE_SEQ", "WIGGLE_ALL"):
            self._wiggle_cfg = self._load_wiggle_config()
            self._wiggle_start = time.time()
            self._wiggle_joint_idx = 0

        elif new_state == "ESTOP":
            # Send zero torque immediately
            self._send_zero_torque()

        self.get_logger().info(f'State: {old} → {new_state}')

    def _loop(self):
        """50Hz control loop — behavior depends on state."""
        if self._state == "STAND":
            self._handle_stand()
        elif self._state == "WALK":
            # Policy node handles WALK — just ensure zero cmd if needed
            pass
        elif self._state == "SIM_WALK":
            self._handle_stand_hold()
        elif self._state == "WIGGLE_SEQ":
            self._handle_wiggle_sequential()
        elif self._state == "WIGGLE_ALL":
            self._handle_wiggle_all()
        elif self._state == "ESTOP":
            self._send_zero_torque()

    def _handle_stand(self):
        """Soft start: interpolate to default pose, ramp gains."""
        elapsed = time.time() - self._state_start_time

        # Phase 1: interpolate position (0 → ramp_time)
        pos_alpha = min(elapsed / self._ramp_time, 1.0)

        # Phase 2: ramp gains (ramp_time → ramp_time + gain_ramp_time)
        gain_elapsed = max(0, elapsed - self._ramp_time)
        gain_alpha = min(gain_elapsed / self._gain_ramp_time, 1.0)

        # Soft Kp/Kd
        soft_kp_scale = 0.1 + 0.9 * gain_alpha  # 10% → 100%
        soft_kd_scale = 0.2 + 0.8 * gain_alpha   # 20% → 100%

        # Build interpolated command
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in JOINT_ORDER:
            start_pos = self._stand_start_positions.get(name, DEFAULT_POSITIONS[name])
            target_pos = DEFAULT_POSITIONS[name]
            current_target = start_pos + (target_pos - start_pos) * pos_alpha

            kp_base, kd_base = DEFAULT_GAINS[name]

            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = current_target
            cmd.velocity = 0.0
            gs = float(self.get_parameter("gain_scale").value)
            cmd.kp = kp_base * soft_kp_scale * gs
            cmd.kd = kd_base * soft_kd_scale * gs
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

        # Publish zero velocity during stand
        vel_msg = Twist()
        self._pub_vel.publish(vel_msg)

    def _handle_stand_hold(self):
        """Hold default stand position with full gains. Used during SIM_WALK."""
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        gs = float(self.get_parameter("gain_scale").value)

        for name in JOINT_ORDER:
            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = DEFAULT_POSITIONS[name]
            cmd.velocity = 0.0
            cmd.kp = kp_base * gs
            cmd.kd = kd_base * gs
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _handle_wiggle_sequential(self):
        """One joint at a time through sine wave. Others hold default."""
        elapsed = time.time() - self._wiggle_start
        dur = self._wiggle_cfg['duration_per_joint']
        joint_idx = int(elapsed / dur)

        if joint_idx >= len(JOINT_ORDER):
            self.get_logger().info('Wiggle sequential complete')
            self._transition("STAND")
            return

        active_name = JOINT_ORDER[joint_idx]
        phase_t = elapsed - joint_idx * dur

        if joint_idx != self._wiggle_joint_idx:
            self._wiggle_joint_idx = joint_idx
            self.get_logger().info(f'Wiggle: {active_name} ({joint_idx+1}/{len(JOINT_ORDER)})')

        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        gs = float(self.get_parameter("gain_scale").value)

        for name in JOINT_ORDER:
            default = DEFAULT_POSITIONS[name]
            if name == active_name:
                jcfg = self._wiggle_cfg['joints'][name]
                freq = jcfg['freq']
                sin_val = math.sin(2 * math.pi * freq * phase_t)
                if sin_val >= 0:
                    offset = jcfg['pos'] * sin_val
                else:
                    offset = -jcfg['neg'] * sin_val  # neg is negative, so -neg*sin gives positive when sin<0
                target = default + offset
            else:
                target = default

            # Safety clamp to URDF limits
            if name in JOINT_LIMITS:
                lo, hi = JOINT_LIMITS[name]
                target = max(lo, min(hi, target))

            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = target
            cmd.velocity = 0.0
            cmd.kp = kp_base * gs
            cmd.kd = kd_base * gs
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _handle_wiggle_all(self):
        """All joints wiggle simultaneously."""
        elapsed = time.time() - self._wiggle_start

        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        gs = float(self.get_parameter("gain_scale").value)

        for name in JOINT_ORDER:
            default = DEFAULT_POSITIONS[name]
            jcfg = self._wiggle_cfg['joints'][name]
            freq = jcfg['freq']
            sin_val = math.sin(2 * math.pi * freq * elapsed)
            if sin_val >= 0:
                offset = jcfg['pos'] * sin_val
            else:
                offset = -jcfg['neg'] * sin_val

            target = default + offset

            # Safety clamp to URDF limits
            if name in JOINT_LIMITS:
                lo, hi = JOINT_LIMITS[name]
                target = max(lo, min(hi, target))

            kp_base, kd_base = DEFAULT_GAINS[name]
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = target
            cmd.velocity = 0.0
            cmd.kp = kp_base * gs
            cmd.kd = kd_base * gs
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _send_zero_torque(self):
        """Send zero Kp/Kd/torque to all joints (e-stop)."""
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in JOINT_ORDER:
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = 0.0
            cmd.velocity = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

    def _publish_state(self):
        """Publish state at 10Hz."""
        state_msg = String()
        state_msg.data = self._state
        self._pub_state.publish(state_msg)

        robot_msg = RobotState()
        robot_msg.header.stamp = self.get_clock().now().to_msg()
        robot_msg.fsm_state = self._state
        robot_msg.safety_ok = self._safety_ok
        robot_msg.all_motors_enabled = self._state not in ("IDLE", "ESTOP")
        self._pub_robot.publish(robot_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
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


if __name__ == '__main__':
    main()
