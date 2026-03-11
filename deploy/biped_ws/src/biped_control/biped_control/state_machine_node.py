"""State machine node — IDLE → STAND → WALK → ESTOP.

Manages robot lifecycle transitions. Publishes current state.
Reads safety status and gamepad buttons for transitions.

States:
    IDLE    — motors disabled, waiting for START command
    STAND   — soft start, then policy with zero velocity
    WALK    — policy with cmd_vel input
    ESTOP   — zero torque, requires manual reset
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from biped_msgs.msg import MITCommand, MITCommandArray, RobotState
from biped_control.obs_builder import ISAAC_JOINT_ORDER, DEFAULT_POSITIONS


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

        self._ramp_time = float(self.get_parameter('stand_ramp_time').value)
        self._gain_ramp_time = float(self.get_parameter('stand_gain_ramp_time').value)
        self._stable_time = float(self.get_parameter('stand_stable_time').value)

        # State
        self._state = "IDLE"
        self._state_start_time = time.time()
        self._safety_ok = True
        self._current_positions = {n: 0.0 for n in ISAAC_JOINT_ORDER}
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
        elif cmd == "STOP" and self._state == "WALK":
            self._transition("STAND")
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

        from biped_control.policy_node import DEFAULT_GAINS

        for name in ISAAC_JOINT_ORDER:
            start_pos = self._stand_start_positions.get(name, DEFAULT_POSITIONS[name])
            target_pos = DEFAULT_POSITIONS[name]
            current_target = start_pos + (target_pos - start_pos) * pos_alpha

            kp_base, kd_base = DEFAULT_GAINS[name]

            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = current_target
            cmd.velocity = 0.0
            cmd.kp = kp_base * soft_kp_scale
            cmd.kd = kd_base * soft_kd_scale
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)

        # Publish zero velocity during stand
        vel_msg = Twist()
        self._pub_vel.publish(vel_msg)

    def _send_zero_torque(self):
        """Send zero Kp/Kd/torque to all joints (e-stop)."""
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in ISAAC_JOINT_ORDER:
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
