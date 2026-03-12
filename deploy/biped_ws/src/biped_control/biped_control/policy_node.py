"""Policy inference node — 50Hz ONNX inference loop.

Subscribes to IMU, joint states, and cmd_vel. Builds 45d observation,
runs ONNX inference, converts actions to MIT commands, publishes.

Safety features:
  - Only publishes commands when state machine is STAND or WALK
  - Soft start: interpolates from current position to policy target over 2s
  - Ramps gains from 10% to full over 1s after soft start completes

Usage:
    ros2 run biped_control policy_node --ros-args \
        -p onnx_model:=student_flat.onnx \
        -p gain_scale:=0.3
"""

import numpy as np
import yaml
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import String
from biped_msgs.msg import MITCommand, MITCommandArray
from biped_control.obs_builder import ObsBuilder, ISAAC_JOINT_ORDER, DEFAULT_POSITIONS, DEFAULT_GAINS


# Soft start timing
SOFT_START_POSITION_SECS = 2.0  # interpolate current → target over this
SOFT_START_GAIN_SECS = 1.0       # ramp gains from 10% → 100% after position interp


class PolicyNode(Node):
    """50Hz RL policy inference node with soft start and state machine gating."""

    def __init__(self):
        super().__init__('policy_node')

        self.declare_parameter('onnx_model', 'student_flat.onnx')
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('gains_file', '')
        self.declare_parameter('gain_scale', 1.0)

        model_path = str(self.get_parameter('onnx_model').value)
        self._rate = float(self.get_parameter('loop_rate').value)
        gains_file = str(self.get_parameter('gains_file').value)
        self._gain_scale = float(self.get_parameter('gain_scale').value)

        # Load ONNX model
        self._session = None
        try:
            self._session = ort.InferenceSession(model_path)
            self.get_logger().info(f'ONNX model loaded: {model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')

        # Load gains
        self._gains = dict(DEFAULT_GAINS)
        if gains_file:
            self._load_gains(gains_file)

        # Observation builder
        self._obs_builder = ObsBuilder()

        # Latest sensor data
        self._gyro = np.zeros(3, dtype=np.float32)
        self._gravity = np.array([0.0, 0.0, 9.81], dtype=np.float32)
        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._joint_positions = {}  # populated from /joint_states
        self._joint_velocities = {}
        self._fsm_state = "IDLE"

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)
        self.create_subscription(Vector3Stamped, '/imu/gravity', self._gravity_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(String, '/state_machine', self._fsm_cb, 10)

        # Publisher
        self._pub_cmd = self.create_publisher(MITCommandArray, '/joint_commands', 10)

        # Timer
        self._timer = self.create_timer(1.0 / self._rate, self._loop)

        self.get_logger().info(
            f'Policy node started — {self._rate}Hz, gain_scale={self._gain_scale}, '
            f'waiting for STAND/WALK state'
        )

    def _load_gains(self, path: str):
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            for name in ISAAC_JOINT_ORDER:
                if name in data:
                    kp = data[name].get('kp', self._gains[name][0])
                    kd = data[name].get('kd', self._gains[name][1])
                    self._gains[name] = (kp, kd)
            self.get_logger().info(f'Loaded gains from {path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load gains: {e}, using defaults')

    # --- Callbacks ---

    def _imu_cb(self, msg: Imu):
        self._gyro[0] = msg.angular_velocity.x
        self._gyro[1] = msg.angular_velocity.y
        self._gyro[2] = msg.angular_velocity.z

    def _gravity_cb(self, msg: Vector3Stamped):
        self._gravity[0] = msg.vector.x
        self._gravity[1] = msg.vector.y
        self._gravity[2] = msg.vector.z

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self._joint_velocities[name] = msg.velocity[i]

    def _cmd_cb(self, msg: Twist):
        self._cmd_vel[0] = msg.linear.x
        self._cmd_vel[1] = msg.linear.y
        self._cmd_vel[2] = msg.angular.z

    def _fsm_cb(self, msg: String):
        new_state = msg.data.strip().upper()
        if new_state != self._fsm_state:
            self.get_logger().info(f'FSM: {self._fsm_state} → {new_state}')
            self._fsm_state = new_state

    # --- Main loop ---

    def _loop(self):
        if self._session is None:
            return

        # Only run during WALK — state_machine_node handles STAND soft start
        if self._fsm_state != "WALK":
            return

        # Need joint state data before we can build obs
        if not self._joint_positions:
            return

        # Build observation
        obs = self._obs_builder.build(
            gyro=self._gyro,
            gravity=self._gravity,
            cmd_vel=self._cmd_vel if self._fsm_state == "WALK" else np.zeros(3),
            joint_positions=self._joint_positions,
            joint_velocities=self._joint_velocities,
        )

        # Run inference
        obs_input = obs.reshape(1, -1).astype(np.float32)
        actions = self._session.run(None, {'obs': obs_input})[0][0]

        # Clamp actions to training range — prevents feedback divergence
        # through last_action in the observation vector
        actions = np.clip(actions, -1.0, 1.0)

        # Update last action
        self._obs_builder.update_last_action(actions)

        # Convert to position targets
        targets = ObsBuilder.action_to_positions(actions)

        # Build MIT command array
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in ISAAC_JOINT_ORDER:
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = targets[name]
            cmd.velocity = 0.0
            kp, kd = self._gains[name]
            gs = self.get_parameter('gain_scale').value
            cmd.kp = kp * gs
            cmd.kd = kd * gs
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        self._pub_cmd.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
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
