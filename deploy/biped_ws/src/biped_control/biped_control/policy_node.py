"""Policy inference node — 50Hz ONNX inference loop.

Subscribes to IMU, joint states, and cmd_vel. Builds 45d observation,
runs ONNX inference, converts actions to MIT commands, publishes.

Usage:
    ros2 run biped_control policy_node --ros-args \
        -p onnx_model:=student_flat.onnx \
        -p gains_file:=gains.yaml
"""

import numpy as np
import yaml
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist, Vector3Stamped
from biped_msgs.msg import MITCommand, MITCommandArray
from biped_control.obs_builder import ObsBuilder, ISAAC_JOINT_ORDER, DEFAULT_POSITIONS

# Default PD gains (from training config, halved Berkeley values)
DEFAULT_GAINS = {
    "L_hip_pitch": (15.0, 3.0), "R_hip_pitch": (15.0, 3.0),
    "L_hip_roll":  (10.0, 3.0), "R_hip_roll":  (10.0, 3.0),
    "L_hip_yaw":   (10.0, 3.0), "R_hip_yaw":   (10.0, 3.0),
    "L_knee":      (15.0, 3.0), "R_knee":      (15.0, 3.0),
    "L_foot_pitch": (2.0, 0.2), "R_foot_pitch": (2.0, 0.2),
    "L_foot_roll":  (2.0, 0.2), "R_foot_roll":  (2.0, 0.2),
}


class PolicyNode(Node):
    """50Hz RL policy inference node.

    Publishes:
        /joint_commands   MITCommandArray  @ 50Hz
        /policy/obs       Float32MultiArray (debug)

    Subscribes:
        /imu/data         Imu
        /imu/gravity      Vector3Stamped
        /joint_states     JointState
        /cmd_vel          Twist
        /state_machine    String (only runs when STAND or WALK)
    """

    def __init__(self):
        super().__init__('policy_node')

        self.declare_parameter('onnx_model', 'student_flat.onnx')
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('gains_file', '')
        self.declare_parameter('gain_scale', 1.0)  # multiply all gains by this

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
        self._gravity = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._joint_positions = {n: DEFAULT_POSITIONS[n] for n in ISAAC_JOINT_ORDER}
        self._joint_velocities = {n: 0.0 for n in ISAAC_JOINT_ORDER}
        self._fsm_state = "IDLE"
        self._enabled = False

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)
        self.create_subscription(Vector3Stamped, '/imu/gravity', self._gravity_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

        # Publisher
        self._pub_cmd = self.create_publisher(MITCommandArray, '/joint_commands', 10)

        # Timer
        self._timer = self.create_timer(1.0 / self._rate, self._loop)

        self.get_logger().info(
            f'Policy node started — {self._rate}Hz, gain_scale={self._gain_scale}'
        )

    def _load_gains(self, path: str):
        """Load per-joint Kp/Kd from YAML."""
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

    # --- Subscriber callbacks ---

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
            if name in self._joint_positions:
                if i < len(msg.position):
                    self._joint_positions[name] = msg.position[i]
                if i < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[i]

    def _cmd_cb(self, msg: Twist):
        self._cmd_vel[0] = msg.linear.x
        self._cmd_vel[1] = msg.linear.y
        self._cmd_vel[2] = msg.angular.z

    # --- Main loop ---

    def _loop(self):
        if self._session is None:
            return

        # Build observation
        obs = self._obs_builder.build(
            gyro=self._gyro,
            gravity=self._gravity,
            cmd_vel=self._cmd_vel,
            joint_positions=self._joint_positions,
            joint_velocities=self._joint_velocities,
        )

        # Run inference
        obs_input = obs.reshape(1, -1).astype(np.float32)
        actions = self._session.run(None, {'obs': obs_input})[0][0]  # (12,)

        # Update last action for next obs
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
            cmd.kp = kp * self._gain_scale
            cmd.kd = kd * self._gain_scale
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
