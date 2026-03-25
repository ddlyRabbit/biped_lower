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

import time
import numpy as np
import yaml
import onnxruntime as ort
import rclpy
import rcl_interfaces.msg
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
# JointState also used for /policy_viz_joints
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import String
from biped_msgs.msg import MITCommand, MITCommandArray
from biped_control.obs_builder import ObsBuilder, ISAAC_JOINT_ORDER, DEFAULT_POSITIONS, DEFAULT_GAINS, ACTION_ORDER


# Soft start timing
SOFT_START_POSITION_SECS = 2.0  # interpolate current → target over this
SOFT_START_GAIN_SECS = 1.0       # ramp gains from 10% → 100% after position interp
WALK_GAIN_RAMP_SECS = 5.0        # ramp gains from 10% → 100% on WALK entry


class PolicyNode(Node):
    """50Hz RL policy inference node with soft start and state machine gating."""

    def __init__(self):
        super().__init__('policy_node')

        self.declare_parameter('onnx_model', 'student_flat.onnx')
        self.declare_parameter('loop_rate', 50.0)
        self.declare_parameter('gains_file', '')
        self.declare_parameter('gain_scale', 1.0,
            rcl_interfaces.msg.ParameterDescriptor(dynamic_typing=True))

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
        self._walk_start_time = None  # set on WALK entry for gain ramp

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)
        self.create_subscription(Vector3Stamped, '/imu/gravity', self._gravity_cb, sensor_qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(String, '/state_machine', self._fsm_cb, 10)

        # Publishers
        self._pub_cmd = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._pub_viz = self.create_publisher(MITCommandArray, '/policy_viz', 10)
        self._pub_viz_js = self.create_publisher(JointState, '/policy_viz_joints', 10)

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
            if new_state in ("WALK", "SIM_WALK"):
                self._walk_start_time = time.time()
            self._fsm_state = new_state

    # --- Main loop ---

    def _loop(self):
        if self._session is None:
            return

        # Run during WALK (motors) or SIM_WALK (viz only)
        if self._fsm_state not in ("WALK", "SIM_WALK"):
            return

        # Need joint state data before we can build obs
        if not self._joint_positions:
            return

        # Build observation
        obs = self._obs_builder.build(
            gyro=self._gyro,
            gravity=self._gravity,
            cmd_vel=self._cmd_vel,  # pass cmd_vel in both WALK and SIM_WALK
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

        # Debug: comprehensive logging every 10 seconds
        if not hasattr(self, '_debug_timer'):
            self._debug_timer = 0
        self._debug_timer += 1
        if self._debug_timer % (int(self._rate) * 10) == 1:  # every 10s
            self.get_logger().info(f'[RAW] gyro={self._gyro} gravity={self._gravity} cmd_vel={self._cmd_vel}')
            self.get_logger().info(f'[RAW] joint_pos={{{", ".join(f"{k}:{v:.4f}" for k,v in sorted(self._joint_positions.items()))}}}')
            self.get_logger().info(f'[RAW] joint_vel={{{", ".join(f"{k}:{v:.4f}" for k,v in sorted(self._joint_velocities.items()))}}}')
            self.get_logger().info(f'[OBS] full_45d={np.array2string(obs, precision=4, separator=",")}')
            self.get_logger().info(f'[ACT] actions={np.array2string(actions, precision=4, separator=",")}')

        # Convert to position targets
        targets = ObsBuilder.action_to_positions(actions)

        if self._debug_timer % (int(self._rate) * 10) == 1:
            tgt_str = ' '.join(f'{n}={targets[n]:+.3f}' for n in ACTION_ORDER)
            self.get_logger().info(f'[TGT] {tgt_str}')


        # Build MIT command array
        cmd_msg = MITCommandArray()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        # Gain ramp on WALK entry — prevent violent transition from STAND
        walk_ramp = 1.0
        if self._walk_start_time is not None:
            walk_elapsed = time.time() - self._walk_start_time
            walk_ramp = min(1.0, 0.1 + 0.9 * (walk_elapsed / WALK_GAIN_RAMP_SECS))

        for name in ISAAC_JOINT_ORDER:
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = targets[name]
            cmd.velocity = 0.0
            kp, kd = self._gains[name]
            gs = float(self.get_parameter("gain_scale").value)
            cmd.kp = kp * gs * walk_ramp
            cmd.kd = kd * gs * walk_ramp
            cmd.torque_ff = 0.0
            cmd_msg.commands.append(cmd)

        if self._debug_timer % (int(self._rate) * 10) == 1:
            for cmd in cmd_msg.commands:
                self.get_logger().info(f'[CMD] {cmd.joint_name} pos={cmd.position:+.4f} kp={cmd.kp:.1f} kd={cmd.kd:.1f}')

        if self._fsm_state == "SIM_WALK":
            self._pub_viz.publish(cmd_msg)  # viz only, motors hold STAND
            # Also publish as JointState for Foxglove plotting
            js = JointState()
            js.header = cmd_msg.header
            js.name = [cmd.joint_name for cmd in cmd_msg.commands]
            js.position = [cmd.position for cmd in cmd_msg.commands]
            js.velocity = [0.0] * len(cmd_msg.commands)
            js.effort = [0.0] * len(cmd_msg.commands)
            self._pub_viz_js.publish(js)
        else:
            self._pub_cmd.publish(cmd_msg)  # actual motor commands


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
