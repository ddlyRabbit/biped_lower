"""ROS2 node for ZMP-based walking trajectory generation and execution.

Subscribes to /state_machine for WALK_ZMP / WALK_SIM_ZMP triggers.
Publishes joint commands at 50Hz stepping through pre-computed trajectory.

Config loaded from zmp_config.yaml (re-read on every WALK_ZMP command).
"""

import os
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

from biped_msgs.msg import MITCommand, MITCommandArray

from .zmp_walker import ZMPWalker
from .footstep_planner import FootstepPlanner, check_zmp_stability
from .biped_ik import BipedIK


# Default config path — same location as other bringup configs
DEFAULT_CONFIG = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..',
    'biped_bringup', 'config', 'zmp_config.yaml'
)


class ZMPTrajectoryNode(Node):
    """Generates and executes ZMP walking trajectories."""

    def __init__(self):
        super().__init__('zmp_trajectory_node')

        # gain_scale from launch file (same param as other nodes)
        self.declare_parameter('gain_scale', 0.3)

        # Config path — can override via ROS param
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            # Try default relative path
            config_path = os.path.expanduser(
                '~/biped_lower/deploy/biped_ws/src/biped_bringup/config/zmp_config.yaml'
            )
        self._config_path = config_path

        # Load config once to get URDF path and init IK
        cfg = self._load_config()
        urdf_path = cfg.get('urdf_path', '')
        if not urdf_path or not os.path.exists(urdf_path):
            self.get_logger().error(f'URDF not found: {urdf_path}')
            return

        self.ik = BipedIK(urdf_path)
        self.get_logger().info(f'IK loaded from {urdf_path}')

        # Default PD gains (same as obs_builder.py)
        self.gains = {
            "L_hip_pitch": (200.0, 7.5), "R_hip_pitch": (200.0, 7.5),
            "L_hip_roll":  (150.0, 5.5), "R_hip_roll":  (150.0, 5.5),
            "L_hip_yaw":   (150.0, 5.0), "R_hip_yaw":   (150.0, 5.0),
            "L_knee":      (200.0, 5.0), "R_knee":      (200.0, 5.0),
            "L_foot_pitch": (30.0, 2.0), "R_foot_pitch": (30.0, 2.0),
            "L_foot_roll":  (30.0, 2.0), "R_foot_roll":  (30.0, 2.0),
        }

        # State
        self._trajectory = None
        self._traj_index = 0
        self._active = False
        self._sim_only = False
        self._last_state = ""
        self._gain_scale = float(self.get_parameter('gain_scale').value)
        self._dt = cfg.get('dt', 0.02)
        self._start_time = None           # set on WALK_ZMP / WALK_SIM_ZMP entry
        self._gain_ramp_secs = 1.0        # 1s gain ramp on entry
        self._ramp_in_secs = 2.0          # 2s interpolation from current pose to ZMP start
        self._ramp_in_frames = int(self._ramp_in_secs / self._dt)
        self._current_positions = {}      # latest joint positions from /joint_states
        self._active_trajectory = None    # ramp_in + trajectory, rebuilt on each activation

        # Subscriptions
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, sensor_qos)
        self.create_subscription(String, '/state_machine', self._state_cb, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._viz_pub = self.create_publisher(JointState, '/policy_viz_joints', 10)

        # Timer at control rate
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(f'ZMP trajectory node ready (config: {self._config_path})')

    def _load_config(self) -> dict:
        """Load config from YAML file. Re-read on every trajectory generation."""
        try:
            with open(self._config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.get_logger().info(f'Config loaded from {self._config_path}')
            return cfg or {}
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            return {}

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._current_positions[name] = msg.position[i]

    def _state_cb(self, msg: String):
        """Handle state machine transitions. Ignore repeated same-state messages."""
        state = msg.data.strip().upper()

        if state == self._last_state:
            return
        self._last_state = state

        if state == 'WALK_SIM_ZMP':
            self._sim_only = True
            self._generate_trajectory()
            ramp_in = self._build_ramp_in()
            self._active_trajectory = ramp_in + self._trajectory
            self._traj_index = 0
            self._start_time = time.time()
            self._active = True
            self.get_logger().info(
                f'ZMP SIM started, {len(self._active_trajectory)} steps '
                f'({len(ramp_in)} ramp-in + {len(self._trajectory)} walk+ramp-down), '
                f'gain ramp {self._gain_ramp_secs}s'
            )

        elif state == 'WALK_ZMP':
            if self._trajectory is None:
                self.get_logger().error(
                    'No trajectory! Run WALK_SIM_ZMP first to generate.'
                )
                return
            self._sim_only = False
            ramp_in = self._build_ramp_in()
            self._active_trajectory = ramp_in + self._trajectory
            self._traj_index = 0
            self._start_time = time.time()
            self._active = True
            self.get_logger().info(
                f'ZMP REAL started, {len(self._active_trajectory)} steps '
                f'({len(ramp_in)} ramp-in + {len(self._trajectory)} walk+ramp-down), '
                f'gain ramp {self._gain_ramp_secs}s'
            )

        elif state in ('STAND', 'IDLE', 'ESTOP'):
            if self._active:
                self._active = False
                self.get_logger().info('ZMP walk stopped')

    def _generate_trajectory(self):
        """Pre-compute full joint trajectory from ZMP. Re-reads config."""
        cfg = self._load_config()

        step_length = cfg.get('step_length', 0.10)
        step_width = cfg.get('step_width', 0.25)
        step_height = cfg.get('step_height', 0.05)
        step_period = cfg.get('step_period', 0.8)
        num_steps = int(cfg.get('num_steps', 10))
        ds_ratio = cfg.get('double_support_ratio', 0.1)
        com_height = cfg.get('com_height', 0.40)
        preview_horizon = int(cfg.get('preview_horizon', 320))
        self._gain_scale = float(self.get_parameter('gain_scale').value)
        self._dt = cfg.get('dt', 0.02)

        self.get_logger().info(
            f'Generating ZMP: length={step_length}m, width={step_width}m, '
            f'height={step_height}m, period={step_period}s, steps={num_steps}, '
            f'com_h={com_height}m, gain_scale={self._gain_scale}'
        )

        # 1. Footstep plan
        planner = FootstepPlanner(
            step_length=step_length,
            step_width=step_width,
            step_height=step_height,
            step_period=step_period,
            num_steps=num_steps,
            double_support_ratio=ds_ratio,
            dt=self._dt,
        )
        plan = planner.plan()

        # 2. ZMP preview control → CoM
        walker = ZMPWalker(
            com_height=com_height,
            dt=self._dt,
            preview_horizon=preview_horizon,
        )
        com_x, com_y, _, _ = walker.generate(plan.zmp_ref_x, plan.zmp_ref_y)

        # 2b. Stability check — ZMP within support polygon (10% buffer)
        stability = check_zmp_stability(
            plan, com_x, com_y,
            foot_length=0.15,
            foot_width=0.08,
            buffer=0.10,
        )
        if stability['stable']:
            self.get_logger().info(
                f'ZMP stability OK — min margin: {stability["min_margin"]:.4f}m'
            )
        else:
            self.get_logger().warn(
                f'ZMP UNSTABLE — {len(stability["violations"])} violations, '
                f'min margin: {stability["min_margin"]:.4f}m at t={stability["worst_time"]:.2f}s. '
                f'Consider reducing step_length or increasing step_period.'
            )

        # 3. IK → joint angles
        self.ik.reset()
        trajectory = []
        for i in range(plan.n_samples):
            joints = self.ik.solve(
                com_x=com_x[i],
                com_y=com_y[i],
                left_foot_pos=plan.left_foot[i],
                right_foot_pos=plan.right_foot[i],
            )
            trajectory.append(joints)

        # Ramp down: IK phase (1s) then joint interpolation to default (1s)
        ramp_ik_frames = int(1.0 / self._dt)    # 1s IK ramp
        ramp_joint_frames = int(1.0 / self._dt)  # 1s joint interpolation

        # Phase 1: IK ramp — move CoM + feet back toward standing
        end_com_x = com_x[-1]
        end_com_y = com_y[-1]
        end_l_foot = plan.left_foot[-1].copy()
        end_r_foot = plan.right_foot[-1].copy()

        # Standing foot positions (relative to CoM=0): centered
        stand_l_foot = np.array([0.0, self.ik.left_foot_standing[1] - self.ik.right_foot_standing[1], 0.0]) * 0.5
        stand_r_foot = -stand_l_foot.copy()
        # Actually just use zero offset (feet directly under torso at default)
        stand_l_foot = np.array([0.0, end_l_foot[1], 0.0])
        stand_r_foot = np.array([0.0, end_r_foot[1], 0.0])
        center_x = (end_l_foot[0] + end_r_foot[0]) / 2.0

        for i in range(1, ramp_ik_frames + 1):
            alpha = i / ramp_ik_frames
            alpha = 0.5 * (1 - np.cos(np.pi * alpha))

            ramp_com_x = end_com_x + alpha * (center_x - end_com_x)
            ramp_com_y = end_com_y + alpha * (0.0 - end_com_y)

            # Interpolate foot positions back to standing (under torso)
            ramp_l = end_l_foot + alpha * (stand_l_foot - end_l_foot)
            ramp_r = end_r_foot + alpha * (stand_r_foot - end_r_foot)

            joints = self.ik.solve(ramp_com_x, ramp_com_y, ramp_l, ramp_r)
            trajectory.append(joints)

        # Phase 2: smooth joint interpolation from last IK frame to exact default
        last_ik_joints = trajectory[-1]
        default_joints = {
            "R_hip_pitch": 0.08, "R_hip_roll": 0.0, "R_hip_yaw": 0.0,
            "R_knee": 0.25, "R_foot_pitch": -0.17, "R_foot_roll": 0.0,
            "L_hip_pitch": -0.08, "L_hip_roll": 0.0, "L_hip_yaw": 0.0,
            "L_knee": 0.25, "L_foot_pitch": -0.17, "L_foot_roll": 0.0,
        }

        for i in range(1, ramp_joint_frames + 1):
            alpha = i / ramp_joint_frames
            alpha = 0.5 * (1 - np.cos(np.pi * alpha))
            frame = {}
            for name in last_ik_joints:
                frame[name] = last_ik_joints[name] + alpha * (default_joints[name] - last_ik_joints[name])
            trajectory.append(frame)

        ramp_down_frames = ramp_ik_frames + ramp_joint_frames

        self._trajectory = trajectory
        self._ramp_in_frames = int(self._ramp_in_secs / self._dt)
        self.get_logger().info(
            f'Trajectory generated: {len(trajectory)} walk + {ramp_down_frames} ramp-down '
            f'(ramp-in {self._ramp_in_frames} frames prepended at activation)'
        )

    def _build_ramp_in(self) -> list:
        """Build ramp-in frames from current joint positions to first trajectory frame."""
        if not self._trajectory:
            return []
        first_ik_joints = self._trajectory[0]
        start_joints = {}
        for name in first_ik_joints:
            start_joints[name] = self._current_positions.get(name, first_ik_joints[name])

        ramp_in = []
        for i in range(self._ramp_in_frames):
            alpha = (i + 1) / self._ramp_in_frames
            alpha = 0.5 * (1 - np.cos(np.pi * alpha))  # cosine ease
            frame = {}
            for name in first_ik_joints:
                frame[name] = start_joints[name] + alpha * (first_ik_joints[name] - start_joints[name])
            ramp_in.append(frame)
        return ramp_in

    def _step(self):
        """Execute one timestep of the trajectory."""
        if not self._active or self._active_trajectory is None:
            return

        if self._traj_index >= len(self._active_trajectory):
            self._active = False
            self.get_logger().info('ZMP trajectory complete')
            return

        joints = self._active_trajectory[self._traj_index]

        if self._traj_index % 50 == 0:
            self.get_logger().info(
                f'ZMP step {self._traj_index}/{len(self._active_trajectory)}'
            )

        self._traj_index += 1

        if self._sim_only:
            self._publish_viz(joints)
        else:
            self._publish_commands(joints)
            self._publish_viz(joints)

    def _publish_commands(self, joints: dict):
        """Publish joint commands to real motors with 1s gain ramp."""
        # Gain ramp: 10% → 100% over _gain_ramp_secs
        ramp = 1.0
        if self._start_time is not None:
            elapsed = time.time() - self._start_time
            ramp = min(1.0, 0.1 + 0.9 * (elapsed / self._gain_ramp_secs))

        msg = MITCommandArray()
        gs = float(self.get_parameter('gain_scale').value)
        for name, angle in joints.items():
            kp, kd = self.gains.get(name, (0.0, 0.0))
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = float(angle)
            cmd.velocity = 0.0
            cmd.kp = float(kp * gs * ramp)
            cmd.kd = float(kd * gs * ramp)
            cmd.torque_ff = 0.0
            msg.commands.append(cmd)
        self._cmd_pub.publish(msg)

    def _publish_viz(self, joints: dict):
        """Publish joint states for visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, angle in joints.items():
            msg.name.append(name)
            msg.position.append(float(angle))
        self._viz_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZMPTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
