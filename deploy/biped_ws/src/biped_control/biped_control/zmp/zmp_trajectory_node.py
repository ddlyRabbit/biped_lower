"""ROS2 node for ZMP-based walking trajectory generation and execution.

Subscribes to /state_machine for WALK_ZMP / WALK_SIM_ZMP triggers.
Publishes joint commands at 50Hz stepping through pre-computed trajectory.

Config loaded from zmp_config.yaml (re-read on every WALK_ZMP command).
"""

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from biped_msgs.msg import MITCommand, MITCommandArray

from .zmp_walker import ZMPWalker
from .footstep_planner import FootstepPlanner
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
        self._gain_scale = cfg.get('gain_scale', 0.3)
        self._dt = cfg.get('dt', 0.02)

        # Subscriptions
        self.create_subscription(String, '/state_machine', self._state_cb, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._viz_pub = self.create_publisher(JointState, '/policy_viz', 10)

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

    def _state_cb(self, msg: String):
        """Handle state machine transitions."""
        state = msg.data.strip().upper()

        if state in ('WALK_ZMP', 'WALK_SIM_ZMP'):
            self._sim_only = (state == 'WALK_SIM_ZMP')
            self._generate_trajectory()
            self._traj_index = 0
            self._active = True
            mode = 'SIM' if self._sim_only else 'REAL'
            self.get_logger().info(
                f'ZMP walk started ({mode}), {len(self._trajectory)} steps'
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
        self._gain_scale = cfg.get('gain_scale', 0.3)
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

        self._trajectory = trajectory
        self.get_logger().info(f'Trajectory generated: {len(trajectory)} frames')

    def _step(self):
        """Execute one timestep of the trajectory."""
        if not self._active or self._trajectory is None:
            return

        if self._traj_index >= len(self._trajectory):
            self._active = False
            self.get_logger().info('ZMP trajectory complete')
            return

        joints = self._trajectory[self._traj_index]
        self._traj_index += 1

        if self._sim_only:
            self._publish_viz(joints)
        else:
            self._publish_commands(joints)
            self._publish_viz(joints)

    def _publish_commands(self, joints: dict):
        """Publish joint commands to real motors."""
        msg = MITCommandArray()
        for name, angle in joints.items():
            kp, kd = self.gains.get(name, (0.0, 0.0))
            cmd = MITCommand()
            cmd.joint_name = name
            cmd.position = float(angle)
            cmd.velocity = 0.0
            cmd.kp = float(kp * self._gain_scale)
            cmd.kd = float(kd * self._gain_scale)
            cmd.torque = 0.0
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
