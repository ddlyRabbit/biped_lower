"""ROS2 node for ZMP-based walking trajectory generation and execution.

Subscribes to /state_machine for WALK_ZMP / WALK_SIM_ZMP triggers.
Publishes joint commands at 50Hz stepping through pre-computed trajectory.

In WALK_SIM_ZMP mode, publishes to /policy_viz only (ghost URDF).
In WALK_ZMP mode, publishes to /joint_commands (real motors).
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

from biped_msgs.msg import MITCommand, MITCommandArray

from .zmp_walker import ZMPWalker
from .footstep_planner import FootstepPlanner
from .biped_ik import BipedIK


class ZMPTrajectoryNode(Node):
    """Generates and executes ZMP walking trajectories."""

    def __init__(self):
        super().__init__('zmp_trajectory_node')

        # Parameters — all configurable via launch file
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('gain_scale', 1.0)

        # ZMP parameters
        self.declare_parameter('step_length', 0.10)
        self.declare_parameter('step_width', 0.25)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('step_period', 0.8)
        self.declare_parameter('num_steps', 10)
        self.declare_parameter('double_support_ratio', 0.1)
        self.declare_parameter('com_height', 0.43)
        self.declare_parameter('preview_horizon', 320)

        # Read params
        urdf_path = self.get_parameter('urdf_path').value
        self.dt = self.get_parameter('dt').value
        self.gain_scale = self.get_parameter('gain_scale').value

        if not urdf_path or not os.path.exists(urdf_path):
            self.get_logger().error(f'URDF not found: {urdf_path}')
            return

        # Initialize IK solver
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
        self._trajectory = None  # List of Dict[joint_name → angle]
        self._traj_index = 0
        self._active = False
        self._sim_only = False

        # Subscriptions
        self.create_subscription(String, '/state_machine', self._state_cb, 10)

        # Publishers
        self._cmd_pub = self.create_publisher(MITCommandArray, '/joint_commands', 10)
        self._viz_pub = self.create_publisher(JointState, '/policy_viz', 10)

        # Timer at control rate
        self._timer = self.create_timer(self.dt, self._step)

        self.get_logger().info('ZMP trajectory node ready')

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
        """Pre-compute full joint trajectory from ZMP."""
        # Read current parameters (supports runtime reconfiguration)
        step_length = self.get_parameter('step_length').value
        step_width = self.get_parameter('step_width').value
        step_height = self.get_parameter('step_height').value
        step_period = self.get_parameter('step_period').value
        num_steps = int(self.get_parameter('num_steps').value)
        ds_ratio = self.get_parameter('double_support_ratio').value
        com_height = self.get_parameter('com_height').value
        preview_horizon = int(self.get_parameter('preview_horizon').value)
        self.gain_scale = self.get_parameter('gain_scale').value

        self.get_logger().info(
            f'Generating ZMP: length={step_length}m, width={step_width}m, '
            f'height={step_height}m, period={step_period}s, steps={num_steps}, '
            f'com_h={com_height}m, gain_scale={self.gain_scale}'
        )

        # 1. Footstep plan → ZMP reference + foot trajectories
        planner = FootstepPlanner(
            step_length=step_length,
            step_width=step_width,
            step_height=step_height,
            step_period=step_period,
            num_steps=num_steps,
            double_support_ratio=ds_ratio,
            dt=self.dt,
        )
        plan = planner.plan()

        # 2. ZMP preview control → CoM trajectory
        walker = ZMPWalker(
            com_height=com_height,
            dt=self.dt,
            preview_horizon=preview_horizon,
        )
        com_x, com_y, _, _ = walker.generate(plan.zmp_ref_x, plan.zmp_ref_y)

        # 3. IK → joint angles per timestep
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
            cmd.kp = float(kp * self.gain_scale)
            cmd.kd = float(kd * self.gain_scale)
            cmd.torque = 0.0
            msg.commands.append(cmd)
        self._cmd_pub.publish(msg)

    def _publish_viz(self, joints: dict):
        """Publish joint states for visualization (ghost URDF in Foxglove)."""
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
