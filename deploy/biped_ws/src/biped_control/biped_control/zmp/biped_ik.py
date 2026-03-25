"""Inverse kinematics for biped using pinocchio + URDF.

Uses pinocchio's CLIK (Closed-Loop IK) with position-only Jacobian.
Each leg solved independently using frame Jacobian masking.

The full trajectory is pre-computed before execution (not real-time).
Requires: pip install pin
"""

import numpy as np
from pathlib import Path
from typing import Dict, Tuple

import pinocchio as pin


# Deploy joint names — pinocchio joint order is alphabetical from URDF
# pinocchio joints[1..12]: left first, then right
PIN_JOINT_TO_DEPLOY = {
    "left_hip_pitch_04": "L_hip_pitch",
    "left_hip_roll_03": "L_hip_roll",
    "left_hip_yaw_03": "L_hip_yaw",
    "left_knee_04": "L_knee",
    "left_foot_pitch_02": "L_foot_pitch",
    "left_foot_roll_02": "L_foot_roll",
    "right_hip_pitch_04": "R_hip_pitch",
    "right_hip_roll_03": "R_hip_roll",
    "right_hip_yaw_03": "R_hip_yaw",
    "right_knee_04": "R_knee",
    "right_foot_pitch_02": "R_foot_pitch",
    "right_foot_roll_02": "R_foot_roll",
}


class BipedIK:
    """IK solver using pinocchio CLIK."""

    def __init__(self, urdf_path: str, dt: float = 0.1, max_iter: int = 100,
                 eps: float = 1e-4, reg_weight: float = 0.1):
        """
        Args:
            urdf_path: Path to robot URDF.
            dt: CLIK step size.
            max_iter: Max CLIK iterations.
            eps: Convergence threshold (meters).
            reg_weight: Regularization weight pulling toward default pose.
        """
        self.urdf_path = str(Path(urdf_path).resolve())
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.dt = dt
        self.max_iter = max_iter
        self.eps = eps
        self.reg_weight = reg_weight

        # Find foot frame IDs
        self.r_foot_id = self.model.getFrameId("foot_6061")
        self.l_foot_id = self.model.getFrameId("foot_6061_2")

        # Build joint index mapping: pinocchio idx_q → deploy name
        self._pin_to_deploy = {}
        self._deploy_to_pin_idx = {}
        for i in range(1, self.model.njoints):
            pin_name = self.model.names[i]
            deploy_name = PIN_JOINT_TO_DEPLOY.get(pin_name)
            if deploy_name:
                idx = self.model.joints[i].idx_q
                self._pin_to_deploy[idx] = deploy_name
                self._deploy_to_pin_idx[deploy_name] = idx

        # Right/left leg joint indices in q vector (for Jacobian masking)
        self._r_indices = []
        self._l_indices = []
        for i in range(1, self.model.njoints):
            idx = self.model.joints[i].idx_q
            if self.model.names[i].startswith("right"):
                self._r_indices.append(idx)
            else:
                self._l_indices.append(idx)

        # Default config
        self._q_default = pin.neutral(self.model)
        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx = self.model.joints[i].idx_q
            if "right_hip_pitch" in name:
                self._q_default[idx] = 0.08
            elif "left_hip_pitch" in name:
                self._q_default[idx] = -0.08
            elif "knee" in name:
                self._q_default[idx] = 0.25
            elif "foot_pitch" in name:
                self._q_default[idx] = -0.17

        self._last_q = self._q_default.copy()

        # Standing foot positions (torso-relative)
        pin.forwardKinematics(self.model, self.data, self._q_default)
        pin.updateFramePlacements(self.model, self.data)
        self.right_foot_standing = self.data.oMf[self.r_foot_id].translation.copy()
        self.left_foot_standing = self.data.oMf[self.l_foot_id].translation.copy()

    def _solve_one_foot(self, q: np.ndarray, foot_id: int,
                        target_pos: np.ndarray, leg_indices: list) -> np.ndarray:
        """CLIK for one foot, only moving the specified leg joints."""
        q = q.copy()
        oMdes = pin.SE3(np.eye(3), target_pos)

        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            dMf = oMdes.inverse() * self.data.oMf[foot_id]
            err = pin.log(dMf).vector[:3]

            if np.linalg.norm(err) < self.eps:
                break

            # Full Jacobian, position rows only
            J_full = pin.computeFrameJacobian(
                self.model, self.data, q, foot_id, pin.LOCAL_WORLD_ALIGNED
            )[:3, :]

            # Mask: only use columns for this leg
            J_leg = np.zeros_like(J_full)
            J_leg[:, leg_indices] = J_full[:, leg_indices]

            # Damped least-squares with regularization toward default pose
            # min ||J*dq - err||^2 + reg * ||q + dq - q_default||^2
            n = J_leg.shape[1]
            reg = self.reg_weight * np.eye(n)
            A = np.vstack([J_leg, reg])
            q_err = self._q_default - q  # pull toward default
            b = np.concatenate([err, self.reg_weight * q_err])
            dq = np.linalg.lstsq(A, b, rcond=None)[0]
            q = pin.integrate(self.model, q, dq * self.dt)

            # Clamp to joint limits
            for idx in leg_indices:
                lo = self.model.lowerPositionLimit[idx]
                hi = self.model.upperPositionLimit[idx]
                q[idx] = np.clip(q[idx], lo, hi)

        return q

    def solve(
        self,
        com_x: float,
        com_y: float,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
    ) -> Dict[str, float]:
        """Solve IK for both legs.

        Foot positions in world frame → torso-relative → IK.
        Torso XY ≈ CoM XY.
        """
        # Foot targets relative to torso
        r_rel = right_foot_pos.copy()
        r_rel[0] -= com_x
        r_rel[1] -= com_y

        l_rel = left_foot_pos.copy()
        l_rel[0] -= com_x
        l_rel[1] -= com_y

        # Solve right leg
        q = self._solve_one_foot(self._last_q, self.r_foot_id, r_rel, self._r_indices)
        # Solve left leg (from updated q)
        q = self._solve_one_foot(q, self.l_foot_id, l_rel, self._l_indices)

        self._last_q = q.copy()

        # Convert to deploy names
        joints = {}
        for idx, name in self._pin_to_deploy.items():
            joints[name] = float(q[idx])

        return joints

    def forward_kinematics(self, joints: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """Compute foot positions from joint angles (torso-relative)."""
        q = self._q_default.copy()
        for name, angle in joints.items():
            if name in self._deploy_to_pin_idx:
                q[self._deploy_to_pin_idx[name]] = angle

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        r = self.data.oMf[self.r_foot_id].translation.copy()
        l = self.data.oMf[self.l_foot_id].translation.copy()
        return r, l

    def reset(self):
        """Reset warm-start cache."""
        self._last_q = self._q_default.copy()
