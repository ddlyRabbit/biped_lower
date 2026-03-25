"""Inverse kinematics for biped using pinocchio CLIK.

Uses damped least-squares with regularization toward default pose.
Each leg solved independently using frame Jacobian column masking.

The full trajectory is pre-computed before execution (not real-time).
Requires: pip install pin
"""

import numpy as np
from pathlib import Path
from typing import Dict, Tuple

import pinocchio as pin


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

# Mirroring map: how to initialize left leg from right leg solution
# The URDF has mirrored joint frames (π rotations), so joint angles
# have opposite signs for pitch/yaw, same sign for roll/knee
MIRROR_MAP = {
    # right_joint_idx → (left_joint_idx, sign)
    # Pitch axes are inverted between L/R due to frame π rotation
    # Roll keeps sign, Yaw inverts
}


class BipedIK:
    """IK solver using pinocchio CLIK with proper L/R initialization."""

    def __init__(self, urdf_path: str, dt: float = 0.1, max_iter: int = 150,
                 eps: float = 5e-4, damping: float = 1e-3):
        self.urdf_path = str(Path(urdf_path).resolve())
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.dt = dt
        self.max_iter = max_iter
        self.eps = eps
        self.damping = damping

        self.r_foot_id = self.model.getFrameId("foot_6061")
        self.l_foot_id = self.model.getFrameId("foot_6061_2")

        # Build index maps
        self._pin_to_deploy = {}
        self._deploy_to_pin_idx = {}
        self._r_indices = []
        self._l_indices = []

        for i in range(1, self.model.njoints):
            name = self.model.names[i]
            idx = self.model.joints[i].idx_q
            deploy_name = PIN_JOINT_TO_DEPLOY.get(name)
            if deploy_name:
                self._pin_to_deploy[idx] = deploy_name
                self._deploy_to_pin_idx[deploy_name] = idx
                if name.startswith("right"):
                    self._r_indices.append(idx)
                else:
                    self._l_indices.append(idx)

        # Default config
        self._q_default = pin.neutral(self.model)
        defaults = {
            "R_hip_pitch": 0.08, "L_hip_pitch": -0.08,
            "R_knee": 0.25, "L_knee": 0.25,
            "R_foot_pitch": -0.17, "L_foot_pitch": -0.17,
        }
        for name, val in defaults.items():
            if name in self._deploy_to_pin_idx:
                self._q_default[self._deploy_to_pin_idx[name]] = val

        self._last_q = self._q_default.copy()

        # Standing foot positions
        pin.forwardKinematics(self.model, self.data, self._q_default)
        pin.updateFramePlacements(self.model, self.data)
        self.right_foot_standing = self.data.oMf[self.r_foot_id].translation.copy()
        self.left_foot_standing = self.data.oMf[self.l_foot_id].translation.copy()

    def _solve_one_foot(self, q: np.ndarray, foot_id: int,
                        target_pos: np.ndarray, leg_indices: list) -> np.ndarray:
        """Damped least-squares CLIK for one foot."""
        q = q.copy()
        oMdes = pin.SE3(np.eye(3), target_pos)

        for it in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            current_pos = self.data.oMf[foot_id].translation
            err = target_pos - current_pos

            if np.linalg.norm(err) < self.eps:
                break

            J_full = pin.computeFrameJacobian(
                self.model, self.data, q, foot_id, pin.LOCAL_WORLD_ALIGNED
            )[:3, :]

            # Extract only this leg's columns
            J_leg = J_full[:, leg_indices]

            # Damped least-squares: dq = J^T (J J^T + λI)^{-1} err
            JJT = J_leg @ J_leg.T + self.damping * np.eye(3)
            dq_leg = J_leg.T @ np.linalg.solve(JJT, err)

            # Apply to full q
            dq = np.zeros(self.model.nv)
            for i, idx in enumerate(leg_indices):
                dq[idx] = dq_leg[i]

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
        """Solve IK for both legs."""
        # Foot targets relative to torso
        # Planner gives world-frame XY + swing Z offset (ground = 0).
        # IK needs torso-frame coordinates where standing foot Z ≈ -0.74.
        # Convert: torso-relative = world_foot - torso_pos
        # torso_pos ≈ (com_x, com_y, 0) in world (torso on ground plane + leg height)
        # But in torso frame, foot Z = standing_z + swing_offset
        r_rel = np.array([
            right_foot_pos[0] - com_x,
            right_foot_pos[1] - com_y,
            self.right_foot_standing[2] + right_foot_pos[2],  # standing Z + swing clearance
        ])

        l_rel = np.array([
            left_foot_pos[0] - com_x,
            left_foot_pos[1] - com_y,
            self.left_foot_standing[2] + left_foot_pos[2],  # standing Z + swing clearance
        ])

        # Solve right leg first
        q = self._solve_one_foot(self._last_q, self.r_foot_id, r_rel, self._r_indices)

        # Initialize left leg from mirrored right leg solution
        # All left joint axes are negated relative to right in our URDF,
        # so q_left = -q_right gives the same physical pose (mirrored).
        for r_idx, l_idx in zip(self._r_indices, self._l_indices):
            q[l_idx] = -q[r_idx]

        # Solve left leg (CLIK fine-tunes from mirrored init)
        q = self._solve_one_foot(q, self.l_foot_id, l_rel, self._l_indices)

        self._last_q = q.copy()

        joints = {}
        for idx, name in self._pin_to_deploy.items():
            joints[name] = float(q[idx])
        return joints

    def forward_kinematics(self, joints: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        q = self._q_default.copy()
        for name, angle in joints.items():
            if name in self._deploy_to_pin_idx:
                q[self._deploy_to_pin_idx[name]] = angle
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return (self.data.oMf[self.r_foot_id].translation.copy(),
                self.data.oMf[self.l_foot_id].translation.copy())

    def reset(self):
        self._last_q = self._q_default.copy()
