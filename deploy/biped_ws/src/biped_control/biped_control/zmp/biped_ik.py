"""Inverse kinematics for biped using ikpy + URDF.

Solves for 12 joint angles given desired CoM position and foot positions.
Uses ikpy's numerical IK with our URDF kinematic chain.

The full trajectory is pre-computed before execution, so IK speed per frame
is not critical (not real-time).
"""

import numpy as np
import ikpy.chain
from typing import Dict, Tuple, Optional


class BipedIK:
    """IK solver for biped robot using ikpy."""

    # URDF joint limits (from robot.urdf)
    # Format: [base(ignored), hip_pitch, hip_roll, hip_yaw, knee, foot_pitch, foot_roll]
    RIGHT_LIMITS_LOW = np.array([0, -2.217, -2.269, -1.571, 0.0, -0.873, -0.262])
    RIGHT_LIMITS_HIGH = np.array([0, 1.047, 0.209, 1.571, 2.705, 0.524, 0.262])
    LEFT_LIMITS_LOW = np.array([0, -1.047, -0.209, -1.571, 0.0, -0.873, -0.262])
    LEFT_LIMITS_HIGH = np.array([0, 2.217, 2.269, 1.571, 2.705, 0.524, 0.262])

    def __init__(self, urdf_path: str):
        """
        Args:
            urdf_path: Path to robot URDF file.
        """
        self.urdf_path = urdf_path

        # Build kinematic chains from URDF
        self.right_leg = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["assy_formfg___kd_b_102b_torso_btm", "right_hip_pitch_04"],
            name="right_leg",
        )
        self.left_leg = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["assy_formfg___kd_b_102b_torso_btm", "left_hip_pitch_04"],
            name="left_leg",
        )

        # Default joint angles for initial guess — physically reasonable standing pose
        # [base, hip_pitch, hip_roll, hip_yaw, knee, foot_pitch, foot_roll]
        self.right_default = np.array([0, 0.08, 0, 0, 0.25, -0.17, 0])
        self.left_default = np.array([0, -0.08, 0, 0, 0.25, -0.17, 0])

        # Cache last solution for warm-starting
        self._last_right = self.right_default.copy()
        self._last_left = self.left_default.copy()

        # Compute standing foot positions via FK
        r_fk = self.right_leg.forward_kinematics(self.right_default)
        l_fk = self.left_leg.forward_kinematics(self.left_default)
        self.right_foot_standing = r_fk[:3, 3].copy()
        self.left_foot_standing = l_fk[:3, 3].copy()

    def _clamp_to_limits(self, angles: np.ndarray, side: str) -> np.ndarray:
        """Clamp joint angles to URDF limits."""
        if side == "right":
            return np.clip(angles, self.RIGHT_LIMITS_LOW, self.RIGHT_LIMITS_HIGH)
        else:
            return np.clip(angles, self.LEFT_LIMITS_LOW, self.LEFT_LIMITS_HIGH)

    def solve(
        self,
        com_x: float,
        com_y: float,
        left_foot_pos: np.ndarray,
        right_foot_pos: np.ndarray,
    ) -> Dict[str, float]:
        """Solve IK for both legs.

        Foot positions are in world frame; converted to torso-relative for IK.
        Torso XY ≈ CoM XY (approximation — torso is heaviest single body).

        Args:
            com_x: CoM X position in world frame.
            com_y: CoM Y position in world frame.
            left_foot_pos: (3,) XYZ of left foot in world frame.
            right_foot_pos: (3,) XYZ of right foot in world frame.

        Returns:
            Dict mapping joint name → angle (radians).
        """
        # Foot targets relative to torso
        r_target = right_foot_pos.copy()
        r_target[0] -= com_x
        r_target[1] -= com_y

        l_target = left_foot_pos.copy()
        l_target[0] -= com_x
        l_target[1] -= com_y

        # Solve IK with warm start from last solution
        r_angles = self.right_leg.inverse_kinematics(
            r_target,
            initial_position=self._last_right,
        )
        l_angles = self.left_leg.inverse_kinematics(
            l_target,
            initial_position=self._last_left,
        )

        # Clamp to joint limits
        r_angles = self._clamp_to_limits(r_angles, "right")
        l_angles = self._clamp_to_limits(l_angles, "left")

        # Cache for next iteration
        self._last_right = r_angles.copy()
        self._last_left = l_angles.copy()

        # Extract active joint angles (skip base link at index 0)
        joints = {
            "R_hip_pitch": float(r_angles[1]),
            "R_hip_roll": float(r_angles[2]),
            "R_hip_yaw": float(r_angles[3]),
            "R_knee": float(r_angles[4]),
            "R_foot_pitch": float(r_angles[5]),
            "R_foot_roll": float(r_angles[6]),
            "L_hip_pitch": float(l_angles[1]),
            "L_hip_roll": float(l_angles[2]),
            "L_hip_yaw": float(l_angles[3]),
            "L_knee": float(l_angles[4]),
            "L_foot_pitch": float(l_angles[5]),
            "L_foot_roll": float(l_angles[6]),
        }

        return joints

    def forward_kinematics(self, joints: Dict[str, float]) -> Tuple[np.ndarray, np.ndarray]:
        """Compute foot positions from joint angles.

        Returns:
            (right_foot_pos, left_foot_pos) each (3,) in torso frame.
        """
        r_angles = np.array([
            0,
            joints.get("R_hip_pitch", 0),
            joints.get("R_hip_roll", 0),
            joints.get("R_hip_yaw", 0),
            joints.get("R_knee", 0),
            joints.get("R_foot_pitch", 0),
            joints.get("R_foot_roll", 0),
        ])
        l_angles = np.array([
            0,
            joints.get("L_hip_pitch", 0),
            joints.get("L_hip_roll", 0),
            joints.get("L_hip_yaw", 0),
            joints.get("L_knee", 0),
            joints.get("L_foot_pitch", 0),
            joints.get("L_foot_roll", 0),
        ])

        r_fk = self.right_leg.forward_kinematics(r_angles)[:3, 3]
        l_fk = self.left_leg.forward_kinematics(l_angles)[:3, 3]
        return r_fk, l_fk

    def reset(self):
        """Reset warm-start cache to defaults."""
        self._last_right = self.right_default.copy()
        self._last_left = self.left_default.copy()
