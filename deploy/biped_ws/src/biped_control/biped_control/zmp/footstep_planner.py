"""Footstep planner for ZMP walking.

Generates:
1. ZMP reference trajectory from footstep sequence
2. Foot swing trajectories (cubic spline in Z, linear in XY)
3. Per-timestep foot target positions for IK
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class FootstepPlan:
    """Complete walking plan output."""
    zmp_ref_x: np.ndarray       # (N,) ZMP reference X
    zmp_ref_y: np.ndarray       # (N,) ZMP reference Y
    left_foot: np.ndarray       # (N, 3) left foot XYZ targets
    right_foot: np.ndarray      # (N, 3) right foot XYZ targets
    support_foot: np.ndarray    # (N,) 0=double, 1=left, 2=right (which foot is support)
    dt: float
    n_samples: int


class FootstepPlanner:
    """Generate footstep sequence and ZMP/swing trajectories."""

    def __init__(
        self,
        step_length: float = 0.10,
        step_width: float = 0.25,
        step_height: float = 0.05,
        step_period: float = 0.8,
        num_steps: int = 10,
        double_support_ratio: float = 0.1,
        dt: float = 0.02,
    ):
        """
        Args:
            step_length: Forward distance per step (meters).
            step_width: Lateral foot-to-foot distance (meters). From FK.
            step_height: Swing foot clearance (meters).
            step_period: Duration of one step (seconds).
            num_steps: Number of steps to take.
            double_support_ratio: Fraction of step period in double support.
            dt: Control timestep (seconds).
        """
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.step_period = step_period
        self.num_steps = num_steps
        self.ds_ratio = double_support_ratio
        self.dt = dt

        self.samples_per_step = int(step_period / dt)
        self.ds_samples = int(self.samples_per_step * double_support_ratio)
        self.ss_samples = self.samples_per_step - self.ds_samples

    def plan(self) -> FootstepPlan:
        """Generate complete walking plan.

        Sequence: idle → walk N steps → idle
        Total phases: 1 idle + N steps + 1 idle = N+2 phases
        """
        n_phases = self.num_steps + 2
        n_total = n_phases * self.samples_per_step

        zmp_x = np.zeros(n_total)
        zmp_y = np.zeros(n_total)
        left_foot = np.zeros((n_total, 3))
        right_foot = np.zeros((n_total, 3))
        support = np.zeros(n_total, dtype=int)  # 0=double, 1=left_support, 2=right_support

        # Initial foot positions (standing)
        half_width = self.step_width / 2.0
        l_pos = np.array([0.0, half_width, 0.0])
        r_pos = np.array([0.0, -half_width, 0.0])

        # Phase 0: Idle (double support, feet at start)
        s = 0
        e = self.samples_per_step
        zmp_x[s:e] = 0.0
        zmp_y[s:e] = 0.0
        left_foot[s:e] = l_pos
        right_foot[s:e] = r_pos
        support[s:e] = 0

        # Walking steps
        for step_idx in range(1, self.num_steps + 1):
            s = step_idx * self.samples_per_step
            e = s + self.samples_per_step

            # Alternating: odd=left swing (right support), even=right swing (left support)
            # First step: right foot is support, left swings forward
            left_is_swing = (step_idx % 2 == 1)

            # Target positions for swing foot
            forward = (step_idx) * self.step_length
            # First step is half length, last step brings feet together
            if step_idx == 1:
                swing_target_x = self.step_length * 0.5
            elif step_idx == self.num_steps:
                swing_target_x = forward - self.step_length * 0.5
            else:
                swing_target_x = forward

            if left_is_swing:
                swing_target = np.array([swing_target_x, half_width, 0.0])
                support_pos = r_pos.copy()
                support[s:e] = 2  # right is support
                # ZMP under support foot
                zmp_x[s:e] = support_pos[0]
                zmp_y[s:e] = support_pos[1]
            else:
                swing_target = np.array([swing_target_x, -half_width, 0.0])
                support_pos = l_pos.copy()
                support[s:e] = 1  # left is support
                zmp_x[s:e] = support_pos[0]
                zmp_y[s:e] = support_pos[1]

            # Generate swing trajectory
            if left_is_swing:
                swing_start = l_pos.copy()
                # Support foot stays
                right_foot[s:e] = r_pos
                # Swing foot trajectory
                for i in range(self.samples_per_step):
                    t = i / self.samples_per_step
                    # Double support at start and end
                    if i < self.ds_samples // 2 or i >= self.samples_per_step - self.ds_samples // 2:
                        left_foot[s + i] = swing_start if i < self.ds_samples // 2 else swing_target
                    else:
                        # Swing phase: linear XY, parabolic Z
                        ss_t = (i - self.ds_samples // 2) / self.ss_samples
                        xy = swing_start[:2] + ss_t * (swing_target[:2] - swing_start[:2])
                        z = 4 * self.step_height * ss_t * (1 - ss_t)  # parabola
                        left_foot[s + i] = [xy[0], xy[1], z]
                l_pos = swing_target.copy()
            else:
                swing_start = r_pos.copy()
                left_foot[s:e] = l_pos
                for i in range(self.samples_per_step):
                    t = i / self.samples_per_step
                    if i < self.ds_samples // 2 or i >= self.samples_per_step - self.ds_samples // 2:
                        right_foot[s + i] = swing_start if i < self.ds_samples // 2 else swing_target
                    else:
                        ss_t = (i - self.ds_samples // 2) / self.ss_samples
                        xy = swing_start[:2] + ss_t * (swing_target[:2] - swing_start[:2])
                        z = 4 * self.step_height * ss_t * (1 - ss_t)
                        right_foot[s + i] = [xy[0], xy[1], z]
                r_pos = swing_target.copy()

        # Final idle phase
        s = (self.num_steps + 1) * self.samples_per_step
        e = n_total
        zmp_x[s:e] = (l_pos[0] + r_pos[0]) / 2.0
        zmp_y[s:e] = 0.0
        left_foot[s:e] = l_pos
        right_foot[s:e] = r_pos
        support[s:e] = 0

        return FootstepPlan(
            zmp_ref_x=zmp_x,
            zmp_ref_y=zmp_y,
            left_foot=left_foot,
            right_foot=right_foot,
            support_foot=support,
            dt=self.dt,
            n_samples=n_total,
        )
