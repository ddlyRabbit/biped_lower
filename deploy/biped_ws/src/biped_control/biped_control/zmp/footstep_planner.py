"""Footstep planner for ZMP walking.

Generates:
1. ZMP reference trajectory from footstep sequence
2. Foot swing trajectories (cubic spline in Z, linear in XY)
3. Per-timestep foot target positions for IK
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional


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
                prev_support_pos = l_pos.copy()
                support[s:e] = 2  # right is support
            else:
                swing_target = np.array([swing_target_x, -half_width, 0.0])
                support_pos = l_pos.copy()
                prev_support_pos = r_pos.copy()
                support[s:e] = 1  # left is support

            # ZMP trajectory: interpolate during double support, hold during single
            ds_half = self.ds_samples // 2
            for i in range(self.samples_per_step):
                if i < ds_half:
                    # Initial double support: ZMP ramps from previous foot to current support
                    t_ds = i / max(ds_half, 1)
                    zmp_x[s + i] = prev_support_pos[0] + t_ds * (support_pos[0] - prev_support_pos[0])
                    zmp_y[s + i] = prev_support_pos[1] + t_ds * (support_pos[1] - prev_support_pos[1])
                    support[s + i] = 0  # double support during ramp
                elif i >= self.samples_per_step - ds_half:
                    # Final double support: ZMP stays at support foot (next step will ramp)
                    zmp_x[s + i] = support_pos[0]
                    zmp_y[s + i] = support_pos[1]
                    support[s + i] = 0  # double support
                else:
                    # Single support: ZMP at support foot
                    zmp_x[s + i] = support_pos[0]
                    zmp_y[s + i] = support_pos[1]

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


def check_zmp_stability(
    plan: FootstepPlan,
    zmp_x: np.ndarray,
    zmp_y: np.ndarray,
    foot_length: float = 0.15,
    foot_width: float = 0.08,
    buffer: float = 0.10,
) -> dict:
    """Check if ZMP stays within the support polygon with a safety buffer.

    Args:
        plan: Footstep plan with foot positions and support phase.
        zmp_x, zmp_y: Realized ZMP trajectory from preview controller.
        foot_length: Foot sole length (meters), centered on foot frame.
        foot_width: Foot sole width (meters), centered on foot frame.
        buffer: Safety buffer as fraction of polygon (0.10 = 10% from edges).

    Returns:
        dict with:
          stable: bool — True if ZMP stays within buffered polygon everywhere.
          violations: list of (timestep, margin) where margin < 0.
          min_margin: minimum margin across trajectory.
          worst_phase: which phase (step index) has worst margin.
    """
    n = plan.n_samples
    shrink = 1.0 - buffer  # shrink polygon by buffer fraction

    violations = []
    min_margin = float('inf')
    worst_step = -1

    for i in range(n):
        sup = plan.support_foot[i]
        lf = plan.left_foot[i]
        rf = plan.right_foot[i]

        if sup == 0:
            # Double support: polygon is convex hull of both feet
            # Simplified: rectangle from min/max of both foot bounds
            x_lo = min(lf[0], rf[0]) - foot_length / 2
            x_hi = max(lf[0], rf[0]) + foot_length / 2
            y_lo = min(lf[1], rf[1]) - foot_width / 2
            y_hi = max(lf[1], rf[1]) + foot_width / 2
        elif sup == 1:
            # Left foot support
            x_lo = lf[0] - foot_length / 2
            x_hi = lf[0] + foot_length / 2
            y_lo = lf[1] - foot_width / 2
            y_hi = lf[1] + foot_width / 2
        else:
            # Right foot support
            x_lo = rf[0] - foot_length / 2
            x_hi = rf[0] + foot_length / 2
            y_lo = rf[1] - foot_width / 2
            y_hi = rf[1] + foot_width / 2

        # Shrink by buffer
        cx = (x_lo + x_hi) / 2
        cy = (y_lo + y_hi) / 2
        hw = (x_hi - x_lo) / 2 * shrink
        hh = (y_hi - y_lo) / 2 * shrink
        x_lo_buf = cx - hw
        x_hi_buf = cx + hw
        y_lo_buf = cy - hh
        y_hi_buf = cy + hh

        # Margin: positive = inside, negative = outside
        margin_x = min(zmp_x[i] - x_lo_buf, x_hi_buf - zmp_x[i])
        margin_y = min(zmp_y[i] - y_lo_buf, y_hi_buf - zmp_y[i])
        margin = min(margin_x, margin_y)

        if margin < min_margin:
            min_margin = margin
            worst_step = i

        if margin < 0:
            violations.append((i, margin))

    return {
        'stable': len(violations) == 0,
        'violations': violations,
        'min_margin': min_margin,
        'worst_step': worst_step,
        'worst_time': worst_step * plan.dt,
    }


def find_ideal_config(
    com_height: float = 0.40,
    foot_width: float = 0.08,
    foot_length: float = 0.15,
    dt: float = 0.02,
    buffer: float = 0.10,
) -> dict:
    """Find ideal first-step walking config for ZMP stability.

    Searches step_length, step_width, and step_period for the most stable
    configuration. Returns the config with the largest minimum margin.

    The ideal config balances:
    - Step period: longer = more time for CoM transfer = more stable
    - Step width: wider = larger support polygon = more stable laterally
    - Step length: shorter = less forward CoM shift needed = more stable
    - CoM height: affects pendulum dynamics (T = 2π√(h/g))

    Returns:
        dict with best config parameters and stability margin.
    """
    from .zmp_walker import ZMPWalker

    best_config = None
    best_margin = -float('inf')

    for step_length in [0.03, 0.05, 0.08, 0.10, 0.12, 0.15]:
        for step_width in [0.20, 0.25, 0.30]:
            for step_period in [0.6, 0.8, 1.0, 1.2]:
                planner = FootstepPlanner(
                    step_length=step_length,
                    step_width=step_width,
                    step_height=0.03,
                    step_period=step_period,
                    num_steps=4,
                    dt=dt,
                )
                plan = planner.plan()

                walker = ZMPWalker(com_height=com_height, dt=dt)
                _, _, zmp_x, zmp_y = walker.generate(plan.zmp_ref_x, plan.zmp_ref_y)

                result = check_zmp_stability(
                    plan, zmp_x, zmp_y,
                    foot_length=foot_length,
                    foot_width=foot_width,
                    buffer=buffer,
                )

                if result['min_margin'] > best_margin:
                    best_margin = result['min_margin']
                    best_config = {
                        'step_length': step_length,
                        'step_width': step_width,
                        'step_period': step_period,
                        'com_height': com_height,
                        'min_margin': result['min_margin'],
                        'stable': result['stable'],
                    }

    return best_config
