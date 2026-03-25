"""Custom MDP functions for Unitree G1-style biped training.

Contains: energy reward, feet_gait reward, foot_clearance_reward,
lin_vel_cmd_levels curriculum, UniformLevelVelocityCommandCfg.
"""

from __future__ import annotations

import math
import torch
from dataclasses import MISSING
from typing import TYPE_CHECKING

from isaaclab.managers import CommandTermCfg, SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils import configclass
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Reward functions
# ---------------------------------------------------------------------------


def energy(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Energy penalty: sum of |torque * joint_vel| over all joints."""
    return torch.sum(
        torch.abs(
            env.scene["robot"].data.applied_torque
            * env.scene["robot"].data.joint_vel
        ),
        dim=1,
    )


def feet_gait(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
    command_name: str,
    period: float,
    offset: list[float],
    threshold: float,
) -> torch.Tensor:
    """Gait reward — encourages alternating foot contacts at desired phase.

    Each foot should be in contact when cos(phase + offset) > 0, and in air otherwise.
    Phase = 2π * (t % period) / period.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w_history[:, 0, sensor_cfg.body_ids, :]
    in_contact = forces.norm(dim=-1) > 1.0  # [num_envs, num_feet]

    # Time since episode start
    t = env.episode_length_buf * env.step_dt  # [num_envs]
    phase = 2 * math.pi * t / period  # [num_envs]

    num_feet = len(offset)
    offsets = torch.tensor(offset, device=phase.device, dtype=phase.dtype)  # [num_feet]

    # Desired contact state
    desired_contact = torch.cos(phase.unsqueeze(1) + 2 * math.pi * offsets.unsqueeze(0)) > 0  # [envs, feet]

    reward = torch.where(
        desired_contact == in_contact,
        torch.ones_like(in_contact, dtype=torch.float),
        torch.zeros_like(in_contact, dtype=torch.float),
    ).mean(dim=1)

    # Scale by command magnitude — no gait reward when standing still
    cmd = env.command_manager.get_command(command_name)
    cmd_norm = torch.norm(cmd[:, :2], dim=1)
    moving = cmd_norm > 0.1

    # Standing: reward for both feet on ground
    both_contact = in_contact.all(dim=1).float()
    reward = torch.where(moving, reward, both_contact)

    return reward


def foot_clearance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_height: float,
    std: float,
    tanh_mult: float,
) -> torch.Tensor:
    """Reward feet for reaching target clearance height during swing.

    Only rewards when foot is moving (has nonzero velocity).
    Uses exp(-((h - target)^2) / (2*std^2)) * tanh(speed).
    """
    asset = env.scene[asset_cfg.name]
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]  # [envs, feet]
    foot_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]  # [envs, feet, 3]
    foot_speed = foot_vel.norm(dim=-1)  # [envs, feet]

    height_error = (foot_z - target_height) ** 2
    height_reward = torch.exp(-height_error / (2 * std**2))

    # Only reward moving feet (swing phase)
    speed_mask = torch.tanh(tanh_mult * foot_speed)

    reward = (height_reward * speed_mask).mean(dim=1)
    return reward


# ---------------------------------------------------------------------------
# Curriculum functions
# ---------------------------------------------------------------------------


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    term_name: str,
) -> float:
    """Curriculum that linearly expands velocity command ranges over training.

    Ramps from initial ranges to limit_ranges over 10000 iterations.
    """
    cmd_term = env.command_manager.get_term("base_velocity")
    cfg = cmd_term.cfg

    if not hasattr(cfg, "limit_ranges") or cfg.limit_ranges is None:
        return 0.0

    # Linear ramp over 10000 iterations (each iter = num_steps_per_env steps)
    steps_per_iter = 24  # num_steps_per_env
    ramp_iters = 10000
    progress = min(env.common_step_counter / (ramp_iters * steps_per_iter), 1.0)

    # Interpolate ranges
    for attr in ["lin_vel_x", "lin_vel_y", "ang_vel_z"]:
        initial = getattr(cfg.ranges, attr)
        limit = getattr(cfg.limit_ranges, attr)
        if initial is not None and limit is not None:
            new_low = initial[0] + (limit[0] - initial[0]) * progress
            new_high = initial[1] + (limit[1] - initial[1]) * progress
            setattr(cfg.ranges, attr, (new_low, new_high))

    return progress


# ---------------------------------------------------------------------------
# Custom command: UniformLevelVelocityCommand with limit_ranges
# ---------------------------------------------------------------------------


@configclass
class UniformLevelVelocityCommandCfg(UniformVelocityCommandCfg):
    """Uniform velocity command config with limit_ranges for curriculum."""

    limit_ranges: UniformVelocityCommandCfg.Ranges | None = None
