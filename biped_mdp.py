"""Custom MDP functions for Unitree G1-style biped training.

Copied exactly from unitree_rl_lab G1 29dof config:
  https://github.com/unitreerobotics/unitree_rl_lab

Contains: energy, feet_gait, foot_clearance_reward, lin_vel_cmd_levels,
           UniformLevelVelocityCommandCfg.
"""

from __future__ import annotations

import torch
from dataclasses import MISSING
from typing import TYPE_CHECKING
from collections.abc import Sequence

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.envs.mdp.commands import UniformVelocityCommandCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Command config
# ---------------------------------------------------------------------------

@configclass
class UniformLevelVelocityCommandCfg(UniformVelocityCommandCfg):
    """Uniform velocity command config with limit_ranges for curriculum."""
    limit_ranges: UniformVelocityCommandCfg.Ranges = MISSING


# ---------------------------------------------------------------------------
# Reward functions — EXACT G1 implementations
# ---------------------------------------------------------------------------

def energy(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize the energy used by the robot's joints.

    Exact G1: sum(|joint_vel| * |applied_torque|)
    """
    asset: Articulation = env.scene[asset_cfg.name]
    qvel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    qfrc = asset.data.applied_torque[:, asset_cfg.joint_ids]
    return torch.sum(torch.abs(qvel) * torch.abs(qfrc), dim=-1)


def feet_gait(
    env: ManagerBasedRLEnv,
    period: float,
    offset: list[float],
    sensor_cfg: SceneEntityCfg,
    threshold: float = 0.5,
    command_name=None,
) -> torch.Tensor:
    """Phase-based gait reward — EXACT G1 implementation.

    Each foot has a phase offset. During stance phase (phase < threshold),
    foot should be in contact. During swing (phase >= threshold), foot
    should be in air. Reward = count of correct feet.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    is_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0

    global_phase = ((env.episode_length_buf * env.step_dt) % period / period).unsqueeze(1)
    phases = []
    for offset_ in offset:
        phase = (global_phase + offset_) % 1.0
        phases.append(phase)
    leg_phase = torch.cat(phases, dim=-1)

    reward = torch.zeros(env.num_envs, dtype=torch.float, device=env.device)
    for i in range(len(sensor_cfg.body_ids)):
        is_stance = leg_phase[:, i] < threshold
        reward += ~(is_stance ^ is_contact[:, i])

    if command_name is not None:
        cmd_norm = torch.norm(env.command_manager.get_command(command_name), dim=1)
        reward *= cmd_norm > 0.1
    return reward


def foot_clearance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_height: float,
    std: float,
    tanh_mult: float,
) -> torch.Tensor:
    """Reward swinging feet for clearing target height — EXACT G1 implementation.

    reward = exp(-sum((foot_z - target)^2 * tanh(mult * |foot_xy_vel|)) / std)
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    foot_z_target_error = torch.square(
        asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - target_height
    )
    foot_velocity_tanh = torch.tanh(
        tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2)
    )
    reward = foot_z_target_error * foot_velocity_tanh
    return torch.exp(-torch.sum(reward, dim=1) / std)


# ---------------------------------------------------------------------------
# Curriculum functions — EXACT G1 implementation
# ---------------------------------------------------------------------------

def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy",
) -> torch.Tensor:
    """Expand velocity command ranges when tracking reward is good.

    Exact G1: at each episode boundary, if mean reward > weight * 0.8,
    expand lin_vel_x and lin_vel_y by ±0.1, clamped to limit_ranges.
    """
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    limit_ranges = command_term.cfg.limit_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    reward = torch.mean(
        env.reward_manager._episode_sums[reward_term_name][env_ids]
    ) / env.max_episode_length_s

    if env.common_step_counter % env.max_episode_length == 0:
        if reward > reward_term.weight * 0.8:
            delta_command = torch.tensor([-0.1, 0.1], device=env.device)
            ranges.lin_vel_x = torch.clamp(
                torch.tensor(ranges.lin_vel_x, device=env.device) + delta_command,
                limit_ranges.lin_vel_x[0],
                limit_ranges.lin_vel_x[1],
            ).tolist()
            ranges.lin_vel_y = torch.clamp(
                torch.tensor(ranges.lin_vel_y, device=env.device) + delta_command,
                limit_ranges.lin_vel_y[0],
                limit_ranges.lin_vel_y[1],
            ).tolist()

    return torch.tensor(ranges.lin_vel_x[1], device=env.device)
