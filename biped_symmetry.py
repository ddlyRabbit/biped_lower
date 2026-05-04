"""Adaptive symmetry augmentation for biped — mirrors left↔right.

Handles both 48-dim (teacher, with base_lin_vel) and 45-dim (student, without base_lin_vel).

Joint order (MASTER_JOINT_ORDER):
    [0] L_hip_pitch, [1] R_hip_pitch
    [2] L_hip_roll,  [3] R_hip_roll
    [4] L_hip_yaw,   [5] R_hip_yaw
    [6] L_knee,      [7] R_knee
    [8] L_foot_pitch,[9] R_foot_pitch
   [10] L_foot_roll, [11] R_foot_roll

Mirror rules:
    hip_pitch: swap+negate
    hip_roll:  swap+negate
    hip_yaw:   swap only
    knee:      swap only
    foot_pitch: swap only
    foot_roll:  swap only
"""
import torch
from tensordict import TensorDict

# (left_idx, right_idx, negate)
JOINT_MIRROR = [
    (0,  1,  True),   # hip_pitch
    (2,  3,  True),   # hip_roll
    (4,  5,  False),  # hip_yaw
    (6,  7,  False),  # knee
    (8,  9,  False),  # foot_pitch
    (10, 11, False),  # foot_roll
]

def _mirror_actions(actions: torch.Tensor) -> torch.Tensor:
    m = actions.clone()
    for l, r, neg in JOINT_MIRROR:
        if neg:
            m[:, l] = -actions[:, r]
            m[:, r] = -actions[:, l]
        else:
            m[:, l] = actions[:, r]
            m[:, r] = actions[:, l]
    return m

def _mirror_joint_slice(obs: torch.Tensor, start: int) -> torch.Tensor:
    for l, r, neg in JOINT_MIRROR:
        li = start + l
        ri = start + r
        if neg:
            obs[:, li], obs[:, ri] = -obs[:, ri].clone(), -obs[:, li].clone()
        else:
            obs[:, li], obs[:, ri] = obs[:, ri].clone(), obs[:, li].clone()
    return obs

def _mirror_obs(obs: torch.Tensor) -> torch.Tensor:
    m = obs.clone()
    dim = obs.shape[-1]

    if dim == 48:
        # [0:3] base_lin_vel: negate y
        m[:, 1] *= -1
        offset = 3
    elif dim == 45:
        offset = 0
    else:
        raise ValueError(f"Unsupported obs dim: {dim}")

    # [offset+0 : offset+3] ang_vel: negate x and z? 
    # Wait, mirroring Y plane (left-right reflection):
    # y changes sign. x and z stay same.
    # So lin_vel_y negates.
    # For angular velocity (yaw/roll), ang_vel_x (roll) and ang_vel_z (yaw) negate. ang_vel_y (pitch) stays same.
    m[:, offset + 0] *= -1  # roll
    m[:, offset + 2] *= -1  # yaw

    # [offset+3 : offset+6] proj_gravity: negate y
    m[:, offset + 4] *= -1

    # [offset+6 : offset+9] cmd_vel: negate vy, negate wz
    m[:, offset + 7] *= -1
    m[:, offset + 8] *= -1

    # [offset+9 : offset+21] joint_pos
    _mirror_joint_slice(m, offset + 9)

    # [offset+21 : offset+33] joint_vel
    _mirror_joint_slice(m, offset + 21)

    # [offset+33 : offset+45] last_action
    _mirror_joint_slice(m, offset + 33)

    return m

def biped_symmetry_augmentation(obs, actions, env):
    obs_out = None
    actions_out = None

    if obs is not None:
        policy_obs = obs["policy"]
        policy_mirror = _mirror_obs(policy_obs)

        mirror_dict = {}
        for key in obs.keys():
            if key == "policy":
                mirror_dict[key] = policy_mirror
            else:
                mirror_dict[key] = obs[key].clone()

        obs_mirror = TensorDict(mirror_dict, batch_size=[policy_mirror.shape[0]])
        obs_out = torch.cat([obs, obs_mirror], dim=0)

    if actions is not None:
        actions_mirror = _mirror_actions(actions)
        actions_out = torch.cat([actions, actions_mirror], dim=0)

    return obs_out, actions_out
