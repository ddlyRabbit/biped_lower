"""Adaptive symmetry augmentation for biped — mirrors left↔right.

Handles both 48-dim (teacher, with base_lin_vel) and 45-dim (student, without base_lin_vel).

rsl_rl interface:
    data_augmentation_func(obs=TensorDict, actions=tensor, env=env) -> (obs, actions)

Joint order (Isaac, 12d):
    [0] R_hip_yaw,  [1] R_hip_roll,  [2] R_hip_pitch,
    [3] R_knee,     [4] R_foot_pitch,[5] R_foot_roll,
    [6] L_hip_yaw,  [7] L_hip_roll,  [8] L_hip_pitch,
    [9] L_knee,    [10] L_foot_pitch,[11] L_foot_roll

Mirror rules (from URDF joint limits):
    hip_yaw:   swap only  (symmetric limits ±1.57)
    hip_roll:  swap+negate (R:[-2.27,+0.21] ↔ L:[-0.21,+2.27])
    hip_pitch: swap+negate (R:[-2.22,+1.05] ↔ L:[-1.05,+2.22])
    knee:      swap only  (symmetric [0, 2.71])
    foot_pitch: swap only (symmetric [-0.87, +0.52])
    foot_roll:  swap only (symmetric ±0.26)

Observation layout:
    48-dim (teacher): [base_lin_vel(3), ang_vel(3), proj_gravity(3), cmd_vel(3),
                       hip_pos(6), knee_pos(2), foot_pitch_pos(2), foot_roll_pos(2),
                       joint_vel(12), last_action(12)]
    45-dim (student): [ang_vel(3), proj_gravity(3), cmd_vel(3),
                       hip_pos(6), knee_pos(2), foot_pitch_pos(2), foot_roll_pos(2),
                       joint_vel(12), last_action(12)]
"""
import torch
from tensordict import TensorDict


# ── Joint mirror mapping ─────────────────────────────────────────────────────
# (right_idx, left_idx, negate_on_swap)
# Derived from URDF: hip_roll and hip_pitch have negated limits across L/R.
JOINT_MIRROR = [
    (0,  6,  False),  # hip_yaw:   swap
    (1,  7,  True),   # hip_roll:  swap + negate
    (2,  8,  True),   # hip_pitch: swap + negate
    (3,  9,  False),  # knee:      swap
    (4, 10,  False),  # foot_pitch: swap
    (5, 11,  False),  # foot_roll:  swap
]


def _mirror_actions(actions: torch.Tensor) -> torch.Tensor:
    """Mirror actions (N, 12)."""
    m = actions.clone()
    for r, l, neg in JOINT_MIRROR:
        if neg:
            m[:, r] = -actions[:, l]
            m[:, l] = -actions[:, r]
        else:
            m[:, r] = actions[:, l]
            m[:, l] = actions[:, r]
    return m


def _mirror_joint_slice(obs: torch.Tensor, start: int) -> torch.Tensor:
    """Mirror a 12-dim joint slice starting at `start` in obs (N, obs_dim).
    Returns modified obs (in-place on clone).
    """
    for r, l, neg in JOINT_MIRROR:
        ri = start + r
        li = start + l
        if neg:
            obs[:, ri], obs[:, li] = -obs[:, li].clone(), -obs[:, ri].clone()
        else:
            obs[:, ri], obs[:, li] = obs[:, li].clone(), obs[:, ri].clone()
    return obs


def _mirror_obs(obs: torch.Tensor) -> torch.Tensor:
    """Mirror policy observation. Auto-detects 45-dim vs 48-dim."""
    m = obs.clone()
    dim = obs.shape[-1]

    if dim == 48:
        # [0:3] base_lin_vel: negate y
        m[:, 1] *= -1
        offset = 3
    elif dim == 45:
        offset = 0
    else:
        raise ValueError(f"Unsupported obs dim: {dim}. Expected 45 or 48.")

    # [offset+0 : offset+3] ang_vel: negate y
    m[:, offset + 1] *= -1
    # [offset+3 : offset+6] proj_gravity: negate y
    m[:, offset + 4] *= -1
    # [offset+6 : offset+9] cmd_vel: negate vy, negate wz
    m[:, offset + 7] *= -1
    m[:, offset + 8] *= -1

    # [offset+9 : offset+15] hip_pos (6d): swap L↔R per joint
    # hip_pos order: [R_hip_roll, R_hip_yaw, R_hip_pitch, L_hip_roll, L_hip_yaw, L_hip_pitch]
    # (grouped by joint type in ObsCfg: ".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*")
    # Isaac returns joints matching regex in order they appear in articulation.
    # With regex ".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*":
    #   result = [R_hip_roll, L_hip_roll, R_hip_yaw, L_hip_yaw, R_hip_pitch, L_hip_pitch]
    hip_base = offset + 9
    # hip_roll pair: indices 0,1 → R_hip_roll ↔ L_hip_roll (negate)
    m[:, hip_base + 0], m[:, hip_base + 1] = -m[:, hip_base + 1].clone(), -m[:, hip_base + 0].clone()
    # hip_yaw pair: indices 2,3 → R_hip_yaw ↔ L_hip_yaw (swap only)
    m[:, hip_base + 2], m[:, hip_base + 3] = m[:, hip_base + 3].clone(), m[:, hip_base + 2].clone()
    # hip_pitch pair: indices 4,5 → R_hip_pitch ↔ L_hip_pitch (negate)
    m[:, hip_base + 4], m[:, hip_base + 5] = -m[:, hip_base + 5].clone(), -m[:, hip_base + 4].clone()

    # [offset+15 : offset+17] knee_pos: [R_knee, L_knee] → swap
    m[:, offset + 15], m[:, offset + 16] = m[:, offset + 16].clone(), m[:, offset + 15].clone()
    # [offset+17 : offset+19] foot_pitch: [R_fp, L_fp] → swap
    m[:, offset + 17], m[:, offset + 18] = m[:, offset + 18].clone(), m[:, offset + 17].clone()
    # [offset+19 : offset+21] foot_roll: [R_fr, L_fr] → swap
    m[:, offset + 19], m[:, offset + 20] = m[:, offset + 20].clone(), m[:, offset + 19].clone()

    # [offset+21 : offset+33] joint_vel (12d): mirror per JOINT_MIRROR
    _mirror_joint_slice(m, offset + 21)

    # [offset+33 : offset+45] last_action (12d): mirror per JOINT_MIRROR
    _mirror_joint_slice(m, offset + 33)

    return m


def biped_symmetry_augmentation(obs, actions, env):
    """Data augmentation for rsl_rl symmetry.

    Args:
        obs: TensorDict with "policy" key (N, obs_dim) and possibly "critic".
        actions: tensor (N, 12) or None.
        env: environment (passed by rsl_rl via symmetry_cfg["_env"]).

    Returns:
        Concatenated [original, mirrored] along batch dim.
    """
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
