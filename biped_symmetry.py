"""Symmetry augmentation for biped — mirrors left↔right.

obs_batch is a TensorDict with keys "policy" (N, 45) and "critic" (N, ...).
We mirror the "policy" observations. Critic obs get duplicated without mirroring.

Observation "policy" (45d):
  [0:3]   ang_vel: negate y
  [3:6]   proj_gravity: negate y
  [6:9]   cmd_vel: negate vy, negate wz
  [9:15]  hip_pos (Lr,Rr,Ly,Ry,Lp,Rp): swap+negate all (roll/yaw/pitch all flip)
  [15:17] knee_pos (L,R): swap only
  [17:19] foot_pitch (L,R): swap only
  [19:21] foot_roll (L,R): swap+negate
  [21:33] joint_vel (Isaac order): mirror per joint type
  [33:45] last_action (12d): mirror same as actions

Actions (12d Isaac order): Lhp,Rhp,Lhr,Rhr,Lhy,Rhy,Lkn,Rkn,Lfp,Rfp,Lfr,Rfr
"""
import torch
from tensordict import TensorDict


def _mirror_actions_tensor(a):
    """Mirror actions (N, 12). Isaac order."""
    m = a.clone()
    m[:, 0] = -a[:, 1]   # Lhp ← -Rhp
    m[:, 1] = -a[:, 0]   # Rhp ← -Lhp
    m[:, 2] = -a[:, 3]   # Lhr ← -Rhr
    m[:, 3] = -a[:, 2]   # Rhr ← -Lhr
    m[:, 4] = -a[:, 5]   # Lhy ← -Rhy
    m[:, 5] = -a[:, 4]   # Rhy ← -Lhy
    m[:, 6] = a[:, 7]    # Lkn ← Rkn
    m[:, 7] = a[:, 6]    # Rkn ← Lkn
    m[:, 8] = a[:, 9]    # Lfp ← Rfp
    m[:, 9] = a[:, 8]    # Rfp ← Lfp
    m[:, 10] = -a[:, 11]  # Lfr ← -Rfr
    m[:, 11] = -a[:, 10]  # Rfr ← -Lfr
    return m


def _mirror_obs_flat(obs):
    """Mirror flat policy observation (N, 45)."""
    m = obs.clone()
    # [0:3] ang_vel: negate y
    m[:, 1] *= -1
    # [3:6] proj_gravity: negate y
    m[:, 4] *= -1
    # [6:9] cmd_vel: negate vy, negate wz
    m[:, 7] *= -1
    m[:, 8] *= -1
    # [9:15] hip_pos: swap L↔R + negate all (roll, yaw, pitch all have mirrored axes)
    m[:, 9]  = -obs[:, 10]  # Lr ← -Rr
    m[:, 10] = -obs[:, 9]   # Rr ← -Lr
    m[:, 11] = -obs[:, 12]  # Ly ← -Ry
    m[:, 12] = -obs[:, 11]  # Ry ← -Ly
    m[:, 13] = -obs[:, 14]  # Lp ← -Rp
    m[:, 14] = -obs[:, 13]  # Rp ← -Lp
    # [15:17] knee_pos: swap only
    m[:, 15] = obs[:, 16]
    m[:, 16] = obs[:, 15]
    # [17:19] foot_pitch: swap only
    m[:, 17] = obs[:, 18]
    m[:, 18] = obs[:, 17]
    # [19:21] foot_roll: swap + negate
    m[:, 19] = -obs[:, 20]
    m[:, 20] = -obs[:, 19]
    # [21:33] joint_vel: mirror per Isaac order
    v = obs[:, 21:33]
    m[:, 21] = -v[:, 1]   # Lhp ← -Rhp
    m[:, 22] = -v[:, 0]   # Rhp ← -Lhp
    m[:, 23] = -v[:, 3]   # Lhr ← -Rhr
    m[:, 24] = -v[:, 2]   # Rhr ← -Lhr
    m[:, 25] = -v[:, 5]   # Lhy ← -Rhy
    m[:, 26] = -v[:, 4]   # Rhy ← -Lhy
    m[:, 27] = v[:, 7]    # Lkn ← Rkn
    m[:, 28] = v[:, 6]    # Rkn ← Lkn
    m[:, 29] = v[:, 9]    # Lfp ← Rfp
    m[:, 30] = v[:, 8]    # Rfp ← Lfp
    m[:, 31] = -v[:, 11]  # Lfr ← -Rfr
    m[:, 32] = -v[:, 10]  # Rfr ← -Lfr
    # [33:45] last_action: mirror
    m[:, 33:45] = _mirror_actions_tensor(obs[:, 33:45])
    return m


def biped_symmetry_augmentation(obs, actions, env):
    """Data augmentation for rsl_rl symmetry.
    
    obs: TensorDict with "policy" key (N, 45) and possibly "critic"
    actions: tensor (N, 12) or None
    env: unused
    
    Returns concatenated [original, mirrored] along batch dim.
    """
    obs_out = None
    actions_out = None
    
    if obs is not None:
        # Mirror policy observations
        policy_obs = obs["policy"]
        policy_mirror = _mirror_obs_flat(policy_obs)
        
        # Build mirrored TensorDict
        mirror_dict = {}
        for key in obs.keys():
            if key == "policy":
                mirror_dict[key] = policy_mirror
            else:
                # Critic obs: just duplicate (not mirrored)
                mirror_dict[key] = obs[key].clone()
        
        obs_mirror = TensorDict(mirror_dict, batch_size=[policy_mirror.shape[0]])
        obs_out = torch.cat([obs, obs_mirror], dim=0)
    
    if actions is not None:
        actions_mirror = _mirror_actions_tensor(actions)
        actions_out = torch.cat([actions, actions_mirror], dim=0)
    
    return obs_out, actions_out
