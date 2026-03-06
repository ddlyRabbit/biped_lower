# Register biped locomotion environments with gymnasium

import gymnasium as gym

gym.register(
    id="Isaac-Velocity-Flat-Biped-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg",
        "skrl_cfg_entry_point": "agents/skrl_flat_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Velocity-Flat-Biped-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "biped_env_cfg:BipedFlatEnvCfg_PLAY",
        "skrl_cfg_entry_point": "agents/skrl_flat_ppo_cfg.yaml",
    },
)
