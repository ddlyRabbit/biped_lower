# Biped Rough Terrain Config
# Inherits from flat config, adds:
#   - Terrain generator (Berkeley ROUGH_TERRAINS_CFG adapted for our robot)
#   - Height scanner (RayCaster on torso)
#   - height_scan observation term
#   - terrain_levels curriculum
#   - Relaxed flat_orientation penalty
#
# NOTE: Policy obs_dim changes (adds height scan), so flat checkpoints
#       cannot be directly resumed. Use transfer learning or train from scratch.

import math
import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.terrain_generator_cfg import TerrainGeneratorCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.assets import Articulation
from isaaclab.terrains import TerrainImporter
from isaaclab.envs import mdp as base_mdp

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

from biped_env_cfg import BipedFlatEnvCfg, BipedFlatEnvCfg_PLAY


###############################################################################
# Terrain Generator — Berkeley-style with our robot's scale
#
# Berkeley original uses step_height 0-0.1m, slope 0-0.4, noise 0-0.06m.
# We use similar values — our robot is ~0.75m tall, similar to Berkeley.
# Added 30% flat proportion for safe curriculum start (Berkeley uses 30%).
###############################################################################

BIPED_ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,            # 10 difficulty levels
    num_cols=20,            # 20 copies per level → 200 sub-terrains
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "flat": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.3,
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1,
            slope_range=(0.0, 0.4),
            platform_width=2.0,
            border_width=0.25,
        ),
        "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1,
            slope_range=(0.0, 0.4),
            platform_width=2.0,
            border_width=0.25,
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.05,
            step_height_range=(0.0, 0.1),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.05,
            step_height_range=(0.0, 0.1),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "wave_terrain": terrain_gen.HfWaveTerrainCfg(
            proportion=0.2,
            amplitude_range=(0.0, 0.2),
            num_waves=4,
            border_width=0.25,
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2,
            noise_range=(0.0, 0.06),
            noise_step=0.02,
            border_width=0.25,
        ),
    },
)


###############################################################################
# Terrain Curriculum — Berkeley exact
###############################################################################

def terrain_levels_vel(
    env: "ManagerBasedRLEnv",
    env_ids: Sequence[int],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Curriculum based on distance walked vs commanded velocity.

    Promotes robots to harder terrain when they walk far enough (> half terrain size).
    Demotes when they walk less than half the commanded distance.
    Exact Berkeley implementation.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    command = env.command_manager.get_command("base_velocity")

    distance = torch.norm(
        asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1
    )
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up

    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())


###############################################################################
# Rough Env Config
###############################################################################

@configclass
class BipedRoughEnvCfg(BipedFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # --- Terrain: generator instead of plane ---
        self.scene.terrain = TerrainImporterCfg(
            prim_path="/World/ground",
            terrain_type="generator",
            terrain_generator=BIPED_ROUGH_TERRAINS_CFG,
            max_init_terrain_level=0,  # start all robots on easiest terrain
            collision_group=-1,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
            visual_material=sim_utils.MdlFileCfg(
                mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
                project_uvw=True,
            ),
            debug_vis=False,
        )

        # Enable terrain curriculum in generator
        self.scene.terrain.terrain_generator.curriculum = True

        # --- Height Scanner: RayCaster on torso ---
        # 1.6m × 1.0m grid, 0.1m resolution → 16×10 = 160 rays
        self.scene.height_scanner = RayCasterCfg(
            prim_path="{ENV_REGEX_NS}/Robot/assy_formfg___kd_b_102b_torso_btm",
            offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
            attach_yaw_only=True,
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
            debug_vis=False,
            mesh_prim_paths=["/World/ground"],
        )

        # Update height scanner period to match control frequency
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt

        # --- Height scan observation ---
        # Add to policy observations (appended after existing 48-dim obs)
        self.observations.policy.height_scan = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )
        # Add to critic observations too
        self.observations.critic.height_scan = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

        # --- Terrain curriculum ---
        self.curriculums.terrain_levels = CurrTerm(
            func="biped_rough_env_cfg:terrain_levels_vel",
            params={"asset_cfg": SceneEntityCfg("robot")},
        )

        # --- Relax penalties for rough terrain (Berkeley rough defaults) ---
        # flat_orientation: -0.5 → 0.0 (Berkeley rough sets weight=0.0)
        self.rewards.flat_orientation_l2.weight = 0.0
        # dof_pos_limits: -1.0 → 0.0 (Berkeley rough sets weight=0.0)
        self.rewards.dof_pos_limits.weight = 0.0

        # --- Reduce num_envs for rough (terrain mesh uses more VRAM) ---
        self.scene.num_envs = 8192
        self.scene.env_spacing = 2.5


@configclass
class BipedRoughEnvCfg_PLAY(BipedRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # Smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5

        # Reduce terrain grid to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # Spawn randomly instead of at terrain levels
        self.scene.terrain.max_init_terrain_level = None

        # Disable noise for play
        self.observations.policy.enable_corruption = False

        # Disable pushes
        self.events.push_robot = None
