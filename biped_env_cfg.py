# Biped V58 — +X forward axis, dual URDF support (heavy/light)
# Source: https://github.com/HybridRobotics/isaac_berkeley_humanoid
#
# MATCHED TO BERKELEY:
#   REWARDS:      13 terms, same weights. feet_air_time uses adaptive mode
#                 (continuous 0-500 iters, then impact-based)
#   CURRICULUM:   push_force_levels + command_vel (expands lin_vel_x for +X forward)
#   PPO:          [128,128,128], init_noise_std=1.0, entropy_coef=0.005
#   OBSERVATIONS: base_lin_vel in policy, per-joint-group noise, obs_dim=48
#   ACTIONS:      scale=0.5
#   COMMANDS:     lin_vel_x=(-0.5, 1.0) forward, lin_vel_y=(-0.3, 0.3) lateral
#   TERMINATIONS: base_contact (torso, threshold=1.0), time_out
#   EVENTS:       All Berkeley events + scale_all_actuator_gains (extra)
#   DECIMATION:   4 (50 Hz control)
#
# DIFFERENCES FROM BERKELEY:
#   ACTUATORS:    DelayedPDActuator with friction + action delay for sim-to-real.
#                 Gains matched to deploy: hip_pitch/knee Kp=30, hip_roll/yaw Kp=20,
#                 foot Kp=25. Friction: 1.0Nm (hip_pitch/knee), 0.75Nm (hip_roll/yaw),
#                 0.5Nm (foot). Foot effort 30Nm (parallel linkage).
#   EVENTS:       scale_all_actuator_gains added (not in Berkeley)
#   FORWARD AXIS: +X (Berkeley uses custom joint naming, ours uses Onshape URDF)
#   FEET_AIR:     Adaptive (continuous → impact), Berkeley uses contact-sensor based

###############################################################################
# URDF selection — use --urdf heavy|light flag in training scripts
# Both URDFs use +X forward axis. Heavy=original material (~30.7kg), Light=~15.6kg
###############################################################################
URDF_HEAVY = "/uploads/heavy/robot.urdf"
URDF_LIGHT = "/uploads/light/robot.urdf"
URDF_DEFAULT = URDF_HEAVY

import math
import torch
import numpy as np
from typing import Literal, TYPE_CHECKING
from collections.abc import Sequence

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg
from isaaclab.sim.converters.urdf_converter_cfg import UrdfConverterCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.sensors import ContactSensor, ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.envs import ManagerBasedRLEnvCfg, ManagerBasedEnv
from isaaclab.envs import mdp as base_mdp
from isaaclab.envs.mdp.events import _randomize_prop_by_op

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


ALL_JOINTS = [
    "right_hip_yaw_03", "right_hip_roll_03", "right_hip_pitch_04",
    "right_knee_04", "right_foot_pitch_02", "right_foot_roll_02",
    "left_hip_yaw_03", "left_hip_roll_03", "left_hip_pitch_04",
    "left_knee_04", "left_foot_pitch_02", "left_foot_roll_02",
]


###############################################################################
# Custom reward functions — EXACT Berkeley implementations
###############################################################################


def stand_still(
    env: "ManagerBasedRLEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names="foot_6061.*"),
    vel_threshold: float = 0.1,
) -> torch.Tensor:
    """Penalize foot body velocity when commanded velocity is near zero.

    Encourages the robot to stand still and not shuffle feet.
    G1-style contact_no_vel.
    """
    cmd = env.command_manager.get_command(command_name)
    cmd_norm = torch.norm(cmd[:, :2], dim=1)
    near_zero = cmd_norm < vel_threshold  # True when should stand still

    asset: Articulation = env.scene[asset_cfg.name]
    foot_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]  # XY velocity
    foot_speed = torch.sum(foot_vel.norm(dim=-1), dim=1)  # sum over both feet

    # Only penalize when cmd is near zero
    return foot_speed * near_zero.float()


def feet_air_time_berkeley(
    env: "ManagerBasedRLEnv",
    command_name: str,
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
    threshold_min: float = 0.2,
    threshold_max: float = 0.5,
) -> torch.Tensor:
    """Pure Berkeley impact-based air time reward.

    Uses ContactSensor.compute_first_contact() directly.
    Rewards at the moment of first ground contact after an air phase.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold_min) * first_contact, dim=1)
    reward = torch.clamp(reward, min=-0.25, max=threshold_max - threshold_min)

    reward *= torch.norm(
        env.command_manager.get_command(command_name)[:, :2], dim=1
    ) > 0.1

    return reward


def feet_slide_berkeley(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Berkeley contact-sensor feet slide penalty."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset: Articulation = env.scene[asset_cfg.name]
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    return reward

def feet_air_time(
    env: "ManagerBasedRLEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg,
    height_threshold: float = 0.058,
) -> torch.Tensor:
    """Continuous single-stance reward — foot HEIGHT contact detection.

    Fixed reward every step exactly ONE foot is airborne (height > threshold).
    Uses foot Z position — immune to self-collision noise.
    Foot center rests at ~0.053m; threshold 0.058m = 0.5cm clearance.
    """
    asset = env.scene[asset_cfg.name]
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]  # [num_envs, num_feet]

    in_air = foot_z > height_threshold
    num_airborne = in_air.sum(dim=1)

    # Reward only when exactly 1 foot is in the air
    single_foot_air = (num_airborne == 1).float()
    reward = single_foot_air * 0.1

    # No reward for zero command
    reward *= torch.norm(
        env.command_manager.get_command(command_name)[:, :2], dim=1
    ) > 0.1

    # Debug every 50 steps
    if not hasattr(env, "_feet_air_debug_step"):
        env._feet_air_debug_step = 0
    env._feet_air_debug_step += 1
    if env._feet_air_debug_step % 50 == 0:
        n = in_air.shape[0]
        both_ground = (num_airborne == 0).sum().item()
        one_air = (num_airborne == 1).sum().item()
        both_air = (num_airborne == 2).sum().item()
        mean_h0 = foot_z[:, 0].mean().item()
        mean_h1 = foot_z[:, 1].mean().item()
        mean_r = reward.mean().item()
        with open("/results/feet_air_debug.txt", "a") as f:
            f.write(
                f"step={env._feet_air_debug_step} "
                f"both_ground={both_ground}/{n} one_air={one_air}/{n} both_air={both_air}/{n} "
                f"h0={mean_h0:.4f} h1={mean_h1:.4f} "
                f"reward={mean_r:.6f}\n"
            )

    return reward


def feet_air_time_adaptive(
    env: "ManagerBasedRLEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg,
    threshold_min: float = 0.05,
    threshold_max: float = 0.5,
    height_threshold: float = 0.058,
    switch_step: int = 500 * 24,  # switch from continuous to impact after 500 iters
) -> torch.Tensor:
    """Adaptive feet air time: continuous for first N steps, then impact-based.

    Continuous reward bootstraps stepping behavior early in training.
    Impact-based reward refines gait quality once basic walking is established.
    switch_step = 500 iters * 24 steps/env = 12000 steps.
    """
    if env.common_step_counter < switch_step:
        return feet_air_time(env, command_name, asset_cfg, height_threshold)
    else:
        return feet_air_time_impact(
            env, command_name, asset_cfg,
            threshold_min, threshold_max, height_threshold,
        )


def feet_air_time_impact(
    env: "ManagerBasedRLEnv",
    command_name: str,
    asset_cfg: SceneEntityCfg,
    threshold_min: float = 0.05,
    threshold_max: float = 0.5,
    height_threshold: float = 0.058,
) -> torch.Tensor:
    """Impact-based feet air time reward — height detection.

    Rewards at landing proportional to air time. Penalizes short steps
    (< threshold_min). Caps at threshold_max. Uses foot Z for contact.
    """
    asset = env.scene[asset_cfg.name]
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]

    in_air = foot_z > height_threshold
    in_contact = ~in_air

    # Persistent tracking
    if not hasattr(env, "_impact_timer"):
        env._impact_timer = torch.zeros_like(foot_z)
        env._impact_was_air = torch.zeros(
            *foot_z.shape, dtype=torch.bool, device=foot_z.device
        )

    dt = env.step_dt
    timer = env._impact_timer
    was_air = env._impact_was_air

    # First contact: was airborne, now on ground
    first_contact = was_air & in_contact

    # Accumulate air time, reset on contact
    timer[in_air] += dt
    last_air_time = timer.clone()
    timer[in_contact] = 0.0
    was_air[:] = in_air

    # Impact reward at landing
    air_time = (last_air_time - threshold_min) * first_contact
    air_time = torch.clamp(air_time, max=threshold_max - threshold_min)
    reward = torch.sum(air_time, dim=1)

    # No reward for zero command
    reward *= torch.norm(
        env.command_manager.get_command(command_name)[:, :2], dim=1
    ) > 0.1

    # Debug every 50 steps
    if not hasattr(env, "_impact_debug_step"):
        env._impact_debug_step = 0
    env._impact_debug_step += 1
    if env._impact_debug_step % 50 == 0:
        n = in_air.shape[0]
        n_land = first_contact.any(dim=1).sum().item()
        mean_air_t = last_air_time[first_contact].mean().item() if first_contact.any() else 0.0
        mean_r = reward.mean().item()
        with open("/results/feet_air_debug_impact.txt", "a") as f:
            f.write(
                f"step={env._impact_debug_step} "
                f"landings={n_land}/{n} mean_air_t={mean_air_t:.4f} "
                f"reward={mean_r:.6f}\n"
            )

    return reward


def feet_slide(
    env: "ManagerBasedRLEnv",
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    height_threshold: float = 0.02,
) -> torch.Tensor:
    """Penalize feet sliding — height-based contact detection.

    Penalizes horizontal foot velocity when foot is on/near ground.
    Uses foot Z position (not contact forces) to avoid self-collision noise.
    """
    asset = env.scene[asset_cfg.name]
    # Foot height for contact detection
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]  # [num_envs, num_feet]
    on_ground = foot_z < height_threshold  # [num_envs, num_feet]
    # Horizontal velocity of feet
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]  # [num_envs, num_feet, 2]
    reward = torch.sum(body_vel.norm(dim=-1) * on_ground, dim=1)
    return reward


###############################################################################
# Custom event functions
###############################################################################

def randomize_joint_default_pos(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "abs",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """Randomize joint default positions — exact Berkeley."""
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    if asset_cfg.joint_ids == slice(None):
        joint_ids = slice(None)
    else:
        joint_ids = torch.tensor(asset_cfg.joint_ids, dtype=torch.int, device=asset.device)
    if pos_distribution_params is not None:
        pos = asset.data.default_joint_pos.to(asset.device).clone()
        pos = _randomize_prop_by_op(
            pos, pos_distribution_params, env_ids, joint_ids,
            operation=operation, distribution=distribution,
        )[env_ids][:, joint_ids]
        if env_ids != slice(None) and joint_ids != slice(None):
            env_ids = env_ids[:, None]
        asset.data.default_joint_pos[env_ids, joint_ids] = pos


###############################################################################
# Custom curriculum functions — EXACT Berkeley implementations
###############################################################################

def modify_push_force(
    env: "ManagerBasedRLEnv",
    env_ids: Sequence[int],
    term_name: str,
    max_velocity: Sequence[float],
    interval: int,
    starting_step: float = 0.0,
):
    """Adaptive push force curriculum — exact Berkeley.

    Increases push when robot is stable (few falls vs timeouts).
    Decreases when falling too much.
    """
    try:
        term_cfg = env.event_manager.get_term_cfg("push_robot")
    except Exception:
        return 0.0
    curr_setting = term_cfg.params["velocity_range"]["x"][1]
    if env.common_step_counter < starting_step:
        return curr_setting
    if env.common_step_counter % interval == 0:
        base_contact_count = torch.sum(
            env.termination_manager.get_term("base_contact")
        )
        time_out_count = torch.sum(
            env.termination_manager.get_term("time_out")
        )
        # Stable → increase push
        if base_contact_count < time_out_count * 2:
            term_cfg = env.event_manager.get_term_cfg("push_robot")
            curr_setting = term_cfg.params["velocity_range"]["x"][1]
            curr_setting = np.clip(curr_setting * 1.5, 0.0, max_velocity[0])
            term_cfg.params["velocity_range"]["x"] = (-curr_setting, curr_setting)
            curr_setting_y = term_cfg.params["velocity_range"]["y"][1]
            curr_setting_y = np.clip(curr_setting_y * 1.5, 0.0, max_velocity[1])
            term_cfg.params["velocity_range"]["y"] = (-curr_setting_y, curr_setting_y)
            env.event_manager.set_term_cfg("push_robot", term_cfg)
        # Falling too much → decrease push
        if base_contact_count > time_out_count / 2:
            term_cfg = env.event_manager.get_term_cfg("push_robot")
            curr_setting = term_cfg.params["velocity_range"]["x"][1]
            curr_setting = np.clip(curr_setting - 0.2, 0.0, max_velocity[0])
            term_cfg.params["velocity_range"]["x"] = (-curr_setting, curr_setting)
            curr_setting_y = term_cfg.params["velocity_range"]["y"][1]
            curr_setting_y = np.clip(curr_setting_y - 0.2, 0.0, max_velocity[1])
            term_cfg.params["velocity_range"]["y"] = (-curr_setting_y, curr_setting_y)
            env.event_manager.set_term_cfg("push_robot", term_cfg)
    return curr_setting


def modify_command_velocity(
    env: "ManagerBasedRLEnv",
    env_ids: Sequence[int],
    term_name: str,
    max_velocity: Sequence[float],
    interval: int,
    starting_step: float = 0.0,
):
    """Adaptive command velocity curriculum — exact Berkeley.

    Expands lin_vel_x range when tracking reward is good (>80% of max). Forward is +X.
    """
    command_cfg = env.command_manager.get_term("base_velocity").cfg
    curr_lin_vel_x = command_cfg.ranges.lin_vel_x
    if env.common_step_counter < starting_step:
        return curr_lin_vel_x[1]
    if env.common_step_counter % interval == 0:
        term_cfg = env.reward_manager.get_term_cfg(term_name)
        rew = env.reward_manager._episode_sums[term_name][env_ids]
        if (
            torch.mean(rew) / env.max_episode_length
            > 0.8 * term_cfg.weight * env.step_dt
        ):
            curr_lin_vel_x = (
                np.clip(curr_lin_vel_x[0] - 0.5, max_velocity[0], 0.0),
                np.clip(curr_lin_vel_x[1] + 0.5, 0.0, max_velocity[1]),
            )
            command_cfg.ranges.lin_vel_x = curr_lin_vel_x
    return curr_lin_vel_x[1]


###############################################################################
# Robot config — DelayedPDActuator with friction + 15-25ms action delay
###############################################################################

BIPED_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=URDF_DEFAULT,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
        fix_base=False,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            ),
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.80),
        joint_pos={
            "right_hip_pitch.*": 0.08,
            "left_hip_pitch.*": -0.08,
            ".*hip_roll.*": 0.0,
            ".*hip_yaw.*": 0.0,
            ".*knee.*": 0.25,
            ".*foot_pitch.*": -0.17,
            ".*foot_roll.*": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "hip_roll": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_roll.*"],
            effort_limit=50.0, velocity_limit=10.0,
            stiffness=150.0, damping=5.5, armature=0.01,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_yaw": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_yaw.*"],
            effort_limit=50.0, velocity_limit=10.0,
            stiffness=150.0, damping=5.0, armature=0.01,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_pitch.*"],
            effort_limit=100.0, velocity_limit=10.0,
            stiffness=200.0, damping=7.5, armature=0.01,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "knee": DelayedPDActuatorCfg(
            joint_names_expr=[".*knee.*"],
            effort_limit=100.0, velocity_limit=10.0,
            stiffness=200.0, damping=5.0, armature=0.01,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "foot_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_pitch.*"],
            effort_limit=30.0, velocity_limit=10.0,
            stiffness=30.0, damping=2.0, armature=0.01,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
        "foot_roll": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_roll.*"],
            effort_limit=30.0, velocity_limit=10.0,
            stiffness=30.0, damping=2.0, armature=0.01,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
    },
)


###############################################################################
# Scene
###############################################################################

@configclass
class BipedSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        terrain_generator=None,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
        debug_vis=False,
    )
    robot: ArticulationCfg = BIPED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        history_length=3,
        track_air_time=True,
        track_pose=True,
    )
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0),
    )


###############################################################################
# Commands — EXACT Berkeley
###############################################################################

@configclass
class CommandsCfg:
    base_velocity = base_mdp.UniformVelocityCommandCfg(
        resampling_time_range=(10.0, 10.0),
        debug_vis=True,
        asset_name="robot",
        heading_command=True,
        heading_control_stiffness=0.5,
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        ranges=base_mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),      # forward=+X (biased positive)
            lin_vel_y=(-0.3, 0.3),      # lateral (small)
            ang_vel_z=(-0.2, 0.2),
            heading=(-math.pi, math.pi),
        ),
    )


###############################################################################
# Observations — EXACT Berkeley
# base_lin_vel IN policy (with noise), per-joint-group noise levels
# obs_dim = 3+3+3+3+6+2+2+2+12+12 = 48
###############################################################################

@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        base_lin_vel = ObsTerm(
            func=base_mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        base_ang_vel = ObsTerm(
            func=base_mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        projected_gravity = ObsTerm(
            func=base_mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(
            func=base_mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        # Per-joint-group noise — Berkeley exact
        hip_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[
                        ".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*",
                    ],
                ),
            },
            noise=Unoise(n_min=-0.03, n_max=0.03),
        )
        knee_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*knee.*"],
                ),
            },
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        foot_pitch_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*foot_pitch.*"],
                ),
            },
            noise=Unoise(n_min=-0.08, n_max=0.08),
        )
        foot_roll_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*foot_roll.*"],
                ),
            },
            noise=Unoise(n_min=-0.03, n_max=0.03),
        )
        joint_vel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        actions = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(PolicyCfg):
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


###############################################################################
# Actions — EXACT Berkeley (scale=0.5)
###############################################################################

@configclass
class ActionsCfg:
    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=ALL_JOINTS,
        scale={
            "right_foot_roll_02": 0.25, "left_foot_roll_02": 0.25,
            "right_hip_yaw_03": 0.5, "right_hip_roll_03": 0.5, "right_hip_pitch_04": 0.5,
            "right_knee_04": 0.5, "right_foot_pitch_02": 0.5,
            "left_hip_yaw_03": 0.5, "left_hip_roll_03": 0.5, "left_hip_pitch_04": 0.5,
            "left_knee_04": 0.5, "left_foot_pitch_02": 0.5,
        },
        preserve_order=True,
        use_default_offset=True,
    )


###############################################################################
# Rewards — EXACT Berkeley flat config (13 terms)
#
# Berkeley rough base → flat overrides:
#   flat_orientation: -0.5 → -5.0
#   joint_torques:   -1e-5 → -2.5e-5
#   feet_air_time:     2.0 → 0.5
#   dof_pos_limits:    0.0 → -1.0 (set by rough, not changed by flat)
###############################################################################

@configclass
class RewardsCfg:
    # -- tracking (positive)
    track_lin_vel_xy_exp = RewTerm(
        func=base_mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=base_mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    # -- penalties
    lin_vel_z_l2 = RewTerm(func=base_mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=base_mdp.ang_vel_xy_l2, weight=-0.05)
    joint_torques_l2 = RewTerm(
        func=base_mdp.joint_torques_l2,
        weight=-1.0e-5,
    )
    action_rate_l2 = RewTerm(func=base_mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(
        func="biped_env_cfg:feet_air_time_berkeley",
        weight=10.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
            "threshold_min": 0.2,
            "threshold_max": 0.5,
        },
    )
    feet_slide = RewTerm(
        func="biped_env_cfg:feet_slide_berkeley",
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names="foot_6061.*"),
        },
    )
    undesired_contacts = RewTerm(
        func=base_mdp.undesired_contacts,
        weight=-1.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=[
                    "kd_d_102r_6061", "kd_d_102l_6061",  # hip yaw links (≈ HAA)
                    "kd_d_201r_6061", "rs03",              # hip pitch links (≈ HFE)
                ],
            ),
            "threshold": 1.0,
        },
    )
    joint_deviation_hip = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*hip_roll.*", ".*hip_yaw.*"],
            ),
        },
    )
    joint_deviation_knee = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-0.01,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"]),
        },
    )
    joint_deviation_foot_roll = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_roll.*"]),
        },
    )
    stand_still = RewTerm(
        func="biped_env_cfg:stand_still",
        weight=-0.2,
        params={
            "command_name": "base_velocity",
            "asset_cfg": SceneEntityCfg("robot", body_names="foot_6061.*"),
        },
    )
    flat_orientation_l2 = RewTerm(func=base_mdp.flat_orientation_l2, weight=-0.5)
    dof_pos_limits = RewTerm(func=base_mdp.joint_pos_limits, weight=-1.0)


###############################################################################
# Terminations — EXACT Berkeley
###############################################################################

@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=base_mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names="assy_formfg___kd_b_102b_torso_btm",
            ),
            "threshold": 1.0,
        },
    )


###############################################################################
# Events — Berkeley events + push_robot for curriculum
###############################################################################

@configclass
class EventsCfg:
    # --- Startup ---
    physics_material = EventTerm(
        func=base_mdp.randomize_rigid_body_material,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.0),
            "dynamic_friction_range": (0.3, 1.0),
            "restitution_range": (0.0, 0.1),
            "num_buckets": 64,
        },
        mode="startup",
    )
    scale_all_link_masses = EventTerm(
        func=base_mdp.randomize_rigid_body_mass,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "mass_distribution_params": (1.0, 1.2),
            "operation": "scale",
        },
        mode="startup",
    )
    add_base_mass = EventTerm(
        func=base_mdp.randomize_rigid_body_mass,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", body_names="assy_formfg___kd_b_102b_torso_btm",
            ),
            "mass_distribution_params": (-1.0, 1.0),
            "operation": "add",
        },
        mode="startup",
    )
    add_all_joint_default_pos = EventTerm(
        func="biped_env_cfg:randomize_joint_default_pos",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "pos_distribution_params": (-0.05, 0.05),
            "operation": "add",
        },
        mode="startup",
    )
    scale_all_joint_armature = EventTerm(
        func=base_mdp.randomize_joint_parameters,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "armature_distribution_params": (1.0, 1.05),
            "operation": "scale",
        },
        mode="startup",
    )
    scale_all_joint_friction = EventTerm(
        func=base_mdp.randomize_joint_parameters,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "friction_distribution_params": (0.9, 1.1),
            "operation": "scale",
        },
        mode="startup",
    )
    scale_all_actuator_gains = EventTerm(
        func=base_mdp.randomize_actuator_gains,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
        },
        mode="startup",
    )

    # --- Reset ---
    reset_base = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
        mode="reset",
    )
    reset_robot_joints = EventTerm(
        func=base_mdp.reset_joints_by_scale,
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
        mode="reset",
    )
    base_external_force_torque = EventTerm(
        func=base_mdp.apply_external_force_torque,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", body_names="assy_formfg___kd_b_102b_torso_btm",
            ),
            "force_range": (0.0, 0.0),
            "torque_range": (0.0, 0.0),
        },
        mode="reset",
    )

    # --- Interval ---
    push_robot = EventTerm(
        func=base_mdp.push_by_setting_velocity,
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
        mode="interval",
        interval_range_s=(10.0, 15.0),
    )


###############################################################################
# Curriculum — Berkeley-style (push_force + command_vel)
#
# push_force_levels: after iter 1000, adaptively increase/decrease push
#   based on base_contact vs time_out ratio (Berkeley starts at 1500)
# command_vel: after iter 3000, expand lin_vel_x when tracking > 80% of max
#   (Berkeley starts at 5000). Forward is +X.
###############################################################################

@configclass
class CurriculumsCfg:
    push_force_levels = CurrTerm(
        func=modify_push_force,
        params={
            "term_name": "push_robot",
            "max_velocity": [0.75, 0.75],     # max push 0.75 m/s
            "interval": 200 * 24,
            "starting_step": 1000 * 24,     # start after 1000 iterations
        },
    )
    command_vel = CurrTerm(
        func=modify_command_velocity,
        params={
            "term_name": "track_lin_vel_xy_exp",
            "max_velocity": [-0.5, 1.0],
            "interval": 200 * 24,
            "starting_step": 5000 * 24,     # start after 5000 iterations
        },
    )


###############################################################################
# Main Env Config
###############################################################################

@configclass
class BipedFlatEnvCfg(ManagerBasedRLEnvCfg):
    scene: BipedSceneCfg = BipedSceneCfg(num_envs=16384, env_spacing=2.5)
    commands: CommandsCfg = CommandsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    curriculum: CurriculumsCfg = CurriculumsCfg()

    def __post_init__(self):
        self.decimation = 4  # 50 Hz control (Berkeley exact)
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = False  # required for self-collisions
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt


@configclass
class BipedFlatEnvCfg_PLAY(BipedFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 100
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None  # disabled for this test
