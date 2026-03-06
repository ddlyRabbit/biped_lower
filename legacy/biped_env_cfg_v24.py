# Biped flat terrain velocity tracking - V24
# V22 + knee bend + BUGFIXES:
#   1. hip_abduction_penalty was wrong — didn't account for mirrored L/R hip roll ranges
#      Right hip roll: [-2.27, 0.21], Left: [-0.21, 2.27] — abs() doesn't work
#      FIX: Remove hip_abduction, rely on joint_deviation instead
#   2. joint_deviation_all at -0.5 fought knee_bend incentive (init knee=0.4, target=0.3)
#      FIX: Exclude knees from joint_deviation, let knee_bend handle them
#   3. knee_bend was too weak (1.5) vs conflicting penalties
#      FIX: Increase to 2.5, raise target to 0.4 (match init pose)
#   4. knee_overbend threshold 0.4 == knee_bend target, creating dead zone
#      FIX: Raise threshold to 0.8 (45°) — allow natural walking flexion
# Continuing from V22 best checkpoint.

import math
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim.converters.urdf_converter_cfg import UrdfConverterCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab.envs import mdp as base_mdp
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
    TerminationsCfg as BaseTerminationsCfg,
)


##
# Custom reward functions — V22
##

def track_vel_vector(env, target_vel_x: float = 0.0, target_vel_y: float = -0.5, std: float = 0.25):
    """PRIMARY REWARD: Track commanded velocity vector in WORLD frame.
    Fixed direction [0, -0.5] regardless of robot orientation.
    """
    root_vel_w = env.scene["robot"].data.root_lin_vel_w[:, :2]
    target = torch.tensor([[target_vel_x, target_vel_y]], device=root_vel_w.device)
    error = torch.sum(torch.square(root_vel_w - target), dim=1)
    return torch.exp(-error / (std ** 2))


def yaw_rate_penalty(env):
    """Penalize yaw angular velocity — keeps robot facing initial direction.
    Without this, robot can yaw freely and walk sideways while still
    tracking world-frame velocity, making target appear to move with robot.
    """
    ang_vel = env.scene["robot"].data.root_ang_vel_w[:, 2]  # yaw rate (Z axis)
    return torch.square(ang_vel)


def base_height_penalty(env, min_height: float = 0.55):
    """CONSTRAINT: Quadratic penalty when height drops below min_height."""
    root_pos = env.scene["robot"].data.root_pos_w
    height = root_pos[:, 2]
    deficit = torch.clamp(min_height - height, min=0.0)
    return deficit ** 2


def gait_phase_reward(
    env,
    sensor_cfg: SceneEntityCfg,
    command_name: str = "base_velocity",
    gait_freq: float = 1.5,
):
    """Reward alternating foot contacts matching a sinusoidal phase clock."""
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    contact = torch.norm(forces, dim=-1) > 1.0

    phase = env.episode_length_buf * env.step_dt * gait_freq * 2 * math.pi
    sin_phase = torch.sin(phase)

    left_expected = (sin_phase > 0).float()
    right_expected = (sin_phase <= 0).float()
    expected = torch.stack([left_expected, right_expected], dim=-1)

    agreement = (contact.float() * expected + (1 - contact.float()) * (1 - expected))
    reward = agreement.mean(dim=-1)

    commands = env.command_manager.get_command(command_name)
    move_mask = (torch.norm(commands[:, :2], dim=-1) > 0.1).float()
    return reward * move_mask


def foot_clearance_reward(
    env,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    target_height: float = 0.04,
):
    """Reward feet achieving clearance during swing phase."""
    contact_sensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    in_contact = torch.norm(forces, dim=-1) > 1.0
    in_swing = ~in_contact

    asset = env.scene[asset_cfg.name]
    foot_pos = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]

    clearance_achieved = (foot_pos > target_height).float()
    swing_clearance = in_swing.float() * clearance_achieved

    has_swing = in_swing.any(dim=-1).float()
    reward = swing_clearance.sum(dim=-1) / (in_swing.float().sum(dim=-1) + 1e-6)
    return reward * has_swing


def knee_bend_incentive(env, asset_cfg: SceneEntityCfg, target_bend: float = 0.3, std: float = 0.2):
    """Reward knees being near a target bend angle during walking.
    Encourages natural knee flexion rather than stiff-legged shuffling.
    Gaussian peak at target_bend, decays for straight or over-bent knees.
    """
    asset = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]  # [num_envs, num_knees]
    # Reward each knee for being near target bend
    error = torch.square(joint_pos - target_bend)
    reward_per_knee = torch.exp(-error / (std ** 2))
    return reward_per_knee.mean(dim=-1)


def hip_abduction_penalty(env, asset_cfg: SceneEntityCfg, threshold: float = 0.12):
    """Penalize excessive hip roll (wide stance)."""
    asset = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    excess = torch.abs(joint_pos) - threshold
    return torch.sum(torch.clamp(excess, min=0.0), dim=-1)


def knee_overbend_penalty(env, asset_cfg: SceneEntityCfg, threshold: float = 0.4):
    """Penalize deep knee flexion (squat)."""
    asset = env.scene[asset_cfg.name]
    joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    excess = joint_pos - threshold
    return torch.sum(torch.clamp(excess, min=0.0), dim=-1)


##
# Biped Articulation Config
##

BIPED_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path="/uploads/robot.urdf",
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
        pos=(0.0, 0.0, 0.75),
        joint_pos={
            ".*hip_pitch.*": -0.2,
            ".*hip_roll.*": 0.0,
            ".*hip_yaw.*": 0.0,
            ".*knee.*": 0.4,
            ".*foot_pitch.*": -0.2,
            ".*foot_roll.*": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "hips": ImplicitActuatorCfg(
            joint_names_expr=[".*hip_pitch.*", ".*hip_roll.*", ".*hip_yaw.*"],
            effort_limit_sim=200,
            stiffness={
                ".*hip_pitch.*": 150.0,
                ".*hip_roll.*": 150.0,
                ".*hip_yaw.*": 100.0,
            },
            damping={
                ".*hip_pitch.*": 10.0,
                ".*hip_roll.*": 10.0,
                ".*hip_yaw.*": 10.0,
            },
        ),
        "knees": ImplicitActuatorCfg(
            joint_names_expr=[".*knee.*"],
            effort_limit_sim=200,
            stiffness={".*knee.*": 200.0},
            damping={".*knee.*": 10.0},
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*foot_pitch.*", ".*foot_roll.*"],
            effort_limit_sim=100,
            stiffness={
                ".*foot_pitch.*": 40.0,
                ".*foot_roll.*": 40.0,
            },
            damping={
                ".*foot_pitch.*": 5.0,
                ".*foot_roll.*": 5.0,
            },
        ),
    },
)


##
# V22 Rewards
#
# V21 base + yaw stability + leg lift incentives
##

@configclass
class BipedRewards(RewardsCfg):

    # ============ OVERRIDE PARENT TERMS ============
    track_lin_vel_xy_exp = None
    track_ang_vel_z_exp = None
    feet_air_time = None
    dof_acc_l2 = None

    # ============ PRIMARY: VELOCITY VECTOR TRACKING ============
    track_vel_vector = RewTerm(
        func="biped_env_cfg:track_vel_vector",
        weight=10.0,
        params={"target_vel_x": 0.0, "target_vel_y": -0.5, "std": 0.25},
    )

    # ============ NEW: YAW STABILITY ============
    # Prevents robot from rotating — locks world-frame direction
    yaw_rate_penalty = RewTerm(
        func="biped_env_cfg:yaw_rate_penalty",
        weight=-2.0,
    )

    # ============ NEW: LEG LIFT INCENTIVES ============
    # Light weights — bootstrap gait without being farmable
    gait_phase = RewTerm(
        func="biped_env_cfg:gait_phase_reward",
        weight=2.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
            "command_name": "base_velocity",
            "gait_freq": 1.5,
        },
    )
    foot_clearance = RewTerm(
        func="biped_env_cfg:foot_clearance_reward",
        weight=1.5,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names="foot_6061.*"),
            "target_height": 0.03,
        },
    )

    # ============ NEW: KNEE BEND INCENTIVE ============
    # Reward moderate knee flexion — prevents stiff-legged shuffling
    # Target 0.4 rad (~23°) matches init pose, std 0.3 for wider acceptance
    knee_bend = RewTerm(
        func="biped_env_cfg:knee_bend_incentive",
        weight=2.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"]),
            "target_bend": 0.4,
            "std": 0.3,
        },
    )

    # ============ CONSTRAINTS ============
    base_height_penalty = RewTerm(
        func="biped_env_cfg:base_height_penalty",
        weight=-8.0,
        params={"min_height": 0.55},
    )

    flat_orientation_l2 = RewTerm(func=base_mdp.flat_orientation_l2, weight=-6.0)

    knee_overbend = RewTerm(
        func="biped_env_cfg:knee_overbend_penalty",
        weight=-5.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"]),
            "threshold": 0.8,  # Was 0.4 — too restrictive, blocked natural walking flexion
        },
    )

    # hip_abduction REMOVED — was using abs() on mirrored L/R joints, causing asymmetric penalty
    # joint_deviation handles hip drift instead

    # Exclude knees — they're managed by knee_bend incentive + knee_overbend penalty
    joint_deviation_all = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*hip.*", ".*foot.*"])},
    )

    ang_vel_xy_l2 = RewTerm(func=base_mdp.ang_vel_xy_l2, weight=-0.3)
    lin_vel_z_l2 = RewTerm(func=base_mdp.lin_vel_z_l2, weight=-0.2)

    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.2,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names="foot_6061.*"),
        },
    )

    action_rate_l2 = RewTerm(func=base_mdp.action_rate_l2, weight=-0.01)
    dof_torques_l2 = RewTerm(func=base_mdp.joint_torques_l2, weight=-1.0e-4)
    dof_pos_limits = RewTerm(func=base_mdp.joint_pos_limits, weight=-1.0)

    undesired_contacts = RewTerm(
        func=base_mdp.undesired_contacts,
        weight=-1.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=[".*kd_d_102r.*", ".*kd_b_102b.*"]),
            "threshold": 1.0,
        },
    )

    no_fly = RewTerm(
        func=base_mdp.desired_contacts,
        weight=-0.5,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*")},
    )

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)


##
# Terminations
##

@configclass
class BipedTerminations(BaseTerminationsCfg):
    base_contact = DoneTerm(
        func=base_mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="assy_formfg___kd_b_102b_torso_btm"),
            "threshold": 50.0,
        },
    )
    bad_orientation = DoneTerm(
        func=base_mdp.bad_orientation,
        params={"limit_angle": 0.5},
    )


##
# Environment Config
##

@configclass
class BipedFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    rewards: BipedRewards = BipedRewards()
    terminations: BipedTerminations = BipedTerminations()

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = BIPED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None
        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/Robot/.*"

        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.base_com = None
        self.events.reset_robot_joints.params["position_range"] = (0.95, 1.05)
        self.events.base_external_force_torque.params["asset_cfg"].body_names = [
            "assy_formfg___kd_b_102b_torso_btm"
        ]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (0.0, 0.0)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        }

        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, -0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.0

        self.scene.num_envs = 4096
        self.decimation = 4
        self.sim.dt = 0.005
        self.episode_length_s = 60.0


@configclass
class BipedFlatEnvCfg_PLAY(BipedFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.8, -0.8)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.rel_standing_envs = 0.0
