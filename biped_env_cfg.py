# Biped — Unitree G1-style rewards, obs, curriculum
# Migrated from Berkeley-style to match G1 29dof training setup.
# Keeps our URDF, actuators, joint names, and hardware.

###############################################################################
# URDF selection — use --urdf heavy|light flag in training scripts
###############################################################################
URDF_HEAVY = "/uploads/heavy/robot.urdf"
URDF_LIGHT = "/uploads/light/robot.urdf"
URDF_DEFAULT = URDF_HEAVY

import math
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
from isaaclab.actuators import DelayedPDActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
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
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp as base_mdp

from biped_mdp import UniformLevelVelocityCommandCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


###############################################################################
# Robot config — DelayedPDActuator with friction + action delay
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
# Commands — G1-style with curriculum limit_ranges
###############################################################################

@configclass
class CommandsCfg:
    base_velocity = UniformLevelVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=False,
        debug_vis=True,
        ranges=UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.3, 0.3),
            lin_vel_y=(-0.3, 0.3),
            ang_vel_z=(-0.3, 0.3),
            heading=(-math.pi, math.pi),
        ),
        limit_ranges=UniformLevelVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),
            lin_vel_y=(-0.3, 0.3),
            ang_vel_z=(-0.2, 0.2),
            heading=(-math.pi, math.pi),
        ),
    )


###############################################################################
# Observations — G1-style with history_length=5
# Per frame: ang_vel(3) + gravity(3) + cmd(3) + joint_pos_rel(12) +
#            joint_vel_rel(12) + last_action(12) = 45
###############################################################################

@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(func=base_mdp.base_ang_vel, scale=0.2, noise=Unoise(n_min=-0.2, n_max=0.2))
        projected_gravity = ObsTerm(func=base_mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        velocity_commands = ObsTerm(func=base_mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos_rel = ObsTerm(func=base_mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel_rel = ObsTerm(func=base_mdp.joint_vel_rel, scale=0.05, noise=Unoise(n_min=-1.5, n_max=1.5))
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=base_mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=base_mdp.base_ang_vel, scale=0.2)
        projected_gravity = ObsTerm(func=base_mdp.projected_gravity)
        velocity_commands = ObsTerm(func=base_mdp.generated_commands, params={"command_name": "base_velocity"})
        joint_pos_rel = ObsTerm(func=base_mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=base_mdp.joint_vel_rel, scale=0.05)
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


###############################################################################
# Actions — uniform scale=0.25, wildcard joint order (Isaac alphabetical)
###############################################################################

@configclass
class ActionsCfg:
    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],
        scale=0.25,
        use_default_offset=True,
    )


###############################################################################
# Rewards — G1-style (17 terms)
###############################################################################

@configclass
class RewardsCfg:
    track_lin_vel_xy = RewTerm(
        func="biped_mdp:track_lin_vel_xy_yaw_frame_exp",
        weight=4.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z = RewTerm(
        func=base_mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    alive = RewTerm(func=base_mdp.is_alive, weight=0.15)
    base_linear_velocity = RewTerm(func=base_mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=base_mdp.ang_vel_xy_l2, weight=-0.05)
    joint_vel = RewTerm(func=base_mdp.joint_vel_l2, weight=-0.001)
    joint_acc = RewTerm(func=base_mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate = RewTerm(func=base_mdp.action_rate_l2, weight=-0.05)
    dof_pos_limits = RewTerm(func=base_mdp.joint_pos_limits, weight=-5.0)
    energy = RewTerm(
        func="biped_mdp:energy",
        weight=-2e-5,
    )
    joint_deviation_legs = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*hip_roll.*", ".*hip_yaw.*"])},
    )
    flat_orientation_l2 = RewTerm(func=base_mdp.flat_orientation_l2, weight=-5.0)
    base_height = RewTerm(
        func=base_mdp.base_height_l2,
        weight=-0.5,
        params={"target_height": 0.8},
    )
    gait = RewTerm(
        func="biped_mdp:feet_gait",
        weight=0.5,
        params={
            "period": 0.8,
            "offset": [0.0, 0.5],
            "threshold": 0.55,
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
        },
    )
    feet_slide = RewTerm(
        func="biped_mdp:feet_slide",
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="foot_6061.*"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names="foot_6061.*"),
        },
    )
    feet_clearance = RewTerm(
        func="biped_mdp:foot_clearance_reward",
        weight=1.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.1,
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
                    "assy_formfg___kd_b_102b_torso_btm",
                    "kd_d_102r_6061", "kd_d_102l_6061",
                    "kd_d_201r_6061", "rs03",
                    "kd_d_301r_6061", "kd_d_301l_6061",
                    "kd_d_401r_6061", "kd_d_401l_6061",
                    "arb_uj111_cross_bearing", "arb_uj111_cross_bearing_2",
                ],
            ),
            "threshold": 1.0,
        },
    )


###############################################################################
# Terminations
###############################################################################

@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    base_height = DoneTerm(
        func=base_mdp.root_height_below_minimum,
        params={"minimum_height": 0.2},
    )
    bad_orientation = DoneTerm(
        func=base_mdp.bad_orientation,
        params={"limit_angle": 0.8},
    )


###############################################################################
# Events — G1-style domain randomization
###############################################################################

@configclass
class EventsCfg:
    physics_material = EventTerm(
        func=base_mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.3, 1.0),
            "dynamic_friction_range": (0.3, 1.0),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )
    add_base_mass = EventTerm(
        func=base_mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="assy_formfg___kd_b_102b_torso_btm"),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
        },
    )
    base_external_force_torque = EventTerm(
        func=base_mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="assy_formfg___kd_b_102b_torso_btm"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )
    reset_base = EventTerm(
        func=base_mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
                "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
            },
        },
    )
    reset_robot_joints = EventTerm(
        func=base_mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (1.0, 1.0),
            "velocity_range": (-1.0, 1.0),
        },
    )
    push_robot = EventTerm(
        func=base_mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(5.0, 5.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )


###############################################################################
# Curriculum — G1-style velocity command levels
###############################################################################

@configclass
class CurriculumCfg:
    lin_vel_cmd_levels = CurrTerm(func="biped_mdp:lin_vel_cmd_levels")


###############################################################################
# Main Env Config
###############################################################################

@configclass
class BipedFlatEnvCfg(ManagerBasedRLEnvCfg):
    scene: BipedSceneCfg = BipedSceneCfg(num_envs=4096, env_spacing=2.5)
    commands: CommandsCfg = CommandsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.disable_contact_processing = False
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
        self.events.push_robot = None
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
