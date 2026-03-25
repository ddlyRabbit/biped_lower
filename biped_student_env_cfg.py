# Biped Student Config — G1-style teacher-student distillation
#
# Student obs: No base_lin_vel (not available on real hardware).
#   Flat: 45d per frame × 5 history = 225d
# Teacher obs: Full privileged observations.
#   Flat: 48d per frame × 5 history = 240d (includes base_lin_vel)
#
# Matches G1-style obs: wildcard joint order, obs scaling (ang_vel×0.2,
# joint_vel×0.05), history_length=5.

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.envs import mdp as base_mdp

from biped_env_cfg import BipedFlatEnvCfg


###############################################################################
# Flat Student Observations
#   Student: 45d/frame × 5 = 225d (no base_lin_vel)
#   Teacher: 48d/frame × 5 = 240d (with base_lin_vel)
###############################################################################

@configclass
class FlatStudentObservationsCfg:

    @configclass
    class StudentPolicyCfg(ObsGroup):
        """Student: 45d per frame (no base_lin_vel), history=5 → 225d total."""
        # NOTE: base_lin_vel intentionally omitted
        base_ang_vel = ObsTerm(
            func=base_mdp.base_ang_vel,
            scale=0.2,
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
        joint_pos_rel = ObsTerm(
            func=base_mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel_rel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            scale=0.05,
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class TeacherPolicyCfg(ObsGroup):
        """Teacher: 48d per frame (with base_lin_vel), history=5 → 240d total."""
        base_lin_vel = ObsTerm(
            func=base_mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        base_ang_vel = ObsTerm(
            func=base_mdp.base_ang_vel,
            scale=0.2,
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
        joint_pos_rel = ObsTerm(
            func=base_mdp.joint_pos_rel,
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel_rel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            scale=0.05,
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic: same as teacher, no noise, history=5."""
        base_lin_vel = ObsTerm(func=base_mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=base_mdp.base_ang_vel, scale=0.2)
        projected_gravity = ObsTerm(func=base_mdp.projected_gravity)
        velocity_commands = ObsTerm(
            func=base_mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos_rel = ObsTerm(func=base_mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=base_mdp.joint_vel_rel, scale=0.05)
        last_action = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.history_length = 5
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: StudentPolicyCfg = StudentPolicyCfg()
    teacher: TeacherPolicyCfg = TeacherPolicyCfg()
    critic: CriticCfg = CriticCfg()


###############################################################################
# Environment Configs
###############################################################################

@configclass
class BipedStudentFlatEnvCfg(BipedFlatEnvCfg):
    """Flat env for student distillation."""
    observations: FlatStudentObservationsCfg = FlatStudentObservationsCfg()


@configclass
class BipedStudentFlatEnvCfg_PLAY(BipedStudentFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 100
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
        self.commands.base_velocity.ranges = self.commands.base_velocity.limit_ranges
