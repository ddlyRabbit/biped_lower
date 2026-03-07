# Biped Student Config — G1-style teacher-student distillation
#
# Removes base_lin_vel from policy observations (not available on real hardware).
# Teacher obs group retains base_lin_vel for the DistillationRunner.
#
# Student obs_dim = 48 - 3 = 45
# Teacher obs_dim = 48 (same as original flat config)
#
# Flow:
#   1. Train teacher: biped_train_rsl.py (standard PPO, flat config with base_lin_vel)
#   2. Distill student: biped_distill_rsl.py --teacher_checkpoint <path>
#      - Loads teacher weights, trains student MLP via MSE(student_actions, teacher_actions)
#   3. Fine-tune student: biped_distill_rsl.py --finetune --resume <distilled_model>
#      - Standard PPO using student obs only

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.envs import mdp as base_mdp

from biped_env_cfg import BipedFlatEnvCfg


@configclass
class StudentObservationsCfg:
    """Observation config with separate student (policy) and teacher obs groups.

    Student (policy): No base_lin_vel — only real-sensor observations.
    Teacher: Full observations including base_lin_vel (privileged).
    """

    @configclass
    class StudentPolicyCfg(ObsGroup):
        """Student observations — real-sensor only (no base_lin_vel)."""

        # NOTE: base_lin_vel intentionally omitted
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
    class TeacherPolicyCfg(ObsGroup):
        """Teacher observations — includes privileged base_lin_vel."""

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
    class CriticCfg(ObsGroup):
        """Critic sees everything (same as teacher)."""

        base_lin_vel = ObsTerm(func=base_mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=base_mdp.base_ang_vel)
        projected_gravity = ObsTerm(func=base_mdp.projected_gravity)
        velocity_commands = ObsTerm(
            func=base_mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
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
        )
        knee_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*knee.*"],
                ),
            },
        )
        foot_pitch_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*foot_pitch.*"],
                ),
            },
        )
        foot_roll_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot", joint_names=[".*foot_roll.*"],
                ),
            },
        )
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)
        actions = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: StudentPolicyCfg = StudentPolicyCfg()
    teacher: TeacherPolicyCfg = TeacherPolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class BipedStudentFlatEnvCfg(BipedFlatEnvCfg):
    """Flat env for student distillation — adds teacher obs group."""

    observations: StudentObservationsCfg = StudentObservationsCfg()


@configclass
class BipedStudentFlatEnvCfg_PLAY(BipedStudentFlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 100
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
