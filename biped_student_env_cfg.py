# Biped Student Config — G1-style teacher-student distillation
#
# Student obs: No base_lin_vel (not available on real hardware).
#   Flat: 45-dim | Rough: 232-dim (with height_scan, no base_lin_vel)
# Teacher obs: Full privileged observations.
#   Flat: 48-dim (includes base_lin_vel)
#   Rough: 235-dim (includes base_lin_vel + height_scan)
#
# Student is NOT terrain-blind — has height_scan (real sensor).
# Only base_lin_vel is removed (not available on real hardware).
#
# Flow:
#   1. Train teacher: biped_train_rsl.py (standard PPO, with base_lin_vel + height_scan)
#   2. Distill student: biped_distill_rsl.py --teacher_checkpoint <path>
#      - Loads teacher weights, trains student MLP via MSE(student_actions, teacher_actions)
#   3. Deploy student on real hardware (45-dim obs only)

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from isaaclab.envs import mdp as base_mdp

from biped_env_cfg import BipedFlatEnvCfg
from biped_rough_env_cfg import BipedRoughEnvCfg


###############################################################################
# Shared observation terms (student = real-sensor only)
###############################################################################

def _student_obs_terms():
    """Returns obs terms available on real hardware (no base_lin_vel, no height_scan)."""
    return dict(
        base_ang_vel=ObsTerm(
            func=base_mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        ),
        projected_gravity=ObsTerm(
            func=base_mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        velocity_commands=ObsTerm(
            func=base_mdp.generated_commands,
            params={"command_name": "base_velocity"},
        ),
        hip_pos=ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg(
                    "robot",
                    joint_names=[".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*"],
                ),
            },
            noise=Unoise(n_min=-0.03, n_max=0.03),
        ),
        knee_pos=ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"]),
            },
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        foot_pitch_pos=ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_pitch.*"]),
            },
            noise=Unoise(n_min=-0.08, n_max=0.08),
        ),
        foot_roll_pos=ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_roll.*"]),
            },
            noise=Unoise(n_min=-0.03, n_max=0.03),
        ),
        joint_vel=ObsTerm(
            func=base_mdp.joint_vel_rel,
            noise=Unoise(n_min=-1.5, n_max=1.5),
        ),
        actions=ObsTerm(func=base_mdp.last_action),
    )


###############################################################################
# Flat Student Observations (student=45d, teacher=48d)
###############################################################################

@configclass
class FlatStudentObservationsCfg:

    @configclass
    class StudentPolicyCfg(ObsGroup):
        """Student: 45-dim (no base_lin_vel)."""

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class TeacherPolicyCfg(ObsGroup):
        """Teacher: 48-dim (same as flat training policy obs)."""

        base_lin_vel = ObsTerm(
            func=base_mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic: same as teacher, no noise."""

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
                    "robot", joint_names=[".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*"],
                ),
            },
        )
        knee_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"])},
        )
        foot_pitch_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_pitch.*"])},
        )
        foot_roll_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_roll.*"])},
        )
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)
        actions = ObsTerm(func=base_mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: StudentPolicyCfg = StudentPolicyCfg()
    teacher: TeacherPolicyCfg = TeacherPolicyCfg()
    critic: CriticCfg = CriticCfg()


###############################################################################
# Rough Student Observations (student=45d, teacher=235d)
###############################################################################

@configclass
class RoughStudentObservationsCfg:

    @configclass
    class StudentPolicyCfg(ObsGroup):
        """Student: 232-dim (no base_lin_vel, WITH height_scan)."""

        height_scan = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class TeacherPolicyCfg(ObsGroup):
        """Teacher: 235-dim (base_lin_vel + all obs + height_scan)."""

        base_lin_vel = ObsTerm(
            func=base_mdp.base_lin_vel,
            noise=Unoise(n_min=-0.1, n_max=0.1),
        )
        height_scan = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        """Critic: same as teacher, no noise."""

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
                    "robot", joint_names=[".*hip_roll.*", ".*hip_yaw.*", ".*hip_pitch.*"],
                ),
            },
        )
        knee_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*knee.*"])},
        )
        foot_pitch_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_pitch.*"])},
        )
        foot_roll_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*foot_roll.*"])},
        )
        joint_vel = ObsTerm(func=base_mdp.joint_vel_rel)
        actions = ObsTerm(func=base_mdp.last_action)
        height_scan = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: StudentPolicyCfg = StudentPolicyCfg()
    teacher: TeacherPolicyCfg = TeacherPolicyCfg()
    critic: CriticCfg = CriticCfg()


###############################################################################
# Populate student and teacher obs terms dynamically
# (Student groups get _student_obs_terms; teacher groups get them + privileged)
###############################################################################

# Flat student policy — 45-dim
for _name, _term in _student_obs_terms().items():
    setattr(FlatStudentObservationsCfg.StudentPolicyCfg, _name, _term)

# Flat teacher policy — 48-dim (base_lin_vel already defined + student terms)
for _name, _term in _student_obs_terms().items():
    setattr(FlatStudentObservationsCfg.TeacherPolicyCfg, _name, _term)

# Rough student policy — 45-dim (same as flat student, terrain blind)
for _name, _term in _student_obs_terms().items():
    setattr(RoughStudentObservationsCfg.StudentPolicyCfg, _name, _term)

# Rough teacher policy — 235-dim (base_lin_vel + height_scan already defined + student terms)
for _name, _term in _student_obs_terms().items():
    setattr(RoughStudentObservationsCfg.TeacherPolicyCfg, _name, _term)


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


@configclass
class BipedStudentRoughEnvCfg(BipedRoughEnvCfg):
    """Rough env for student distillation.
    
    Student keeps height_scan (real sensor), only base_lin_vel is removed.
    BipedRoughEnvCfg.__post_init__ adds height_scan to policy/critic — we keep it.
    """
    observations: RoughStudentObservationsCfg = RoughStudentObservationsCfg()


@configclass
class BipedStudentRoughEnvCfg_PLAY(BipedStudentRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False
        self.scene.terrain.max_init_terrain_level = 9
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
