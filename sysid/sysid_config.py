"""Sysid robot configuration — V73 actuators, light URDF."""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import DelayedPDActuatorCfg

URDF_LIGHT = "/uploads/light/robot.urdf"

SYSID_ROBOT_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UrdfFileCfg(
        asset_path=URDF_LIGHT,
        activate_contact_sensors=False,
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
            fix_root_link=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.30),
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
            stiffness=80.0, damping=3.0, armature=0.0112,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_yaw": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_yaw.*"],
            effort_limit=50.0, velocity_limit=10.0,
            stiffness=80.0, damping=3.0, armature=0.0112,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_pitch.*"],
            effort_limit=100.0, velocity_limit=10.0,
            stiffness=120.0, damping=3.0, armature=0.0152,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "knee": DelayedPDActuatorCfg(
            joint_names_expr=[".*knee.*"],
            effort_limit=100.0, velocity_limit=10.0,
            stiffness=120.0, damping=3.0, armature=0.024,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "foot_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_pitch.*"],
            effort_limit=30.0, velocity_limit=10.0,
            stiffness=64.0, damping=2.0, armature=0.0112,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
        "foot_roll": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_roll.*"],
            effort_limit=30.0, velocity_limit=10.0,
            stiffness=64.0, damping=2.0, armature=0.001,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
    },
)

# Physics config
SIM_DT = 0.005        # 200Hz physics
DECIMATION = 4        # control at 50Hz (matching training)
CONTROL_DT = SIM_DT * DECIMATION  # 0.02s

# Joint index in the articulation (Isaac joint order from URDF)
# Verified from robot.data.joint_names after spawning
JOINT_INDEX = {
    "R_hip_pitch": 5,
    "R_hip_roll": 1,
    "R_hip_yaw": 3,
    "R_knee": 7,
    "R_foot_pitch": 9,
    "R_foot_roll": 11,
    "L_hip_pitch": 4,
    "L_hip_roll": 0,
    "L_hip_yaw": 2,
    "L_knee": 6,
    "L_foot_pitch": 8,
    "L_foot_roll": 10,
}

# Default positions (same as init_state, in joint index order)
DEFAULT_POS = {
    "R_hip_pitch": 0.08, "L_hip_pitch": -0.08,
    "R_hip_roll": 0.0, "L_hip_roll": 0.0,
    "R_hip_yaw": 0.0, "L_hip_yaw": 0.0,
    "R_knee": 0.25, "L_knee": 0.25,
    "R_foot_pitch": -0.17, "L_foot_pitch": -0.17,
    "R_foot_roll": 0.0, "L_foot_roll": 0.0,
}

MOTOR_TYPES = {
    "R_hip_pitch": "RS04", "R_hip_roll": "RS03", "R_hip_yaw": "RS03",
    "R_knee": "RS04", "R_foot_pitch": "RS02", "R_foot_roll": "RS02",
}

# Test parameters per joint
TEST_PARAMS = {
    "R_hip_pitch": {"step": 0.4, "amp": 0.3, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
    "R_hip_roll": {"step": -0.3, "amp": 0.3, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
    "R_hip_yaw": {"step": 0.3, "amp": 0.3, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
    "R_knee": {"step": 0.8, "amp": 0.3, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
    "R_foot_pitch": {"step": 0.1, "amp": 0.15, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
    "R_foot_roll": {"step": 0.1, "amp": 0.1, "freqs": [0.5, 1.0, 2.0, 5.0, 10.0]},
}
