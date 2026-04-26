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
        fix_base=True,
        joint_drive=sim_utils.UrdfFileCfg.JointDriveCfg(
            target_type="position",
            gains=sim_utils.UrdfFileCfg.JointDriveCfg.PDGainsCfg(
                stiffness=0.0,
                damping=0.0,
            ),
        ),
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
        # V74 gains (current training: 10/10/15/15/16/16)
        "hip_roll": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_roll.*"],
            effort_limit=50.0, velocity_limit=21.0,
            stiffness=10.0, damping=3.0, armature=0.0112,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_yaw": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_yaw.*"],
            effort_limit=50.0, velocity_limit=21.0,
            stiffness=10.0, damping=3.0, armature=0.0112,
            friction=0.375,
            min_delay=0, max_delay=1,
        ),
        "hip_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*hip_pitch.*"],
            effort_limit=100.0, velocity_limit=21.0,
            stiffness=15.0, damping=3.0, armature=0.0152,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "knee": DelayedPDActuatorCfg(
            joint_names_expr=[".*knee.*"],
            effort_limit=100.0, velocity_limit=21.0,
            stiffness=15.0, damping=3.0, armature=0.024,
            friction=0.5,
            min_delay=0, max_delay=1,
        ),
        "foot_pitch": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_pitch.*"],
            effort_limit=30.0, velocity_limit=30.0,
            stiffness=16.0, damping=2.0, armature=0.0112,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
        "foot_roll": DelayedPDActuatorCfg(
            joint_names_expr=[".*foot_roll.*"],
            effort_limit=30.0, velocity_limit=30.0,
            stiffness=16.0, damping=2.0, armature=0.001,
            friction=0.25,
            min_delay=0, max_delay=1,
        ),
    },
)

# Physics config
SIM_DT = 0.0005       # 2kHz physics
DECIMATION = 40       # control at 50Hz (2kHz physics, PD at 2kHz)
CONTROL_DT = SIM_DT * DECIMATION  # 0.02s (50Hz control, 2kHz PD)

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

import os
import yaml
import math

_this_dir = os.path.dirname(os.path.abspath(__file__))
CHIRP_YAML = os.path.abspath(os.path.join(_this_dir, "../deploy/biped_ws/src/biped_bringup/config/chirp.yaml"))
with open(CHIRP_YAML, 'r') as f:
    _wcfg = yaml.safe_load(f)["chirp"]["joints"]

# Dynamically loaded limits for sysid (Hips 50%, Knee 50%, Ankle 70%)
JOINT_LIMITS_CONFIG = {
    j: (math.radians(data["min"]), math.radians(data["max"]))
    for j, data in _wcfg.items()
}

SINE_FREQS = [0.5, 1.0, 2.0, 5.0, 10.0]


def compute_test_params(joint_name):
    """Compute step target and sine amplitude from config limits.

    Matches deploy/scripts/motor_sysid.py exactly:
        mid = (lo + hi) / 2
        half_range = (hi - lo) / 2
        step_target = mid + half_range * 0.5
        sine_amplitude = half_range * 0.5
    """
    lo, hi = JOINT_LIMITS_CONFIG.get(joint_name, (-0.5, 0.5))
    mid = (lo + hi) / 2.0
    half_range = (hi - lo) / 2.0
    return {
        "step": mid + half_range * 0.5,
        "amp": half_range * 0.5,  # Base amplitude
        "freqs": SINE_FREQS,
        "amps": [half_range * 0.5 / max(1.0, f) for f in SINE_FREQS],
    }


# Pre-computed for reference
TEST_PARAMS = {name: compute_test_params(name) for name in JOINT_LIMITS_CONFIG}
