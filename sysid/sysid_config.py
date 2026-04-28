"""Sysid robot configuration — V73 actuators, light URDF."""

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import DelayedPDActuatorCfg, ActuatorNetMLPCfg

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
        "hip_roll": ActuatorNetMLPCfg(
            joint_names_expr=[".*hip_roll.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/hip_roll_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
        "hip_yaw": ActuatorNetMLPCfg(
            joint_names_expr=[".*hip_yaw.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/hip_yaw_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
        "hip_pitch": ActuatorNetMLPCfg(
            joint_names_expr=[".*hip_pitch.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/hip_pitch_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
        "knee": ActuatorNetMLPCfg(
            joint_names_expr=[".*knee.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/knee_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
        "foot_pitch": ActuatorNetMLPCfg(
            joint_names_expr=[".*foot_pitch.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/foot_pitch_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
        "foot_roll": ActuatorNetMLPCfg(
            joint_names_expr=[".*foot_roll.*"],
            network_file="/home/ubuntu/workspace/biped_locomotion/actuator_net/foot_roll_net.pt",
            pos_scale=1.0, vel_scale=1.0, torque_scale=1.0,
            input_idx=[0, 1, 2], saturation_effort=100.0, input_order="pos_vel",
        ),
    },
)

# Physics config
SIM_DT = 0.0005       # 2kHz physics
DECIMATION = 40       # control at 50Hz (2kHz physics, PD at 2kHz)
CONTROL_DT = SIM_DT * DECIMATION  # 0.02s (50Hz control, 2kHz PD)

# Joint index in the articulation (Isaac joint order from URDF)
# Verified from robot.data.joint_names after spawning
# Joint index in the articulation (from robot.data.joint_names after spawning)
# Order: left_hip_pitch_04(0), right_hip_pitch_04(1), left_hip_roll_03(2), right_hip_roll_03(3),
#        left_hip_yaw_03(4), right_hip_yaw_03(5), left_knee_04(6), right_knee_04(7),
#        left_foot_pitch_02(8), right_foot_pitch_02(9), left_foot_roll_02(10), right_foot_roll_02(11)
JOINT_INDEX = {
    "L_hip_pitch": 0,
    "R_hip_pitch": 1,
    "L_hip_roll": 2,
    "R_hip_roll": 3,
    "L_hip_yaw": 4,
    "R_hip_yaw": 5,
    "L_knee": 6,
    "R_knee": 7,
    "L_foot_pitch": 8,
    "R_foot_pitch": 9,
    "L_foot_roll": 10,
    "R_foot_roll": 11,
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
