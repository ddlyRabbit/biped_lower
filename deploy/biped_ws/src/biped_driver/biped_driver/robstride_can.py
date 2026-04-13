"""Biped-specific adapter around the RobStride shared library.

Adds:
  - Ankle parallel linkage transform (2× RS02 → foot pitch/roll)
  - Multi-bus manager (routes joints to correct CAN bus)
  - Soft stop joint protection (2° buffer with restoring torque)
  - Convenience config loader from robot.yaml format

All motor limits come from calibration data.  URDF values are only
used as fallback when no calibration is loaded.

Upstream library: biped_driver.robstride_dynamics (vendored from Seeed)
"""

import math
from dataclasses import dataclass
from typing import Optional

from .robstride_dynamics import RobstrideBus, Motor, MotorFeedback


# ── Ankle parallel linkage ──────────────────────────────────────────
#
# Reference: github.com/asimovinc/asimov-v0  mechanical/ankle_mechanism.md
#
# Two RS02 motors per ankle, U-joint to foot plate.
# Motor A (upper, knee-side) and Motor B (lower, foot-side) have
# INVERTED mounting: positive rotation on B pushes its linkage DOWN
# while positive rotation on A pushes its linkage UP.
#
#   y_A =  r × sin(θ_A)     (positive rotation → up)
#   y_B = -r × sin(θ_B)     (positive rotation → down)
#
# From Onshape CAD:
#   Crank radius (r):  32.249 mm
#   d (pivot→bar):     41.14 mm forward
#   c/2 (bar half):    31.398 mm lateral
#
# K_P = d / r        = 41.14  / 32.249 = 1.2757
# K_R = (c/2) / r    = 31.398 / 32.249 = 0.9736
# INV_P = 1/(2×K_P)  = 0.3919
# INV_R = 1/(2×K_R)  = 0.5136
#
# Forward (joint → motors):
#   motor_A (upper) =  K_P × pitch − K_R × roll
#   motor_B (lower) = −K_P × pitch − K_R × roll
#
# Inverse (motors → joint):
#   pitch =  INV_P × (A − B)
#   roll  = −INV_R × (A + B)

_PITCH_GAIN = 41.14 / 32.249    # K_P = 1.2757
_ROLL_GAIN  = 31.398 / 32.249   # K_R = 0.9736
_INV_PITCH  = 32.249 / (2 * 41.14)   # 1/(2*K_P) = 0.3919
_INV_ROLL   = 32.249 / (2 * 31.398)  # 1/(2*K_R) = 0.5136


def ankle_command_to_motors(
    pitch_cmd: float, roll_cmd: float, pitch_sign: int = 1
) -> tuple[float, float]:
    """Forward: joint-space (pitch, roll) → motor-space (upper, lower).

    motor_A (upper) =  pitch_sign × K_P × pitch − K_R × roll
    motor_B (lower) = −pitch_sign × K_P × pitch − K_R × roll

    pitch_sign: +1 for R ankle, −1 for L ankle (mirrored pitch axis).
    """
    motor_upper =  pitch_sign * _PITCH_GAIN * pitch_cmd - _ROLL_GAIN * roll_cmd
    motor_lower = -pitch_sign * _PITCH_GAIN * pitch_cmd - _ROLL_GAIN * roll_cmd
    return motor_upper, motor_lower


def ankle_motors_to_feedback(
    upper_pos: float, lower_pos: float, pitch_sign: int = 1
) -> tuple[float, float]:
    """Inverse: motor-space (upper, lower) → joint-space (pitch, roll).

    pitch =  pitch_sign × INV_P × (A − B)
    roll  = −INV_R × (A + B)

    pitch_sign: +1 for R ankle, −1 for L ankle (mirrored pitch axis).
    """
    foot_pitch =  pitch_sign * _INV_PITCH * (upper_pos - lower_pos)
    foot_roll  = -_INV_ROLL  * (upper_pos + lower_pos)
    return foot_pitch, foot_roll



def ankle_motor_theoretical_limits(is_upper: bool, pitch_sign: int = 1) -> tuple[float, float]:
    """Theoretical motor command range from URDF joint limits.

    Fallback when no calibration is loaded.
    Returns (cmd_lo, cmd_hi) for the given ankle motor.

    pitch_sign: +1 for R ankle, -1 for L ankle.
    upper: cmd =  pitch_sign * K_P * pitch - K_R * roll
    lower: cmd = -pitch_sign * K_P * pitch - K_R * roll
    """
    pitch_min = -0.87267
    pitch_max =  0.52360
    roll_min  = -0.26180
    roll_max  =  0.26180

    # Sign flip changes which corners give min/max — enumerate all four
    sign = pitch_sign if is_upper else -pitch_sign
    corners = [
        sign * _PITCH_GAIN * p - _ROLL_GAIN * r
        for p in (pitch_min, pitch_max)
        for r in (roll_min, roll_max)
    ]
    return min(corners), max(corners)


# Ankle pairs: top_motor → bottom_motor (upper/knee-side → lower/foot-side)
ANKLE_PAIRS = {
    "L_foot_top": "L_foot_bottom",
    "R_foot_top": "R_foot_bottom",
}

ANKLE_TOP_MOTORS = set(ANKLE_PAIRS.keys())       # upper / knee-side
ANKLE_BOTTOM_MOTORS = set(ANKLE_PAIRS.values())   # lower / foot-side
ANKLE_ALL = ANKLE_TOP_MOTORS | ANKLE_BOTTOM_MOTORS

# Motor name → joint-space name (for /joint_states, policy compatibility)
ANKLE_MOTOR_TO_JOINT = {
    "L_foot_top": "L_foot_pitch",  "L_foot_bottom": "L_foot_roll",
    "R_foot_top": "R_foot_pitch",  "R_foot_bottom": "R_foot_roll",
}


# ── Soft stop config ────────────────────────────────────────────────

SOFTSTOP_BUFFER_RAD = math.radians(2.0)  # 0.0349 rad
SOFTSTOP_KP = 20.0  # Nm/rad restoring spring


# ── Multi-bus manager ───────────────────────────────────────────────

@dataclass
class JointConfig:
    """Per-joint configuration.

    All limits are in **motor command-space** and come from calibration.
    For normal joints motor command-space equals joint-space (offset
    just shifts the encoder reference).  For ankle joints motor
    command-space is the post-linkage space.
    """
    name: str
    can_bus: str          # "can0"
    can_id: int
    model: str            # "rs-02", "rs-03", "rs-04"
    offset: float = 0.0   # bus homing_offset (motor-space)
    direction: int = 1    # +1 normal, -1 inverted encoder
    is_ankle: bool = False
    # Motor command-space limits (from calibration, URDF fallback)
    motor_cmd_lo: float = -12.57
    motor_cmd_hi: float = 12.57
    motor_softstop_lo: float = -12.57
    motor_softstop_hi: float = 12.57


class BipedMotorManager:
    """Manages all motors across CAN buses via RobstrideBus instances.

    All soft-stop and clamping logic operates in motor command-space.
    Limits come from calibration data; URDF values are used only as
    a fallback when no calibration is loaded.
    """

    # From URDF — used ONLY as fallback when calibration is absent.
    JOINT_LIMITS = {
        "R_hip_pitch":  (-2.21657, 1.04720),
        "R_hip_roll":   (-2.26893, 0.20944),
        "R_hip_yaw":    (-1.57080, 1.57080),
        "R_knee":       ( 0.00000, 2.70526),
        "L_hip_pitch":  (-1.04720, 2.21657),
        "L_hip_roll":   (-0.20944, 2.26893),
        "L_hip_yaw":    (-1.57080, 1.57080),
        "L_knee":       ( 0.00000, 2.70526),
    }

    def __init__(self, joints: list[JointConfig]):
        self.joints = {j.name: j for j in joints}
        self._buses: dict[str, RobstrideBus] = {}
        self._joint_to_bus: dict[str, str] = {}

        bus_motors: dict[str, dict[str, Motor]] = {}
        bus_calibration: dict[str, dict[str, dict]] = {}

        for j in joints:
            if j.can_bus not in bus_motors:
                bus_motors[j.can_bus] = {}
                bus_calibration[j.can_bus] = {}

            bus_motors[j.can_bus][j.name] = Motor(
                id=j.can_id,
                model=j.model.lower(),
            )
            bus_calibration[j.can_bus][j.name] = {
                "direction": j.direction,
                "homing_offset": j.offset,
            }
            self._joint_to_bus[j.name] = j.can_bus

        for channel, motors in bus_motors.items():
            self._buses[channel] = RobstrideBus(
                channel=channel,
                motors=motors,
                calibration=bus_calibration[channel],
            )

    @classmethod
    def from_robot_yaml(cls, config: dict, offsets: Optional[dict] = None) -> "BipedMotorManager":
        """Build manager from robot.yaml config + optional calibration offsets.

        Motor command-space limits are derived from calibration data
        (motor_min / motor_max).  When calibration is absent, URDF
        joint limits (normal) or theoretical linkage limits (ankle)
        are used as a fallback.
        """
        joints = []
        for bus_key, bus_cfg in config.items():
            if not isinstance(bus_cfg, dict) or "motors" not in bus_cfg:
                continue
            iface = bus_cfg.get("interface", bus_key)
            for name, mcfg in bus_cfg["motors"].items():
                is_ankle = name in ANKLE_ALL
                urdf = cls.JOINT_LIMITS.get(name, (-12.57, 12.57))

                offset = 0.0
                direction = 1
                motor_cmd_lo = -12.57
                motor_cmd_hi = 12.57

                if offsets and name in offsets:
                    cal = offsets[name]
                    m_min = cal.get("motor_min", 0.0)
                    m_max = cal.get("motor_max", 0.0)
                    direction = cal.get("direction", 1)

                    if direction == -1 and not is_ankle:
                        # Inverted encoder: pos = -(encoder - offset)
                        # offset = motor_max maps encoder_max → 0 (urdf_lower)
                        offset = m_max + urdf[0]
                    else:
                        offset = cal.get("offset", 0.0)

                    # Limits from calibration (motor command-space)
                    if direction == -1:
                        # pos = -(encoder - offset) = offset - encoder
                        motor_cmd_lo = -(m_max - offset)
                        motor_cmd_hi = -(m_min - offset)
                    else:
                        motor_cmd_lo = m_min - offset
                        motor_cmd_hi = m_max - offset
                else:
                    # No calibration — URDF / theoretical fallback
                    if is_ankle:
                        is_upper = name in ANKLE_TOP_MOTORS
                        pitch_sign = -1 if name.startswith("L") else 1
                        motor_cmd_lo, motor_cmd_hi = ankle_motor_theoretical_limits(is_upper, pitch_sign)
                    else:
                        motor_cmd_lo = urdf[0]
                        motor_cmd_hi = urdf[1]

                motor_softstop_lo = motor_cmd_lo + SOFTSTOP_BUFFER_RAD
                motor_softstop_hi = motor_cmd_hi - SOFTSTOP_BUFFER_RAD

                joints.append(JointConfig(
                    name=name,
                    can_bus=iface,
                    can_id=mcfg["id"],
                    model=f"rs-{mcfg['type'][2:].zfill(2)}",
                    offset=offset,
                    direction=direction,
                    is_ankle=is_ankle,
                    motor_cmd_lo=motor_cmd_lo,
                    motor_cmd_hi=motor_cmd_hi,
                    motor_softstop_lo=motor_softstop_lo,
                    motor_softstop_hi=motor_softstop_hi,
                ))
        return cls(joints)

    def _bus_for(self, joint_name: str) -> RobstrideBus:
        return self._buses[self._joint_to_bus[joint_name]]

    # ── Connection ──────────────────────────────────────────────────

    def connect_all(self):
        for bus in self._buses.values():
            if not bus.is_connected:
                bus.connect()

    def disconnect_all(self):
        for bus in self._buses.values():
            if bus.is_connected:
                bus.disconnect()

    def enable_all(self):
        for bus in self._buses.values():
            bus.enable_and_set_mit_all()

    def disable_all(self, clear_fault: bool = False):
        for bus in self._buses.values():
            bus.disable_all(clear_fault)

    def flush_all(self):
        for bus in self._buses.values():
            bus.flush_rx()

    # ── Motor-space soft stop ───────────────────────────────────────

    def compute_motor_softstop_torque(self, name: str, actual_pos: float) -> float:
        """Restoring torque in motor command-space when near calibration limits.

        Works for both normal joints (command-space = joint-space)
        and ankle motors (command-space = post-linkage motor-space).
        """
        j = self.joints[name]

        if actual_pos < j.motor_softstop_lo:
            penetration = j.motor_softstop_lo - actual_pos
            return SOFTSTOP_KP * min(penetration, SOFTSTOP_BUFFER_RAD)
        elif actual_pos > j.motor_softstop_hi:
            penetration = actual_pos - j.motor_softstop_hi
            return -SOFTSTOP_KP * min(penetration, SOFTSTOP_BUFFER_RAD)
        return 0.0

    # ── Motor commands ──────────────────────────────────────────────

    def send_mit_command(
        self,
        joint_name: str,
        position: float,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque_ff: float = 0.0,
        actual_pos: float = None,
    ) -> None:
        """Send MIT command for a NORMAL (non-ankle) joint.

        Clamps + soft-stop in motor command-space (= joint-space for
        normal joints).  Limits come from calibration.
        """
        j = self.joints[joint_name]

        # Hard clamp → soft-stop clamp
        position = max(j.motor_cmd_lo, min(j.motor_cmd_hi, position))
        position = max(j.motor_softstop_lo, min(j.motor_softstop_hi, position))

        if actual_pos is not None:
            torque_ff += self.compute_motor_softstop_torque(joint_name, actual_pos)

        bus = self._bus_for(joint_name)
        bus.write_operation_frame(joint_name, position, kp, kd, velocity, torque_ff)

    def send_ankle_mit_command(
        self,
        motor_name: str,
        motor_position: float,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque_ff: float = 0.0,
        actual_pos: float = None,
    ) -> None:
        """Send MIT command for an ankle motor (post-linkage motor-space).

        Clamps + soft-stop in motor command-space using calibration limits.
        """
        j = self.joints[motor_name]

        # Hard clamp → soft-stop clamp
        motor_position = max(j.motor_cmd_lo, min(j.motor_cmd_hi, motor_position))
        motor_position = max(j.motor_softstop_lo, min(j.motor_softstop_hi, motor_position))

        if actual_pos is not None:
            torque_ff += self.compute_motor_softstop_torque(motor_name, actual_pos)

        bus = self._bus_for(motor_name)
        bus.write_operation_frame(motor_name, motor_position, kp, kd, velocity, torque_ff)

    def read_feedback(
        self, joint_name: str, timeout: float = 0.005
    ) -> Optional[MotorFeedback]:
        bus = self._bus_for(joint_name)
        return bus.read_operation_frame(joint_name, timeout)

    # ── Ankle helpers ───────────────────────────────────────────────

    def is_ankle_top(self, name: str) -> bool:
        return name in ANKLE_PAIRS

    def is_ankle_bottom(self, name: str) -> bool:
        return name in ANKLE_PAIRS.values()

    def get_ankle_pair(self, name: str) -> Optional[tuple[str, str]]:
        if name in ANKLE_PAIRS:
            return (name, ANKLE_PAIRS[name])
        for t, b in ANKLE_PAIRS.items():
            if name == b:
                return (t, b)
        return None


__all__ = [
    "RobstrideBus", "Motor", "MotorFeedback",
    "BipedMotorManager", "JointConfig",
    "ankle_command_to_motors", "ankle_motors_to_feedback",
    "ankle_motor_theoretical_limits",
    "ANKLE_PAIRS", "ANKLE_ALL", "ANKLE_MOTOR_TO_JOINT",
    "_PITCH_GAIN", "_ROLL_GAIN", "_INV_PITCH", "_INV_ROLL",
    "SOFTSTOP_BUFFER_RAD", "SOFTSTOP_KP",
]
