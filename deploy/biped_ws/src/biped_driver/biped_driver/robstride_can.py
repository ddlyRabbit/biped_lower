"""Biped-specific adapter around the RobStride shared library.

Adds:
  - Ankle parallel linkage transform (2× RS02 → foot pitch/roll)
  - Multi-bus manager (can0 + can1)
  - Soft stop joint protection (2° buffer with restoring torque)
  - Convenience config loader from robot.yaml format

Upstream library: biped_driver.robstride_dynamics (vendored from Seeed)
"""

import math
from dataclasses import dataclass
from typing import Optional

from .robstride_dynamics import RobstrideBus, Motor, MotorFeedback


# ── Ankle parallel linkage ──────────────────────────────────────────
#
# Two RS02 motors per ankle, asymmetric rod lengths, U-joint to foot.
# From Onshape CAD:
#   Crank radius:     32.249 mm
#   Upper rod:        192 mm (knee-side motor = foot_pitch CAN ID)
#   Lower rod:        98 mm  (foot-side motor = foot_roll CAN ID)
#   Foot attachment:  ±31.398 mm lateral, 41.14 mm forward from U-joint
#
# Forward (joint → motors):
#   motor_upper = pitch_gain × pitch + roll_gain × roll
#   motor_lower = pitch_gain × pitch - roll_gain × roll
#
# Inverse (motors → joint):
#   pitch = inv_pitch × (upper + lower)
#   roll  = inv_roll  × (upper - lower)

_PITCH_GAIN = 41.14 / 32.249    # 1.2757 — motor rad per foot pitch rad
_ROLL_GAIN  = 31.398 / 32.249   # 0.9736 — motor rad per foot roll rad
_INV_PITCH  = 32.249 / (2 * 41.14)   # 0.3919
_INV_ROLL   = 32.249 / (2 * 31.398)  # 0.5136


def ankle_command_to_motors(pitch_cmd: float, roll_cmd: float) -> tuple[float, float]:
    """Convert foot pitch/roll targets → (upper_motor, lower_motor) positions."""
    motor_upper = _PITCH_GAIN * pitch_cmd + _ROLL_GAIN * roll_cmd
    motor_lower = _PITCH_GAIN * pitch_cmd - _ROLL_GAIN * roll_cmd
    return motor_upper, motor_lower


def ankle_motors_to_feedback(
    upper_pos: float, lower_pos: float
) -> tuple[float, float]:
    """Convert (upper, lower) motor positions → (foot_pitch, foot_roll)."""
    foot_pitch = _INV_PITCH * (upper_pos + lower_pos)
    foot_roll  = _INV_ROLL  * (upper_pos - lower_pos)
    return foot_pitch, foot_roll


def ankle_motor_cmd_at_joint_min(is_upper: bool) -> float:
    """Motor command value when the ankle joint is at its most negative position.

    Used to compute the correct bus offset: offset = encoder_min - motor_cmd_at_min.
    This ensures that motor_cmd=0 maps to encoder_at_joint_zero.

    For upper motor: motor = PG × pitch_min + RG × roll_min
    For lower motor: motor = PG × pitch_min - RG × roll_max  (note: -RG × roll_max is more negative)
    """
    pitch_min = -0.87267  # from URDF foot_pitch lower limit
    roll_min = -0.26180
    roll_max = 0.26180
    if is_upper:
        return _PITCH_GAIN * pitch_min + _ROLL_GAIN * roll_min
    else:
        return _PITCH_GAIN * pitch_min - _ROLL_GAIN * roll_max


# Ankle pairs: pitch_joint → roll_joint (upper_motor → lower_motor)
ANKLE_PAIRS = {
    "L_foot_pitch": "L_foot_roll",
    "R_foot_pitch": "R_foot_roll",
}

ANKLE_PITCH_MOTORS = set(ANKLE_PAIRS.keys())
ANKLE_ROLL_MOTORS = set(ANKLE_PAIRS.values())
ANKLE_ALL = ANKLE_PITCH_MOTORS | ANKLE_ROLL_MOTORS


# ── Soft stop config ────────────────────────────────────────────────

SOFTSTOP_BUFFER_RAD = math.radians(2.0)  # 0.0349 rad
SOFTSTOP_KP = 20.0  # Nm/rad restoring spring


# ── Multi-bus manager ───────────────────────────────────────────────

@dataclass
class JointConfig:
    """Per-joint configuration."""
    name: str
    can_bus: str          # "can0" or "can1" or "/dev/ttyUSB0"
    can_id: int
    model: str            # "rs-02", "rs-03", "rs-04"
    offset: float = 0.0   # bus homing_offset (motor-space)
    is_ankle: bool = False
    # Joint-space limits (for clamping + soft stops)
    limit_lo: float = -12.57
    limit_hi: float = 12.57
    softstop_lo: float = -12.57
    softstop_hi: float = 12.57
    # Motor-space command limits (for ankle motors, post-linkage clamping)
    # These are in command-space: physical_encoder = command + offset
    motor_cmd_lo: float = -12.57
    motor_cmd_hi: float = 12.57


class BipedMotorManager:
    """Manages all motors across both CAN buses via RobstrideBus instances.

    For normal joints: limits, soft stops, and commands all operate in joint-space.
    For ankle joints: limits and soft stops operate in joint-space (before linkage).
      The linkage forward transform converts to motor-space for the CAN command.
      The bus offset maps motor-command-space to raw encoder values.
    """

    # From URDF: urdf/heavy/robot.urdf joint limits (rad)
    JOINT_LIMITS = {
        "R_hip_pitch":  (-2.21657, 1.04720),
        "R_hip_roll":   (-2.26893, 0.20944),
        "R_hip_yaw":    (-1.57080, 1.57080),
        "R_knee":       ( 0.00000, 2.70526),
        "R_foot_pitch": (-0.87267, 0.52360),
        "R_foot_roll":  (-0.26180, 0.26180),
        "L_hip_pitch":  (-1.04720, 2.21657),
        "L_hip_roll":   (-0.20944, 2.26893),
        "L_hip_yaw":    (-1.57080, 1.57080),
        "L_knee":       ( 0.00000, 2.70526),
        "L_foot_pitch": (-0.87267, 0.52360),
        "L_foot_roll":  (-0.26180, 0.26180),
    }

    def __init__(self, joints: list[JointConfig], backend: str = "socketcan"):
        self.joints = {j.name: j for j in joints}
        self.backend = backend
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
                "direction": 1,
                "homing_offset": j.offset,
            }
            self._joint_to_bus[j.name] = j.can_bus

        for channel, motors in bus_motors.items():
            self._buses[channel] = RobstrideBus(
                channel=channel,
                motors=motors,
                calibration=bus_calibration[channel],
                backend=backend,
            )

    @classmethod
    def from_robot_yaml(cls, config: dict, offsets: Optional[dict] = None, backend: str = "socketcan") -> "BipedMotorManager":
        joints = []
        for bus_key, bus_cfg in config.items():
            if not isinstance(bus_cfg, dict) or "motors" not in bus_cfg:
                continue
            iface = bus_cfg.get("interface", bus_key)
            for name, mcfg in bus_cfg["motors"].items():
                is_ankle = name in ANKLE_ALL
                urdf = cls.JOINT_LIMITS.get(name, (-12.57, 12.57))

                # Always use URDF joint-space limits for clamping/soft stops
                limit_lo = urdf[0]
                limit_hi = urdf[1]

                # Compute bus offset from calibration
                offset = 0.0
                if offsets and name in offsets:
                    cal = offsets[name]
                    if is_ankle:
                        # Ankle offset: encoder_at_joint_zero
                        # = encoder_min - motor_cmd_at_joint_min
                        motor_min = cal.get("motor_min", 0.0)
                        is_upper = name in ANKLE_PITCH_MOTORS
                        cmd_at_min = ankle_motor_cmd_at_joint_min(is_upper)
                        offset = motor_min - cmd_at_min
                    else:
                        offset = cal.get("offset", 0.0)

                softstop_lo = limit_lo + SOFTSTOP_BUFFER_RAD
                softstop_hi = limit_hi - SOFTSTOP_BUFFER_RAD

                # Motor command-space limits for ankle motors
                # command + offset = physical encoder → command = physical - offset
                motor_cmd_lo = -12.57
                motor_cmd_hi = 12.57
                if is_ankle and offsets and name in offsets:
                    cal = offsets[name]
                    m_min = cal.get("motor_min", -12.57)
                    m_max = cal.get("motor_max", 12.57)
                    # Add safety buffer in motor-space too (2°)
                    motor_cmd_lo = (m_min + SOFTSTOP_BUFFER_RAD) - offset
                    motor_cmd_hi = (m_max - SOFTSTOP_BUFFER_RAD) - offset

                joints.append(JointConfig(
                    name=name,
                    can_bus=iface,
                    can_id=mcfg["id"],
                    model=f"rs-{mcfg['type'][2:].zfill(2)}",
                    offset=offset,
                    is_ankle=is_ankle,
                    limit_lo=limit_lo,
                    limit_hi=limit_hi,
                    softstop_lo=softstop_lo,
                    softstop_hi=softstop_hi,
                    motor_cmd_lo=motor_cmd_lo,
                    motor_cmd_hi=motor_cmd_hi,
                ))
        return cls(joints, backend=backend)

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
            bus.enable_all()
            bus.set_mode_all(0)
            bus.flush_rx()

    def disable_all(self, clear_fault: bool = False):
        for bus in self._buses.values():
            bus.disable_all(clear_fault)

    def flush_all(self):
        for bus in self._buses.values():
            bus.flush_rx()

    # ── Joint-level API ─────────────────────────────────────────────

    def clamp_joint(self, name: str, position: float) -> float:
        """Hard-clamp position to URDF joint-space limits."""
        j = self.joints[name]
        return max(j.limit_lo, min(j.limit_hi, position))

    # Keep old name for compatibility
    def clamp(self, name: str, position: float) -> float:
        return self.clamp_joint(name, position)

    def softstop_clamp(self, name: str, position: float) -> float:
        """Clamp position to soft stop boundaries (inside hard limits by 2°)."""
        j = self.joints[name]
        return max(j.softstop_lo, min(j.softstop_hi, position))

    def compute_softstop_torque(self, name: str, actual_pos: float) -> float:
        """Compute restoring torque in joint-space when near limits.

        For ankle joints: actual_pos must be in joint-space (after linkage inverse).
        For normal joints: actual_pos is the motor feedback position (= joint-space).
        """
        j = self.joints[name]

        if actual_pos < j.softstop_lo:
            penetration = j.softstop_lo - actual_pos
            return SOFTSTOP_KP * min(penetration, SOFTSTOP_BUFFER_RAD)
        elif actual_pos > j.softstop_hi:
            penetration = actual_pos - j.softstop_hi
            return -SOFTSTOP_KP * min(penetration, SOFTSTOP_BUFFER_RAD)
        return 0.0

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

        Applies joint-space soft stops + restoring torque.
        For ankle joints, use send_ankle_mit_command() instead.
        """
        j = self.joints[joint_name]

        # Hard clamp + soft stop clamp (joint-space)
        position = max(j.limit_lo, min(j.limit_hi, position))
        position = max(j.softstop_lo, min(j.softstop_hi, position))

        if actual_pos is not None:
            torque_ff += self.compute_softstop_torque(joint_name, actual_pos)

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
    ) -> None:
        """Send MIT command for an ankle motor (motor-space, post-linkage).

        Joint-space soft stops are applied by the caller (can_bus_node._handle_ankle)
        before the linkage forward transform. Here we enforce motor-space hard limits
        from calibration to prevent the motor exceeding its physical range.
        """
        j = self.joints[motor_name]
        motor_position = max(j.motor_cmd_lo, min(j.motor_cmd_hi, motor_position))

        bus = self._bus_for(motor_name)
        bus.write_operation_frame(motor_name, motor_position, kp, kd, velocity, torque_ff)

    def read_feedback(
        self, joint_name: str, timeout: float = 0.005
    ) -> Optional[MotorFeedback]:
        bus = self._bus_for(joint_name)
        return bus.read_operation_frame(joint_name, timeout)

    def is_ankle_pitch(self, name: str) -> bool:
        return name in ANKLE_PAIRS

    def is_ankle_roll(self, name: str) -> bool:
        return name in ANKLE_PAIRS.values()

    def get_ankle_pair(self, name: str) -> Optional[tuple[str, str]]:
        if name in ANKLE_PAIRS:
            return (name, ANKLE_PAIRS[name])
        for p, r in ANKLE_PAIRS.items():
            if name == r:
                return (p, r)
        return None


__all__ = [
    "RobstrideBus", "Motor", "MotorFeedback",
    "BipedMotorManager", "JointConfig",
    "ankle_command_to_motors", "ankle_motors_to_feedback",
    "ankle_motor_cmd_at_joint_min",
    "ANKLE_PAIRS", "ANKLE_ALL",
    "_PITCH_GAIN", "_ROLL_GAIN",
    "SOFTSTOP_BUFFER_RAD", "SOFTSTOP_KP",
]
