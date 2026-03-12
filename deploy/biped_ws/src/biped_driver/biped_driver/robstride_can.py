"""Biped-specific adapter around the RobStride shared library.

Adds:
  - Ankle parallel linkage transform (2× RS02 → foot pitch/roll)
  - Multi-bus manager (can0 + can1)
  - Convenience config loader from robot.yaml format

Upstream library: biped_driver.robstride_dynamics (vendored from Seeed)
"""

import math
from dataclasses import dataclass, field
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
# Linearized mapping (<9% error at max pitch, <1% at walking range):

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


# Ankle pairs: (pitch_joint=upper_motor, roll_joint=lower_motor)
ANKLE_PAIRS = {
    "L_foot_pitch": "L_foot_roll",
    "R_foot_pitch": "R_foot_roll",
}


# ── Multi-bus manager ───────────────────────────────────────────────

# 2° safety buffer in radians
SOFTSTOP_BUFFER_RAD = math.radians(2.0)  # 0.0349 rad

# Soft stop spring gain: Nm/rad of penetration into buffer zone.
# Produces restoring torque that ramps linearly as joint approaches hard limit.
# At full 2° penetration: 20 Nm/rad × 0.035 rad ≈ 0.7 Nm restoring torque.
SOFTSTOP_KP = 20.0


@dataclass
class JointConfig:
    """Per-joint configuration."""
    name: str
    can_bus: str          # "can0" or "can1"
    can_id: int
    model: str            # "rs-02", "rs-03", "rs-04"
    offset: float = 0.0   # calibration offset (rad)
    limit_lo: float = -12.57   # hard limit (from URDF or calibration)
    limit_hi: float = 12.57    # hard limit
    softstop_lo: float = -12.57  # soft stop = hard limit + buffer (inward)
    softstop_hi: float = 12.57   # soft stop = hard limit - buffer (inward)


class BipedMotorManager:
    """Manages all motors across both CAN buses via RobstrideBus instances.

    Provides a joint-level API that transparently handles:
    - Routing commands to the correct CAN bus
    - Ankle parallel linkage transforms
    - Calibration offsets
    """

    # URDF joint limits (rad)
    JOINT_LIMITS = {
        "L_hip_pitch":  (-2.222, 1.047),
        "R_hip_pitch":  (-1.047, 2.222),
        "L_hip_roll":   (-0.209, 2.269),
        "R_hip_roll":   (-2.269, 0.209),
        "L_hip_yaw":    (-2.094, 2.094),
        "R_hip_yaw":    (-2.094, 2.094),
        "L_knee":       ( 0.000, 2.618),
        "R_knee":       ( 0.000, 2.618),
        "L_foot_pitch": (-1.047, 0.524),
        "R_foot_pitch": (-1.047, 0.524),
        "L_foot_roll":  (-0.262, 0.262),
        "R_foot_roll":  (-0.262, 0.262),
    }

    def __init__(self, joints: list[JointConfig]):
        self.joints = {j.name: j for j in joints}
        self._buses: dict[str, RobstrideBus] = {}
        self._joint_to_bus: dict[str, str] = {}

        # Group joints by CAN interface and build bus objects
        bus_motors: dict[str, dict[str, Motor]] = {}
        bus_calibration: dict[str, dict[str, dict]] = {}

        for j in joints:
            if j.can_bus not in bus_motors:
                bus_motors[j.can_bus] = {}
                bus_calibration[j.can_bus] = {}

            bus_motors[j.can_bus][j.name] = Motor(
                id=j.can_id,
                model=j.model.lower(),  # table keys are lowercase
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
            )

    @classmethod
    def from_robot_yaml(cls, config: dict, offsets: Optional[dict] = None) -> "BipedMotorManager":
        """Build from robot.yaml config dict.

        Expected format:
            can0:
              interface: can0
              motors:
                R_hip_pitch: {id: 1, type: RS04}
                ...
            can1:
              ...
        """
        joints = []
        for bus_key, bus_cfg in config.items():
            if not isinstance(bus_cfg, dict) or "motors" not in bus_cfg:
                continue  # skip non-bus keys (comments, metadata, etc.)
            iface = bus_cfg.get("interface", bus_key)
            for name, mcfg in bus_cfg["motors"].items():
                offset = 0.0
                cal_lo = None
                cal_hi = None
                if offsets and name in offsets:
                    cal = offsets[name]
                    offset = cal.get("offset", 0.0)
                    # Calibration provides measured mechanical limits
                    # (already in joint-space after offset correction)
                    if "limit_lo" in cal and "limit_hi" in cal:
                        cal_lo = cal["limit_lo"]
                        cal_hi = cal["limit_hi"]
                    elif "encoder_limit_a" in cal and "encoder_limit_b" in cal:
                        a = cal["encoder_limit_a"] - offset
                        b = cal["encoder_limit_b"] - offset
                        cal_lo = min(a, b)
                        cal_hi = max(a, b)

                # Use calibration limits if available, else URDF defaults
                urdf = cls.JOINT_LIMITS.get(name, (-12.57, 12.57))
                limit_lo = cal_lo if cal_lo is not None else urdf[0]
                limit_hi = cal_hi if cal_hi is not None else urdf[1]

                # Soft stops: inset by safety buffer from hard limits
                softstop_lo = limit_lo + SOFTSTOP_BUFFER_RAD
                softstop_hi = limit_hi - SOFTSTOP_BUFFER_RAD

                joints.append(JointConfig(
                    name=name,
                    can_bus=iface,
                    can_id=mcfg["id"],
                    model=f"rs-{mcfg['type'][2:].zfill(2)}",
                    offset=offset,
                    limit_lo=limit_lo,
                    limit_hi=limit_hi,
                    softstop_lo=softstop_lo,
                    softstop_hi=softstop_hi,
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
            bus.enable_all()
            bus.set_mode_all(0)  # MIT mode
            bus.flush_rx()

    def disable_all(self, clear_fault: bool = False):
        for bus in self._buses.values():
            bus.disable_all(clear_fault)

    def flush_all(self):
        for bus in self._buses.values():
            bus.flush_rx()

    # ── Joint-level API ─────────────────────────────────────────────

    def clamp(self, name: str, position: float) -> float:
        """Hard-clamp position to joint limits."""
        j = self.joints[name]
        return max(j.limit_lo, min(j.limit_hi, position))

    def compute_softstop_torque(self, name: str, actual_pos: float) -> float:
        """Compute restoring torque if joint is in the soft stop buffer zone.

        Returns 0.0 if within safe range. Returns a torque (Nm) pushing
        the joint away from the limit if within the 2° buffer zone.

        The torque ramps linearly from 0 at the soft stop boundary to
        SOFTSTOP_KP × SOFTSTOP_BUFFER_RAD at the hard limit.
        """
        j = self.joints[name]

        if actual_pos < j.softstop_lo:
            # Below low soft stop — push positive (away from limit)
            penetration = j.softstop_lo - actual_pos
            return SOFTSTOP_KP * min(penetration, SOFTSTOP_BUFFER_RAD)
        elif actual_pos > j.softstop_hi:
            # Above high soft stop — push negative (away from limit)
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
        """Send one MIT command with soft stop protection.

        If actual_pos is provided and the joint is in the soft stop zone:
        1. Position command is clamped to the soft stop boundary (not the hard limit)
        2. A restoring torque is added to torque_ff pushing the joint inward

        Hard clamp at mechanical limits is always applied regardless.
        """
        j = self.joints[joint_name]

        # Hard clamp — never command beyond mechanical limits
        position = max(j.limit_lo, min(j.limit_hi, position))

        # Soft stop: clamp command to soft stop zone + add restoring torque
        position = max(j.softstop_lo, min(j.softstop_hi, position))

        if actual_pos is not None:
            torque_ff += self.compute_softstop_torque(joint_name, actual_pos)

        bus = self._bus_for(joint_name)
        bus.write_operation_frame(joint_name, position, kp, kd, velocity, torque_ff)

    def read_feedback(
        self, joint_name: str, timeout: float = 0.005
    ) -> Optional[MotorFeedback]:
        """Read one feedback frame from the motor."""
        bus = self._bus_for(joint_name)
        return bus.read_operation_frame(joint_name, timeout)

    def is_ankle_pitch(self, name: str) -> bool:
        return name in ANKLE_PAIRS

    def is_ankle_roll(self, name: str) -> bool:
        return name in ANKLE_PAIRS.values()

    def get_ankle_pair(self, name: str) -> Optional[tuple[str, str]]:
        """If name is part of an ankle pair, return (pitch_name, roll_name)."""
        if name in ANKLE_PAIRS:
            return (name, ANKLE_PAIRS[name])
        for p, r in ANKLE_PAIRS.items():
            if name == r:
                return (p, r)
        return None


# Re-export for convenience
__all__ = [
    "RobstrideBus",
    "Motor",
    "MotorFeedback",
    "BipedMotorManager",
    "JointConfig",
    "ankle_command_to_motors",
    "ankle_motors_to_feedback",
    "ANKLE_PAIRS",
]
