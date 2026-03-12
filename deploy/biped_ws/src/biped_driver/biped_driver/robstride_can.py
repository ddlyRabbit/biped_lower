"""Biped-specific adapter around the RobStride shared library.

Adds:
  - Ankle parallel linkage transform (2× RS02 → foot pitch/roll)
  - Multi-bus manager (can0 + can1)
  - Convenience config loader from robot.yaml format

Upstream library: biped_driver.robstride_dynamics (vendored from Seeed)
"""

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

@dataclass
class JointConfig:
    """Per-joint configuration."""
    name: str
    can_bus: str          # "can0" or "can1"
    can_id: int
    model: str            # "rs-02", "rs-03", "rs-04"
    offset: float = 0.0   # calibration offset (rad)
    limit_lo: float = -12.57
    limit_hi: float = 12.57


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
        for bus_key in config:
            bus_cfg = config[bus_key]
            iface = bus_cfg.get("interface", bus_key)
            for name, mcfg in bus_cfg.get("motors", {}).items():
                offset = 0.0
                if offsets and name in offsets:
                    offset = offsets[name].get("offset", 0.0)
                limits = cls.JOINT_LIMITS.get(name, (-12.57, 12.57))
                joints.append(JointConfig(
                    name=name,
                    can_bus=iface,
                    can_id=mcfg["id"],
                    model=f"rs-{mcfg['type'][2:].zfill(2)}",
                    offset=offset,
                    limit_lo=limits[0],
                    limit_hi=limits[1],
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
        j = self.joints[name]
        return max(j.limit_lo, min(j.limit_hi, position))

    def send_mit_command(
        self,
        joint_name: str,
        position: float,
        kp: float,
        kd: float,
        velocity: float = 0.0,
        torque_ff: float = 0.0,
    ) -> None:
        """Send one MIT command (handles routing to correct bus)."""
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
