"""
robstride_mit.py — faithful Python port of the ROBSTRIDE MIT-mode CAN codec
from ``rust/motor_control/src/robstride.rs`` (the deployed driver, RS03-EN spec).

WHY NOT REUSE ``tests/direct_can/sine_wave_test.py``:
That script uses the T-Motor / AK "MIT-Cheetah" 8-byte packing
(pos16 | vel12 | kp12 | kd12 | tau12, torque in the DATA) with fixed ±18 Nm /
±30 rad·s⁻¹ scaling. ROBSTRIDE's native control frame is different:
    * DATA[0:8] = four big-endian u16: position | velocity | kp | kd
    * TORQUE u16 is carried in the ARBITRATION ID (bits 8..23)
    * scaling is PER-MODEL (RS02 ≠ RS03 ≠ RS04 ≠ RS00)
Using the AK packing / fixed scaling mis-drives RS03/RS04 (a 120 Nm shoulder
clips at ±18 Nm) and garbles telemetry. This module reproduces the Rust codec
exactly so the characterization numbers are trustworthy.

Per-model MIT ranges (source of truth: robstride.rs MotorModel::mit_* methods):
    model  vel_max(rad/s)  torque_max(Nm)  kp_max   kd_max
    RS04   15              120             5000     100
    RS03   50              60              5000     100
    RS02   44              17              500      5
    RS00   30              2  (!)          100      5
Position range is ±4π for every model. The RS00 ±2 Nm torque scale is far below
its ~14 Nm spec peak — small-joint (gripper/wrist-roll/head) torque telemetry
may saturate; VERIFY against the Robstride manual (see harness Stage 1e).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

HOST_CAN_ID = 0xAA
POS_MAX = 4.0 * math.pi          # ±4π rad, all models
_EXT_ID_MASK = 0x1FFFFFFF        # 29-bit extended CAN id


class MotorMsg(IntEnum):
    INFO = 0
    CONTROL = 1
    FEEDBACK = 2
    ENABLE = 3
    DISABLE = 4
    ZERO_POS = 6
    SET_ID = 7
    READ_PARAM = 17
    WRITE_PARAM = 18
    SAVE_CONFIG = 22


class MotorMode(IntEnum):
    RESET = 0
    CALIBRATION = 1
    RUN = 2
    UNKNOWN = 3


# Robstride param IDs (from robstride.rs::params) — handy for reading MECHPOS,
# MECHVEL, VBUS, temperature, etc. during characterization.
class Param(IntEnum):
    RUN_MODE = 0x7005
    IQ_REF = 0x7006
    SPD_REF = 0x700A
    LIMIT_TORQUE = 0x700B
    LOC_REF = 0x7016
    LIMIT_SPD = 0x7017
    LIMIT_CUR = 0x7018
    MECHPOS = 0x7019        # motor-side (PRE-gearbox) mechanical position
    IQF = 0x701A            # filtered iq (current) — proxy for the Kt fit
    MECHVEL = 0x701B
    VBUS = 0x701C           # bus voltage (affects torque/speed headroom)


@dataclass(frozen=True)
class ModelScale:
    name: str
    vel_max: float      # rad/s
    torque_max: float   # Nm
    kp_max: float
    kd_max: float


MODELS: dict[str, ModelScale] = {
    "RS04": ModelScale("RS04", 15.0, 120.0, 5000.0, 100.0),
    "RS03": ModelScale("RS03", 50.0, 60.0, 5000.0, 100.0),
    "RS02": ModelScale("RS02", 44.0, 17.0, 500.0, 5.0),
    "RS00": ModelScale("RS00", 30.0, 2.0, 100.0, 5.0),
}


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _enc_signed(value: float, vmax: float) -> int:
    """Symmetric ±vmax float -> u16 with 0x7FFF as the midpoint (Rust convention)."""
    u = (_clamp(value, -vmax, vmax) / vmax + 1.0) * float(0x7FFF)
    return int(_clamp(u, 0.0, float(0xFFFF)))


def _enc_unsigned(value: float, vmax: float) -> int:
    u = _clamp(value, 0.0, vmax) / vmax * float(0xFFFF)
    return int(_clamp(u, 0.0, float(0xFFFF)))


def build_control(model: str, motor_id: int, pos: float, vel: float,
                  kp: float, kd: float, torque: float) -> tuple[int, bytes]:
    """MIT control frame -> (extended arbitration_id, 8 data bytes).
    Mirrors robstride.rs::build_control_frame exactly."""
    m = MODELS[model]
    pos_u = _enc_signed(pos, POS_MAX)
    vel_u = _enc_signed(vel, m.vel_max)
    kp_u = _enc_unsigned(kp, m.kp_max)
    kd_u = _enc_unsigned(kd, m.kd_max)
    tau_u = _enc_signed(torque, m.torque_max)
    arb = ((int(MotorMsg.CONTROL) << 24) | (tau_u << 8) | (motor_id & 0xFF)) & _EXT_ID_MASK
    data = bytes((
        (pos_u >> 8) & 0xFF, pos_u & 0xFF,
        (vel_u >> 8) & 0xFF, vel_u & 0xFF,
        (kp_u >> 8) & 0xFF, kp_u & 0xFF,
        (kd_u >> 8) & 0xFF, kd_u & 0xFF,
    ))
    return arb, data


def build_simple(motor_id: int, msg: MotorMsg, data: bytes = b"\x00" * 8) -> tuple[int, bytes]:
    """Enable / Disable / ZeroPos / SaveConfig — host id in bits 8..15
    (robstride.rs::build_arb_id)."""
    arb = ((int(msg) << 24) | (HOST_CAN_ID << 8) | (motor_id & 0xFF)) & _EXT_ID_MASK
    data = (bytes(data) + b"\x00" * 8)[:8]
    return arb, data


def build_enable(motor_id: int):       return build_simple(motor_id, MotorMsg.ENABLE)
def build_disable(motor_id: int):      return build_simple(motor_id, MotorMsg.DISABLE)
def build_zero_pos(motor_id: int):     return build_simple(motor_id, MotorMsg.ZERO_POS, bytes((1, 0, 0, 0, 0, 0, 0, 0)))
def build_save_config(motor_id: int):  return build_simple(motor_id, MotorMsg.SAVE_CONFIG)


def build_read_param(motor_id: int, param: int) -> tuple[int, bytes]:
    arb = ((int(MotorMsg.READ_PARAM) << 24) | (HOST_CAN_ID << 8) | (motor_id & 0xFF)) & _EXT_ID_MASK
    data = bytes((param & 0xFF, (param >> 8) & 0xFF, 0, 0, 0, 0, 0, 0))  # param id little-endian
    return arb, data


@dataclass
class Feedback:
    motor_id: int
    mode: MotorMode
    position: float     # rad, MOTOR-SIDE (pre-gearbox) — do not trust as output angle
    velocity: float     # rad/s
    torque: float       # Nm (from current * Kt, i.e. what we validate externally)
    temperature: float  # °C
    error_bits: int


def is_feedback_frame(arb_id: int) -> bool:
    return ((arb_id >> 24) & 0x1F) == int(MotorMsg.FEEDBACK)


def decode_feedback(arb_id: int, data: bytes, model: str) -> Feedback:
    """Type-2 feedback frame -> Feedback. Mirrors robstride.rs::parse_feedback."""
    if len(data) != 8:
        raise ValueError(f"feedback needs 8 data bytes, got {len(data)}")
    m = MODELS[model]
    motor_id = (arb_id >> 8) & 0xFF
    error_bits = (arb_id >> 16) & 0x1F
    mode = MotorMode((arb_id >> 22) & 0x03)
    angle_raw = (data[0] << 8) | data[1]
    position = angle_raw / 65535.0 * (8.0 * math.pi) - (4.0 * math.pi)
    vel_raw = (data[2] << 8) | data[3]
    velocity = vel_raw / 65535.0 * (2.0 * m.vel_max) - m.vel_max
    tau_raw = (data[4] << 8) | data[5]
    torque = tau_raw / 65535.0 * (2.0 * m.torque_max) - m.torque_max
    temp_raw = (data[6] << 8) | data[7]
    temperature = temp_raw / 10.0
    return Feedback(motor_id, mode, position, velocity, torque, temperature, error_bits)
