"""so101_leader.py — SO-101 leader arm controller module.

Minimal Feetech STS3215 serial reader — no SDK dependency beyond pyserial.

Controller protocol (duck-typed, works anywhere in the teleop system):

    arm = So101Leader('/dev/ttyACM0')
    arm.connect()
    while True:
        targets = arm.poll()   # Optional[np.ndarray]  7 AIZEE joint targets [rad]
    arm.close()

Install dependency:  pip install pyserial
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import serial
    import serial.serialutil
except ImportError:
    serial = None  # type: ignore

# ---------------------------------------------------------------------------
# Feetech STS3215 protocol constants
# ---------------------------------------------------------------------------
_BAUD      = 1_000_000
_HEADER    = b"\xFF\xFF"
_READ      = 0x02
_REG_POS   = 0x38   # Present_Position — 2 bytes, little-endian
_CENTER    = 2048   # raw ticks at mechanical center
_TICKS     = 4096   # ticks per full revolution (12-bit encoder)

CALIB_PATH = Path("config/so101_calibration.json")

# Default AIZEE arm target range [rad_min, rad_max] per SO-101 joint.
# Order matches JOINTS below.  Edit via the calibration JSON after recording.
AIZEE_DEFAULTS: list[tuple[float, float]] = [
    (-1.57,  1.57),   # swivel       ← shoulder_pan
    (-1.57,  1.57),   # gantry_base  ← shoulder_lift
    (-1.57,  0.50),   # gantry_mid   ← elbow_flex
    (-0.50,  1.57),   # gantry_end   ← wrist_flex
    (-1.00,  1.00),   # wrist_pitch  ← wrist_yaw (SO-101 servo 5)
    (-1.57,  1.57),   # wrist_roll   ← wrist_roll
    ( 0.00,  0.50),   # gripper      ← gripper  (0=open, 0.5=closed)
]


def ticks_to_rad(ticks: int) -> float:
    """Convert raw encoder ticks to radians relative to center (0 = neutral)."""
    return (ticks - _CENTER) * (2.0 * math.pi / _TICKS)


# ---------------------------------------------------------------------------
# So101Leader
# ---------------------------------------------------------------------------

class So101Leader:
    """SO-101 7-DOF leader arm.  Reads STS3215 servo positions over USB serial
    and converts them to AIZEE arm joint targets using a calibration file.

    Servo IDs 1-7 map to joints in order:
        1 shoulder_pan  → swivel      (rover base)
        2 shoulder_lift → gantry_base
        3 elbow_flex    → gantry_mid
        4 wrist_flex    → gantry_end
        5 wrist_yaw     → wrist_pitch
        6 wrist_roll    → wrist_roll
        7 gripper       → gripper
    """

    JOINTS = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_yaw",
        "wrist_roll",
        "gripper",
    ]
    AIZEE_JOINTS = [
        "swivel",
        "gantry_base",
        "gantry_mid",
        "gantry_end",
        "wrist_pitch",
        "wrist_roll",
        "gripper",
    ]

    def __init__(
        self,
        port: str,
        baud: int = _BAUD,
        calib: Path | str = CALIB_PATH,
    ) -> None:
        if serial is None:
            raise ImportError("pyserial not installed — run: pip install pyserial")
        self.port   = port
        self.baud   = baud
        self._ser: Optional[serial.Serial] = None
        self._calib_path = Path(calib)
        self._calib = _load_calib(self._calib_path)
        # Per-joint unwrap state — tracks rollovers across 0/4095 boundary
        self._prev_raw:   dict[str, Optional[int]] = {j: None for j in self.JOINTS}
        self._unwrap_off: dict[str, int]           = {j: 0    for j in self.JOINTS}
        # Set by poll(): True if that joint's raw frac was outside [0,1] (clamped)
        self._clamped:    list[bool]               = [False] * len(self.JOINTS)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open serial port.  Returns True on success."""
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(0.1)
            # Reset unwrap state so seeding runs again on first read.
            for j in self.JOINTS:
                self._prev_raw[j]   = None
                self._unwrap_off[j] = 0
            self._clamped = [False] * len(self.JOINTS)
            return True
        except serial.SerialException as exc:
            print(f"[SO-101] connect failed on {self.port}: {exc}")
            return False

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    @property
    def clamped_joints(self) -> list[bool]:
        """Per-joint clamp flag from the last poll().

        True means the joint's encoder position was outside its calibrated
        [min_raw, max_raw] range and the output was clamped to rad_min/rad_max.
        This usually means the SO-101 servo is out of its calibrated window.
        """
        return list(self._clamped)

    # ------------------------------------------------------------------
    # Encoder unwrapping
    # ------------------------------------------------------------------

    def _unwrap(self, joint: str, raw: int) -> int:
        """Return a continuously unwrapped position for *joint*.

        On the very first reading after connect(), seeds _unwrap_off so that
        the initial physical tick maps correctly into the calibrated range:
        - Wrapped ranges (min_raw > max_raw): if the initial tick is in the
          lower segment [0, max_raw], add one full revolution so the value
          sits above the upper segment [min_raw, 4095].

        Subsequent calls track rollovers by checking whether the step between
        consecutive readings exceeds half the encoder range (2048 ticks).
        _prev_raw stores the UNWRAPPED value so that the delta computation
        stays valid even after the offset has been adjusted.
        """
        prev_u = self._prev_raw[joint]     # stored as unwrapped (may be >4095)
        if prev_u is None:
            # First read — seed offset for genuine wrapped calibration ranges.
            # A genuine wrap crosses the 0/4095 boundary: the "short" path between
            # min_raw and max_raw passes through 0.  This happens when
            # min_raw > max_raw AND min_raw - max_raw > TICKS//2.
            # Non-wrap inverted ranges (physical min = high encoder, physical max = low,
            # but entirely within one revolution) have mn > mx with mn - mx < TICKS//2
            # and do NOT need the unwrap offset.
            if self._calib:
                jc = self._calib["joints"].get(joint, {})
                mn = jc.get("min_raw", 0)
                mx = jc.get("max_raw", _TICKS - 1)
                if mn > mx and (mn - mx) > _TICKS // 2 and raw <= mx:
                    # raw is in the lower segment of a genuine wrapped range;
                    # shift up by one revolution so it reads above min_raw.
                    self._unwrap_off[joint] = _TICKS
            self._prev_raw[joint] = raw + self._unwrap_off[joint]
            return raw + self._unwrap_off[joint]

        prev_raw = prev_u % _TICKS         # physical tick implied by last unwrapped
        delta = raw - prev_raw
        if delta > _TICKS // 2:            # backward wrap: e.g. 5 → 4090
            self._unwrap_off[joint] -= _TICKS
        elif delta < -(_TICKS // 2):       # forward wrap:  e.g. 4090 → 5
            self._unwrap_off[joint] += _TICKS
        unwrapped = raw + self._unwrap_off[joint]
        self._prev_raw[joint] = unwrapped
        return unwrapped

    # ------------------------------------------------------------------
    # Controller interface  (poll every control cycle)
    # ------------------------------------------------------------------

    def poll(self) -> Optional[np.ndarray]:
        """Read all 6 servos and return AIZEE joint targets [rad].

        Handles three encoder range types (see calibration JSON):
          Normal    (min_raw <= max_raw): simple linear interpolation.
          Wrap      (min_raw > max_raw, gap > 2048): genuine 0/4095 crossing;
                    upper bound is lifted into unwrapped space.
          Inverted  (min_raw > max_raw, gap <= 2048): physical min = high
                    encoder value; interpolation runs in reverse so that
                    u=min_raw→rad_min and u=max_raw→rad_max.

        Falls back to raw-ticks-to-radians if no calibration is loaded.
        Returns None if any servo read fails.
        """
        unwrapped = self.read_unwrapped()
        if unwrapped is None:
            return None

        out = np.zeros(len(self.JOINTS), dtype=np.float32)
        for i, joint in enumerate(self.JOINTS):
            u = unwrapped[joint]           # continuously varying (may be outside 0-4095)
            if self._calib:
                jc    = self._calib["joints"].get(joint, {})
                mn    = jc.get("min_raw",  0)
                mx    = jc.get("max_raw",  _TICKS - 1)
                r_min = jc.get("rad_min",  AIZEE_DEFAULTS[i][0])
                r_max = jc.get("rad_max",  AIZEE_DEFAULTS[i][1])
                # Three encoder range types:
                #   Normal:       mn <= mx — simple ascending range.
                #   Genuine wrap: mn > mx, mn - mx > TICKS//2 — crosses 0/4095.
                #   Inverted:     mn > mx, mn - mx < TICKS//2 — physical min is
                #                 high encoder value, physical max is low.  Does NOT
                #                 cross 0/4095; the old code misidentified this as a
                #                 wrap and produced frac ≤ 0 for all mid-range values.
                if mn <= mx:
                    # Normal ascending range.
                    span = mx - mn
                    raw_frac = (u - mn) / span if span else 0.5
                elif (mn - mx) > _TICKS // 2:
                    # Genuine wrap: lift upper bound into unwrapped space.
                    mx_u = mn + (_TICKS - mn + mx)
                    span = mx_u - mn
                    raw_frac = (u - mn) / span if span else 0.5
                else:
                    # Non-wrap inverted: encoder decreases from physical min → max.
                    # u = mn → frac = 0 (rad_min); u = mx → frac = 1 (rad_max).
                    span = mn - mx
                    raw_frac = (mn - u) / span if span else 0.5
                self._clamped[i] = (raw_frac < 0.0 or raw_frac > 1.0)
                frac = max(0.0, min(1.0, raw_frac))
                out[i] = r_min + frac * (r_max - r_min)
            else:
                self._clamped[i] = False
                out[i] = ticks_to_rad(u % _TICKS)
        return out

    # ------------------------------------------------------------------
    # Zero-offset / direction mapping
    # ------------------------------------------------------------------

    @property
    def zero_offsets(self) -> np.ndarray:
        """Per-joint zero offsets [rad] loaded from calibration.
        Subtracted from poll() output before sending to AIZEE arm."""
        out = np.zeros(len(self.JOINTS), dtype=np.float32)
        if self._calib:
            for i, j in enumerate(self.JOINTS):
                out[i] = float(self._calib["joints"].get(j, {}).get("zero_offset", 0.0))
        return out

    @property
    def directions(self) -> np.ndarray:
        """Per-joint direction signs (+1 or -1) loaded from calibration.
        Multiplied after zero subtraction to align rotation direction with AIZEE arm."""
        out = np.ones(len(self.JOINTS), dtype=np.float32)
        if self._calib:
            for i, j in enumerate(self.JOINTS):
                out[i] = float(self._calib["joints"].get(j, {}).get("direction", 1))
        return out

    def save_zero(self, offsets: np.ndarray) -> None:
        """Persist zero_offset values to the calibration JSON on disk."""
        if not self._calib:
            return
        for i, joint in enumerate(self.JOINTS):
            if joint in self._calib["joints"]:
                self._calib["joints"][joint]["zero_offset"] = round(float(offsets[i]), 4)
        with open(self._calib_path, "w") as f:
            import json as _json
            _json.dump(self._calib, f, indent=2)

    # ------------------------------------------------------------------
    # Raw / unwrapped reads
    # ------------------------------------------------------------------

    def read_raw(self) -> Optional[dict[str, int]]:
        """Return {joint: ticks} in [0, 4095] for all 7 servos."""
        result: dict[str, int] = {}
        for servo_id, joint in enumerate(self.JOINTS, start=1):
            val = self._read_u16(servo_id, _REG_POS)
            if val is None:
                return None
            result[joint] = val
        return result

    def read_unwrapped(self) -> Optional[dict[str, int]]:
        """Read all servos and return continuously unwrapped positions.

        Values may be outside [0, 4095] when the servo has crossed the
        encoder boundary since connect().  Updates internal unwrap state.
        """
        result: dict[str, int] = {}
        for servo_id, joint in enumerate(self.JOINTS, start=1):
            val = self._read_u16(servo_id, _REG_POS)
            if val is None:
                return None
            result[joint] = self._unwrap(joint, val)
        return result

    def read_positions(self) -> Optional[dict[str, float]]:
        """Return {joint: radians} (center = 0), or None on error."""
        raw = self.read_raw()
        return {j: ticks_to_rad(v) for j, v in raw.items()} if raw else None

    # ------------------------------------------------------------------
    # Feetech packet I/O
    # ------------------------------------------------------------------

    def _read_u16(self, servo_id: int, reg: int) -> Optional[int]:
        """Send a READ instruction and parse the 2-byte response.

        Packet:   FF FF [ID] [LEN=4] [INST=0x02] [REG] [COUNT=2] [CHECKSUM]
        Response: FF FF [ID] [LEN=4] [ERROR]      [D0]  [D1]      [CHECKSUM]
        """
        body     = bytes([servo_id, 4, _READ, reg, 2])
        checksum = (~sum(body)) & 0xFF
        pkt      = _HEADER + body + bytes([checksum])
        try:
            self._ser.reset_input_buffer()
            self._ser.write(pkt)
            self._ser.flush()
            resp = self._ser.read(8)          # 8 bytes total
        except serial.SerialException:
            return None

        if len(resp) < 8 or resp[0] != 0xFF or resp[1] != 0xFF:
            return None
        if resp[4] != 0:                      # error byte
            return None
        return resp[5] | (resp[6] << 8)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_calib(path: Path) -> Optional[dict]:
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return None
