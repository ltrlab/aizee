"""so101_leader.py — SO-101 leader arm controller module.

Minimal Feetech STS3215 serial reader — no SDK dependency beyond pyserial.

Controller protocol (duck-typed, works anywhere in the teleop system):

    arm = So101Leader('/dev/ttyACM0')
    arm.connect()
    while True:
        targets = arm.poll()   # Optional[np.ndarray]  6 AIZEE joint targets [rad]
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
    (-1.57,  1.57),   # gantry_base  ← shoulder_pan
    (-1.57,  0.50),   # gantry_mid   ← shoulder_lift
    (-0.50,  1.57),   # gantry_end   ← elbow_flex
    (-1.00,  1.00),   # wrist_pitch  ← wrist_flex
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
    """SO-101 6-DOF leader arm.  Reads STS3215 servo positions over USB serial
    and converts them to AIZEE arm joint targets using a calibration file.

    Servo IDs 1-6 map to joints in order:
        1 shoulder_pan → gantry_base
        2 shoulder_lift → gantry_mid
        3 elbow_flex    → gantry_end
        4 wrist_flex    → wrist_pitch
        5 wrist_roll    → wrist_roll
        6 gripper       → gripper
    """

    JOINTS = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]
    AIZEE_JOINTS = [
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
        self._calib = _load_calib(Path(calib))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        """Open serial port.  Returns True on success."""
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(0.1)
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

    # ------------------------------------------------------------------
    # Controller interface  (poll every control cycle)
    # ------------------------------------------------------------------

    def poll(self) -> Optional[np.ndarray]:
        """Read all 6 servos and return AIZEE joint targets [rad].

        Uses calibration (min/max raw → rad_min/rad_max per joint).
        Falls back to raw-ticks-to-radians conversion if no calibration.
        Returns None if any servo read fails.
        """
        raw = self.read_raw()
        if raw is None:
            return None

        out = np.zeros(6, dtype=np.float32)
        for i, joint in enumerate(self.JOINTS):
            ticks = raw[joint]
            if self._calib:
                jc    = self._calib["joints"].get(joint, {})
                mn    = jc.get("min_raw",  0)
                mx    = jc.get("max_raw",  _TICKS - 1)
                r_min = jc.get("rad_min",  AIZEE_DEFAULTS[i][0])
                r_max = jc.get("rad_max",  AIZEE_DEFAULTS[i][1])
                span  = mx - mn
                frac  = max(0.0, min(1.0, (ticks - mn) / span)) if span else 0.5
                out[i] = r_min + frac * (r_max - r_min)
            else:
                out[i] = ticks_to_rad(ticks)
        return out

    # ------------------------------------------------------------------
    # Raw reads
    # ------------------------------------------------------------------

    def read_raw(self) -> Optional[dict[str, int]]:
        """Return {joint: ticks} for all 6 servos, or None on any error."""
        result: dict[str, int] = {}
        for servo_id, joint in enumerate(self.JOINTS, start=1):
            val = self._read_u16(servo_id, _REG_POS)
            if val is None:
                return None
            result[joint] = val
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
