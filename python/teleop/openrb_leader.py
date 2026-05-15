"""openrb_leader.py — OpenRB-150 + Dynamixel XL330 leader arm controller.

The OpenRB-150 board reads 7 Dynamixel XL330-M077-T servos over its built-in
TTL bus and exposes a tiny binary protocol over USB-CDC.  This module talks
that protocol and presents the same controller interface as So101Leader, so
the rest of the AIZEE teleop pipeline can use either arm transparently.

Controller protocol (duck-typed; identical to So101Leader):

    arm = OpenRBLeader('/dev/ttyACM0')
    arm.connect()
    while True:
        targets = arm.poll()   # Optional[np.ndarray]  7 AIZEE joint targets [rad]
    arm.close()

USB wire protocol (host <-> OpenRB-150):

    Host -> MCU:
        0x50          probe   (reply: ASCII "AIZEE-OPENRB-LEADER\\n")
        0xA5          poll    (reply: 0xA5 [N=7] [pos x 4 bytes int32 LE]*N
                                       [int16 joy_x][int16 joy_y]
                                       [uint8 joy_btn][uint8 joy_status][crc8])

    The MCU keeps torque disabled on every servo and sync-reads
    Present_Position (DXL register 132, 4 bytes) on each poll.  It also
    polls an optional M5Stack Joystick2 (I2C addr 0x63 on the firmware's
    Wire bus) and embeds X/Y/button state in the same reply — used by
    collect_demo.py for operator drive + recording start/stop.  When no
    joystick is present, joy_status is non-zero and the host ignores the
    fields.

Servo mapping (mirrors SO-101 leader):
    1 shoulder_pan  -> swivel
    2 shoulder_lift -> gantry_base
    3 elbow_flex    -> gantry_mid
    4 wrist_flex    -> gantry_end
    5 wrist_yaw     -> wrist_pitch
    6 wrist_roll    -> wrist_roll
    7 gripper       -> gripper

Install dependency:  pip install pyserial
"""

from __future__ import annotations

import json
import math
import struct
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
# Wire protocol constants  (must match firmware/openrb_leader/src/main.cpp)
# ---------------------------------------------------------------------------
_BAUD       = 1_000_000   # USB-CDC ignores this, but pyserial wants a value
_CMD_POLL   = 0xA5        # request positions
_CMD_IDENT  = 0x50        # identify (returns ASCII line)
_CMD_SCAN   = 0x53        # bus scan (returns list of (id, baud_code))
_CMD_REID   = 0x52        # re-assign single-servo ID + baud (setup wizard)
_CMD_CENTER = 0xC0        # drive single servo to encoder centre (2048)
_REPLY_HDR  = 0xA5
_REPLY_SCAN_HDR   = 0x53
_REPLY_REID_HDR   = 0x52
_REPLY_CENTER_HDR = 0xC0
_IDENT_STR  = b"AIZEE-OPENRB-LEADER"
_N_SERVOS   = 7

# REID status codes (from firmware).
REID_OK             = 0x00
REID_NOT_FOUND      = 0x01
REID_AMBIGUOUS      = 0x02
REID_WRITE_FAILED   = 0x03
REID_BAUD_FAILED    = 0x04
REID_VERIFY_FAILED  = 0x05
REID_STATUS_NAMES   = {
    REID_OK:            "ok",
    REID_NOT_FOUND:     "no servo detected on bus",
    REID_AMBIGUOUS:     "multiple servos on bus (unplug others)",
    REID_WRITE_FAILED:  "set-id write failed",
    REID_BAUD_FAILED:   "set-baudrate write failed",
    REID_VERIFY_FAILED: "post-write ping at 1Mbps failed (servo may be on wrong baud)",
}

# CENTER status codes (firmware handle_center).
CENTER_OK         = 0x00
CENTER_NOT_FOUND  = 0x01
CENTER_TIMEOUT    = 0x02
CENTER_FAILED     = 0x03
CENTER_STATUS_NAMES = {
    CENTER_OK:        "ok",
    CENTER_NOT_FOUND: "servo did not respond",
    CENTER_TIMEOUT:   "movement timed out before reaching centre",
    CENTER_FAILED:    "torque/mode/goal write failed",
}

# baud_code -> baud rate, matching firmware BAUDS[] order.
BAUD_CODES = {0: 1_000_000, 1: 57_600, 2: 115_200, 3: 2_000_000}

# XL330 encoder is 12-bit (0..4095) over a single revolution.  Present_Position
# is a 32-bit signed value that accumulates across multi-turn extended-position
# mode, but we treat it modulo 4096 for the same calibration math used by SO-101.
_TICKS  = 4096
_CENTER = 2048

# M5Stack Joystick2 status codes (firmware appends as last byte of POLL reply).
JOY_STATUS_OK          = 0x00
JOY_STATUS_NOT_PRESENT = 0x01
JOY_STATUS_READ_ERROR  = 0x02

# Joystick2 12-bit centred range — reading is int16 nominally in [-2048, +2047].
# Divided by this to produce a [-1, +1] float for downstream consumers.
_JOY_HALF_RANGE = 2048.0

CALIB_PATH = Path("config/openrb_calibration.json")

# Default AIZEE arm target range [rad_min, rad_max] per joint (matches SO-101).
AIZEE_DEFAULTS: list[tuple[float, float]] = [
    (-1.57,  1.57),   # swivel
    (-1.57,  1.57),   # gantry_base
    (-1.57,  0.50),   # gantry_mid
    (-0.50,  1.57),   # gantry_end
    (-1.00,  1.00),   # wrist_pitch
    (-1.57,  1.57),   # wrist_roll
    ( 0.00,  0.50),   # gripper
]


def ticks_to_rad(ticks: int) -> float:
    """Convert raw encoder ticks to radians relative to center (0 = neutral)."""
    return (ticks - _CENTER) * (2.0 * math.pi / _TICKS)


def _crc8(data: bytes) -> int:
    """Dallas/Maxim CRC-8 (polynomial 0x31).  Matches the firmware implementation."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


# ---------------------------------------------------------------------------
# OpenRBLeader
# ---------------------------------------------------------------------------

class OpenRBLeader:
    """OpenRB-150 + 7x XL330 leader arm.

    Mirrors the So101Leader interface so it is a drop-in replacement.
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
        self.port  = port
        self.baud  = baud
        self._ser: Optional[serial.Serial] = None
        self._calib_path = Path(calib)
        self._calib = _load_calib(self._calib_path)
        self._prev_raw:   dict[str, Optional[int]] = {j: None for j in self.JOINTS}
        self._unwrap_off: dict[str, int]           = {j: 0    for j in self.JOINTS}
        self._clamped:    list[bool]               = [False] * len(self.JOINTS)
        self._last_clean: Optional[np.ndarray]     = None
        self._reject_count: int                    = 0
        # Joystick2 state — updated each successful poll.  Consumers read
        # `last_joystick` to get the current snapshot; the press_counter
        # increments on every released→pressed edge so a slow main loop
        # can never miss a quick click between samples.
        self._joy_x:        float = 0.0    # normalised [-1, +1]
        self._joy_y:        float = 0.0
        self._joy_button:   bool  = False  # True while pressed (after debounce)
        self._joy_prev_btn: bool  = False
        self._joy_press_counter: int = 0
        self._joy_status:   int   = JOY_STATUS_NOT_PRESENT
        # Button debounce.  The M5 Joystick2's click switch sits under the
        # thumb-stick and false-fires when the operator pushes sideways with
        # any downward bias; single-sample I2C noise on the button register
        # also lands as 0x00 occasionally.  We require N consecutive same-
        # state reads (~30 ms at 174 Hz) to flip the public `button` state.
        # Symmetric: filters both released→pressed and pressed→released
        # transients, so quick legit clicks held >30 ms still come through.
        self._JOY_DEBOUNCE_N: int = 5
        self._joy_btn_streak: int = 0      # >0 = streak of "pressed" raw reads,
                                            # <0 = streak of "released"

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
            # OpenRB-150 USB-CDC resets the MCU on port open; give the
            # bootloader and sketch time to come up before the first poll.
            time.sleep(0.5)
            self._ser.reset_input_buffer()
            for j in self.JOINTS:
                self._prev_raw[j]   = None
                self._unwrap_off[j] = 0
            self._clamped = [False] * len(self.JOINTS)
            self._last_clean = None
            self._reject_count = 0
            return True
        except serial.SerialException as exc:
            print(f"[OpenRB] connect failed on {self.port}: {exc}")
            return False

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()

    @property
    def connected(self) -> bool:
        return bool(self._ser and self._ser.is_open)

    @property
    def clamped_joints(self) -> list[bool]:
        return list(self._clamped)

    @property
    def last_joystick(self) -> dict:
        """Snapshot of the M5 Joystick2 state from the last successful poll.

        Keys:
            x, y           floats in [-1, +1] (0 when joystick absent)
            button         True while currently pressed
            press_counter  monotonic; increments on each press edge so a
                           slow consumer never misses a quick click
            status         JOY_STATUS_OK / NOT_PRESENT / READ_ERROR
            present        convenience bool: status == JOY_STATUS_OK
        """
        return {
            "x":             self._joy_x,
            "y":             self._joy_y,
            "button":        self._joy_button,
            "press_counter": self._joy_press_counter,
            "status":        self._joy_status,
            "present":       self._joy_status == JOY_STATUS_OK,
        }

    # ------------------------------------------------------------------
    # Encoder unwrapping (identical to SO-101)
    # ------------------------------------------------------------------

    def _unwrap(self, joint: str, raw: int) -> int:
        prev_u = self._prev_raw[joint]
        if prev_u is None:
            if self._calib:
                jc = self._calib["joints"].get(joint, {})
                mn = jc.get("min_raw", 0)
                mx = jc.get("max_raw", _TICKS - 1)
                if mn > mx and (mn - mx) > _TICKS // 2 and raw <= mx:
                    self._unwrap_off[joint] = _TICKS
            self._prev_raw[joint] = raw + self._unwrap_off[joint]
            return raw + self._unwrap_off[joint]

        prev_raw = prev_u % _TICKS
        delta = raw - prev_raw
        if delta > _TICKS // 2:
            self._unwrap_off[joint] -= _TICKS
        elif delta < -(_TICKS // 2):
            self._unwrap_off[joint] += _TICKS
        unwrapped = raw + self._unwrap_off[joint]
        self._prev_raw[joint] = unwrapped
        return unwrapped

    # ------------------------------------------------------------------
    # Poll
    # ------------------------------------------------------------------

    def poll(self) -> Optional[np.ndarray]:
        unwrapped = self.read_unwrapped()
        if unwrapped is None:
            return None

        out = np.zeros(len(self.JOINTS), dtype=np.float32)
        for i, joint in enumerate(self.JOINTS):
            u = unwrapped[joint]
            if self._calib:
                jc    = self._calib["joints"].get(joint, {})
                mn    = jc.get("min_raw",  0)
                mx    = jc.get("max_raw",  _TICKS - 1)
                r_min = jc.get("rad_min",  AIZEE_DEFAULTS[i][0])
                r_max = jc.get("rad_max",  AIZEE_DEFAULTS[i][1])
                if mn <= mx:
                    span = mx - mn
                    raw_frac = (u - mn) / span if span else 0.5
                elif (mn - mx) > _TICKS // 2:
                    mx_u = mn + (_TICKS - mn + mx)
                    span = mx_u - mn
                    raw_frac = (u - mn) / span if span else 0.5
                else:
                    span = mn - mx
                    raw_frac = (mn - u) / span if span else 0.5
                self._clamped[i] = (raw_frac < 0.0 or raw_frac > 1.0)
                frac = max(0.0, min(1.0, raw_frac))
                out[i] = r_min + frac * (r_max - r_min)
            else:
                self._clamped[i] = False
                out[i] = ticks_to_rad(u % _TICKS)

        # Multi-joint glitch rejection — catches USB-CDC byte-misalignment
        # errors, which produce wild values (~0.5-π rad off) on *every*
        # joint of the corrupted sync-read frame.
        #
        # Tuning notes:
        # - THRESH 0.15 rad/poll at ~500 Hz polls = 75 rad/s of "real" motion
        #   — well above any human-driven leader speed, but well below the
        #   sub-rad deltas a corrupted frame produces.  The previous value
        #   (0.008) tripped on normal fast teleop, freezing telemetry until
        #   MAX_REJECTS expired and then snapping forward — that "pause +
        #   jump" was perceived as an arm jerk.
        # - MIN_JOINTS 6 of 7 means real coordinated motion (which rarely
        #   slings every joint past 0.15 rad in one tick) passes through;
        #   only a frame-wide corruption fires the filter.
        # - MAX_REJECTS 3 (~6 ms at 500 Hz) is enough to absorb a 1-3 frame
        #   burst of corruption without producing a visible telemetry stall.
        _GLITCH_JOINT_THRESH = 0.15
        _GLITCH_MIN_JOINTS   = 6
        _MAX_REJECTS         = 3

        if self._last_clean is not None:
            n_big = int(np.sum(np.abs(out - self._last_clean) > _GLITCH_JOINT_THRESH))
            if n_big >= _GLITCH_MIN_JOINTS:
                self._reject_count += 1
                if self._reject_count < _MAX_REJECTS:
                    return self._last_clean.copy()

        self._last_clean = out.copy()
        self._reject_count = 0
        return out

    # ------------------------------------------------------------------
    # Zero-offset / direction mapping
    # ------------------------------------------------------------------

    @property
    def zero_offsets(self) -> np.ndarray:
        out = np.zeros(len(self.JOINTS), dtype=np.float32)
        if self._calib:
            for i, j in enumerate(self.JOINTS):
                out[i] = float(self._calib["joints"].get(j, {}).get("zero_offset", 0.0))
        return out

    @property
    def directions(self) -> np.ndarray:
        out = np.ones(len(self.JOINTS), dtype=np.float32)
        if self._calib:
            for i, j in enumerate(self.JOINTS):
                out[i] = float(self._calib["joints"].get(j, {}).get("direction", 1))
        return out

    def save_zero(self, offsets: np.ndarray) -> None:
        if not self._calib:
            return
        for i, joint in enumerate(self.JOINTS):
            if joint in self._calib["joints"]:
                self._calib["joints"][joint]["zero_offset"] = round(float(offsets[i]), 4)
        with open(self._calib_path, "w") as f:
            json.dump(self._calib, f, indent=2)

    def save_limits(self, limits: dict[str, tuple[int, int]]) -> None:
        """Persist per-joint min_raw/max_raw to the calibration JSON.

        *limits* is {joint_name: (min_raw, max_raw)} in physical encoder
        ticks (0..4095).  Joints not present in the dict are left untouched.
        """
        if not self._calib:
            return
        for joint, (mn, mx) in limits.items():
            jc = self._calib["joints"].get(joint)
            if jc is None:
                continue
            jc["min_raw"] = int(mn)
            jc["max_raw"] = int(mx)
        with open(self._calib_path, "w") as f:
            json.dump(self._calib, f, indent=2)

    # ------------------------------------------------------------------
    # Wire protocol
    # ------------------------------------------------------------------

    def _read_exact(self, n: int) -> Optional[bytes]:
        """Block until *n* bytes are read or the serial timeout elapses."""
        if self._ser is None:
            return None
        buf = bytearray()
        deadline = time.monotonic() + 0.1
        while len(buf) < n and time.monotonic() < deadline:
            chunk = self._ser.read(n - len(buf))
            if not chunk:
                return None if not buf else None
            buf.extend(chunk)
        return bytes(buf) if len(buf) == n else None

    def _sync_read_positions(self) -> Optional[dict[str, int]]:
        """Send a poll command and parse the response frame.

        Frame:
            [0xA5][N=7][pos x 4 bytes signed LE]*N
            [int16 LE joy_x][int16 LE joy_y][uint8 joy_btn][uint8 joy_status]
            [crc8]
        Total: 1 + 1 + 7*4 + 4 + 2 + 1 = 37 bytes.

        Joystick fields are parsed and stashed on the instance (see
        last_joystick) on every successful poll.  When joy_status != 0
        the joystick state is reported as neutral so consumers don't act
        on garbage.
        """
        if self._ser is None:
            return None
        try:
            self._ser.reset_input_buffer()
            self._ser.write(bytes([_CMD_POLL]))
            self._ser.flush()
            # Find frame start (tolerates one stray byte).
            hdr = self._read_exact(2)
            if hdr is None:
                return None
            if hdr[0] != _REPLY_HDR:
                # Try resyncing once: read one more byte and check.
                tail = self._read_exact(1)
                if tail is None or hdr[1] != _REPLY_HDR:
                    return None
                n_byte = tail[0]
            else:
                n_byte = hdr[1]
            if n_byte != _N_SERVOS:
                return None
            payload = self._read_exact(_N_SERVOS * 4 + 6 + 1)
            if payload is None:
                return None
            data = payload[:-1]
            crc  = payload[-1]
            if _crc8(bytes([_REPLY_HDR, n_byte]) + data) != crc:
                return None
            positions = struct.unpack("<7i", data[: _N_SERVOS * 4])
            joy_x_raw, joy_y_raw, joy_btn, joy_status = struct.unpack(
                "<hhBB", data[_N_SERVOS * 4 :]
            )
        except (serial.SerialException, OSError):
            return None

        # Update joystick snapshot.  Edge-detect on the released→pressed
        # transition; press_counter is monotonic so a 30 Hz main loop can
        # poll it without missing a quick click that happened at 500 Hz.
        self._joy_status = int(joy_status)
        if joy_status == JOY_STATUS_OK:
            # X is negated so the public convention is "+x = stick pushed
            # right" from the operator's point of view (the M5 unit's raw
            # X reports the opposite sign as mounted on the leader board).
            # +y already means "stick pushed forward" so it passes through.
            self._joy_x = max(-1.0, min(1.0, -joy_x_raw / _JOY_HALF_RANGE))
            self._joy_y = max(-1.0, min(1.0,  joy_y_raw / _JOY_HALF_RANGE))

            # Debounce: walk a saturated counter toward the current raw
            # reading; only update the public button state when the streak
            # reaches the debounce threshold in either direction.  Filters
            # both single-sample I2C noise and brief mechanical contact
            # bounces from off-axis stick pressure.
            raw_pressed = (joy_btn == 0)
            if raw_pressed:
                self._joy_btn_streak = min(self._joy_btn_streak + 1,
                                            self._JOY_DEBOUNCE_N)
            else:
                self._joy_btn_streak = max(self._joy_btn_streak - 1,
                                            -self._JOY_DEBOUNCE_N)

            if self._joy_btn_streak >= self._JOY_DEBOUNCE_N:
                btn_pressed = True
            elif self._joy_btn_streak <= -self._JOY_DEBOUNCE_N:
                btn_pressed = False
            else:
                # Between thresholds — hold the current debounced state.
                btn_pressed = self._joy_button

            if btn_pressed and not self._joy_prev_btn:
                self._joy_press_counter += 1
            self._joy_button   = btn_pressed
            self._joy_prev_btn = btn_pressed
        elif joy_status == JOY_STATUS_NOT_PRESENT:
            # Only latched at boot by the firmware — true device absence.
            # Force everything to neutral so consumers see "no joystick".
            self._joy_x = 0.0
            self._joy_y = 0.0
            self._joy_button     = False
            self._joy_prev_btn   = False
            self._joy_btn_streak = 0
        # JOY_STATUS_READ_ERROR: a single I2C transaction failed.  Hold the
        # previous x / y / button / prev_btn values — on the next successful
        # poll we re-sync.  The `present` field (status == OK) goes False
        # for this one sample, so the main loop transparently falls back to
        # xbox / WASD for that tick rather than acting on stale state.

        # XL330 in extended-position mode returns a signed multi-turn value.
        # Reduce to [0, 4095] for the calibration math (which already handles
        # wrap and inverted ranges).
        return {joint: positions[i] % _TICKS for i, joint in enumerate(self.JOINTS)}

    def read_raw(self) -> Optional[dict[str, int]]:
        return self._sync_read_positions()

    def read_unwrapped(self) -> Optional[dict[str, int]]:
        raw = self._sync_read_positions()
        if raw is None:
            return None
        return {joint: self._unwrap(joint, raw[joint]) for joint in self.JOINTS}

    def read_positions(self) -> Optional[dict[str, float]]:
        raw = self.read_raw()
        return {j: ticks_to_rad(v) for j, v in raw.items()} if raw else None

    # ------------------------------------------------------------------
    # Centering (one servo at a time — the firmware blocks until done)
    # ------------------------------------------------------------------

    def center_one(self, servo_id: int, timeout: float = 6.0) -> tuple[int, int, int]:
        """Drive one servo to encoder centre (2048) and disable torque.

        The firmware enables position control with a slow profile velocity,
        commands GOAL_POSITION = 2048, waits for arrival, then turns torque
        back off.  Sequential per-servo invocation avoids the inrush spike
        of energising all 7 at once on USB power.

        Returns (status, found_id, final_position) where status is one of
        the CENTER_* codes.  *timeout* must exceed the firmware's own
        CENTER_TIMEOUT_MS (4 s) by enough margin for USB-CDC round trip.
        """
        if self._ser is None:
            return (CENTER_NOT_FOUND, servo_id, 0)
        try:
            self._ser.reset_input_buffer()
            self._ser.write(bytes([_CMD_CENTER, servo_id & 0xFF]))
            self._ser.flush()
        except (serial.SerialException, OSError):
            return (CENTER_NOT_FOUND, servo_id, 0)
        # Reuse the module-level _read_exact helper so we get a generous timeout.
        payload = _read_exact(self._ser, 8, timeout=timeout)
        if payload is None:
            return (CENTER_TIMEOUT, servo_id, 0)
        if payload[0] != _REPLY_CENTER_HDR:
            return (CENTER_FAILED, servo_id, 0)
        if _crc8(bytes(payload[:7])) != payload[7]:
            return (CENTER_FAILED, servo_id, 0)
        pos = struct.unpack("<i", payload[3:7])[0]
        return (payload[1], payload[2], pos)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_calib(path: Path) -> Optional[dict]:
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return None


# ---------------------------------------------------------------------------
# Auto-detection
# ---------------------------------------------------------------------------

# OpenRB-150 enumerates as a Robotis-vendor USB-CDC device.  Some bootloader
# variants (the SAMD UF2 fallback) show up under Arduino's VID — both are
# accepted here; the probe handshake is the real gate.
_OPENRB_KNOWN_VIDS = {
    0x2F5D,   # ROBOTIS
    0x2341,   # Arduino LLC
    0x239A,   # Adafruit (sometimes used by SAMD UF2 variants)
}


def _probe_openrb(device: str, baud: int = _BAUD, timeout: float = 0.4) -> tuple[bool, str]:
    """Send the IDENT command and check for the AIZEE-OPENRB-LEADER reply."""
    if serial is None:
        return False, "pyserial not installed"
    try:
        ser = serial.Serial(device, baud, timeout=timeout)
    except (serial.SerialException, OSError) as exc:
        return False, f"open failed ({exc.__class__.__name__})"
    try:
        # OpenRB-150 resets on USB-CDC open — wait for the sketch to come up.
        time.sleep(0.6)
        try:
            ser.reset_input_buffer()
            ser.write(bytes([_CMD_IDENT]))
            ser.flush()
        except (serial.SerialException, OSError) as exc:
            return False, f"io error ({exc.__class__.__name__})"
        # Read up to one full reply line; tolerate leading framing noise.
        deadline = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if chunk:
                buf.extend(chunk)
                if _IDENT_STR in buf:
                    return True, "identified"
            else:
                break
        return False, "no identify reply"
    finally:
        try:
            ser.close()
        except Exception:
            pass


def find_openrb_port(
    exclude: Optional[list[str]] = None,
    baud: int = _BAUD,
    verbose: bool = False,
) -> Optional[str]:
    """Auto-detect the OpenRB-150 leader board's USB-CDC port."""
    if serial is None:
        return None
    try:
        from serial.tools import list_ports
    except ImportError:
        return None

    excl  = set(exclude or [])
    ports = [p for p in list_ports.comports() if p.device not in excl]
    known  = [p for p in ports if p.vid in _OPENRB_KNOWN_VIDS]
    others = [p for p in ports if p.vid not in _OPENRB_KNOWN_VIDS]
    ordered = known + others

    if verbose:
        if not ordered:
            print("  (no serial ports enumerated)")
        for p in ordered:
            vid_pid = (f"{p.vid:04X}:{p.pid:04X}"
                       if p.vid is not None and p.pid is not None else "????:????")
            desc = p.description or ""
            print(f"  {p.device:<10}  VID:PID={vid_pid}  {desc}")

    for p in ordered:
        ok, detail = _probe_openrb(p.device, baud)
        if verbose:
            print(f"  probe {p.device}: {'OK' if ok else 'fail'} — {detail}")
        if ok:
            return p.device
    return None


# ---------------------------------------------------------------------------
# Setup-mode client helpers (used by openrb_setup_arm.py)
# ---------------------------------------------------------------------------

def _read_exact(ser, n: int, timeout: float = 1.5) -> Optional[bytes]:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while len(buf) < n and time.monotonic() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf) if len(buf) == n else None


def bus_scan(ser) -> Optional[list[tuple[int, int]]]:
    """Send CMD_SCAN to the OpenRB-150; return list of (servo_id, baud_code) or None.

    The firmware sweeps every supported baud rate and broadcast-pings on each.
    Use BAUD_CODES to look up the actual baud from baud_code.
    """
    try:
        ser.reset_input_buffer()
        ser.write(bytes([_CMD_SCAN]))
        ser.flush()
    except (serial.SerialException, OSError):
        return None
    # Bus scan can take ~400 ms per baud * 4 bauds = ~1.6s — be generous.
    hdr = _read_exact(ser, 2, timeout=4.0)
    if hdr is None or hdr[0] != _REPLY_SCAN_HDR:
        return None
    n = hdr[1]
    payload = _read_exact(ser, n * 2 + 1, timeout=2.0)
    if payload is None:
        return None
    data = payload[:-1]
    crc  = payload[-1]
    if _crc8(bytes([_REPLY_SCAN_HDR, n]) + data) != crc:
        return None
    return [(data[i * 2], data[i * 2 + 1]) for i in range(n)]


def reassign_id(ser, target_id: int) -> tuple[int, int, int]:
    """Send CMD_REID to assign whatever servo is on the bus to *target_id* @ 1 Mbps.

    Returns (status, found_id, baud_code) — status is one of REID_*.
    """
    try:
        ser.reset_input_buffer()
        ser.write(bytes([_CMD_REID, target_id & 0xFF]))
        ser.flush()
    except (serial.SerialException, OSError):
        return (REID_NOT_FOUND, 0, 0xFF)
    payload = _read_exact(ser, 5, timeout=4.0)
    if payload is None:
        return (REID_NOT_FOUND, 0, 0xFF)
    if payload[0] != _REPLY_REID_HDR:
        return (REID_NOT_FOUND, 0, 0xFF)
    if _crc8(bytes(payload[:4])) != payload[4]:
        return (REID_NOT_FOUND, 0, 0xFF)
    return (payload[1], payload[2], payload[3])
