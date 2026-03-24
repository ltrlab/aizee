#!/usr/bin/env python3
"""so101_teleop.py — Teleoperate the AIZEE arm using the SO-101 leader arm.

Reads SO-101 joint positions at 20 Hz, maps them to AIZEE arm targets via
the calibration file, and sends arm_joints commands over ZMQ.

The SO-101 is a drop-in controller module — same poll() interface that any
other controller (keyboard, gamepad) would expose.

Usage:
    python so101_teleop.py --port /dev/ttyACM0
    python so101_teleop.py --port COM4 \\
        --cmd   tcp://192.168.0.27:5555 \\
        --telem tcp://192.168.0.27:5556

Controls (keyboard, while script is running):
    E    enable all arm joints on the AIZEE arm
    I    idle — enable motors with zero torque (see actual positions)
    H    hold — freeze target at current actual position
    X    soft shutdown — hold 1 s, return to zero, disable
    Q    quit  (Ctrl-C also works)

Gamepad: A=enable  B=shutdown/cancel  Start=hold  Back=quit

"""

from __future__ import annotations

import argparse
import enum
import json
import sys
import threading
import time
import yaml
from pathlib import Path
from typing import Optional

import numpy as np
import zmq

try:
    import pygame
    _pygame_available = True
except ImportError:
    _pygame_available = False

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from so101_leader import So101Leader, CALIB_PATH

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import ARM_JOINTS, KP, KD, setup_keyboard, load_arm_limits, clamp_arm_positions

TELEOP_HZ = 30   # main loop rate (independent of recording rate)

# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_W = 76   # total visible width including box borders (inner = 74)
# All 7 SO-101 servos map 1-to-1 to AIZEE joints.
# Servo 1 (shoulder_pan) → swivel, servos 2-7 → arm joints.
_LEADER_JOINTS = ["swivel", "gantry_base", "gantry_mid", "gantry_end",
                  "wrist_pitch", "wrist_roll", "gripper"]

# Per-joint torque saturation thresholds (Nm) — matches max_torque in hardware_jetson_rover.yaml
_SAT_TORQUE = {
    "swivel":      12.0,   # RS03 nominal
    "gantry_base": 24.0,   # RS04 nominal
    "gantry_mid":  12.0,   # RS03 nominal
    "gantry_end":   5.0,   # RS02 nominal
    "wrist_pitch":  5.0,   # RS02 nominal
    "wrist_roll":   0.5,   # RS00 nominal
    "gripper":      0.5,   # RS00 nominal
}

# UPS voltage thresholds (V)
_UPS_OK   = 11.7
_UPS_WARN = 10.8
_UPS_CRIT = 10.0

# ANSI color codes (enabled on Windows via _ansi_on())
_GRN = "\033[1;32m"
_YEL = "\033[1;33m"
_RED = "\033[1;31m"
_RST = "\033[0m"
_BG_YEL = "\033[103m"   # bright yellow background (warnings)
_BG_RED  = "\033[101m"  # bright red background (critical)

_TEMP_WARN = 65.0   # °C — yellow background
_TEMP_CRIT = 80.0   # °C — red background
_VBUS_WARN = 20.0   # V  — yellow background
_VBUS_CRIT = 18.0   # V  — red background


def _render(
    leader_rad:      Optional[np.ndarray],
    target:          Optional[np.ndarray],
    actual:          Optional[np.ndarray],
    status:          str,
    hint:            str,
    robot_ok:        bool  = False,
    telem_age:       float = 999.0,
    ups_data:        Optional[dict] = None,
    clamped:         Optional[list[bool]] = None,
    torque:          Optional[np.ndarray] = None,
    temp:            Optional[np.ndarray] = None,
    battery_voltage: Optional[float]      = None,
) -> list[str]:
    _IW = _W - 2   # inner visible width (74)

    TOP = "\u2554" + "\u2550" * _IW + "\u2557"   # ╔═══╗
    MID = "\u2560" + "\u2550" * _IW + "\u2563"   # ╠═══╣
    BOT = "\u255a" + "\u2550" * _IW + "\u255d"   # ╚═══╝
    SEP = "\u2551  " + "\u2500" * (_IW - 4) + "  \u2551"  # ║  ───  ║

    def _row(text: str, vis: int = -1) -> str:
        """Wrap text in box borders, padding to _IW visible chars."""
        vlen = len(text) if vis < 0 else vis
        return "\u2551" + text + " " * max(0, _IW - vlen) + "\u2551"

    # Title line — status right-aligned
    # "  SO-101 → AIZEE Teleop" is 23 visible chars (→ is single-width)
    title_vis = 23
    gap = max(1, _IW - title_vis - len(status))
    title_line = _row(
        f"  SO-101 \u2192 AIZEE Teleop{' ' * gap}{status}",
        title_vis + gap + len(status),
    )

    # Column header  (leader col is 9 wide: 8 value + 1 clamp flag)
    header_line = _row(
        f"  {'joint':<18} {'leader':>9}  {'target':>8}  {'actual':>8}   {'err':>7}  {'torq':>5}  {'temp':>4}"
    )

    # Joint data rows
    joint_lines = []
    for i, jname in enumerate(_LEADER_JOINTS):
        is_clamped = (clamped is not None and i < len(clamped) and clamped[i])
        if leader_rad is not None:
            clamp_flag = f"{_YEL}!{_RST}" if is_clamped else " "
            l_s  = f"{float(leader_rad[i]):>+8.3f}{clamp_flag}"
        else:
            l_s  = "      -- "
        l_vis = 9   # 8-char value + 1 flag/space  (format: >+8.3f + clamp or padding)
        t_ok = target is not None and not np.isnan(target[i])
        a_ok = actual is not None and not np.isnan(actual[i])
        t_s  = f"{float(target[i]):>+8.3f}"             if t_ok else "      --"
        a_s  = f"{float(actual[i]):>+8.3f}"             if a_ok else "      --"
        e_s  = f"{float(target[i] - actual[i]):>+7.3f}" if (t_ok and a_ok) else "     --"
        tq_ok = torque is not None and i < len(torque) and not np.isnan(torque[i])
        if tq_ok:
            tq    = float(torque[i])
            thresh = _SAT_TORQUE.get(jname, 999.0)
            ratio  = abs(tq) / thresh if thresh > 0 else 0.0
            if ratio >= 0.85:
                tq_s = f"{_BG_RED}{tq:>+5.1f}{_RST}"
            elif ratio >= 0.60:
                tq_s = f"{_BG_YEL}{tq:>+5.1f}{_RST}"
            else:
                tq_s = f"{tq:>+5.1f}"
        else:
            tq_s = "   --"
        temp_ok = temp is not None and i < len(temp) and not np.isnan(temp[i])
        if temp_ok:
            tc = float(temp[i])
            if tc >= _TEMP_CRIT:
                temp_s = f"{_BG_RED}{tc:>3.0f}\u00b0{_RST}"
            elif tc >= _TEMP_WARN:
                temp_s = f"{_BG_YEL}{tc:>3.0f}\u00b0{_RST}"
            else:
                temp_s = f"{tc:>3.0f}\u00b0"
        else:
            temp_s = "  --"
        row_text = f"  {jname:<18} {l_s}  {t_s}  {a_s}   {e_s}  {tq_s}  {temp_s}"
        row_vis  = 2 + 18 + 1 + l_vis + 2 + 8 + 2 + 8 + 3 + 7 + 2 + 5 + 2 + 4
        joint_lines.append(_row(row_text, row_vis))

    # Robot / UPS status line
    if robot_ok and telem_age < 2.0:
        robot_text    = "robot: connected"
        robot_display = f"{_GRN}{robot_text}{_RST}"
    elif robot_ok:
        robot_text    = f"robot: stale {telem_age:.0f}s"
        robot_display = f"{_YEL}{robot_text}{_RST}"
    else:
        robot_text    = "robot: offline"
        robot_display = robot_text
    robot_pad = " " * max(2, 24 - len(robot_text))

    if ups_data:
        v   = float(ups_data.get("voltage",    0.0))
        c   = float(ups_data.get("current",    0.0))
        p   = float(ups_data.get("power",      0.0))
        pct = float(ups_data.get("percentage", 0.0))
        if   v >= _UPS_OK:   col, ups_st = _GRN, "OK"
        elif v >= _UPS_WARN: col, ups_st = _YEL, "WARN"
        elif v >= _UPS_CRIT: col, ups_st = _RED, "CRIT"
        else:                col, ups_st = _RED, "SHUTDOWN"
        ups_body = f"UPS  {v:.2f}V  {c:.2f}A  {p:.1f}W  ({pct:.0f}%)"
        ups_line = f"{ups_body}  {col}[{ups_st}]{_RST}"
        ups_vis  = len(ups_body) + 2 + 1 + len(ups_st) + 1   # "  [STATUS]"
    else:
        ups_line = "UPS  --"
        ups_vis  = 7

    if battery_voltage is not None:
        vbus_v = battery_voltage
        if vbus_v < _VBUS_CRIT:
            vbus_s = f"  {_BG_RED}VBUS {vbus_v:.1f}V{_RST}"
        elif vbus_v < _VBUS_WARN:
            vbus_s = f"  {_BG_YEL}VBUS {vbus_v:.1f}V{_RST}"
        else:
            vbus_s = f"  VBUS {vbus_v:.1f}V"
        vbus_vis = 2 + 5 + len(f"{vbus_v:.1f}") + 1
    else:
        vbus_s, vbus_vis = "", 0

    robot_line = _row(
        f"  {robot_display}{robot_pad}{ups_line}{vbus_s}",
        2 + len(robot_text) + len(robot_pad) + ups_vis + vbus_vis,
    )

    # Controls / hint line
    ctrl_text = hint if hint else "Q quit"
    ctrl_line = _row(f"  {ctrl_text}")

    return [
        TOP,
        title_line,
        MID,
        header_line,
        SEP,
        *joint_lines,
        SEP,
        robot_line,
        MID,
        ctrl_line,
        BOT,
    ]


_N = len(_render(None, None, None, "", "", clamped=None))


def _draw(lines: list[str], first: bool = False) -> None:
    if not first:
        sys.stdout.write(f"\033[{_N}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Gamepad helpers
# ---------------------------------------------------------------------------

def _init_joystick():
    """Initialize pygame and return first usable joystick, or None."""
    if not _pygame_available:
        return None
    try:
        import os
        if sys.platform == "win32":
            os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "-10000,-10000")
        pygame.init()
        if sys.platform == "win32":
            pygame.display.set_mode((1, 1))
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            js = pygame.joystick.Joystick(i)
            js.init()
            if "keyboard" in js.get_name().lower():
                continue
            if js.get_numaxes() >= 2:
                return js
    except Exception:
        pass
    return None


def _read_gamepad(joystick, prev_a: bool, prev_b: bool, prev_start: bool) -> dict:
    """Poll gamepad buttons and return edge-detected events."""
    try:
        pygame.event.pump()
        raw_a     = bool(joystick.get_button(0))
        raw_b     = bool(joystick.get_button(1))
        raw_back  = bool(joystick.get_button(6))
        raw_start = bool(joystick.get_button(7))
        return {
            "enable":    raw_a and not prev_a,
            "shutdown":  raw_b and not prev_b,
            "hold":      raw_start and not prev_start,
            "quit":      raw_back,
            "raw_a":     raw_a,
            "raw_b":     raw_b,
            "raw_start": raw_start,
        }
    except Exception:
        return {
            "enable": False, "shutdown": False, "hold": False, "quit": False,
            "raw_a": False, "raw_b": False, "raw_start": False,
        }


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def _drain(sock) -> Optional[dict]:
    latest = None
    while True:
        try:
            latest = json.loads(sock.recv_string(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def _send(sock, msg: dict) -> None:
    try:
        sock.send_string(json.dumps(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


def _qtorque(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract arm joint torques from telemetry (6-elem, NaN where motor absent)."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("torque", _nan)) if j in motors else _nan for j in ARM_JOINTS],
        dtype=np.float32,
    )


def _qtemp(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract arm joint temperatures (6-elem, NaN where absent)."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("temperature", _nan)) if j in motors else _nan
         for j in ARM_JOINTS], dtype=np.float32)


def _qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract arm joint positions from telemetry.

    Returns None only if telemetry is absent entirely.  If individual motors
    are missing (e.g. failed to enable), their position is set to 0.0 so the
    rest of the joints still show actual data.  Missing motors will appear as
    state="error" in the Rust telemetry rather than being absent from the dict.
    """
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None   # no arm motors at all
    out = []
    for j in ARM_JOINTS:
        m = motors.get(j)
        out.append(float(m.get("position", 0.0)) if m is not None else 0.0)
    return np.array(out, dtype=np.float32)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _load_teleop_yaml() -> dict:
    """Return parsed teleop.yaml, or {} on failure."""
    here = Path(__file__).parent
    for candidate in [
        here / ".." / ".." / "config" / "teleop.yaml",
        Path("config") / "teleop.yaml",
    ]:
        p = candidate.resolve()
        if p.exists():
            return yaml.safe_load(p.read_text()) or {}
    return {}


def _load_endpoints() -> dict:
    return _load_teleop_yaml().get("endpoints", {})


def main() -> None:
    _ep = _load_endpoints()
    ap = argparse.ArgumentParser(
        description="SO-101 leader arm teleop for the AIZEE arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",      required=True,                          help="SO-101 serial port")
    ap.add_argument("--baud",      type=int,  default=1_000_000)
    ap.add_argument("--calib",     default=str(CALIB_PATH),               help="Calibration JSON")
    ap.add_argument("--cmd",       default=_ep.get("command",       "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem",     default=_ep.get("telemetry",     "tcp://192.168.0.27:5556"))
    ap.add_argument("--ups",       default=_ep.get("ups_telemetry", "tcp://192.168.0.27:5562"),
                    help="UPS telemetry address (empty string to disable)")
    ap.add_argument("--max-delta",     type=float, default=0.3, dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.3)")
    ap.add_argument("--robstride-calib", default=None, dest="robstride_calib",
                    help="Path to robstride_calibration.json (default: auto-discover)")
    ap.add_argument("--align-margin",  type=float, default=0.05, dest="align_margin",
                    help="Max per-joint error [rad] to be considered aligned (default 0.05)")
    ap.add_argument("--align-time",    type=float, default=3.0,  dest="align_time",
                    help="Seconds to hold within margin before tracking begins (default 3.0)")
    ap.add_argument("--teleop-pub",    default="tcp://*:5570",   dest="teleop_pub",
                    help="ZMQ PUB endpoint for teleop state (rerun companion); empty to disable")
    args = ap.parse_args()

    _ansi_on()

    # --- SO-101 leader arm ---
    leader = So101Leader(args.port, args.baud, calib=args.calib)
    if not leader.connect():
        sys.exit(1)

    calib_present = Path(args.calib).exists()
    print(f"SO-101 connected on {args.port}")
    print(f"Calibration: {'loaded from ' + args.calib if calib_present else 'NONE — raw ticks->rad (run so101_calibrate.py first)'}")

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    print(f"Arm limits: {'loaded (' + str(len(arm_limits)) + ' joints)' if arm_limits else 'none — run robstride_calibrate.py first'}")

    # Load gains from teleop.yaml (same source as teleop.py).
    # record_replay.py KP/KD are stale 7-elem arrays with wrong wrist values — don't use them.
    _yaml = _load_teleop_yaml()
    _tcfg = _yaml.get("gantry", {})
    _kp: list[float] = _tcfg.get("kp", KP)
    _kd: list[float] = _tcfg.get("kd", KD)
    print(f"Arm gains: kp={_kp}  kd={_kd}")
    _dcfg = _yaml.get("drive", {})
    _swivel_kp: float = float(_dcfg.get("swivel_kp", 80.0))
    _swivel_kd: float = float(_dcfg.get("swivel_kd", 5.0))
    print(f"Swivel gains: kp={_swivel_kp}  kd={_swivel_kd}")

    # --- Leader reader thread ---
    # Runs leader.poll() continuously so serial I/O never blocks the main loop.
    _lr_lock   = threading.Lock()
    _lr_latest: dict = {"rad": None, "clamped": None}

    def _leader_reader(stop: threading.Event) -> None:
        while not stop.is_set():
            r = leader.poll()
            with _lr_lock:
                _lr_latest["rad"]     = r
                _lr_latest["clamped"] = (leader.clamped_joints
                                         if r is not None else _lr_latest["clamped"])

    _lr_stop   = threading.Event()
    _lr_thread = threading.Thread(target=_leader_reader, args=(_lr_stop,), daemon=True)
    _lr_thread.start()

    # Indices into the 7-elem leader array that map to ARM_JOINTS.
    # Servo 1 (swivel) is at index 0 of AIZEE_JOINTS but is not an arm joint,
    # so it is skipped.  Result: [1, 2, 3, 4, 5, 6].
    _arm_joint_set = set(ARM_JOINTS)
    _so101_for_aizee: list[int] = [
        i for i, j in enumerate(leader.AIZEE_JOINTS) if j in _arm_joint_set
    ]

    # Per-joint zero offset and direction — loaded from calibration, updated by Z key.
    # target = directions * (leader_rad - zero_offsets)
    zero_offsets: np.ndarray = leader.zero_offsets
    directions:   np.ndarray = leader.directions

    # --- ZMQ ---
    ctx        = zmq.Context()
    cmd_sock   = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 2)   # never block — drop stale commands
    cmd_sock.setsockopt(zmq.LINGER,  0)  # don't wait on close
    cmd_sock.connect(args.cmd)
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.setsockopt(zmq.CONFLATE, 1)
    telem_sock.connect(args.telem)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    ups_sock: Optional[zmq.Socket] = None
    if args.ups:
        ups_sock = ctx.socket(zmq.SUB)
        ups_sock.setsockopt(zmq.LINGER, 0)
        ups_sock.setsockopt(zmq.CONFLATE, 1)
        ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        ups_sock.connect(args.ups)
    teleop_pub_sock: Optional[zmq.Socket] = None
    if args.teleop_pub:
        teleop_pub_sock = ctx.socket(zmq.PUB)
        teleop_pub_sock.setsockopt(zmq.SNDHWM, 2)
        teleop_pub_sock.setsockopt(zmq.LINGER, 0)
        teleop_pub_sock.bind(args.teleop_pub)
        print(f"Teleop state publisher bound on {args.teleop_pub}")

    get_key = setup_keyboard()

    # Seed actual from first telemetry packet
    q_actual: Optional[np.ndarray] = None
    for _ in range(40):
        telem = _drain(telem_sock)
        if telem:
            q = _qpos(telem)
            if q is not None:
                q_actual = q
                break
        time.sleep(0.05)

    # ---------------------------------------------------------------------------
    # State machine
    # ---------------------------------------------------------------------------
    class State(enum.Enum):
        READY    = "ready"
        IDLE     = "idle"       # enabled, kp=0/kd=0, no tracking — see actual positions
        ALIGNING = "aligning"   # enabled, slowly moving arm to match leader
        TRACKING = "tracking"   # following leader in real time
        HOLD     = "hold"       # target frozen at last actual
        SHUTDOWN = "shutdown"   # hold 1 s, then slowly return to zero

    teleop_state                   = State.READY
    converge_start: Optional[float] = None   # when arm first entered margin
    held_target:    Optional[np.ndarray] = None
    zero_msg:       str   = ""               # status text for zero-capture flash
    zero_msg_until: float = 0.0              # show zero_msg until this time
    shutdown_countdown: float              = 0.0   # seconds remaining to hold
    shutdown_target:    Optional[np.ndarray] = None  # frozen arm positions at shutdown start
    shutdown_swivel:    Optional[float]      = None  # frozen swivel position at shutdown start
    shutdown_zero_since: float              = 0.0   # when ramp first hit zero
    _SHUTDOWN_TIMEOUT                       = 3.0   # force-disable after this many seconds at zero
    swivel_actual:      Optional[float]      = None  # latest swivel position from telemetry
    swivel_torque:      Optional[float]      = None  # latest swivel torque from telemetry
    swivel_temp:        Optional[float]      = None  # latest swivel temperature from telemetry
    arm_torques:        Optional[np.ndarray] = None  # latest arm joint torques (6-elem)
    arm_temps:          Optional[np.ndarray] = None  # latest arm joint temperatures (6-elem)
    held_swivel:        Optional[float]      = None  # swivel position frozen in HOLD
    last_telem_time: float = time.time() if q_actual is not None else 0.0
    ups_data:           Optional[dict]       = None
    battery_voltage:    Optional[float]      = None
    robot_ok = q_actual is not None
    joystick           = _init_joystick() if _pygame_available else None
    prev_gamepad_a:    bool = False
    prev_gamepad_b:    bool = False
    prev_gamepad_start:bool = False

    status = "[ ] ready"
    hint   = "E enable · I idle · Z zero · M mirror · Q quit"

    # Initial draw — q_actual is 6-elem; prepend NaN for swivel slot
    _nan = float("nan")
    _init_actual = (np.concatenate([[_nan], q_actual]) if q_actual is not None else None)
    _draw(_render(None, None, _init_actual, status, hint, robot_ok, 999.0, None, clamped=None), first=True)

    period = 1.0 / TELEOP_HZ

    try:
        while True:
            t0 = time.time()

            # --- Gamepad ---
            key = get_key()
            if joystick is not None:
                gp = _read_gamepad(joystick, prev_gamepad_a, prev_gamepad_b, prev_gamepad_start)
                prev_gamepad_a     = gp["raw_a"]
                prev_gamepad_b     = gp["raw_b"]
                prev_gamepad_start = gp["raw_start"]
                if gp["enable"] and teleop_state in (State.READY, State.IDLE):
                    key = "E"
                if gp["hold"] and teleop_state in (State.ALIGNING, State.TRACKING, State.HOLD):
                    key = "H"
                if gp["shutdown"]:
                    if teleop_state == State.SHUTDOWN:
                        key = "CANCEL_SHUTDOWN"
                    elif teleop_state in (State.ALIGNING, State.TRACKING, State.HOLD, State.IDLE):
                        key = "X"
                if gp["quit"]:
                    key = "Q"

            # --- Keyboard ---
            if key == "Q":
                break

            elif key == "I":
                if teleop_state in (State.READY, State.IDLE):
                    _send(cmd_sock, {"type": "enable", "motor_ids": ["swivel"] + ARM_JOINTS})
                    ref = q_actual if q_actual is not None else [0.0] * len(ARM_JOINTS)
                    ref_list = ref.tolist() if hasattr(ref, "tolist") else list(ref)
                    _send(cmd_sock, {
                        "type":       "arm_joints",
                        "positions":  ref_list,
                        "velocities": [0.0] * len(ARM_JOINTS),
                        "kp":         [0.0] * len(ARM_JOINTS),
                        "kd":         [0.0] * len(ARM_JOINTS),
                        "torques":    [0.0] * len(ARM_JOINTS),
                    })
                    teleop_state = State.IDLE

            elif key == "E":
                # Enable arm motors and enter alignment phase
                _send(cmd_sock, {"type": "enable", "motor_ids": ["swivel"] + ARM_JOINTS})
                teleop_state   = State.ALIGNING
                converge_start = None


            elif key == "H":
                if teleop_state in (State.TRACKING, State.ALIGNING):
                    # Freeze target at current actual position
                    if q_actual is not None:
                        held_target = q_actual.copy()
                    held_swivel  = swivel_actual
                    teleop_state = State.HOLD
                elif teleop_state == State.HOLD:
                    # Return to alignment before tracking resumes
                    teleop_state   = State.ALIGNING
                    converge_start = None

            elif key == "Z":
                # Capture current SO-101 positions as new zero reference.
                with _lr_lock:
                    _z = _lr_latest["rad"]
                if _z is not None:
                    zero_offsets = _z.copy()
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[Z] zeroed — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "M":
                # Mirror: set zero so current SO-101 pose maps to current AIZEE actual.
                # q_actual is 6-elem (arm); zero_offsets/directions are 7-elem (SO-101).
                # Arm joints: zero_offsets[s] = _m[s] - directions[s] * q_actual[j]
                # Swivel (index 0): zero_offsets[0] = _m[0] - directions[0] * swivel_actual
                with _lr_lock:
                    _m = _lr_latest["rad"]
                if _m is not None and q_actual is not None:
                    new_offsets = zero_offsets.copy()
                    for aizee_j, so101_i in enumerate(_so101_for_aizee):
                        new_offsets[so101_i] = _m[so101_i] - directions[so101_i] * q_actual[aizee_j]
                    if swivel_actual is not None:
                        new_offsets[0] = _m[0] - directions[0] * swivel_actual
                    zero_offsets = new_offsets
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[M] mirrored — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "CANCEL_SHUTDOWN" and teleop_state == State.SHUTDOWN:
                teleop_state = State.HOLD
                held_target  = q_actual.copy() if q_actual is not None else held_target
                held_swivel  = swivel_actual

            elif key == "X":
                # Soft shutdown: hold for 1 s, then slowly return to zero
                if teleop_state in (State.ALIGNING, State.TRACKING, State.HOLD, State.IDLE):
                    shutdown_target    = (q_actual.copy() if q_actual is not None
                                          else held_target.copy() if held_target is not None
                                          else None)
                    shutdown_swivel    = (swivel_actual if swivel_actual is not None
                                          else held_swivel if held_swivel is not None
                                          else None)
                    shutdown_countdown  = 1.0
                    shutdown_zero_since = 0.0
                    teleop_state        = State.SHUTDOWN

            # --- Read SO-101 (from background reader thread) ---
            with _lr_lock:
                leader_rad    = _lr_latest["rad"]
                _clamped_live = _lr_latest["clamped"]

            # --- Apply per-joint zero offset + direction ---
            # mapped_rad (7-elem, AIZEE space) = directions * (leader_rad - zero_offsets)
            mapped_rad: Optional[np.ndarray] = (
                directions * (leader_rad - zero_offsets)
                if leader_rad is not None else None
            )
            # aizee_cmd: 6-elem command for Rust arm (skips swivel at index 0)
            aizee_cmd: Optional[np.ndarray] = (
                mapped_rad[_so101_for_aizee] if mapped_rad is not None else None
            )
            # swivel_cmd: scalar command for Rust swivel motor (index 0)
            swivel_cmd: Optional[float] = (
                float(mapped_rad[0]) if mapped_rad is not None else None
            )

            # --- Determine targets ---
            if teleop_state == State.HOLD:
                target       = held_target
                swivel_tgt   = held_swivel
            elif aizee_cmd is not None:
                target       = aizee_cmd
                swivel_tgt   = swivel_cmd
            else:
                target       = q_actual      # no leader data — hold current actual
                swivel_tgt   = swivel_actual

            # --- Send arm + swivel commands ---
            if teleop_state == State.SHUTDOWN:
                dt = period   # fixed timestep
                SHUTDOWN_SPEED = 0.2   # rad/s
                max_change = SHUTDOWN_SPEED * dt
                if shutdown_countdown > 0:
                    # Phase 1: hold position for 1 s
                    shutdown_countdown -= dt
                    if shutdown_target is not None:
                        _send(cmd_sock, {
                            "type":       "arm_joints",
                            "positions":  shutdown_target.tolist(),
                            "velocities": [0.0] * len(ARM_JOINTS),
                            "kp":         _kp,
                            "kd":         _kd,
                            "torques":    [0.0] * len(ARM_JOINTS),
                        })
                    if shutdown_swivel is not None:
                        _send(cmd_sock, {"type": "swivel", "position": shutdown_swivel,
                                         "kp": _swivel_kp, "kd": _swivel_kd})
                else:
                    # Phase 2: slowly move each joint toward zero at 0.2 rad/s
                    if shutdown_target is None:
                        shutdown_target = np.zeros(len(ARM_JOINTS), dtype=np.float32)
                    ref = q_actual if q_actual is not None else shutdown_target
                    new_target = shutdown_target.copy()
                    for i in range(len(new_target)):
                        if abs(new_target[i]) < max_change:
                            new_target[i] = 0.0
                        else:
                            new_target[i] -= np.sign(new_target[i]) * max_change
                    shutdown_target = new_target
                    # Delta-clamp against actual for safety
                    delta = np.clip(shutdown_target - ref, -args.max_delta, args.max_delta)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits), dtype=np.float32)
                    _send(cmd_sock, {
                        "type":       "arm_joints",
                        "positions":  q_cmd.tolist(),
                        "velocities": [0.0] * len(ARM_JOINTS),
                        "kp":         _kp,
                        "kd":         _kd,
                        "torques":    [0.0] * len(ARM_JOINTS),
                    })
                    # Swivel: step toward zero
                    if shutdown_swivel is None:
                        shutdown_swivel = 0.0
                    if abs(shutdown_swivel) < max_change:
                        shutdown_swivel = 0.0
                    else:
                        shutdown_swivel -= np.sign(shutdown_swivel) * max_change
                    _send(cmd_sock, {"type": "swivel", "position": shutdown_swivel,
                                     "kp": _swivel_kp, "kd": _swivel_kd})
                    ramp_done = (np.all(np.abs(shutdown_target) < 0.01)
                                 and abs(shutdown_swivel) < 0.01)
                    if ramp_done and shutdown_zero_since == 0.0:
                        shutdown_zero_since = t0
                    actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
                    timed_out    = (shutdown_zero_since > 0
                                    and t0 - shutdown_zero_since >= _SHUTDOWN_TIMEOUT)
                    if ramp_done and (actual_close or timed_out):
                        _send(cmd_sock, {"type": "disable", "motor_ids": ["swivel"] + ARM_JOINTS})
                        teleop_state = State.READY

            elif teleop_state == State.IDLE:
                # Send kp=0/kd=0 continuously to keep Rust watchdog alive
                if q_actual is not None:
                    _send(cmd_sock, {
                        "type":       "arm_joints",
                        "positions":  q_actual.tolist(),
                        "velocities": [0.0] * len(ARM_JOINTS),
                        "kp":         [0.0] * len(ARM_JOINTS),
                        "kd":         [0.0] * len(ARM_JOINTS),
                        "torques":    [0.0] * len(ARM_JOINTS),
                    })
                if swivel_actual is not None:
                    _send(cmd_sock, {"type": "swivel", "position": swivel_actual,
                                     "kp": _swivel_kp, "kd": _swivel_kd})

            elif teleop_state != State.READY:
                # Normal command sending
                if target is not None:
                    ref   = q_actual if q_actual is not None else target
                    delta = np.clip(target - ref, -args.max_delta, args.max_delta)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits), dtype=np.float32)
                    _send(cmd_sock, {
                        "type":       "arm_joints",
                        "positions":  q_cmd.tolist(),
                        "velocities": [0.0] * len(ARM_JOINTS),
                        "kp":         _kp,
                        "kd":         _kd,
                        "torques":    [0.0] * len(ARM_JOINTS),
                    })
                if swivel_tgt is not None:
                    _send(cmd_sock, {"type": "swivel", "position": swivel_tgt,
                                     "kp": _swivel_kp, "kd": _swivel_kd})

            # --- Alignment convergence check ---
            # Tracks how long max per-joint error < align_margin.
            # Auto-transitions to TRACKING once held for align_time seconds.
            if teleop_state == State.ALIGNING:
                if aizee_cmd is not None and q_actual is not None:
                    max_err = float(np.max(np.abs(q_actual - aizee_cmd)))
                    if max_err < args.align_margin:
                        if converge_start is None:
                            converge_start = t0          # just entered margin
                        elif t0 - converge_start >= args.align_time:
                            teleop_state   = State.TRACKING
                            converge_start = None
                    else:
                        converge_start = None            # diverged — reset timer
                else:
                    converge_start = None                # can't check without data

            # --- Telemetry ---
            telem = _drain(telem_sock)
            q_new = _qpos(telem)
            if q_new is not None:
                q_actual        = q_new
                robot_ok        = True
                last_telem_time = t0
            if telem and "motors" in telem:
                swivel_m = telem["motors"].get("swivel")
                if swivel_m is not None:
                    swivel_actual = float(swivel_m.get("position", 0.0))
                    swivel_torque = float(swivel_m.get("torque", 0.0))
                    swivel_temp   = float(swivel_m.get("temperature", float("nan")))
                tq_new = _qtorque(telem)
                if tq_new is not None:
                    arm_torques = tq_new
                temp_new = _qtemp(telem)
                if temp_new is not None:
                    arm_temps = temp_new
            if telem:
                bv = telem.get("battery_voltage")
                if bv is not None:
                    battery_voltage = float(bv)

            if ups_sock is not None:
                ups_msg = _drain(ups_sock)
                if ups_msg and "ups" in ups_msg:
                    ups_data = ups_msg["ups"]

            # --- Build status + hint ---
            if teleop_state == State.READY:
                status = "[ ] ready"
                hint   = "E enable · I idle · Z zero · M mirror · Q quit"

            elif teleop_state == State.IDLE:
                status = "[I] idle — zero torque"
                hint   = "E track · H hold · X shutdown · Q quit"

            elif teleop_state == State.ALIGNING:
                if aizee_cmd is not None and q_actual is not None:
                    max_err = float(np.max(np.abs(q_actual - aizee_cmd)))
                    if converge_start is not None:
                        held_s = t0 - converge_start
                        status = f"[~] aligned  hold {held_s:.1f}/{args.align_time:.0f}s"
                    else:
                        status = f"[~] aligning  err {max_err:.3f} rad"
                else:
                    status = "[~] aligning..."
                hint = "H hold · X shutdown · Z/M zero · E re-align · Q quit"

            elif teleop_state == State.TRACKING:
                status = "[*] tracking" if leader_rad is not None else "[!] no leader data"
                hint   = "H hold · X shutdown · Z/M zero · E re-align · Q quit"

            elif teleop_state == State.HOLD:
                status = "[H] HOLD"
                hint   = "H resume · X shutdown · Z/M zero · Q quit"

            elif teleop_state == State.SHUTDOWN:
                if shutdown_countdown > 0:
                    status = f"[X] shutdown  hold {shutdown_countdown:.1f}s"
                else:
                    pct = 0
                    if shutdown_target is not None and q_actual is not None:
                        max_dist = float(np.max(np.abs(shutdown_target)))
                        if max_dist > 0.01:
                            pct = max(0, int(100 * (1.0 - max_dist / max(max_dist, 0.001))))
                    status = f"[X] returning to zero  {pct}%"
                hint = ""

            # Zero capture flash overrides status for 2 s
            if t0 < zero_msg_until:
                status = zero_msg

            # --- Render ---
            # Build 7-elem display arrays: index 0 = swivel, indices 1-6 = arm joints.
            # NaN signals missing data to _render() so it shows "--".
            # During SHUTDOWN, show the frozen/ramp target rather than the live leader.
            _nan = float("nan")
            if teleop_state == State.SHUTDOWN:
                _disp_arm    = shutdown_target
                _disp_swivel = shutdown_swivel
            else:
                _disp_arm    = target
                _disp_swivel = swivel_tgt
            if _disp_arm is not None:
                disp_target = np.concatenate([[_disp_swivel if _disp_swivel is not None else _nan], _disp_arm])
            else:
                disp_target = None
            if q_actual is not None:
                disp_actual = np.concatenate([[swivel_actual if swivel_actual is not None else _nan], q_actual])
            else:
                disp_actual = None
            telem_age = t0 - last_telem_time if robot_ok else 999.0
            _clamped = _clamped_live if leader_rad is not None else None
            if arm_torques is not None:
                disp_torque = np.concatenate(
                    [[swivel_torque if swivel_torque is not None else _nan], arm_torques]
                )
            else:
                disp_torque = None
            if arm_temps is not None:
                disp_temp = np.concatenate(
                    [[swivel_temp if swivel_temp is not None else _nan], arm_temps]
                )
            else:
                disp_temp = None
            _draw(_render(leader_rad, disp_target, disp_actual, status, hint,
                          robot_ok, telem_age, ups_data, clamped=_clamped,
                          torque=disp_torque, temp=disp_temp,
                          battery_voltage=battery_voltage))

            # --- Publish teleop state for Rerun companion ---
            if teleop_pub_sock is not None:
                def _jlist(arr):
                    if arr is None:
                        return [None] * 7
                    return [None if not np.isfinite(float(x)) else float(x) for x in arr]
                _send(teleop_pub_sock, {
                    "timestamp": t0,
                    "state":     teleop_state.value,
                    "leader":    _jlist(mapped_rad),
                    "target":    _jlist(disp_target),
                    "actual":    _jlist(disp_actual),
                    "torque":    _jlist(disp_torque),
                    "temp":      _jlist(disp_temp),
                })

            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        _lr_stop.set()
        _lr_thread.join(timeout=1.0)
        print("\nQuit.")
        leader.close()
        cmd_sock.close()
        telem_sock.close()
        if ups_sock is not None:
            ups_sock.close()
        if teleop_pub_sock is not None:
            teleop_pub_sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
