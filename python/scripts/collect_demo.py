#!/usr/bin/env python3
"""collect_demo.py — Motor control + ACT demo recorder for AIZEE arm.

Combines SO-101 teleoperation with demonstration recording.  Optionally
drives the arm via the SO-101 leader arm (--port).  Without --port you
still get full motor control for setup.

Usage:
    python collect_demo.py --port COM4
    python collect_demo.py --port /dev/ttyACM0 \\
        --cmd     tcp://192.168.0.27:5555 \\
        --telem   tcp://192.168.0.27:5556 \\
        --gripper-cam tcp://192.168.0.27:5563

Controls:
    E    enable arm motors (align to leader if --port given)
    I    idle — enable with zero torque (see actual positions)
    H    hold — freeze target at current actual position
    R    toggle recording (TRACKING only)
    X    soft shutdown — hold 1 s, return to zero, disable
    Z    zero — capture current SO-101 pose as zero reference
    M    mirror — set zero so current leader maps to current actual
    P    save current arm position as ready pose (config/ready_pose.json)
    K    mechanical zero — write Robstride hardware zero + SaveConfig to all
         arm joints (motors must be disabled; persists across power cycle)
    Q    quit  (Ctrl-C also works)
    WASD drive wheels (W=fwd S=back A=left D=right; wheels enable with arm)

Gamepad: A=enable  B=shutdown/cancel  Start=hold  Back=quit
         Left stick = drive (wheels enable with arm)

M5 Joystick2 (wired to OpenRB-150 leader, I2C 0x63 on D11/D12):
         Stick = drive (overrides WASD / xbox stick when deflected)
         Button = start/stop recording (no-op unless TRACKING or already recording)
"""

from __future__ import annotations

import argparse
import base64
import enum
import io
import json
import math
import queue
import socket
import sys
import threading
import time
import yaml
from pathlib import Path
from typing import Optional

import h5py
import numpy as np
import zmq
from PIL import Image

try:
    import cv2
    _cv2_available = True
except ImportError:
    _cv2_available = False

try:
    import pygame
    _pygame_available = True
except ImportError:
    _pygame_available = False

try:
    import rerun as rr
    import rerun.blueprint as rrb
    _rerun_available = True
except ImportError:
    _rerun_available = False

try:
    import serial as _serial
    _pyserial_available = True
except ImportError:
    _pyserial_available = False

_so101_available = False
try:
    sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
    from so101_leader import (
        So101Leader, CALIB_PATH as _CALIB_PATH, find_so101_port, _probe_so101,
    )
    _so101_available = True
except ImportError:
    _CALIB_PATH = Path("so101_calibration.json")

# Timeout-bounded serial open (never blocks forever on an unresponsive device
# such as a Bluetooth serial port).  Shared with the leader-arm probes.
try:
    from serial_safe import open_serial as _open_serial
except ImportError:
    _open_serial = None

# OpenRB-150 + Dynamixel XL330 leader (newer build). Same duck-typed interface
# as So101Leader, so the runtime code below is leader-kind agnostic once
# instantiated.
_leader_module_available = False
try:
    from leader import (
        find_any_leader, get_leader_class, default_calib_path,
        identify_port, LEADER_KINDS,
    )
    _leader_module_available = True
    # FF protocol constants (OpenRB only).  Imported once at top so the
    # FF-send block in the main loop stays cheap; getattr fallback keeps
    # this safe if an older openrb_leader.py is on the path.
    try:
        from openrb_leader import FF_MAX_CURRENT_MA, FF_DISABLE_SENTINEL
    except Exception:
        FF_MAX_CURRENT_MA   = 200
        FF_DISABLE_SENTINEL = -32768
except ImportError:
    LEADER_KINDS = ("so101",)
    FF_MAX_CURRENT_MA   = 200
    FF_DISABLE_SENTINEL = -32768

# Quest / WebXR leader (optional — kicks in only when --leader quest).
# Pulled in lazily because it imports IK + aiohttp + cryptography which the
# user may not have installed if they're sticking with the physical leader.
# Imported as a top-level module via the `python/teleop/` path entry that
# line 84 already added (same pattern as so101_leader / leader), not as a
# subpackage of `teleop` — `teleop/teleop.py` would shadow the namespace.
_quest_available = False
try:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    from quest_leader import QuestLeader, QuestLeaderConfig, make_quest_leader_class
    from web import SharedState, start_server_in_thread
    _quest_available = True
except ImportError as _quest_imp_err:
    _quest_imp_err_msg = str(_quest_imp_err)

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import (
    ARM_JOINTS, POLICY_JOINTS, KP, KD,
    setup_keyboard, load_arm_limits, clamp_arm_positions,
)

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_camera, unpack_msg

LOOP_HZ    = 30
REC_HZ     = 20
NUM_JOINTS = len(ARM_JOINTS)   # 7 (swivel + 6 gantry)

# Reduce GIL switch interval so background threads (camera JSON parsing,
# image decode, Rerun logging) yield to the main loop faster.  Default is
# 5 ms — a single large camera JSON parse can stall the main loop that long.
sys.setswitchinterval(0.001)   # 1 ms

# On Windows, time.sleep granularity defaults to ~15.6 ms, which inflates
# the 30 Hz period jitter (we've measured p99 leaking to 100+ ms).  Asking
# the multimedia timer for 1 ms resolution tightens the loop period to the
# OS scheduler floor.  No-op on non-Windows.
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------

_W  = 76
_IW = _W - 2

# Display layout — leader joints in the same swivel-first order as ARM_JOINTS.
_LEADER_JOINTS = list(ARM_JOINTS)

_BASE_MOTORS = ["left_wheel", "right_wheel"]
# Swivel is now part of ARM_JOINTS (joint 0), so no separate entry here.
_ALL_MOTORS  = _BASE_MOTORS + list(ARM_JOINTS)

# ---------------------------------------------------------------------------
# Motor ↔ URDF frame alignment
# ---------------------------------------------------------------------------
# `signs` is a real frame-direction correction — some motor encoders rotate
# opposite to the URDF joint axis (mounting / cabling artefact), and the
# control loop MUST account for that or commands move the wrong way.
#
# `offsets` is a PURELY VISUAL knob for the URDF mirror — used when the
# motor's mechanical zero doesn't visually match the URDF neutral pose,
# but the IK / control loop is otherwise correct.  Applying it in the
# boundary would actively offset the motor command and physically shove
# the joint — which is what just happened to gantry_end.  Offsets are
# forwarded to the browser via telemetry; scene.js / preview.html add
# them only when rendering the mesh.
#
# Boundary math (in collect_demo) is sign-only:
#     q_urdf_ctrl  = q_motor * sign        # both in URDF *control* frame
#     q_motor      = q_urdf_ctrl * sign    # sign² = 1
#     v_urdf       = v_motor * sign
#     τ_urdf       = τ_motor * sign
#
# Loaded from config/joint_align.json at startup; hot-reloaded via mtime.
_ALIGN_OFFSETS = np.zeros(NUM_JOINTS, dtype=np.float32)
_ALIGN_SIGNS   = np.ones(NUM_JOINTS,  dtype=np.float32)
_ALIGN_PATH    = Path(__file__).resolve().parents[2] / "config" / "joint_align.json"
_ALIGN_MTIME   = 0.0   # last mtime we loaded; reload-if-changed gate

def _load_joint_align() -> None:
    """Populate the module-level _ALIGN_* arrays from joint_align.json."""
    global _ALIGN_OFFSETS, _ALIGN_SIGNS, _ALIGN_MTIME
    if not _ALIGN_PATH.exists():
        return
    try:
        _ALIGN_MTIME = _ALIGN_PATH.stat().st_mtime
        data = json.loads(_ALIGN_PATH.read_text())
    except Exception as e:
        print(f"WARNING: joint_align.json parse failed: {e}", flush=True)
        return
    o = data.get("offsets")
    if isinstance(o, list) and len(o) >= NUM_JOINTS:
        _ALIGN_OFFSETS = np.asarray(o[:NUM_JOINTS], dtype=np.float32)
    s = data.get("signs")
    if isinstance(s, list) and len(s) >= NUM_JOINTS:
        _ALIGN_SIGNS = np.asarray(
            [1.0 if float(x) >= 0 else -1.0 for x in s[:NUM_JOINTS]],
            dtype=np.float32,
        )

def _maybe_reload_joint_align() -> bool:
    """Reload joint_align.json if its mtime has changed since last load.

    Cheap to call (one stat per invocation); intended to be hit from the
    main loop a few times per second so /preview Save edits take effect
    without restarting collect_demo.  Returns True if reloaded."""
    try:
        m = _ALIGN_PATH.stat().st_mtime if _ALIGN_PATH.exists() else 0.0
    except OSError:
        return False
    if m == _ALIGN_MTIME:
        return False
    _load_joint_align()
    print(f"[joint_align] reloaded: signs={_ALIGN_SIGNS.tolist()} "
          f"offsets={[round(float(x), 3) for x in _ALIGN_OFFSETS]}", flush=True)
    return True

def _push_visual_offsets_to_leader(leader) -> None:
    """If the active leader exposes set_visual_offsets (QuestLeader does),
    hand it the current _ALIGN_OFFSETS so its IK/FK operate in visual
    frame (= mesh frame).  Physical leaders ignore this."""
    fn = getattr(leader, "set_visual_offsets", None)
    if fn is not None:
        try:
            fn(_ALIGN_OFFSETS)
        except Exception as e:
            print(f"[joint_align] leader.set_visual_offsets failed: {e}", flush=True)

def _motor_to_urdf_pos(q_motor: np.ndarray) -> np.ndarray:
    return np.asarray(q_motor, dtype=np.float32) * _ALIGN_SIGNS

def _urdf_to_motor_pos(q_urdf: np.ndarray) -> np.ndarray:
    # signs are ±1 → 1/sign == sign
    return np.asarray(q_urdf, dtype=np.float32) * _ALIGN_SIGNS

def _flip_sign_vec(v: np.ndarray) -> np.ndarray:
    return np.asarray(v, dtype=np.float32) * _ALIGN_SIGNS

def _urdf_to_motor_arm_payload(arm: dict) -> dict:
    """Transform an arm_joints dict from URDF frame to motor frame.

    Returns a new dict — the input is not mutated (other callers may
    still hold a reference to it, e.g. holder['last_q_cmd'])."""
    if not arm or "positions" not in arm:
        return arm
    out = dict(arm)
    pos = np.asarray(arm["positions"], dtype=np.float32)
    out["positions"] = _urdf_to_motor_pos(pos).tolist()
    if "velocities" in arm and arm["velocities"] is not None:
        vel = np.asarray(arm["velocities"], dtype=np.float32)
        out["velocities"] = _flip_sign_vec(vel).tolist()
    if "torques" in arm and arm["torques"] is not None:
        tq = np.asarray(arm["torques"], dtype=np.float32)
        out["torques"] = _flip_sign_vec(tq).tolist()
    return out

_SAT_TORQUE = {
    "swivel":      12.0,   # RS03 nominal
    "gantry_base": 24.0,   # RS04 nominal
    "gantry_mid":  12.0,   # RS03 nominal
    "gantry_end":   5.0,   # RS02 nominal
    "wrist_pitch":  5.0,   # RS02 nominal
    "wrist_roll":   0.5,   # RS00 nominal
    "gripper":      0.5,   # RS00 nominal
}

_UPS_OK   = 11.7
_UPS_WARN = 10.8
_UPS_CRIT = 10.0

_GRN    = "\033[1;32m"
_YEL    = "\033[1;33m"
_RED    = "\033[1;31m"
_RST    = "\033[0m"
_BG_YEL = "\033[103m"
_BG_RED = "\033[101m"

_TEMP_WARN  = 65.0
_TEMP_CRIT  = 80.0
_VBUS_WARN  = 20.0
_VBUS_CRIT  = 18.0
_CAM_STALE  = 0.5


# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)
        # Set timer resolution to 1 ms so time.sleep() is accurate.
        # Without this, Windows quantizes sleep to ~15 ms (system tick),
        # turning a 30 Hz loop into ~22 Hz with periodic stutter.
        ctypes.windll.winmm.timeBeginPeriod(1)


def _render(
    leader_rad:       Optional[np.ndarray],
    target:           Optional[np.ndarray],
    actual:           Optional[np.ndarray],
    status:           str,
    hint:             str,
    robot_ok:         bool                = False,
    telem_age:        float               = 999.0,
    ups_data:         Optional[dict]      = None,
    clamped:          Optional[list]      = None,
    torque:           Optional[np.ndarray] = None,
    temp:             Optional[np.ndarray] = None,
    battery_voltage:  Optional[float]     = None,
    leader_connected: bool                = False,
    leader_age:       float               = 999.0,
    cam_age:          float               = 999.0,
    rec_steps:        int                 = 0,
    recording:        bool                = False,
    dropped:          int                 = 0,
    estop_active:     bool                = False,
    wheel_states:     Optional[dict]      = None,
    wheels_enabled:   bool                = False,
    drive_linear:     float               = 0.0,
    drive_angular:    float               = 0.0,
    **_ignored,
) -> list[str]:
    TOP = "\u2554" + "\u2550" * _IW + "\u2557"
    MID = "\u2560" + "\u2550" * _IW + "\u2563"
    BOT = "\u255a" + "\u2550" * _IW + "\u255d"
    SEP = "\u2551  " + "\u2500" * (_IW - 4) + "  \u2551"

    def _row(text: str, vis: int = -1) -> str:
        vlen = len(text) if vis < 0 else vis
        return "\u2551" + text + " " * max(0, _IW - vlen) + "\u2551"

    # Title
    title_txt = "  AIZEE Demo Collector"
    title_vis = len(title_txt)
    gap = max(1, _IW - title_vis - len(status))
    title_line = _row(
        f"{title_txt}{' ' * gap}{status}",
        title_vis + gap + len(status),
    )

    # Column header
    header_line = _row(
        f"  {'joint':<18} {'leader':>9}  {'target':>8}  {'actual':>8}   {'err':>7}  {'torq':>5}  {'temp':>4}"
    )

    # Joint rows
    joint_lines = []
    for i, jname in enumerate(_LEADER_JOINTS):
        is_clamped = clamped is not None and i < len(clamped) and clamped[i]
        if leader_rad is not None:
            flag = f"{_YEL}!{_RST}" if is_clamped else " "
            l_s  = f"{float(leader_rad[i]):>+8.3f}{flag}"
        else:
            l_s  = "      -- "
        l_vis = 9
        t_ok = target is not None and not np.isnan(target[i])
        a_ok = actual is not None and not np.isnan(actual[i])
        t_s  = f"{float(target[i]):>+8.3f}" if t_ok else "      --"
        a_s  = f"{float(actual[i]):>+8.3f}" if a_ok else "      --"
        e_s  = f"{float(target[i] - actual[i]):>+7.3f}" if (t_ok and a_ok) else "     --"
        tq_ok = torque is not None and i < len(torque) and not np.isnan(torque[i])
        if tq_ok:
            tq    = float(torque[i])
            ratio = abs(tq) / _SAT_TORQUE.get(jname, 999.0)
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

    # Wheel motor rows
    wheel_lines = []
    for wname in _BASE_MOTORS:
        wm = (wheel_states or {}).get(wname)
        if wm is not None:
            wst = wm.get("state", "?")
            if wst in ("running", "enabled"):
                st_s = f"{_GRN}{wst:<7}{_RST}"
            elif wst == "disabled":
                st_s = f"{wst:<7}"
            else:
                st_s = f"{_RED}{wst:<7}{_RST}"
            st_vis = 7
            wvel = wm.get("velocity")
            vel_s = f"{wvel:>+6.2f}" if wvel is not None else "    --"
            wtq  = wm.get("torque")
            tq_s = f"{wtq:>+5.1f}" if wtq is not None else "   --"
            wtmp = wm.get("temperature")
            if wtmp is not None:
                if wtmp >= _TEMP_CRIT:
                    tmp_s = f"{_BG_RED}{wtmp:>3.0f}\u00b0{_RST}"
                elif wtmp >= _TEMP_WARN:
                    tmp_s = f"{_BG_YEL}{wtmp:>3.0f}\u00b0{_RST}"
                else:
                    tmp_s = f"{wtmp:>3.0f}\u00b0"
            else:
                tmp_s = " --"
            wrow = f"  {wname:<16} {st_s}  vel:{vel_s}  trq:{tq_s}  {tmp_s}"
            wvis = 2 + 16 + 1 + st_vis + 2 + 4 + 6 + 2 + 4 + 5 + 2 + 4
        else:
            wrow = f"  {wname:<16} --"
            wvis = 2 + 16 + 1 + 2
        wheel_lines.append(_row(wrow, wvis))

    # Drive input indicator
    if wheels_enabled:
        dl_s = f"{drive_linear:>+5.2f}"
        da_s = f"{drive_angular:>+5.2f}"
        drive_row = f"  drive: lin {dl_s}  ang {da_s}   {_GRN}ON{_RST}  [WASD/stick]"
        drive_vis = len(f"  drive: lin {dl_s}  ang {da_s}   ON  [WASD/stick]")
    else:
        drive_row = f"  drive: OFF — enable arm to drive"
        drive_vis = -1  # auto
    wheel_lines.append(_row(drive_row, drive_vis))

    # Robot / UPS line
    if robot_ok and telem_age < 2.0:
        robot_txt  = "robot: connected"
        robot_disp = f"{_GRN}{robot_txt}{_RST}"
    elif robot_ok:
        robot_txt  = f"robot: stale {telem_age:.0f}s"
        robot_disp = f"{_YEL}{robot_txt}{_RST}"
    else:
        robot_txt  = "robot: offline"
        robot_disp = robot_txt
    rpad = " " * max(2, 24 - len(robot_txt))

    if ups_data:
        v   = float(ups_data.get("voltage",    0.0))
        c   = float(ups_data.get("current",    0.0))
        p   = float(ups_data.get("power",      0.0))
        pct = float(ups_data.get("percentage", 0.0))
        if v >= _UPS_OK:
            col, st = _GRN, "OK"
        elif v >= _UPS_WARN:
            col, st = _YEL, "WARN"
        elif v >= _UPS_CRIT:
            col, st = _RED, "CRIT"
        else:
            col, st = _RED, "SHUTDOWN"
        ups_body = f"UPS {v:.2f}V {c:.2f}A {p:.1f}W ({pct:.0f}%)"
        ups_disp = f"{ups_body}  {col}[{st}]{_RST}"
        ups_vis  = len(ups_body) + 2 + 1 + len(st) + 1
    else:
        ups_disp = "UPS --"
        ups_vis  = 6

    if battery_voltage is not None:
        bv = battery_voltage
        if bv < _VBUS_CRIT:
            vbus_s = f"  {_BG_RED}VBUS {bv:.1f}V{_RST}"
        elif bv < _VBUS_WARN:
            vbus_s = f"  {_BG_YEL}VBUS {bv:.1f}V{_RST}"
        else:
            vbus_s = f"  VBUS {bv:.1f}V"
        vbus_vis = 2 + 5 + len(f"{bv:.1f}") + 1
    else:
        vbus_s, vbus_vis = "", 0

    robot_line = _row(
        f"  {robot_disp}{rpad}{ups_disp}{vbus_s}",
        2 + len(robot_txt) + len(rpad) + ups_vis + vbus_vis,
    )

    # Leader status (left column of second status row)
    if leader_connected:
        if leader_age > 0.5:
            ldr_txt  = f"leader: STALE ({leader_age:.0f}s)"
            ldr_disp = f"leader: {_RED}STALE ({leader_age:.0f}s){_RST}"
        elif leader_age > 0.1:
            ms       = f"{leader_age * 1000:.0f}ms"
            ldr_txt  = f"leader: {ms}"
            ldr_disp = f"leader: {_YEL}{ms}{_RST}"
        else:
            ms       = f"{leader_age * 1000:.0f}ms"
            ldr_txt  = f"leader: OK ({ms})"
            ldr_disp = f"leader: {_GRN}OK{_RST} ({ms})"
    else:
        ldr_txt  = "leader: --"
        ldr_disp = "leader: --"
    ldr_pad = " " * max(2, 24 - len(ldr_txt))

    # Camera ages
    def _cam(age: float, lbl: str) -> tuple[str, int]:
        if age > _CAM_STALE:
            return f"{lbl}:{_RED}STALE{_RST}", len(lbl) + 1 + 5
        ms  = f"{age * 1000:.0f}ms"
        col = _YEL if age > 0.1 else ""
        rst = _RST if col else ""
        return f"{lbl}:{col}{ms}{rst}", len(lbl) + 1 + len(ms)

    g_s, g_v = _cam(cam_age, "G")
    cam_part = f"cam   {g_s}"
    cam_vis  = 6 + g_v

    # E-stop indicator
    if estop_active:
        estop_disp = f"{_BG_RED}E-STOP{_RST}"
        estop_vis  = 6
    else:
        estop_disp = f"{_GRN}SAFE{_RST}"
        estop_vis  = 4

    # Recording status
    if recording:
        dur     = rec_steps / REC_HZ
        rec_txt = f"REC {rec_steps:4d}  {dur:4.1f}s"
        if dropped:
            rec_txt += f"  drop:{dropped}"
        rec_disp = f"{_BG_RED}{rec_txt}{_RST}"
        rec_vis  = len(rec_txt)
    else:
        rec_disp = "IDLE"
        rec_vis  = 4

    status_line = _row(
        f"  {ldr_disp}{ldr_pad}{cam_part}  {estop_disp}  {rec_disp}",
        2 + len(ldr_txt) + len(ldr_pad) + cam_vis + 2 + estop_vis + 2 + rec_vis,
    )

    ctrl_line = _row(f"  {hint}" if hint else "  Q quit")

    return [
        TOP, title_line, MID,
        header_line, SEP, *joint_lines, SEP,
        *wheel_lines, SEP,
        robot_line, status_line,
        MID, ctrl_line, BOT,
    ]


_N = len(_render(None, None, None, "", ""))


def _draw(lines: list[str], first: bool = False) -> None:
    if not first:
        sys.stdout.write(f"\033[{_N}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()


def _start_display_thread() -> tuple[threading.Event, threading.Thread,
                                      threading.Lock, dict, threading.Event]:
    """Background thread for console rendering.

    Runs both _render() (string formatting, numpy concat) and _draw()
    (stdout write+flush) off the main loop.  Main loop passes a shallow
    dict of display values; this thread does all the work.
    """
    lock   = threading.Lock()
    holder: dict = {"args": None, "first": True}
    signal = threading.Event()
    stop   = threading.Event()

    def _run() -> None:
        while not stop.is_set():
            if not signal.wait(timeout=0.1):
                continue
            signal.clear()
            with lock:
                args  = holder["args"]
                first = holder["first"]
                if first:
                    holder["first"] = False
            if args is not None:
                lines = _render(**args)
                _draw(lines, first=first)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, holder, signal


def _start_rerun_thread(
    dec_lock: threading.Lock,
    dec_cache: dict,
) -> tuple[threading.Event, threading.Thread,
            threading.Lock, dict, threading.Event]:
    """Background thread for Rerun logging.

    Camera frames are pulled from the shared decoder cache (already-decoded
    uint8 RGB) — no JPEG decode or re-encode happens here.  rr.log() can
    still block 5-30 ms when the viewer's ingestion pipe backs up, hence
    the dedicated thread.
    """
    lock   = threading.Lock()
    holder: dict = {"time": 0.0, "joints": None, "leader": None}
    signal = threading.Event()
    stop   = threading.Event()
    prev_gt = 0.0

    def _run() -> None:
        nonlocal prev_gt
        while not stop.is_set():
            if not signal.wait(timeout=0.1):
                continue
            signal.clear()
            with lock:
                ts     = holder["time"]
                joints = holder["joints"]
                leader = holder["leader"]
                holder["joints"] = None
                holder["leader"] = None
            with dec_lock:
                gripper_img = dec_cache["gripper"]
                gripper_t   = dec_cache["gripper_time"]
            try:
                rr.set_time("time", timestamp=ts)
                if gripper_img is not None and gripper_t > prev_gt:
                    rr.log("cameras/gripper", rr.Image(gripper_img))
                    prev_gt = gripper_t
                if joints is not None:
                    for jname, val in joints.items():
                        rr.log(f"joints/{jname}", rr.Scalars(val))
                if leader is not None:
                    for jname, val in leader.items():
                        rr.log(f"leader/{jname}", rr.Scalars(val))
            except Exception:
                pass

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, holder, signal


def _start_image_decoder(
    cam_lock: threading.Lock,
    cam_cache: dict,
    img_size: tuple,
    always_on: bool = False,
    scene_proj_stride: int = 20,
    scene_proj_z_near: float = 0.15,
    scene_proj_z_far:  float = 3.0,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict, threading.Event]:
    """Background thread that decodes + resizes camera JPEGs.

    Gated on *rec_flag* by default (decoding only matters for the record
    buffer).  Pass always_on=True for GUI mode where live preview needs
    decoded frames even outside recording.

    Scene cam additionally has its depth backprojected to a CAMERA-FRAME
    pointcloud right here on the worker thread, so the GUI never pays
    that cost. The final pose transform (R·p + t to world frame) is the
    only remaining numpy work for the GUI side — that's a single matmul
    on ~700 points, sub-millisecond.

    Each `_dec_cache["{cam}"]` is paired with `_dec_cache["{cam}_ts_pub"]`
    (the publisher's capture timestamp of the same frame), so the recorder
    can pull image + ts atomically — eliminates the off-by-one-frame sync
    bug where `latest_*_ts` referred to a newer frame than the decoded
    ndarray. Critical for training-time alignment across streams.
    """
    lock     = threading.Lock()
    decoded: dict = {
        # gripper: image (np.ndarray) + ts_pub (publisher's capture ts) +
        # time (local arrival ts, used to drive the freshness check by
        # callers that already do age-based gating).
        "gripper": None, "gripper_time": 0.0, "gripper_ts_pub": None,
        # scene: same as gripper, plus pre-projected camera-frame point
        # cloud + per-point depth (for the GUI's 3D viz). All five fields
        # are written under the same lock acquisition so consumers get a
        # consistent snapshot of the same frame.
        "scene":            None,
        "scene_time":       0.0,
        "scene_ts_pub":     None,
        "scene_pts_cam":    None,   # (N, 3) float32, camera-frame XYZ
        "scene_pts_depth":  None,   # (N,)  float32, per-point Z for color ramp
    }
    rec_flag = threading.Event()
    stop     = threading.Event()

    def _scene_depth_to_cam_points(depth_blob: dict) -> tuple:
        """Backproject Z16 depth → (N, 3) camera-frame XYZ + (N,) depths.

        Returns (None, None) if the depth payload or intrinsics are
        missing — the decoder skips updating the pointcloud fields in
        that case, leaving the previous cache values intact.

        Camera optical frame here is the librealsense / OpenCV
        convention: +X right, +Y down, +Z forward. The GUI applies a
        single rotation+translation to land in URDF world frame.
        """
        raw   = depth_blob.get("data_bytes")
        intr  = depth_blob.get("intrinsics")
        scale = depth_blob.get("scale")
        w     = depth_blob.get("width")
        h     = depth_blob.get("height")
        if (raw is None or intr is None or scale is None
                or w is None or h is None):
            return None, None
        try:
            depth = np.frombuffer(raw, dtype=np.uint16).reshape(int(h), int(w))
        except Exception:
            return None, None
        fx = float(intr["fx"]); fy = float(intr["fy"])
        cx = float(intr["cx"]); cy = float(intr["cy"])
        us = np.arange(0, int(w), scene_proj_stride, dtype=np.int32)
        vs = np.arange(0, int(h), scene_proj_stride, dtype=np.int32)
        UU, VV = np.meshgrid(us, vs)
        z = depth[VV, UU].astype(np.float32) * float(scale)
        m = (z > scene_proj_z_near) & (z < scene_proj_z_far)
        if not m.any():
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, np.zeros((0,), dtype=np.float32)
        u = UU[m].astype(np.float32); v = VV[m].astype(np.float32)
        zf = z[m]
        X = (u - cx) / fx * zf
        Y = (v - cy) / fy * zf
        pts_cam = np.stack([X, Y, zf], axis=1).astype(np.float32)
        return pts_cam, zf

    def _run() -> None:
        prev_gt = 0.0
        prev_st = 0.0
        while not stop.is_set():
            if not (always_on or rec_flag.is_set()):
                stop.wait(0.05)
                continue

            with cam_lock:
                g_msg = cam_cache["gripper"]
                g_t   = cam_cache["gripper_time"]
                s_msg = cam_cache.get("scene")
                s_t   = cam_cache.get("scene_time", 0.0)

            changed = False
            if g_t > prev_gt and g_msg is not None:
                # Orientation correction is applied at the publisher
                # (config/hardware_jetson_gripper_cam.yaml → streams.color.flip),
                # so the consumer doesn't need to flip anything here.
                img = decode_image(g_msg, img_size)
                pub_ts = g_msg.get("timestamp")
                with lock:
                    decoded["gripper"]        = img
                    decoded["gripper_time"]   = g_t
                    decoded["gripper_ts_pub"] = (
                        float(pub_ts) if pub_ts is not None else None)
                prev_gt = g_t
                changed = True

            if s_t > prev_st and s_msg is not None:
                # Decode color + project depth, then commit both under one
                # lock acquisition so consumers (recorder, GUI push) see a
                # consistent (image, points, ts) triple from the same frame.
                img = decode_image(s_msg, img_size)
                depth_blob = s_msg.get("depth", {}) or {}
                pts_cam, pts_depth = _scene_depth_to_cam_points(depth_blob)
                pub_ts = s_msg.get("timestamp")
                with lock:
                    decoded["scene"]           = img
                    decoded["scene_time"]      = s_t
                    decoded["scene_ts_pub"]    = (
                        float(pub_ts) if pub_ts is not None else None)
                    if pts_cam is not None:
                        decoded["scene_pts_cam"]   = pts_cam
                        decoded["scene_pts_depth"] = pts_depth
                prev_st = s_t
                changed = True

            if not changed:
                stop.wait(0.005)   # 200 Hz poll when idle

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, decoded, rec_flag


# ---------------------------------------------------------------------------
# Main-loop profiler (per-section timing → log file, dump every 10 s)
# ---------------------------------------------------------------------------

class _LoopProfiler:
    def __init__(self, log_path: Optional[Path] = None) -> None:
        self._log = None
        if log_path is not None:
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self._log = open(log_path, "a", buffering=1, encoding="utf-8")
                self._log.write(f"\n=== profiler started {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            except Exception:
                self._log = None
        self._sec: dict[str, list[float]] = {}
        self._gauge: dict[str, list[float]] = {}
        self._work: list[float] = []
        self._period: list[float] = []
        self._t_sec: Optional[float] = None
        self._t_iter: Optional[float] = None
        self._t_prev: Optional[float] = None
        self._next_dump = time.perf_counter() + 10.0

    def begin(self) -> None:
        now = time.perf_counter()
        if self._t_prev is not None:
            self._period.append((now - self._t_prev) * 1000.0)
        self._t_prev = now
        self._t_iter = now
        self._t_sec = now

    def tick(self, name: str) -> None:
        if self._t_sec is None:
            return
        now = time.perf_counter()
        self._sec.setdefault(name, []).append((now - self._t_sec) * 1000.0)
        self._t_sec = now

    def gauge(self, name: str, value: float) -> None:
        self._gauge.setdefault(name, []).append(float(value))

    def end(self) -> None:
        if self._t_iter is None:
            return
        now = time.perf_counter()
        self._work.append((now - self._t_iter) * 1000.0)
        if now > self._next_dump:
            self.dump()
            self._next_dump = now + 10.0

    @staticmethod
    def _stats(arr: list[float]) -> Optional[tuple[float, float, float, float, int]]:
        if not arr:
            return None
        s = sorted(arr)
        n = len(s)
        return (sum(s) / n, s[n // 2], s[min(n - 1, int(n * 0.99))], s[-1], n)

    def dump(self) -> None:
        if self._log is None:
            self._sec.clear(); self._work.clear(); self._period.clear()
            return
        lines = [f"--- {time.strftime('%H:%M:%S')} ---"]
        for label, arr in (("period", self._period), ("work", self._work)):
            st = self._stats(arr)
            if st:
                lines.append(f"{label:8s} mean={st[0]:6.2f} p50={st[1]:6.2f} "
                             f"p99={st[2]:6.2f} max={st[3]:7.2f}  n={st[4]}")
        rows = []
        for name, arr in self._sec.items():
            st = self._stats(arr)
            if st:
                rows.append((st[3], st[2], st[0], name))
        rows.sort(reverse=True)
        for mx, p99, mean, name in rows:
            lines.append(f"  {name:14s} mean={mean:6.2f} p99={p99:6.2f} max={mx:7.2f}")
        for name, arr in self._gauge.items():
            st = self._stats(arr)
            if st:
                lines.append(f"  [g] {name:10s} mean={st[0]:6.2f} p50={st[1]:6.2f} "
                             f"p99={st[2]:6.2f} max={st[3]:7.2f}")
        self._log.write("\n".join(lines) + "\n")
        self._sec.clear(); self._gauge.clear()
        self._work.clear(); self._period.clear()


# ---------------------------------------------------------------------------
# Gamepad helpers
# ---------------------------------------------------------------------------

def _init_joystick():
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


def _apply_deadzone(value: float, deadzone: float = 0.08) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def _apply_curve(value: float, exponent: float = 1.5) -> float:
    sign = 1.0 if value >= 0 else -1.0
    return sign * (abs(value) ** exponent)


def _ramp_toward(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    diff = target - current
    rate = accel if abs(target) > abs(current) else decel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + max_change if diff > 0 else current - max_change


def _read_gamepad(joystick, prev_a: bool, prev_b: bool, prev_start: bool,
                  gp_cfg: Optional[dict] = None) -> dict:
    _empty = {
        "enable": False, "shutdown": False, "hold": False, "quit": False,
        "raw_a": False, "raw_b": False, "raw_start": False,
        "drive_linear": 0.0, "drive_angular": 0.0,
    }
    try:
        pygame.event.pump()
        raw_a     = bool(joystick.get_button(0))
        raw_b     = bool(joystick.get_button(1))
        raw_back  = bool(joystick.get_button(6))
        raw_start = bool(joystick.get_button(7))

        # Left stick axes for driving
        drive_linear  = 0.0
        drive_angular = 0.0
        if gp_cfg is not None:
            axes   = gp_cfg.get("axes", {})
            invert = gp_cfg.get("axis_invert", {})
            dz     = 0.08

            raw_y = joystick.get_axis(axes.get("left_stick_y", 1))
            if invert.get("left_stick_y", False):
                raw_y = -raw_y
            raw_x = joystick.get_axis(axes.get("left_stick_x", 0))
            if invert.get("left_stick_x", False):
                raw_x = -raw_x

            # WORKAROUND: motor controller has linear/angular backwards
            # Y-axis → angular (forward/back), X-axis → linear (turn)
            # Negate angular so stick-forward = robot-forward
            drive_angular = -_apply_curve(_apply_deadzone(raw_y, dz))
            drive_linear  = _apply_curve(_apply_deadzone(raw_x, dz))

        return {
            "enable":    raw_a and not prev_a,
            "shutdown":  raw_b and not prev_b,
            "hold":      raw_start and not prev_start,
            "quit":      raw_back,
            "raw_a":     raw_a,
            "raw_b":     raw_b,
            "raw_start": raw_start,
            "drive_linear":  drive_linear,
            "drive_angular": drive_angular,
        }
    except Exception:
        return _empty


# ---------------------------------------------------------------------------
# ZMQ helpers
# ---------------------------------------------------------------------------

def _drain(sock) -> Optional[dict]:
    latest = None
    while True:
        try:
            latest = unpack_msg(sock.recv(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


_cmd_sock_lock = threading.Lock()  # serialises cmd_sock.send across threads


def _send(sock, msg: dict) -> None:
    """Send one message.  Holds `_cmd_sock_lock` so direct sends from the
    main loop (enable/disable/emergency_stop/replay) don't race with the
    dedicated cmd-sender thread on the same PUSH socket.

    Boundary transform: `arm_joints` messages flow in URDF frame
    internally; convert to motor frame on the way out so callers don't
    need to remember to do it themselves.  `bundle` messages with an
    embedded arm_joints field get the same treatment."""
    if isinstance(msg, dict):
        if msg.get("type") == "arm_joints":
            msg = _urdf_to_motor_arm_payload(msg)
        elif msg.get("type") == "bundle" and isinstance(msg.get("arm_joints"), dict):
            msg = dict(msg)
            msg["arm_joints"] = _urdf_to_motor_arm_payload(msg["arm_joints"])
    try:
        with _cmd_sock_lock:
            sock.send(pack_msg(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


def _build_bundle(
    arm: Optional[dict] = None,
    drive: Optional[dict] = None,
) -> Optional[dict]:
    """Construct a Bundle message.  Returns None if both sub-payloads are None.

    Swivel is part of `arm` (joint 0) — there is no separate swivel field.
    """
    if arm is None and drive is None:
        return None
    msg: dict = {"type": "bundle"}
    if arm is not None:
        msg["arm_joints"] = arm
    if drive is not None:
        msg["drive"] = drive
    return msg


def _send_bundle(
    sock,
    arm: Optional[dict] = None,
    drive: Optional[dict] = None,
) -> None:
    """Send a Bundle once, synchronously.  For one-shots (live replay) and
    shutdown paths.  The 30 Hz teleop path goes through the dedicated
    cmd-sender thread instead — see `_start_cmd_sender`.

    `arm.positions` is expected in URDF frame; this function applies the
    inverse alignment transform on the way out so the Rust driver sees
    motor-frame values.  See _urdf_to_motor_arm_payload for the math."""
    if arm is not None:
        arm = _urdf_to_motor_arm_payload(arm)
    msg = _build_bundle(arm, drive)
    if msg is None:
        return
    try:
        with _cmd_sock_lock:
            sock.send(pack_msg(msg), zmq.NOBLOCK)
    except zmq.Again:
        pass


# ---------------------------------------------------------------------------
# Dedicated command sender thread
# ---------------------------------------------------------------------------
# Re-emits the latest queued Bundle at a higher rate than the 30 Hz main
# loop, so the controller's PD loop sees a fresh frame within ~10 ms even
# when the main loop is mid-tick.  Also pays the msgpack pack + ZMQ send
# cost off the main loop (used to be the dominant `motor` p99 spike).

_CMD_SEND_HZ = 100   # send cadence; 10 ms tick

# Lead-clamp is skipped when telem age exceeds this — see #3 in the
# main-loop lag analysis.  Otherwise a stale q_actual would pin the
# command behind the arm's actual progress and gate fast motion.
_LEAD_CLAMP_TELEM_FRESH_S = 0.10


def _start_cmd_sender(
    sock,
    lr_lock: threading.Lock,
    lr_latest: dict,
    lr_mapping: dict,
    telem_lock: threading.Lock,
    telem_cache: dict,
    hz: int = _CMD_SEND_HZ,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    """Dedicated cmd-sender thread.

    Two modes (set by main loop via the holder):

    * "static"   — re-emits a prebuilt Bundle dict every tick.  Used for
                   IDLE / HOLD / ENGAGING / SHUTDOWN, where the target is
                   either frozen or following a slow ramp the main loop
                   integrates at 30 Hz.

    * "tracking" — computes q_cmd live from `_lr_latest` at the cmd-thread
                   rate (100 Hz), so leader→motor latency is bounded by
                   ~10 ms instead of ~33 ms.  The rate limit is per-time
                   (`max_vel * dt`) rather than per-tick, and the lead-clamp
                   relaxes when telemetry is stale (see _LEAD_CLAMP_TELEM_FRESH_S).

    Holder shape (under the returned lock):
        {
          "mode":     "static" | "tracking" | None,
          "bundle":   {...}  | None,                   # used when mode=="static"
          "tracking": {kp, kd, max_vel, max_lead,
                       vel_ff, drive, arm_limits} | None,
        }
    """
    lock   = threading.Lock()
    # `last_q_cmd` is the current commanded arm pose — main loop reads it
    # for recording / display / shutdown-target seeding regardless of mode.
    holder: dict = {"mode": None, "bundle": None, "tracking": None,
                    "last_q_cmd": None}
    stop   = threading.Event()
    period = 1.0 / hz

    def _run() -> None:
        next_t = time.perf_counter() + period
        last_t: Optional[float] = None
        last_mode: Optional[str] = None
        last_q_cmd: Optional[np.ndarray] = None

        while not stop.is_set():
            now_pc = time.perf_counter()
            dt = (now_pc - last_t) if last_t is not None else period
            last_t = now_pc

            with lock:
                mode     = holder.get("mode")
                bundle   = holder.get("bundle")
                tracking = holder.get("tracking")

            # Reset the rate-limit integrator on any mode transition so the
            # first tick after entering "tracking" doesn't apply a step
            # accumulated from a prior tracking session.
            if mode != last_mode:
                last_q_cmd = None
                last_mode = mode

            msg: Optional[dict] = None

            if mode == "tracking" and tracking is not None:
                msg = _compute_tracking_bundle(
                    tracking, dt, last_q_cmd,
                    lr_lock, lr_latest, lr_mapping,
                    telem_lock, telem_cache,
                )
                if msg is not None:
                    # Pull the q_cmd back out of the bundle for the next
                    # tick's rate-limit integrator and for the main loop
                    # (recording / display).
                    arm = msg.get("arm_joints", {})
                    pos = arm.get("positions")
                    if pos is not None:
                        last_q_cmd = np.asarray(pos, dtype=np.float32)
                        with lock:
                            holder["last_q_cmd"] = last_q_cmd
            elif mode == "static" and bundle is not None:
                msg = bundle
                # Mirror the static bundle's arm positions into last_q_cmd
                # so callers that read `holder["last_q_cmd"]` see the same
                # value regardless of mode.
                arm = bundle.get("arm_joints") or {}
                pos = arm.get("positions")
                if pos is not None:
                    with lock:
                        holder["last_q_cmd"] = np.asarray(pos, dtype=np.float32)

            if msg is not None:
                # Boundary: arm_joints flows in URDF frame internally but the
                # Rust motor driver expects motor frame.  Apply the inverse
                # alignment transform on a SHALLOW COPY so holder['bundle']
                # and last_q_cmd stay in URDF frame for callers that read
                # them.  drive/other top-level fields pass through unchanged.
                arm = msg.get("arm_joints")
                if arm is not None:
                    msg = dict(msg)
                    msg["arm_joints"] = _urdf_to_motor_arm_payload(arm)
                try:
                    with _cmd_sock_lock:
                        sock.send(pack_msg(msg), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                except Exception:
                    pass

            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                stop.wait(sleep_t)
            next_t += period
            if next_t < time.perf_counter():
                next_t = time.perf_counter() + period

    thread = threading.Thread(target=_run, daemon=True, name="CmdTx")
    thread.start()
    return stop, thread, lock, holder


def _compute_tracking_bundle(
    cfg: dict,
    dt: float,
    last_q_cmd: Optional[np.ndarray],
    lr_lock: threading.Lock,
    lr_latest: dict,
    lr_mapping: dict,
    telem_lock: threading.Lock,
    telem_cache: dict,
) -> Optional[dict]:
    """Build a TRACKING-mode arm_joints bundle from the latest leader sample.

    Returns None when the leader has no sample yet, no mapping is
    installed, or the leader pose is too stale to trust.
    """
    # Snapshot leader + mapping under one lock acquisition.
    with lr_lock:
        leader_rad     = lr_latest.get("rad")
        leader_vel     = lr_latest.get("vel")
        leader_t       = lr_latest.get("time", 0.0)
        zero_offsets   = lr_mapping.get("zero_offsets")
        directions     = lr_mapping.get("directions")
        for_aizee      = lr_mapping.get("for_aizee")
        emits_urdf     = lr_mapping.get("emits_urdf", False)

    if (leader_rad is None or zero_offsets is None
            or directions is None or for_aizee is None):
        return None

    # Map leader → AIZEE target.  Physical leaders' `directions` array
    # was pre-tuned to match motor-encoder direction (predating
    # joint_align), so the raw mapped vector is in MOTOR frame.  Fold in
    # _ALIGN_SIGNS to convert to URDF frame here; the cmd-thread boundary
    # transform will fold the same signs back out before sending, leaving
    # the motor command identical to the pre-joint_align behaviour while
    # downstream (recording, holder["last_q_cmd"]) sees URDF frame.
    # QuestLeader already emits URDF, so it skips this fold.
    mapped = directions * (leader_rad - zero_offsets)
    target = mapped[for_aizee]
    if not emits_urdf:
        target = target * _ALIGN_SIGNS
    if leader_vel is not None:
        vel_ff_arr = (directions * leader_vel)[for_aizee]
        if not emits_urdf:
            vel_ff_arr = vel_ff_arr * _ALIGN_SIGNS
    else:
        vel_ff_arr = None

    # Telemetry q_actual + age — used for the lead-clamp.
    with telem_lock:
        telem_msg = telem_cache.get("msg")
        telem_t   = telem_cache.get("time", 0.0)
    q_actual = _qpos(telem_msg) if telem_msg is not None else None
    telem_age = time.time() - telem_t if telem_t > 0 else 999.0

    # Reference for the rate limit.
    if last_q_cmd is not None:
        ref = last_q_cmd.copy()
        # Lead-clamp only when telemetry is fresh.  When telem is stale the
        # clamp would pin the command behind a known-stale q_actual; trust
        # the velocity rate-limit alone in that case.
        if q_actual is not None and telem_age < _LEAD_CLAMP_TELEM_FRESH_S:
            max_lead = float(cfg.get("max_lead", 0.6))
            ref = q_actual + np.clip(ref - q_actual, -max_lead, max_lead)
    elif q_actual is not None:
        ref = q_actual
    else:
        ref = target

    # Velocity rate-limit, per-time (was per-30Hz-tick in the old path).
    max_vel  = float(cfg.get("max_vel", 9.0))   # rad/s
    max_step = max_vel * max(dt, 1e-3)
    delta    = np.clip(target - ref, -max_step, max_step)
    q_cmd    = ref + delta

    arm_limits = cfg.get("arm_limits")
    clamped_mask: Optional[np.ndarray] = None
    if arm_limits:
        q_cmd_clamped = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
        clamped_mask  = q_cmd_clamped != q_cmd
        q_cmd         = q_cmd_clamped

    # Zero vel_ff on any joint pinned at its limit. Without this, the leader's
    # trigger velocity keeps driving firmware kd·(v_cmd − v_actual) torque
    # against the mechanical stop, and trigger chatter knocks the motor back
    # and forth around the limit.
    if cfg.get("vel_ff", True) and vel_ff_arr is not None:
        vel_arr = vel_ff_arr.copy()
        if clamped_mask is not None and clamped_mask.any():
            vel_arr[clamped_mask] = 0.0
        vel = vel_arr.tolist()
    else:
        vel = [0.0] * NUM_JOINTS

    bundle = {
        "type": "bundle",
        "arm_joints": {
            "positions":  q_cmd.tolist(),
            "velocities": vel,
            "kp":         cfg.get("kp"),
            "kd":         cfg.get("kd"),
            "torques":    [0.0] * NUM_JOINTS,
        },
    }
    drive = cfg.get("drive")
    if drive is not None:
        bundle["drive"] = drive
    return bundle


# ---------------------------------------------------------------------------
# Background telemetry receiver
# ---------------------------------------------------------------------------
# Mirrors the camera-receiver pattern: keeps json.loads of fat telem packets
# (multi-motor state) off the main loop.  Caches the latest message; main
# loop reads it under a lock.

def _start_telem_receiver(
    ctx: zmq.Context,
    endpoint: str,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock  = threading.Lock()
    cache: dict = {"msg": None, "time": 0.0}
    stop  = threading.Event()

    def _run() -> None:
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(endpoint)

        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        try:
            while not stop.is_set():
                try:
                    events = dict(poller.poll(timeout=30))
                except zmq.ZMQError:
                    break
                if sock in events:
                    try:
                        msg = unpack_msg(sock.recv(zmq.NOBLOCK))
                    except Exception:
                        continue
                    with lock:
                        cache["msg"]  = msg
                        cache["time"] = time.time()
        finally:
            sock.close()

    thread = threading.Thread(target=_run, daemon=True, name="TelemRx")
    thread.start()
    return stop, thread, lock, cache


# ---------------------------------------------------------------------------
# Background camera / UPS receiver
# ---------------------------------------------------------------------------
# Runs in its own thread so that JSON-parsing large camera frames never
# blocks the motor-command loop.  Caches the latest message per source;
# the main loop reads cached values under a lock.

def _start_cam_receiver(
    ctx: zmq.Context,
    gripper_ep: str,
    ups_ep: Optional[str],
    scene_ep: Optional[str] = None,
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock = threading.Lock()
    cache: dict = {
        "gripper": None, "gripper_time": 0.0, "gripper_ts": None,
        "scene":   None, "scene_time":   0.0, "scene_ts":   None,
        "ups": None,
    }
    stop = threading.Event()

    def _run() -> None:
        # NOTE: zmq.CONFLATE is incompatible with multi-frame messages.  After
        # the camera channel switched to multipart (header msgpack + raw JPEG),
        # CONFLATE on the SUB triggered libzmq's `!_more` assertion in fq.cpp
        # the first time a new frame arrived mid-receive.  We drop CONFLATE
        # and instead set a small RCVHWM and drain to the latest message
        # explicitly inside the recv block.
        gripper_sock = ctx.socket(zmq.SUB)
        gripper_sock.setsockopt(zmq.LINGER, 0)
        gripper_sock.setsockopt(zmq.RCVHWM, 2)
        gripper_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        gripper_sock.connect(gripper_ep)

        scene_sock: Optional[zmq.Socket] = None
        if scene_ep:
            # Same multipart drain pattern as the gripper cam — the scene cam
            # publisher (camera_node.py) also uses pack_camera() multipart.
            scene_sock = ctx.socket(zmq.SUB)
            scene_sock.setsockopt(zmq.LINGER, 0)
            scene_sock.setsockopt(zmq.RCVHWM, 2)
            scene_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            scene_sock.connect(scene_ep)

        ups_sock: Optional[zmq.Socket] = None
        if ups_ep:
            ups_sock = ctx.socket(zmq.SUB)
            ups_sock.setsockopt(zmq.LINGER, 0)
            ups_sock.setsockopt(zmq.CONFLATE, 1)
            ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            ups_sock.connect(ups_ep)

        poller = zmq.Poller()
        poller.register(gripper_sock, zmq.POLLIN)
        if scene_sock:
            poller.register(scene_sock, zmq.POLLIN)
        if ups_sock:
            poller.register(ups_sock, zmq.POLLIN)

        try:
            while not stop.is_set():
                try:
                    # 30 ms ≈ one main-loop tick; matches LOOP_HZ so a stop
                    # request or backlog is noticed within ~one frame.
                    events = dict(poller.poll(timeout=30))
                except zmq.ZMQError:
                    break
                now = time.time()

                if gripper_sock in events:
                    # Drain to the newest available frame (CONFLATE no longer
                    # applies; old frames would otherwise queue up to RCVHWM=2).
                    latest = None
                    while True:
                        try:
                            latest = unpack_camera(gripper_sock.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        except Exception:
                            latest = None
                            break
                    if latest is not None:
                        with lock:
                            cache["gripper"] = latest
                            cache["gripper_time"] = now
                            ts = latest.get("timestamp")
                            if ts is not None:
                                cache["gripper_ts"] = float(ts)

                if scene_sock is not None and scene_sock in events:
                    latest = None
                    while True:
                        try:
                            latest = unpack_camera(scene_sock.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        except Exception:
                            latest = None
                            break
                    if latest is not None:
                        with lock:
                            cache["scene"] = latest
                            cache["scene_time"] = now
                            ts = latest.get("timestamp")
                            if ts is not None:
                                cache["scene_ts"] = float(ts)

                if ups_sock and ups_sock in events:
                    try:
                        msg = unpack_msg(ups_sock.recv(zmq.NOBLOCK))
                        with lock:
                            cache["ups"] = msg
                    except Exception:
                        pass
        finally:
            gripper_sock.close()
            if scene_sock is not None:
                scene_sock.close()
            if ups_sock:
                ups_sock.close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return stop, thread, lock, cache


# ---------------------------------------------------------------------------
# Background e-stop serial reader
# ---------------------------------------------------------------------------
# Reads JSON lines from the ESP32 e-stop receiver over serial.
# Sets/clears a threading.Event so the main loop can gate motor commands.

def _start_estop_reader(
    port: str,
    stop: threading.Event,
    flag: threading.Event,
) -> Optional[threading.Thread]:
    if not _pyserial_available:
        print("WARNING: pyserial not installed — hardware e-stop disabled")
        return None

    def _run() -> None:
        ser = None
        while not stop.is_set():
            if ser is None:
                try:
                    if _open_serial is not None:
                        ser = _open_serial(port, 115200, read_timeout=1)
                    else:
                        ser = _serial.Serial(port, 115200, timeout=1)
                    print(f"E-stop receiver connected on {port}")
                except (_serial.SerialException, OSError, TimeoutError):
                    stop.wait(2)
                    continue
            try:
                raw = ser.readline()
            except _serial.SerialException:
                print(f"E-stop serial error, reconnecting {port}...")
                ser = None
                stop.wait(1)
                continue
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line or line.startswith("#"):
                continue
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            estop = data.get("estop")
            if estop is not None:
                if estop:
                    flag.set()
                else:
                    flag.clear()

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------------------
# Image decoding
# ---------------------------------------------------------------------------

def decode_image(msg: dict, target_size: tuple, flip_v: bool = False) -> Optional[np.ndarray]:
    """Decode a camera message to uint8 [H, W, 3]. target_size is (W, H).

    Uses cv2 (libjpeg-turbo + INTER_AREA) when available — ~5-10x faster
    than PIL+LANCZOS and releases the GIL for the JPEG decode and resize.
    Falls back to PIL when cv2 isn't installed.

    Expects the multipart wire format: `msg["color"]["data_bytes"]` is
    the raw JPEG.  (Old base64 path under `["data"]` is no longer used.)
    """
    color = msg.get("color", {})
    raw   = color.get("data_bytes")
    if raw is None:
        return None
    if _cv2_available:
        arr = np.frombuffer(raw, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        if flip_v:
            bgr = cv2.flip(bgr, 0)
        # target_size is (W, H); cv2.resize takes (W, H) too.  Skip the
        # resize when the publisher already produced the target size.
        if (bgr.shape[1], bgr.shape[0]) != target_size:
            bgr = cv2.resize(bgr, target_size, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    img = Image.open(io.BytesIO(raw)).convert("RGB")
    if flip_v:
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
    if (img.width, img.height) != target_size:
        img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

def _qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract 7-element arm joint positions in URDF frame.

    Motor encoders report in motor frame; we apply the per-joint sign +
    offset from joint_align.json so the rest of collect_demo (IK, engage
    threshold, recording, replay) all work in a single consistent frame.
    Returns None if no arm telemetry is present."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    q_motor = np.array(
        [float(motors[j].get("position", 0.0)) if j in motors else 0.0
         for j in ARM_JOINTS],
        dtype=np.float32,
    )
    return _motor_to_urdf_pos(q_motor)

def _qpos_motor(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Same as _qpos but returns the RAW motor-frame positions.

    Used only for the telem broadcast field `qpos_motor` so the /preview
    calibration UI can compute live offsets/signs against the raw encoder
    reading."""
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    return np.array(
        [float(motors[j].get("position", 0.0)) if j in motors else 0.0
         for j in ARM_JOINTS],
        dtype=np.float32,
    )


def _qtorque(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("torque", _nan)) if j in motors else _nan
         for j in ARM_JOINTS],
        dtype=np.float32,
    )


def _qtemp(telem: Optional[dict]) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    if not any(j in motors for j in ARM_JOINTS):
        return None
    _nan = float("nan")
    return np.array(
        [float(motors[j].get("temperature", _nan)) if j in motors else _nan
         for j in ARM_JOINTS],
        dtype=np.float32,
    )


# ---------------------------------------------------------------------------
# HDF5 episode writer
# ---------------------------------------------------------------------------

def save_episode(
    output_dir, qpos_buf, gripper_buf,
    telem_ts_buf=None, gripper_ts_buf=None,
    qcmd_buf=None, torque_buf=None,
    scene_buf=None, scene_ts_buf=None,
    task_tag: str = "", notes: str = "",
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(output_dir.glob("episode_*.hdf5"))
    ep_num   = (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0
    path     = output_dir / f"episode_{ep_num:04d}.hdf5"

    # Buffers are already 7-DOF (swivel-first) — qpos_buf comes straight from
    # _qpos which iterates ARM_JOINTS, and ARM_JOINTS now includes swivel.
    qpos_arr    = np.stack(qpos_buf,    axis=0).astype(np.float32)   # [T, 7]
    gripper_arr = np.stack(gripper_buf, axis=0)                       # [T, H, W, 3]
    qcmd_arr    = (np.stack(qcmd_buf, axis=0).astype(np.float32)
                   if qcmd_buf and len(qcmd_buf) == len(qpos_buf) else None)
    torque_arr  = (np.stack(torque_buf, axis=0).astype(np.float32)
                   if torque_buf and len(torque_buf) == len(qpos_buf) else None)
    # Scene cam is optional — a session without --scene-cam (or with the
    # service down) produces v4 episodes; a populated scene_buf upgrades
    # to v5 with `observations/images/scene` + `timestamps/camera_scene`.
    scene_arr   = (np.stack(scene_buf, axis=0)
                   if scene_buf and len(scene_buf) == len(qpos_buf) else None)

    # Actions derived from commanded positions (no sag) when available
    act_src  = qcmd_arr if qcmd_arr is not None else qpos_arr
    actions  = np.concatenate([act_src[1:], act_src[-1:]], axis=0).astype(np.float32)  # [T, 7]
    H, W     = gripper_arr.shape[1], gripper_arr.shape[2]

    with h5py.File(path, "w") as f:
        f.attrs["hz"]           = REC_HZ
        f.attrs["arm_joints"]   = ",".join(ARM_JOINTS)
        f.attrs["action_space"] = "absolute"
        # v5 = v4 + `observations/images/scene` (RealSense color) and
        #      `timestamps/camera_scene`. Written only when scene_arr is
        #      populated.
        # v4 = 7-DOF unified arm + single gripper camera (replacing the
        # previous stereo D435 pair). v3 = 7-DOF + stereo left/right images.
        # v2 was also 7-DOF stereo but post-prepend, with a sidecar swivel
        # field; v1 was 6-DOF arm + sidecar swivel. load_recording in
        # record_replay.py handles all four (older versions are dropped on
        # the new single-camera training path).
        f.attrs["format_version"] = 5 if scene_arr is not None else 4
        f.attrs["task_tag"]     = task_tag
        f.attrs["notes"]        = notes
        f.attrs["collected_at"] = float(time.time())
        obs  = f.create_group("observations")
        obs.create_dataset("qpos",   data=qpos_arr,  compression="gzip", compression_opts=4)
        if qcmd_arr is not None:
            obs.create_dataset("qcmd", data=qcmd_arr, compression="gzip", compression_opts=4)
        if torque_arr is not None:
            obs.create_dataset("torques", data=torque_arr, compression="gzip", compression_opts=4)
        imgs = obs.create_group("images")
        imgs.create_dataset("gripper", data=gripper_arr,
                            compression="gzip", compression_opts=4,
                            chunks=(1, H, W, 3))
        if scene_arr is not None:
            sH, sW = scene_arr.shape[1], scene_arr.shape[2]
            imgs.create_dataset("scene", data=scene_arr,
                                compression="gzip", compression_opts=4,
                                chunks=(1, sH, sW, 3))
        f.create_dataset("actions",  data=actions,   compression="gzip", compression_opts=4)
        if telem_ts_buf is not None:
            ts = f.create_group("timestamps")
            ts.create_dataset("telem",          data=np.array(telem_ts_buf,   dtype=np.float64))
            ts.create_dataset("camera_gripper", data=np.array(gripper_ts_buf, dtype=np.float64))
            if scene_ts_buf is not None and len(scene_ts_buf) == len(qpos_buf):
                ts.create_dataset("camera_scene",
                                  data=np.array(scene_ts_buf, dtype=np.float64))

    return path, len(qpos_buf)


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _load_teleop_yaml() -> dict:
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


# ---------------------------------------------------------------------------
# Rover host resolution
#
# The rover (Jetson) is reachable on different networks depending on how the
# operator is connected.  Probe them in priority order and point every rover
# ZMQ endpoint at the first one that answers:
#   1. the configured IP (config/teleop.yaml, default 192.168.0.27 — POE/LAN)
#   2. 10.42.0.1    — direct USB-C ethernet adapter (NetworkManager shared link)
#   3. 192.168.50.1 — the rover's own WiFi access point ("aizee" AP mode)
# ---------------------------------------------------------------------------

# Fallback rover IPs appended after the configured address, in priority order.
_ROVER_FALLBACK_HOSTS = ["10.42.0.1", "192.168.50.1"]


def _split_tcp_endpoint(ep: Optional[str]) -> tuple[Optional[str], Optional[str]]:
    """'tcp://host:port' -> ('host', 'port').  Returns (None, None) for an
    empty / non-tcp:// endpoint, and (host, None) when no port is present."""
    if not ep or "://" not in ep:
        return None, None
    rest = ep.split("://", 1)[1]
    host, sep, port = rest.rpartition(":")
    if not sep:
        return (rest or None), None
    return (host or None), (port or None)


def _host_reachable(host: str, ports, timeout: float = 0.6) -> bool:
    """True if a TCP connection to `host` succeeds on any of `ports`."""
    for port in ports:
        try:
            with socket.create_connection((host, int(port)), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def _resolve_rover_host(primary_host: str, cmd_port: Optional[str],
                        timeout: float = 0.6) -> tuple[str, list]:
    """Probe configured IP -> USB-C ethernet -> WiFi AP and return the first
    reachable host plus the ordered candidate list that was tried.  Falls back
    to `primary_host` if nothing answers.

    Probes both the ZMQ command port (what we actually use) and ssh (22, which
    is always up) so host selection works even before rover services start."""
    candidates: list = []
    for h in [primary_host, *_ROVER_FALLBACK_HOSTS]:
        if h and h not in candidates:
            candidates.append(h)
    ports = [p for p in (cmd_port, "22") if p]
    for h in candidates:
        if _host_reachable(h, ports, timeout):
            return h, candidates
    return primary_host, candidates


# ---------------------------------------------------------------------------
# Live episode replay (on-robot playback inside the main loop)
# ---------------------------------------------------------------------------

def _load_episode_for_replay(path: Path) -> tuple[np.ndarray, float]:
    """Load an episode HDF5 for live replay.

    Returns `(qpos[T, NUM_JOINTS], hz)` — qpos is always 7-DOF in
    ARM_JOINTS order (swivel-first).  Older 6-DOF files with a sidecar
    `swivel` dataset are stitched into the unified shape on the way out.
    """
    with h5py.File(path, "r") as f:
        if "observations" in f and "qpos" in f["observations"]:
            obs = f["observations"]
            raw = obs["qcmd"][:] if "qcmd" in obs else obs["qpos"][:]
            sw  = obs["swivel"][:] if "swivel" in obs else None
            hz  = float(f.attrs.get("hz", 20.0))
        elif "qpos" in f:
            raw = f["qpos"][:]
            sw  = f["swivel"][:] if "swivel" in f else None
            hz  = float(f.attrs.get("hz", 20.0))
        else:
            raise ValueError(f"Unrecognised HDF5 format: {path}")

    raw = raw.astype(np.float32)
    if raw.ndim != 2:
        raise ValueError(f"qpos must be 2D, got shape {raw.shape}")

    # 7-DOF unified — return as-is.
    if raw.shape[1] == NUM_JOINTS:
        return raw, hz

    # Legacy 6-DOF gantry-only with sidecar swivel.
    if raw.shape[1] == NUM_JOINTS - 1:
        if sw is None:
            sw = np.zeros((raw.shape[0],), dtype=np.float32)
        sw_col = sw.astype(np.float32).reshape(-1, 1)
        n      = min(sw_col.shape[0], raw.shape[0])
        return np.hstack([sw_col[:n], raw[:n]]).astype(np.float32), hz

    raise ValueError(
        f"qpos has {raw.shape[1]} columns; expected {NUM_JOINTS} (unified) or "
        f"{NUM_JOINTS - 1} (legacy 6-DOF + sidecar swivel)"
    )


class _LiveReplay:
    """State machine for on-robot episode playback inside collect_demo.

    The main loop owns ZMQ; this class produces lists of motor-command
    dicts via step() / arm() / play() etc., which the caller sends.
    See episode_replay_live.py for the standalone reference.
    """

    class Phase(enum.Enum):
        READY    = "ready"
        ARMING   = "arming"
        PLAYING  = "playing"
        PAUSED   = "paused"
        DONE     = "done"
        SHUTDOWN = "shutdown"

    def __init__(
        self, *, kp, kd,
        max_delta: float, arm_limits, all_motor_ids,
    ):
        self._kp              = list(kp)
        self._kd              = list(kd)
        self._arm_limits      = arm_limits
        self._all_motor_ids   = list(all_motor_ids)

        # Live config (mutable from GUI)
        self.max_delta        = float(max_delta)
        self.speed            = 1.0
        self.loop_mode        = False
        self.goto_start       = True
        self.vel_ff_enabled   = False
        self.ramp_speed       = 1.5   # rad/s approach to start pose.  0.4
                                      # was too slow to traverse end→start
                                      # within a reasonable window when the
                                      # episode ended far from its beginning,
                                      # making re-arming look stuck.
        self._ramp_step       = self.ramp_speed / LOOP_HZ
        self._arm_max_lead    = 0.15  # rad cap on how far the command may
                                      # lead q_actual during ARMING.  At
                                      # kp=30 this puts peak PD torque at
                                      # ~4.5 N·m — enough to overcome gantry
                                      # gravity (was 0.05 / 1.5 N·m, which
                                      # could not move the arm against load
                                      # for re-arming from a distant pose).

        # Episode (set by load()).  qpos is always 7-DOF (swivel-first).
        self.ep_path:       Optional[Path]       = None
        self.ep_name:       str                  = ""
        self.ep_qpos:       Optional[np.ndarray] = None
        self.ep_velocities: Optional[np.ndarray] = None
        self.ep_hz:         float                = 0.0
        self.ep_frames:     int                  = 0

        # Runtime state.  current_target is the 7-DOF arm target — swivel
        # is current_target[0]; there's no separate swivel state anymore.
        self.live:              bool                  = False
        self.phase                                    = self.Phase.READY
        self.frame_idx:         int                   = 0
        self.last_frame_wall:   float                 = 0.0
        self.current_target:    Optional[np.ndarray]  = None
        self.error:             float                 = 0.0
        self.message:           str                   = ""

        # Shutdown state
        self._shutdown_target:      Optional[np.ndarray] = None
        self._shutdown_countdown:   float                = 0.0
        self._shutdown_zero_since:  float                = 0.0
        self._SHUTDOWN_TIMEOUT                           = 3.0

        # Arming-phase stall detection.  ARMING used to require every joint
        # within 0.03 rad of the start pose, which deadlocked when one joint
        # had enough stiction to sit ~0.04 rad off under low PD authority.
        self._arming_start_t:        float = 0.0
        self._ARM_DONE_THRESHOLD            = 0.05   # rad — promote to PLAYING below this
        self._ARM_STALL_ACCEPT              = 0.10   # rad — after stall timeout, accept up to this
        self._ARM_STALL_TIMEOUT             = 4.0    # s — start accepting residual after this

    # -- Episode loading ---------------------------------------------------
    def load(self, path: Path) -> Optional[str]:
        """Load an episode. Returns None on success, error string on failure."""
        try:
            qpos, hz = _load_episode_for_replay(path)
        except Exception as e:
            self.message = f"load error: {e}"
            return str(e)
        self.ep_path       = path
        self.ep_name       = path.name
        self.ep_qpos       = qpos
        self.ep_hz         = hz
        self.ep_frames     = len(qpos)
        self.ep_velocities = self._compute_vel_ff(qpos, hz)
        self.frame_idx     = 0
        self.phase         = self.Phase.READY
        self.current_target = qpos[0].copy() if self.ep_frames > 0 else None
        self.error         = 0.0
        self.message       = f"loaded {path.name}  {self.ep_frames}f @ {hz:.0f} Hz"
        return None

    @staticmethod
    def _compute_vel_ff(qpos: np.ndarray, hz: float) -> Optional[np.ndarray]:
        T = len(qpos)
        if T < 2 or hz <= 0:
            return None
        dt = 1.0 / hz
        dq = np.diff(qpos, axis=0) / dt                        # [T-1, J]
        dq = np.vstack([dq, np.zeros((1, qpos.shape[1]))])     # [T,   J]
        smoothed = dq.copy()
        for i in range(1, T - 1):
            smoothed[i] = (dq[i - 1] + dq[i] + dq[i + 1]) / 3.0
        if T > 1:
            smoothed[0] = (dq[0] + dq[1]) / 2.0
        return smoothed.astype(np.float32)

    # -- Mode transitions --------------------------------------------------
    def enter_live(self) -> bool:
        if self.ep_qpos is None or self.ep_frames == 0:
            return False
        self.live      = True
        self.phase     = self.Phase.READY
        self.frame_idx = 0
        return True

    def exit_live(self) -> bool:
        """Exit live mode. Rejected while motors are actively commanded."""
        if self.phase in (self.Phase.ARMING, self.Phase.PLAYING, self.Phase.SHUTDOWN):
            return False
        self.live  = False
        self.phase = self.Phase.READY
        return True

    # -- Transport ---------------------------------------------------------
    def arm(self, q_actual) -> list[dict]:
        if not self.live or self.ep_qpos is None or self.ep_frames == 0:
            return []
        if self.phase == self.Phase.SHUTDOWN:
            return []
        self.frame_idx = 0
        cmds: list[dict] = [{"type": "enable", "motor_ids": self._all_motor_ids}]
        if self.goto_start and q_actual is not None:
            # Seed the ramp's integrator from the current measured pose so
            # the arming step starts exactly where the arm is.
            self.current_target = q_actual.copy().astype(np.float32)
            self.phase = self.Phase.ARMING
            self._arming_start_t = time.time()
        else:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return cmds

    def play(self, q_actual) -> list[dict]:
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return []
        if self.phase in (self.Phase.READY, self.Phase.DONE):
            return self.arm(q_actual)
        if self.phase == self.Phase.PAUSED:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return []

    def pause(self) -> None:
        if self.live and self.phase == self.Phase.PLAYING:
            self.phase = self.Phase.PAUSED

    def toggle(self, q_actual) -> list[dict]:
        if not self.live:
            return []
        if self.phase == self.Phase.PLAYING:
            self.pause()
            return []
        return self.play(q_actual)

    def restart(self, q_actual) -> list[dict]:
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return []
        self.frame_idx = 0
        cmds: list[dict] = []
        # If we arrived from DONE/PAUSED, motors may have been left disabled
        # by a prior stop; re-enable defensively so ARMING actually moves.
        if self.phase != self.Phase.ARMING and self.phase != self.Phase.PLAYING:
            cmds.append({"type": "enable", "motor_ids": self._all_motor_ids})
        if self.goto_start and q_actual is not None:
            # Reset the ramp integrator from current measured pose so we
            # don't carry the previous run's end-frame target into ARMING
            # (which would make ref ≈ end_pose and queue a giant delta).
            self.current_target = q_actual.copy().astype(np.float32)
            self.phase = self.Phase.ARMING
            self._arming_start_t = time.time()
        else:
            self.phase           = self.Phase.PLAYING
            self.last_frame_wall = time.time()
        return cmds

    def stop(self, q_actual) -> None:
        """Abort: ramp to zero and disable. Phase returns to READY when done."""
        if not self.live or self.phase == self.Phase.SHUTDOWN:
            return
        if self.phase == self.Phase.READY:
            return
        self._shutdown_target = (q_actual.copy() if q_actual is not None
                                 else (self.current_target.copy()
                                       if self.current_target is not None
                                       else np.zeros(NUM_JOINTS, dtype=np.float32)))
        self._shutdown_countdown  = 1.0
        self._shutdown_zero_since = 0.0
        self.phase                = self.Phase.SHUTDOWN

    def set_speed(self, speed: float) -> None:
        self.speed = max(0.1, min(float(speed), 4.0))

    def set_opts(self, **opts) -> None:
        if "loop" in opts:
            self.loop_mode = bool(opts["loop"])
        if "goto_start" in opts:
            self.goto_start = bool(opts["goto_start"])
        if "max_delta" in opts:
            self.max_delta = max(0.01, float(opts["max_delta"]))
        if "vel_ff" in opts:
            self.vel_ff_enabled = bool(opts["vel_ff"])
        if "ramp_speed" in opts:
            self.ramp_speed = max(0.1, float(opts["ramp_speed"]))
            self._ramp_step = self.ramp_speed / LOOP_HZ

    # -- Command builders --------------------------------------------------
    def _safe_cmd(self, target: np.ndarray, ref: Optional[np.ndarray]) -> np.ndarray:
        r = ref if ref is not None else target
        q = r + np.clip(target - r, -self.max_delta, self.max_delta)
        if self._arm_limits:
            q = np.array(clamp_arm_positions(q.tolist(), self._arm_limits))
        return q

    def _arm_cmd(self, q_cmd: np.ndarray, vel_ff: Optional[list]) -> dict:
        return {
            "type":       "arm_joints",
            "positions":  q_cmd.tolist(),
            "velocities": vel_ff if vel_ff is not None else [0.0] * NUM_JOINTS,
            "kp":         self._kp,
            "kd":         self._kd,
            "torques":    [0.0] * NUM_JOINTS,
        }

    # -- Per-tick step -----------------------------------------------------
    def step(
        self, t0: float, q_actual: Optional[np.ndarray], period: float,
    ) -> list[dict]:
        if not self.live:
            return []

        # Tracking error telemetry (max-norm across all 7 joints)
        if q_actual is not None and self.current_target is not None:
            self.error = float(np.max(np.abs(q_actual - self.current_target)))

        cmds: list[dict] = []
        phase = self.phase

        if phase == self.Phase.READY:
            return cmds

        if phase == self.Phase.ARMING:
            tgt = self.ep_qpos[0]
            ref = (self.current_target if self.current_target is not None
                   else (q_actual if q_actual is not None else tgt))
            if q_actual is not None:
                lead = np.clip(ref - q_actual,
                               -self._arm_max_lead, self._arm_max_lead)
                ref  = q_actual + lead
            q_cmd = ref + np.clip(tgt - ref, -self._ramp_step, self._ramp_step)
            if self._arm_limits:
                q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), self._arm_limits))
            cmds.append(self._arm_cmd(q_cmd, None))
            self.current_target = q_cmd
            arm_ok = False
            if q_actual is not None:
                err = float(np.max(np.abs(q_actual - tgt)))
                arm_ok = err < self._ARM_DONE_THRESHOLD
                # Stall escape: if a joint has stiction/PD-authority limits
                # that prevent closing the last sliver, accept a small
                # residual after a few seconds rather than hanging forever.
                if (not arm_ok
                        and self._arming_start_t > 0
                        and t0 - self._arming_start_t > self._ARM_STALL_TIMEOUT
                        and err < self._ARM_STALL_ACCEPT):
                    self.message = f"armed with residual {err:.3f} rad"
                    arm_ok = True
            if arm_ok:
                self.phase           = self.Phase.PLAYING
                self.last_frame_wall = t0
            return cmds

        if phase == self.Phase.PLAYING:
            frame_period = 1.0 / max(self.ep_hz * self.speed, 0.1)
            if t0 - self.last_frame_wall >= frame_period and self.frame_idx < self.ep_frames:
                self.last_frame_wall = t0
                tgt    = self.ep_qpos[self.frame_idx]
                vel_ff = (self.ep_velocities[self.frame_idx].tolist()
                          if (self.ep_velocities is not None and self.vel_ff_enabled)
                          else None)
                cmds.append(self._arm_cmd(
                    self._safe_cmd(tgt, self.current_target), vel_ff))
                self.current_target = tgt
                self.frame_idx += 1
                if self.frame_idx >= self.ep_frames:
                    if self.loop_mode:
                        self.frame_idx = 0
                    else:
                        self.phase = self.Phase.DONE
            return cmds

        if phase in (self.Phase.PAUSED, self.Phase.DONE):
            if self.current_target is not None:
                cmds.append(self._arm_cmd(
                    self._safe_cmd(self.current_target, q_actual), None))
            return cmds

        if phase == self.Phase.SHUTDOWN:
            dt         = period
            max_change = 0.2 * dt   # 0.2 rad/s ramp to zero
            if self._shutdown_countdown > 0:
                self._shutdown_countdown -= dt
                if self._shutdown_target is not None:
                    cmds.append(self._arm_cmd(self._shutdown_target, None))
                return cmds
            if self._shutdown_target is None:
                self._shutdown_target = np.zeros(NUM_JOINTS, dtype=np.float32)
            new_tgt = self._shutdown_target.copy()
            for i in range(len(new_tgt)):
                new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                              else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
            self._shutdown_target = new_tgt
            ramp_done = bool(np.all(np.abs(self._shutdown_target) < 0.01))
            if ramp_done and self._shutdown_zero_since == 0.0:
                self._shutdown_zero_since = t0
            actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
            timed_out    = (self._shutdown_zero_since > 0
                            and t0 - self._shutdown_zero_since >= self._SHUTDOWN_TIMEOUT)
            if ramp_done and (actual_close or timed_out):
                cmds.append({"type": "disable", "motor_ids": self._all_motor_ids})
                self.phase               = self.Phase.READY
                self.frame_idx           = 0
                self._shutdown_target    = None
                self.current_target = (self.ep_qpos[0].copy()
                                       if self.ep_frames > 0 else None)
            else:
                ref   = q_actual if q_actual is not None else self._shutdown_target
                q_cmd = self._safe_cmd(self._shutdown_target, ref)
                cmds.append(self._arm_cmd(q_cmd, None))
            return cmds

        return cmds

    # -- Snapshot / status -------------------------------------------------
    def snapshot_fields(self) -> dict:
        pct = (100.0 * self.frame_idx / self.ep_frames) if self.ep_frames else 0.0
        dur = (self.ep_frames / self.ep_hz) if self.ep_hz > 0 else 0.0
        # 7-DOF target the GUI's joint panel plots against live actual.
        # Layout matches ARM_JOINTS (swivel-first).
        if self.current_target is not None:
            replay_target = [float(x) for x in self.current_target]
        else:
            replay_target = None
        return {
            "replay_live":        self.live,
            "replay_phase":       self.phase.value if self.live else None,
            "replay_frame":       self.frame_idx,
            "replay_frames":      self.ep_frames,
            "replay_pct":         pct,
            "replay_hz":          self.ep_hz,
            "replay_duration":    dur,
            "replay_speed":       self.speed,
            "replay_loop":        self.loop_mode,
            "replay_goto_start":  self.goto_start,
            "replay_vel_ff":      self.vel_ff_enabled,
            "replay_max_delta":   self.max_delta,
            "replay_error":       self.error,
            "replay_path":        str(self.ep_path) if self.ep_path else "",
            "replay_name":        self.ep_name,
            "replay_message":     self.message,
            "replay_target":      replay_target,
        }

    def status_line(self) -> tuple[str, str]:
        """Returns (status, hint) strings, or ('','') when live mode is off."""
        if not self.live:
            return "", ""
        p = self.phase
        if p == self.Phase.READY:
            return (f"[replay] ready — {self.ep_name}  {self.ep_frames}f",
                    "PLAY to arm+play · exit to return to teleop")
        if p == self.Phase.ARMING:
            return (f"[replay] arming — err {self.error:.3f} rad",
                    "STOP to abort")
        if p == self.Phase.PLAYING:
            pct = 100.0 * self.frame_idx / max(self.ep_frames, 1)
            lp  = "  LOOP" if self.loop_mode else ""
            return (f"[replay] PLAYING  {pct:.0f}%  {self.speed:.2f}x{lp}",
                    "PAUSE · STOP")
        if p == self.Phase.PAUSED:
            pct = 100.0 * self.frame_idx / max(self.ep_frames, 1)
            return (f"[replay] PAUSED  {pct:.0f}%", "PLAY · STOP")
        if p == self.Phase.DONE:
            return ("[replay] done", "PLAY to re-run · STOP to disable")
        if p == self.Phase.SHUTDOWN:
            if self._shutdown_countdown > 0:
                return (f"[replay] shutdown  hold {self._shutdown_countdown:.1f}s", "")
            return ("[replay] returning to zero", "")
        return "", ""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    _load_joint_align()  # populate _ALIGN_OFFSETS / _ALIGN_SIGNS from joint_align.json
    _ep = _load_endpoints()
    ap  = argparse.ArgumentParser(
        description="SO-101 leader arm teleop + ACT demo recorder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",            default=None,
                    help="Leader-arm serial port (optional — enables leader tracking)")
    ap.add_argument("--baud",            type=int, default=1_000_000)
    ap.add_argument("--calib",           default=None,
                    help="Leader calibration JSON (defaults to per-leader-kind path)")
    ap.add_argument("--leader",          default="auto",
                    choices=("auto", *LEADER_KINDS),
                    help="Which physical leader arm to use at startup (auto = try "
                         "SO-101 then OpenRB-150).  Quest VR is added/removed at "
                         "runtime via the GUI — not selectable here.")
    # Quest server configuration knobs.  They take effect when the GUI
    # toggles Quest on; ignored otherwise.
    ap.add_argument("--quest-bind",      default="0.0.0.0", dest="quest_bind",
                    help="Bind address for the WebXR server when activated from the GUI")
    ap.add_argument("--quest-port",      type=int, default=8443, dest="quest_port",
                    help="HTTPS port for the WebXR server when activated from the GUI")
    ap.add_argument("--quest-config",    default=None, dest="quest_config",
                    help="Path to quest_teleop.yaml (defaults to config/quest_teleop.yaml)")
    ap.add_argument("--cmd",             default=_ep.get("command",       "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem",           default=_ep.get("telemetry",     "tcp://192.168.0.27:5556"))
    ap.add_argument("--gripper-cam",     default="tcp://192.168.0.27:5563", dest="gripper_cam",
                    help="Gripper camera ZMQ endpoint (single ELP UVC stream)")
    ap.add_argument("--gripper-cam-ctrl", default="tcp://192.168.0.27:5573", dest="gripper_cam_ctrl",
                    help="Gripper camera control ZMQ REP endpoint (V4L2 sliders in GUI). "
                         "Empty string disables the camera-controls panel.")
    ap.add_argument("--scene-cam",       default="tcp://192.168.0.27:5564", dest="scene_cam",
                    help="Scene camera ZMQ endpoint (Intel RealSense RGB-D). "
                         "Empty string disables scene-cam subscribe / record / preview.")
    ap.add_argument("--ups",             default=_ep.get("ups_telemetry", "tcp://192.168.0.27:5562"),
                    help="UPS telemetry address (empty to disable)")
    ap.add_argument("--output-dir",      default="episodes",              dest="output_dir")
    ap.add_argument("--max-steps",       type=int, default=10000,           dest="max_steps",
                    help="Max steps per episode (default: 10000 = 30 s at 20 Hz)")
    ap.add_argument("--image-size",      default="768x1024",              dest="image_size",
                    help="Image size HxW (default: 768x1024 — matches gripper-cam capture)")
    ap.add_argument("--dry-run",         action="store_true",             dest="dry_run")
    ap.add_argument("--max-delta",       type=float, default=0.3,         dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.3)")
    ap.add_argument("--robstride-calib", default=None,                    dest="robstride_calib")
    ap.add_argument("--no-rerun",       action="store_true",             dest="no_rerun",
                    help="Disable Rerun live camera preview")
    ap.add_argument("--gui",            action="store_true",             dest="gui",
                    help="Launch PySide6 control panel (embeds Rerun web viewer)")
    ap.add_argument("--estop-port",    default=None,                    dest="estop_port",
                    help="Serial port for ESP32 e-stop receiver (e.g. /dev/estop-receiver, COM10)")
    ap.add_argument("--task-tag",      default="",                      dest="task_tag",
                    help="Task label written as episode attr (GUI can override live)")
    # Force-feedback on the OpenRB GELLO leader (only).  Off by default —
    # requires the dedicated 5V supply on the OpenRB rail; running on USB
    # power alone will brown out.  Currently scoped to the gripper trigger
    # so the operator feels follower-side gripper load (object squeeze /
    # back-drive from a surface).  Other joints stay passive (torque off).
    ap.add_argument("--ff-leader",     action="store_true",            dest="ff_leader",
                    help="Enable leader force feedback (OpenRB only). "
                         "Reflects follower gripper torque onto the trigger.")
    ap.add_argument("--ff-gripper-gain", type=float, default=350.0,    dest="ff_gripper_gain",
                    help="mA per N·m of follower gripper torque (signed). "
                         "Sign flips which way the trigger pushes; tune empirically "
                         "(default 350 → ~70 mA at typical 0.2 N·m grip load).")
    ap.add_argument("--ff-gripper-deadband", type=float, default=0.05, dest="ff_gripper_deadband",
                    help="N·m of follower gripper torque to ignore before "
                         "applying FF (cancels idle holding torque).")
    args = ap.parse_args()

    _ansi_on()

    # Resolve which network the rover is actually reachable on and repoint every
    # rover ZMQ endpoint accordingly: configured IP → USB-C ethernet → WiFi AP.
    _primary_host, _cmd_port = _split_tcp_endpoint(args.cmd)
    if _primary_host:
        _sel_host, _cands = _resolve_rover_host(_primary_host, _cmd_port)
        _tried = " → ".join(_cands)
        if _sel_host != _primary_host:
            print(f"[net] {_primary_host} unreachable; rover found at {_sel_host} "
                  f"(tried {_tried})", flush=True)
            for _name in ("cmd", "telem", "gripper_cam", "gripper_cam_ctrl",
                          "scene_cam", "ups"):
                _h, _p = _split_tcp_endpoint(getattr(args, _name, None))
                if _h == _primary_host and _p:
                    setattr(args, _name, f"tcp://{_sel_host}:{_p}")
        else:
            print(f"[net] rover reachable at {_sel_host} (priority: {_tried})",
                  flush=True)

    h_s, w_s = args.image_size.split("x")
    img_size  = (int(w_s), int(h_s))   # PIL: (width, height)

    # -------------------------------------------------------------------------
    # Leader arm (optional, hot-pluggable)
    #
    # Two leader kinds are supported, both exposing the same duck-typed
    # interface (poll/connect/JOINTS/AIZEE_JOINTS/zero_offsets/directions):
    #   - so101  : Feetech STS3215 over WaveShare USB-serial bus adapter.
    #   - openrb : Dynamixel XL330 servos behind an OpenRB-150 USB-CDC bridge.
    #
    # The leader is allowed to be absent at startup AND to appear later.  A
    # background watcher polls comports() at low frequency and only probes
    # ports when the set actually changes — no spammy probe loop.
    # -------------------------------------------------------------------------
    leader           = None
    _lr_lock         = threading.Lock()
    _lr_latest: dict = {
        "rad": None, "vel": None, "clamped": None, "time": 0.0,
        # M5 Joystick2 snapshot (populated by _leader_reader when an
        # OpenRB-150 leader is installed; left at neutral defaults
        # otherwise).  See OpenRBLeader.last_joystick for field semantics.
        "joy": {
            "x":             0.0,
            "y":             0.0,
            "button":        False,
            "press_counter": 0,
            "status":        1,    # JOY_STATUS_NOT_PRESENT
            "present":       False,
        },
    }
    # Shared leader-to-AIZEE mapping params.  Written by main loop (Z, M
    # keys; leader install / hot-plug); read by the cmd-sender thread when
    # it computes q_cmd live from leader at 100 Hz.  Held under _lr_lock.
    _lr_mapping: dict = {
        "zero_offsets": None,   # np.ndarray, leader-frame
        "directions":   None,   # np.ndarray, ±1 per leader joint
        "for_aizee":    None,   # list[int], leader-index → AIZEE-arm-index
        "emits_urdf":   False,  # True for QuestLeader; False for physical
                                # leaders whose `directions` was tuned to
                                # the motor-encoder direction pre-joint_align
    }
    _lr_stop         = threading.Event()
    zero_offsets     = None
    directions       = None
    _so101_for_aizee: list[int] = []
    _emits_urdf      = False    # set per-leader in _try_install_leader
    _arm_joint_set   = set(ARM_JOINTS)

    # Selected leader kind ("so101" / "openrb") and its class — set at install
    # time, used by the hot-plug watcher so a re-plug picks the same kind.
    _leader_kind:  Optional[str] = None if args.leader == "auto" else args.leader
    _leader_cls                  = None
    _leader_calib                = args.calib

    # Quest WebXR runtime state.  Lazily built when the GUI toggles Quest
    # on for the first time; persists for the rest of the session so the
    # /preview page stays accessible even after the operator disconnects
    # the VR leader.  Building costs ~1.5 s (cert + URDF + server) which
    # is why we don't do it at boot.
    _quest_state = None
    _quest_cfg   = None
    _quest_server_started = False
    if args.leader != "auto" and _leader_module_available:
        _leader_cls   = get_leader_class(args.leader)
        if _leader_calib is None:
            _leader_calib = str(default_calib_path(args.leader))
    # Back-compat fallback: if the leader module isn't importable for any
    # reason, fall through to the original SO-101-only code path.
    if _leader_cls is None and _so101_available:
        _leader_cls   = So101Leader
        _leader_kind  = _leader_kind or "so101"
        if _leader_calib is None:
            _leader_calib = str(_CALIB_PATH)

    # Atomic single-slot box read by the always-on reader thread.  Updating
    # the dict key is a single bytecode op, so the reader sees None or a
    # complete leader object — never a half-installed one.
    _leader_box: dict = {"leader": None}

    # Hot-plug install hand-off: watcher writes a dict here, main loop pops
    # it at the top of the loop and rebinds `leader`/`zero_offsets`/etc.
    _install_lock = threading.Lock()
    _install_pending: dict = {}

    def _try_install_leader(port: str, kind: Optional[str] = None) -> bool:
        """Connect to *port* and install as the active leader. Returns True on success.

        *kind* overrides the previously-selected leader kind (useful for the
        hot-plug watcher when --leader=auto).  When None, falls back to the
        currently-bound _leader_cls / _leader_kind / _leader_calib.
        """
        nonlocal _leader_cls, _leader_kind, _leader_calib
        if kind is not None and _leader_module_available:
            _leader_cls   = get_leader_class(kind)
            _leader_kind  = kind
            if args.calib is None:
                _leader_calib = str(default_calib_path(kind))
        if _leader_cls is None:
            print(f"No leader class available — cannot install {port}", flush=True)
            return False
        calib_path = _leader_calib if _leader_calib is not None else str(_CALIB_PATH)
        kind_name  = _leader_kind or "leader"
        try:
            ldr = _leader_cls(port, args.baud, calib=calib_path)
        except Exception as exc:
            print(f"{kind_name} init failed on {port}: {exc}", flush=True)
            return False
        try:
            ok = ldr.connect()
        except Exception as exc:
            print(f"{kind_name} connect raised on {port}: {exc}", flush=True)
            return False
        if not ok:
            return False
        for_aizee = [i for i, j in enumerate(ldr.AIZEE_JOINTS) if j in _arm_joint_set]
        # Caller decides whether to write to local rebinds or hand off to main loop.
        with _install_lock:
            _install_pending["data"] = {
                "leader":       ldr,
                "zero_offsets": ldr.zero_offsets,
                "directions":   ldr.directions,
                "for_aizee":    for_aizee,
                # See _apply_leader_to_urdf_frame in _compute_tracking_bundle
                # / the main loop for what this gates.  Physical leaders
                # default to False (their `directions` calibration was tuned
                # to match motor encoder direction, predating joint_align).
                "emits_urdf":   bool(getattr(ldr, "EMITS_URDF_FRAME", False)),
            }
        # Hand the leader the current visual-offset vector so its FK/IK
        # operate in mesh frame (QuestLeader only — others are no-ops).
        _push_visual_offsets_to_leader(ldr)
        _leader_box["leader"] = ldr
        return True

    # ------------------------------------------------------------------
    # Quest WebXR — runtime install/uninstall (driven by the GUI button).
    # The boot path no longer touches these; they're called by the dict
    # command handler when the operator clicks "Connect Quest VR".
    # ------------------------------------------------------------------

    def _ensure_quest_runtime() -> bool:
        """Lazily build SharedState + start the WebXR server.  Idempotent —
        safe to call multiple times; subsequent calls are no-ops."""
        nonlocal _quest_state, _quest_cfg, _quest_server_started
        if not _quest_available:
            print("[quest] support unavailable — install: "
                  "pip install aiohttp cryptography pyyaml", flush=True)
            return False
        if _quest_state is None:
            _quest_state = SharedState()
            _cfg_path = args.quest_config or str(
                Path(__file__).resolve().parents[2] / "config" / "quest_teleop.yaml"
            )
            if Path(_cfg_path).exists():
                _quest_cfg = QuestLeaderConfig.load_yaml(_cfg_path)
                print(f"[quest] loaded config from {_cfg_path}", flush=True)
            else:
                _quest_cfg = QuestLeaderConfig()
                print(f"[quest] no config at {_cfg_path} — using defaults", flush=True)
        if not _quest_server_started:
            start_server_in_thread(
                _quest_state, bind=args.quest_bind, port=args.quest_port,
            )
            _quest_server_started = True
        return True

    def _install_quest_leader() -> bool:
        """Install QuestLeader as the active leader.  Replaces any existing
        serial leader for the duration of the Quest session."""
        nonlocal _leader_cls, _leader_kind, _leader_calib
        if not _ensure_quest_runtime():
            return False
        _leader_cls   = make_quest_leader_class(_quest_state, _quest_cfg)
        _leader_kind  = "quest"
        _leader_calib = None
        return _try_install_leader("webxr://")

    def _uninstall_leader() -> None:
        """Drop the currently-installed leader (no auto-recovery).  The
        WebXR server stays running so /preview is still reachable; only
        the leader-poll path is detached."""
        nonlocal _leader_cls, _leader_kind
        _leader_box["leader"] = None
        with _lr_lock:
            _lr_latest["rad"]     = None
            _lr_latest["vel"]     = None
            _lr_latest["clamped"] = None
            _lr_mapping["zero_offsets"] = None
            _lr_mapping["directions"]   = None
            _lr_mapping["for_aizee"]    = None
            _lr_mapping["emits_urdf"]   = False
        _leader_kind = None

    def _leader_reader(stop: threading.Event) -> None:
        """Always-on reader thread; idles until a leader is installed in _leader_box."""
        prev_r: Optional[np.ndarray] = None
        prev_t: float = 0.0
        ema_v:  Optional[np.ndarray] = None
        # EMA constant for the velocity estimate. Differentiating quantized
        # 12-bit encoders at ~500 Hz produces ~13 mrad/s of LSB noise, so we
        # smooth before forwarding. alpha tuned for ~3 sample time-constant.
        _V_ALPHA = 0.4
        while not stop.is_set():
            ldr = _leader_box["leader"]
            if ldr is None:
                prev_r = None
                ema_v  = None
                time.sleep(0.02)
                continue
            try:
                r = ldr.poll()
            except Exception:
                r = None
            now = time.time()
            v: Optional[np.ndarray] = None
            if r is not None and prev_r is not None and (now - prev_t) > 1e-3:
                inst_v = (r - prev_r) / (now - prev_t)
                ema_v  = inst_v if ema_v is None else (
                    _V_ALPHA * inst_v + (1.0 - _V_ALPHA) * ema_v)
                v = ema_v
            # M5 Joystick2 snapshot — only OpenRBLeader exposes this; older
            # SO-101 leader has no `last_joystick`, so we degrade silently.
            joy_snapshot = getattr(ldr, "last_joystick", None)
            with _lr_lock:
                if r is not None:
                    _lr_latest["rad"]     = r
                    _lr_latest["vel"]     = v
                    _lr_latest["clamped"] = ldr.clamped_joints
                    _lr_latest["time"]    = now
                if joy_snapshot is not None:
                    _lr_latest["joy"] = joy_snapshot
            if r is not None:
                prev_r = r
                prev_t = now

    _lr_thread = threading.Thread(target=_leader_reader, args=(_lr_stop,), daemon=True)
    _lr_thread.start()

    leader_port = args.port
    if leader_port is None and _leader_module_available:
        _excl = [args.estop_port] if args.estop_port else []
        if args.leader == "auto":
            print("Searching for any leader arm...", flush=True)
            leader_port, detected_kind = find_any_leader(exclude=_excl, verbose=True)
        else:
            print(f"Searching for {args.leader} leader arm...", flush=True)
            leader_port, detected_kind = find_any_leader(
                exclude=_excl, verbose=True, prefer=args.leader,
            )
            if detected_kind != args.leader:
                # Honour the explicit --leader choice — don't silently use a different kind.
                leader_port, detected_kind = None, None
        if leader_port:
            print(f"{detected_kind} auto-detected on {leader_port}")
            _leader_kind = detected_kind
            _leader_cls  = get_leader_class(detected_kind)
            if args.calib is None:
                _leader_calib = str(default_calib_path(detected_kind))
        else:
            print("Leader not detected — continuing without leader tracking "
                  "(plug it in any time, or pass --port to force)")
    elif leader_port is None and _so101_available:
        # Fallback path when leader.py is missing — original SO-101-only code.
        _excl = [args.estop_port] if args.estop_port else []
        print("Searching for SO-101 leader arm...", flush=True)
        leader_port = find_so101_port(exclude=_excl, verbose=True)
        if leader_port:
            print(f"SO-101 auto-detected on {leader_port}")

    if leader_port is not None and _leader_cls is None:
        print("Leader-arm support not available (missing leader modules)")
        leader_port = None

    if leader_port is not None:
        if _try_install_leader(leader_port):
            print(f"{_leader_kind or 'leader'} connected on {leader_port}")
            # Drain the pending hand-off into local bindings immediately
            # (main loop hasn't started yet).
            with _install_lock:
                _p = _install_pending.pop("data", None)
            if _p is not None:
                leader           = _p["leader"]
                zero_offsets     = _p["zero_offsets"]
                directions       = _p["directions"]
                _so101_for_aizee = _p["for_aizee"]
                _emits_urdf      = _p.get("emits_urdf", False)
                # Publish mapping to the cmd-thread shared state so live
                # TRACKING-mode q_cmd computation can see it.
                with _lr_lock:
                    _lr_mapping["zero_offsets"] = zero_offsets
                    _lr_mapping["directions"]   = directions
                    _lr_mapping["for_aizee"]    = _so101_for_aizee
                    _lr_mapping["emits_urdf"]   = _emits_urdf
        else:
            print(f"{_leader_kind or 'leader'} connect failed on {leader_port} — "
                  "continuing; will retry when port reappears")

    # Background hot-plug watcher.  Runs whenever a leader is not currently
    # installed; only probes when the port set changes (cheap enumeration is
    # the trigger; expensive sync-read is gated).
    _hp_stop = threading.Event()

    def _leader_hotplug_watcher() -> None:
        if not _pyserial_available:
            return
        try:
            from serial.tools import list_ports
        except ImportError:
            return
        excl = {args.estop_port} if args.estop_port else set()
        # Which kinds the watcher will accept on hot-plug.
        if args.leader == "auto":
            kinds = list(LEADER_KINDS)
        else:
            kinds = [args.leader]
        try:
            prev = {p.device for p in list_ports.comports()}
        except Exception:
            prev = set()
        while not _hp_stop.is_set():
            if _hp_stop.wait(1.5):
                return
            if _leader_box["leader"] is not None:
                continue
            try:
                cur = {p.device for p in list_ports.comports()}
            except Exception:
                continue
            new_ports = (cur - prev) - excl
            prev = cur
            if not new_ports:
                continue
            for dev in sorted(new_ports):
                # Probe for each acceptable kind in the configured order.
                detected = None
                if _leader_module_available:
                    for k in kinds:
                        try:
                            from leader import probe_port
                            ok, _ = probe_port(dev, k)
                        except Exception:
                            ok = False
                        if ok:
                            detected = k
                            break
                else:
                    if _so101_available:
                        ok, _ = _probe_so101(dev)
                        if ok:
                            detected = "so101"
                if detected is None:
                    continue
                if _try_install_leader(dev, kind=detected):
                    print(f"{detected} hot-plugged on {dev}", flush=True)
                    return  # one-shot install; future unplug/replug not supported

    _hp_thread: Optional[threading.Thread] = None
    if _pyserial_available and (_leader_module_available or _so101_available):
        _hp_thread = threading.Thread(target=_leader_hotplug_watcher, daemon=True,
                                      name="LeaderHotPlug")
        _hp_thread.start()

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    _yaml      = _load_teleop_yaml()
    # Arm gains are 7-DOF (swivel + 6 gantry).  Older configs that still have
    # split `gantry.kp` + `drive.swivel_kp` are migrated on the fly.
    _acfg: dict = _yaml.get("arm", {})
    if "kp" in _acfg and "kd" in _acfg:
        _kp: list = list(_acfg["kp"])
        _kd: list = list(_acfg["kd"])
    else:
        _gan = _yaml.get("gantry", {})
        _drv = _yaml.get("drive", {})
        _kp = [float(_drv.get("swivel_kp", KP[0]))] + list(_gan.get("kp", KP[1:]))
        _kd = [float(_drv.get("swivel_kd", KD[0]))] + list(_gan.get("kd", KD[1:]))
    if len(_kp) != NUM_JOINTS or len(_kd) != NUM_JOINTS:
        print(f"WARNING: arm gains length {len(_kp)}/{len(_kd)} != {NUM_JOINTS}; "
              f"falling back to record_replay defaults", flush=True)
        _kp = list(KP)
        _kd = list(KD)
    _dcfg        = _yaml.get("drive", {})
    _max_linear  = float(_dcfg.get("max_linear",  2.0))
    _max_angular = float(_dcfg.get("max_angular", 1.5))
    _drive_kp    = float(_dcfg.get("kp", 0.0))
    _drive_kd    = float(_dcfg.get("kd", 3.0))
    _gp_cfg      = _yaml.get("gamepad", {})

    # -------------------------------------------------------------------------
    # Live replay controller (on-robot playback from GUI Replay tab)
    # -------------------------------------------------------------------------
    live_replay = _LiveReplay(
        kp=_kp, kd=_kd,
        max_delta=args.max_delta,
        arm_limits=arm_limits,
        all_motor_ids=list(ARM_JOINTS),
    )

    # -------------------------------------------------------------------------
    # ZMQ sockets
    # -------------------------------------------------------------------------
    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 4)
    cmd_sock.setsockopt(zmq.LINGER,  0)
    cmd_sock.connect(args.cmd)

    # Telemetry parsing runs in its own thread (json.loads of multi-motor
    # state was a 6-10 ms p99 spike when done in the main loop).  Started
    # before the cmd-sender thread because the cmd-sender consumes
    # _telem_cache for the lead-clamp in TRACKING mode.
    _telem_stop, _telem_thread, _telem_lock, _telem_cache = \
        _start_telem_receiver(ctx, args.telem)
    _telem_last_time = 0.0  # last cache["time"] consumed by main loop

    # Dedicated cmd-sender thread.  Re-emits the latest Bundle at 100 Hz
    # in "static" mode (HOLD/IDLE/ENGAGING/SHUTDOWN), or computes q_cmd
    # live from the leader thread's latest sample in "tracking" mode —
    # which lifts the 30 Hz aliasing cap on the leader→cmd path.
    _cmd_tx_stop, _cmd_tx_thread, _cmd_lock, _cmd_holder = \
        _start_cmd_sender(
            cmd_sock,
            _lr_lock, _lr_latest, _lr_mapping,
            _telem_lock, _telem_cache,
        )

    def _post_bundle(arm=None, drive=None) -> None:
        """Post a static (prebuilt) Bundle.  Used for IDLE/HOLD/ENGAGING/SHUTDOWN."""
        msg = _build_bundle(arm, drive)
        with _cmd_lock:
            _cmd_holder["mode"]     = "static" if msg is not None else None
            _cmd_holder["bundle"]   = msg
            _cmd_holder["tracking"] = None

    def _post_tracking(*, drive: dict, kp: list, kd: list,
                       max_vel: float, max_lead: float,
                       arm_limits, vel_ff: bool = True) -> None:
        """Switch the cmd thread into TRACKING mode.

        The cmd thread will read `_lr_latest` directly at 100 Hz and compute
        q_cmd live with rate limiting per real-time elapsed.  `drive` and
        the gains are still owned by the main loop (so WASD / gain tuning
        flow through the same channel).
        """
        cfg = {
            "kp":         list(kp),
            "kd":         list(kd),
            "max_vel":    float(max_vel),
            "max_lead":   float(max_lead),
            "vel_ff":     bool(vel_ff),
            "drive":      drive,
            "arm_limits": arm_limits,
        }
        with _cmd_lock:
            _cmd_holder["mode"]     = "tracking"
            _cmd_holder["bundle"]   = None
            _cmd_holder["tracking"] = cfg

    def _clear_bundle() -> None:
        """Stop the cmd thread from emitting anything."""
        with _cmd_lock:
            _cmd_holder["mode"]     = None
            _cmd_holder["bundle"]   = None
            _cmd_holder["tracking"] = None

    # Camera + UPS reception runs in a background thread so that
    # JSON-parsing large JPEG frames never delays motor commands.
    # Scene cam is optional (static-mount mode); in rover mode the publisher
    # never streams.  We always subscribe when an endpoint is configured, but
    # treat the scene cam as *present* only once a frame actually arrives
    # (`_scene_cam_seen`, latched below in the main loop).  That keeps rover
    # sessions from (a) showing a dead UI tile and (b) dropping every recorded
    # frame waiting for a scene image that never comes.
    _scene_cam_configured = bool(args.scene_cam)
    _scene_cam_seen       = False   # latched True on first fresh scene frame
    _cam_stop, _cam_thread, _cam_lock, _cam_cache = _start_cam_receiver(
        ctx, args.gripper_cam, args.ups or None,
        scene_ep=(args.scene_cam or None),
    )

    # Background image decoder (base64 + JPEG + resize off main loop).
    # Always-on when the GUI or Rerun viewer needs decoded frames live.
    _dec_always_on = args.gui or (not args.no_rerun and _rerun_available)
    _dec_stop, _dec_thread, _dec_lock, _dec_cache, _rec_flag = \
        _start_image_decoder(_cam_lock, _cam_cache, img_size, always_on=_dec_always_on)

    # -------------------------------------------------------------------------
    # Hardware e-stop (ESP32 serial)
    # -------------------------------------------------------------------------
    _estop_flag = threading.Event()   # set = e-stop active
    _estop_stop = threading.Event()
    _estop_thread: Optional[threading.Thread] = None
    if args.estop_port:
        _estop_thread = _start_estop_reader(args.estop_port, _estop_stop, _estop_flag)

    # -------------------------------------------------------------------------
    # Rerun live camera preview (terminal mode only — GUI uses native Qt
    # widgets for cameras + scalars, which avoids the WASM/gRPC/Chromium
    # pipeline that backs up unboundedly on weak CPUs).
    # -------------------------------------------------------------------------
    use_rerun = not args.no_rerun and not args.gui
    if use_rerun and not _rerun_available:
        print("WARNING: rerun not installed — live camera preview disabled")
        use_rerun = False
    if use_rerun:
        rr.init("aizee_collect")
        rr.spawn(memory_limit="1GiB")
        _joint_names = list(ARM_JOINTS)   # swivel + 6 gantry, in joint order
        rr.set_time("time", timestamp=time.time())
        for _jn in _joint_names:
            rr.log(f"joints/{_jn}", rr.Scalars(0.0))
            rr.log(f"leader/{_jn}", rr.Scalars(0.0))
        rr.send_blueprint(rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial2DView(name="Gripper", origin="cameras/gripper"),
                rrb.TimeSeriesView(
                    name="Joint Positions",
                    contents=[f"joints/{j}" for j in _joint_names]
                            + [f"leader/{j}" for j in _joint_names],
                ),
                row_shares=[2, 1],
            )
        ))

    get_key = setup_keyboard()

    # Seed q_actual from first telemetry packet
    q_actual: Optional[np.ndarray] = None
    for _ in range(40):
        with _telem_lock:
            telem = _telem_cache["msg"]
            tt    = _telem_cache["time"]
        if telem:
            _telem_last_time = tt
            q = _qpos(telem)
            if q is not None:
                q_actual = q
                break
        time.sleep(0.05)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------
    class State(enum.Enum):
        READY    = "ready"
        IDLE     = "idle"
        TRACKING = "tracking"
        HOLD     = "hold"
        ENGAGING = "engaging"   # rate-limited approach to leader before TRACKING
        SHUTDOWN = "shutdown"
        ESTOP    = "estop"

    # Engagement parameters: when E is pressed from READY/IDLE, the arm ramps
    # toward the leader pose at a bounded rate (instead of snapping at full
    # PD authority).  Once the arm is within ENGAGE_DONE_THRESHOLD of the
    # leader on every joint, the state auto-promotes to TRACKING.
    ENGAGE_DELTA          = 0.015  # rad/tick (~0.45 rad/s @ 30 Hz) — slow ramp
    ENGAGE_WARN_THRESHOLD = 0.20   # rad — show warning toast above this gap
    ENGAGE_DONE_THRESHOLD = 0.04   # rad — promote to TRACKING below this gap

    # Per-joint cap on q_cmd lead vs. q_actual during ENGAGING.
    # Sized so each joint can demand exactly its rated motor torque
    # (kp · lead = sat_torque).  A flat 0.05 rad cap left high-kp joints
    # (gantry_base @ kp=200) requesting only 10 N·m — below the stiction +
    # gravity load needed to break the joint loose, leading to permanent
    # stuck-engaging states.  Sizing per-joint avoids that without giving
    # the controller windup margin: at saturation, more lead would not
    # increase delivered torque, just store position error to dump on the
    # joint when it finally breaks free.
    # Per-joint cap on q_cmd lead vs. q_actual during ENGAGING.  Swivel is
    # joint 0 — its sat_torque/kp falls out of the same per-joint formula
    # the gantry uses, so there's no separate `_engage_lead_sw` anymore.
    _engage_lead_arm = np.array(
        [_SAT_TORQUE[j] / float(_kp[i]) for i, j in enumerate(ARM_JOINTS)],
        dtype=np.float32,
    )

    teleop_state                          = State.READY
    held_target:     Optional[np.ndarray] = None
    engage_q_cmd:    Optional[np.ndarray] = None
    engage_warned:   bool                 = False
    shutdown_countdown: float             = 0.0
    shutdown_target: Optional[np.ndarray] = None
    shutdown_zero_since: float            = 0.0   # when ramp first hit zero
    _SHUTDOWN_TIMEOUT                     = 3.0   # force-disable after this many seconds at zero
    arm_torques:     Optional[np.ndarray] = None
    arm_temps:       Optional[np.ndarray] = None
    arm_states:      list                 = ["?"] * NUM_JOINTS
    last_telem_time: float             = time.time() if q_actual is not None else 0.0
    ups_data:        Optional[dict]    = None
    battery_voltage: Optional[float]   = None
    robot_ok = q_actual is not None
    estop_active = False
    prev_estop_hw = False

    # Drive state (wheels)
    drive_linear         = 0.0   # current smoothed linear (-1..+1)
    drive_angular        = 0.0   # current smoothed angular (-1..+1)
    drive_linear_target  = 0.0
    drive_angular_target = 0.0
    _drive_accel         = 50.0  # instant on key press
    _drive_decel         = 8.0   # smooth release
    _last_w_time         = 0.0   # WASD timeout tracking
    _last_s_time         = 0.0
    _last_a_time         = 0.0
    _last_d_time         = 0.0
    _wasd_timeout        = 0.15  # seconds — clear target if no repeat
    wheel_states:  Optional[dict] = None   # telemetry for wheel motors

    zero_msg       = ""
    zero_msg_until = 0.0
    save_msg       = ""
    save_msg_until = 0.0

    joystick           = _init_joystick() if _pygame_available else None
    prev_gp_a:   bool  = False
    prev_gp_b:   bool  = False
    prev_gp_start:bool = False

    # M5 Joystick2 (on the OpenRB-150 leader board, I2C addr 0x63 on D11/D12).
    # `prev_m5_press_counter` lets the 30 Hz main loop edge-detect button
    # presses captured at ~500 Hz by the leader-reader thread without
    # missing quick clicks.  `_m5_dz` is the analog-stick deadzone shared
    # with the curve / ramp code below.
    prev_m5_press_counter: int = 0
    _m5_dz                     = 0.08

    # Recording state.  qpos_buf / qcmd_buf / torque_buf are now 7-DOF
    # (swivel-first, matches ARM_JOINTS) — there's no separate swivel buffer.
    # scene_buf / scene_ts_buf are populated only when --scene-cam is set
    # AND a frame actually decoded — see the recording append below.
    recording      = False
    qpos_buf:     list = []
    qcmd_buf:     list = []
    torque_buf:   list = []
    gripper_buf:  list = []
    scene_buf:    list = []
    telem_ts_buf:   list = []
    gripper_ts_buf: list = []
    scene_ts_buf:   list = []
    dropped_frames = 0
    last_rec_time  = 0.0

    # Episode metadata (GUI can mutate task_tag / notes live via the holder)
    _meta: dict = {"task_tag": args.task_tag, "notes": ""}
    last_saved_path: Optional[Path] = None

    # Camera state
    last_gripper_time   = 0.0
    last_scene_time     = 0.0
    latest_gripper: Optional[dict] = None
    latest_scene:   Optional[dict] = None
    latest_telem_ts: Optional[float] = None
    latest_gripper_ts:  Optional[float] = None
    latest_scene_ts:    Optional[float] = None
    latest_q_cmd: Optional[np.ndarray] = None  # last commanded position sent to motors
    # Scene cam push to the GUI is rate-capped (separate from publisher
    # rate / recording rate). The publisher runs at 30 Hz and the
    # decoder thread keeps _dec_cache fresh at that rate for recording.
    # The GUI feed is dropped to ~6 Hz because the 3D model preview's
    # paint cost (hull projection + pointcloud + frustum + thumbnail)
    # dominates the QtRenderer thread; visually 6 Hz is plenty for a
    # fixed workspace cam and frees ~80% of the per-second paint budget
    # compared to 30 Hz.
    _SCENE_GUI_PERIOD = 0.18
    _last_scene_gui_push = 0.0

    # Leader force-feedback state (OpenRB gripper only).  Smoothed copy of
    # follower gripper torque; LPF coefficient is chosen so a noisy 0.05 N·m
    # ripple decays to <10% within ~5 ticks (~170 ms at 30 Hz) — not so
    # heavy that real grip events feel mushy.
    _ff_gripper_lpf:  float = 0.0
    _FF_LPF_ALPHA            = 0.30
    _ff_was_active           = False   # tracks last-tick FF state for clean release

    status = "[ ] ready — motors off"
    hint   = ("E=hold · I=idle · Q=quit" if leader is None
              else "E=track · I=idle · Z=zero · M=mirror · Q=quit")

    _nan = float("nan")
    # Display arrays now match q_actual shape directly (swivel = index 0).
    _init_actual = q_actual.copy() if q_actual is not None else None

    # -------------------------------------------------------------------------
    # Display: terminal renderer (default) or Qt GUI (--gui)
    # -------------------------------------------------------------------------
    gui_cmd_queue: queue.Queue = queue.Queue(maxsize=32)
    _qt_renderer = None
    _disp_thread = None
    _disp_stop: Optional[threading.Event] = None
    _disp_event: Optional[threading.Event] = None

    if args.gui:
        from collect_demo_gui import QtRenderer

        import os
        def _on_delete_last(path: Path) -> None:
            os.remove(path)

        _qt_renderer = QtRenderer(
            cmd_queue=gui_cmd_queue,
            meta=_meta,
            on_delete_last=_on_delete_last,
            output_dir=Path(args.output_dir),
            camera_ctrl_endpoint=(args.gripper_cam_ctrl or None),
        )
        _disp_lock   = _qt_renderer.lock
        _disp_holder = _qt_renderer.holder
        _disp_cams   = _qt_renderer.cam_holder
        _disp_stop   = _qt_renderer.stop_event
        _qt_renderer.start()
    else:
        _disp_stop, _disp_thread, _disp_lock, _disp_holder, _disp_event = \
            _start_display_thread()
        _disp_cams: Optional[dict] = None

    # Queue the initial frame (first=True is the default in holder)
    with _disp_lock:
        _disp_holder["args"] = dict(
            leader_rad=None, target=None, actual=_init_actual,
            status=status, hint=hint, robot_ok=robot_ok,
            leader_connected=(leader is not None),
            wheel_states=wheel_states, wheels_enabled=False,
        )
    if _disp_event is not None:
        _disp_event.set()

    # -------------------------------------------------------------------------
    # Background Rerun thread (avoids blocking main loop on rr.log IPC)
    # -------------------------------------------------------------------------
    _rr_stop:   Optional[threading.Event]  = None
    _rr_thread: Optional[threading.Thread] = None
    _rr_lock:   Optional[threading.Lock]   = None
    _rr_holder: Optional[dict]             = None
    _rr_event:  Optional[threading.Event]  = None
    if use_rerun:
        _rr_stop, _rr_thread, _rr_lock, _rr_holder, _rr_event = \
            _start_rerun_thread(_dec_lock, _dec_cache)

    frame_counter = 0
    period = 1.0 / LOOP_HZ

    _prof_log_path = Path(__file__).resolve().parent.parent.parent / "logs" / "loop_prof.log"
    _prof = _LoopProfiler(log_path=_prof_log_path)

    _save_thread:        Optional[threading.Thread] = None
    _save_result_holder: list                       = [None]

    def _start_async_save(out_dir, qb, gb, tb, gtb, dur, drop_note, tag="",
                          qcb=None, tqb=None, sb=None, stb=None,
                          task_tag="", notes=""):
        def _run():
            try:
                p, T = save_episode(
                    out_dir, qb, gb,
                    telem_ts_buf=tb, gripper_ts_buf=gtb,
                    qcmd_buf=qcb, torque_buf=tqb,
                    scene_buf=sb, scene_ts_buf=stb,
                    task_tag=task_tag, notes=notes,
                )
                _save_result_holder[0] = (p, f"[SAVED {p.name}  {T} steps  {dur:.1f}s{drop_note}]{tag}")
            except Exception as e:
                _save_result_holder[0] = (None, f"[SAVE ERROR: {e}]")
        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

    def _finalize_recording(reason: str, t_now: float) -> None:
        """Stop recording and dispatch an async save (or dry-run / skip-empty).

        reason is a short free-text suffix shown in status + attached to the
        save-success message ("" = user R toggle; " (hw e-stop)" / " (e-stop)" /
        " (max steps)" for auto-stop paths).
        """
        nonlocal recording, save_msg, save_msg_until, _save_thread
        if not recording:
            return
        recording = False
        _rec_flag.clear()
        steps     = len(qpos_buf)
        dur       = steps / REC_HZ
        drop_note = f"  drop:{dropped_frames}" if dropped_frames else ""
        tag_txt   = reason if reason else ""

        if steps == 0:
            save_msg       = f"[STOPPED{tag_txt} — 0 steps, nothing saved]"
            save_msg_until = t_now + 5.0
            return

        if args.dry_run:
            save_msg       = f"[DRY RUN]{tag_txt} {steps} steps  {dur:.1f}s{drop_note}"
            save_msg_until = t_now + 5.0
            return

        save_msg               = f"[saving {steps} steps{tag_txt}...]"
        save_msg_until         = t_now + 120.0
        _save_result_holder[0] = None
        _save_thread = _start_async_save(
            args.output_dir, qpos_buf, gripper_buf,
            telem_ts_buf, gripper_ts_buf,
            dur, drop_note, tag=tag_txt, qcb=qcmd_buf, tqb=torque_buf,
            sb=scene_buf, stb=scene_ts_buf,
            task_tag=_meta["task_tag"], notes=_meta["notes"],
        )

    try:
        while True:
            t0 = time.time()
            _prof.begin()

            # -----------------------------------------------------------------
            # Hot-plug: install a leader handed off by the watcher thread
            # -----------------------------------------------------------------
            if _install_pending:
                with _install_lock:
                    _p = _install_pending.pop("data", None)
                if _p is not None:
                    leader           = _p["leader"]
                    zero_offsets     = _p["zero_offsets"]
                    directions       = _p["directions"]
                    _so101_for_aizee = _p["for_aizee"]
                    _emits_urdf      = _p.get("emits_urdf", False)
                    # Publish to the cmd-thread mapping state.
                    with _lr_lock:
                        _lr_mapping["zero_offsets"] = zero_offsets
                        _lr_mapping["directions"]   = directions
                        _lr_mapping["for_aizee"]    = _so101_for_aizee
                        _lr_mapping["emits_urdf"]   = _emits_urdf
                    print(f"[hot-plug] leader installed — {len(_so101_for_aizee)} arm joints mapped",
                          flush=True)

            # -----------------------------------------------------------------
            # Pick up completed background save
            # -----------------------------------------------------------------
            if _save_thread is not None and not _save_thread.is_alive():
                if _save_result_holder[0] is not None:
                    _saved_path, save_msg = _save_result_holder[0]
                    save_msg_until  = t0 + 5.0
                    last_saved_path = _saved_path
                    _save_result_holder[0] = None
                _save_thread = None

            # -----------------------------------------------------------------
            # Read cached camera data (populated by background thread)
            # -----------------------------------------------------------------
            with _cam_lock:
                if _cam_cache["gripper"] is not None:
                    latest_gripper    = _cam_cache["gripper"]
                    last_gripper_time = _cam_cache["gripper_time"]
                    latest_gripper_ts = _cam_cache["gripper_ts"]
                if _cam_cache.get("scene") is not None:
                    latest_scene    = _cam_cache["scene"]
                    last_scene_time = _cam_cache["scene_time"]
                    latest_scene_ts = _cam_cache.get("scene_ts")

            cam_age = (t0 - last_gripper_time) if last_gripper_time > 0 else 999.0
            # End-to-end frame age (publisher capture timestamp → host now).
            # Includes any clock skew between Jetson and host; we care about
            # *drift* over time, which is skew-invariant.
            if latest_gripper_ts is not None:
                _prof.gauge("gripper_age_ms", (t0 - latest_gripper_ts) * 1000.0)
            # Time since this loop last *received* a new cam frame (host-only,
            # no clock-skew component) — flags publisher gaps directly.
            _prof.gauge("gripper_recv_age_ms", cam_age * 1000.0)
            if _scene_cam_configured:
                scene_age = (t0 - last_scene_time) if last_scene_time > 0 else 999.0
                if latest_scene_ts is not None:
                    _prof.gauge("scene_age_ms", (t0 - latest_scene_ts) * 1000.0)
                _prof.gauge("scene_recv_age_ms", scene_age * 1000.0)
                # Latch presence the first time a fresh scene frame arrives —
                # distinguishes static mode (scene present) from rover mode.
                if not _scene_cam_seen and scene_age < _CAM_STALE:
                    _scene_cam_seen = True
            else:
                scene_age = 999.0

            # Wake the Rerun thread (~15 Hz, every other frame).  Camera
            # frames are pulled from the shared decoder cache by the
            # Rerun thread itself — no JPEG copy through the main loop.
            # Joint data is queued later, after telemetry + leader are read.
            if _rr_event is not None and (frame_counter % 2 == 0):
                with _rr_lock:
                    _rr_holder["time"] = t0
                _rr_event.set()

            # Push raw JPEG bytes to the GUI's native camera widget — only
            # when a new frame has actually arrived (last_*_time changed),
            # otherwise we'd re-feed the same JPEG every loop tick.
            if _disp_cams is not None:
                push_g = (latest_gripper is not None
                          and last_gripper_time > _disp_cams["gripper_ts"])
                if push_g:
                    gj_bytes = latest_gripper.get("color", {}).get("data_bytes")
                    if gj_bytes is not None:
                        with _disp_lock:
                            _disp_cams["gripper"]    = gj_bytes
                            _disp_cams["gripper_ts"] = last_gripper_time
                        # Mirror to the WebXR client (no-op when not in quest mode).
                        if _quest_state is not None:
                            _quest_state.latest_cam_jpeg = gj_bytes
                            _quest_state.latest_cam_seq += 1
                # Scene cam: rate-cap to _SCENE_GUI_PERIOD so the GUI
                # thread isn't slammed by 30 Hz × (paint + pointcloud).
                # Recording continues at REC_HZ through _dec_cache — this
                # throttle ONLY affects the 3D model preview + scene-tile
                # refresh rate.
                #
                # The GUI gets pre-projected camera-frame points from the
                # decoder thread (heavy numpy already done there), the
                # paired publisher ts, and the raw JPEG bytes for the
                # tile + thumbnail (Qt's native decoder is fast). No depth
                # bytes / intrinsics travel through here anymore.
                push_s = (latest_scene is not None
                          and last_scene_time > _disp_cams.get("scene_ts", 0.0)
                          and (t0 - _last_scene_gui_push) >= _SCENE_GUI_PERIOD)
                if push_s:
                    with _dec_lock:
                        sj_img       = _dec_cache.get("scene")
                        sj_ts_pub    = _dec_cache.get("scene_ts_pub")
                        s_pts_cam    = _dec_cache.get("scene_pts_cam")
                        s_pts_depth  = _dec_cache.get("scene_pts_depth")
                    sj_bytes = latest_scene.get("color", {}).get("data_bytes")
                    if sj_bytes is not None and sj_img is not None:
                        with _disp_lock:
                            _disp_cams["scene"]           = sj_bytes
                            _disp_cams["scene_ts"]        = (
                                sj_ts_pub if sj_ts_pub is not None
                                else last_scene_time)
                            _disp_cams["scene_pts_cam"]   = s_pts_cam
                            _disp_cams["scene_pts_depth"] = s_pts_depth
                        _last_scene_gui_push = t0

            _prof.tick("cam")

            # -----------------------------------------------------------------
            # Gamepad + drive axes
            # -----------------------------------------------------------------
            # When pygame handles WASD, drain terminal buffer discarding WASD
            # so held W doesn't starve command keys (E, Q, etc.)
            if _pygame_available:
                key = None
                while True:
                    _k = get_key()
                    if _k is None:
                        break
                    if _k not in ("W", "A", "S", "D"):
                        key = _k
            else:
                key = get_key()

            # GUI button presses flow through the same key dispatch as keyboard.
            # Drain one per frame so rapid clicks don't starve state transitions.
            try:
                key = gui_cmd_queue.get_nowait()
            except queue.Empty:
                pass

            # Dict commands (live-replay control protocol from GUI)
            if isinstance(key, dict):
                _cmd = key.get("cmd", "")
                if _cmd == "replay_on":
                    if recording:
                        _finalize_recording(" (replay)", t0)
                    _path = Path(key.get("path", ""))
                    err = live_replay.load(_path)
                    if err is None and live_replay.enter_live():
                        # Park teleop while replaying — motors off until user arms
                        teleop_state = State.READY
                        save_msg       = f"[replay loaded] {_path.name}"
                        save_msg_until = t0 + 3.0
                    else:
                        save_msg       = f"[replay load failed] {err or 'no episode'}"
                        save_msg_until = t0 + 5.0
                elif _cmd == "replay_off":
                    if not live_replay.exit_live():
                        save_msg       = "[replay] stop first before exiting live mode"
                        save_msg_until = t0 + 3.0
                elif _cmd == "replay_arm":
                    for _c in live_replay.arm(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_play":
                    for _c in live_replay.play(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_pause":
                    live_replay.pause()
                elif _cmd == "replay_toggle":
                    for _c in live_replay.toggle(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_restart":
                    for _c in live_replay.restart(q_actual):
                        _send(cmd_sock, _c)
                elif _cmd == "replay_stop":
                    live_replay.stop(q_actual)
                elif _cmd == "replay_speed":
                    live_replay.set_speed(float(key.get("speed", 1.0)))
                elif _cmd == "replay_opts":
                    live_replay.set_opts(**{k: v for k, v in key.items() if k != "cmd"})
                elif _cmd == "leader_quest_on":
                    # GUI-driven Quest VR enable.  Builds server lazily on
                    # first toggle; subsequent toggles reuse the same state.
                    if _install_quest_leader():
                        save_msg = f"[quest] connected on :{args.quest_port}"
                        save_msg_until = t0 + 4.0
                    else:
                        save_msg = "[quest] connect failed (see console)"
                        save_msg_until = t0 + 4.0
                elif _cmd == "leader_quest_off":
                    _uninstall_leader()
                    save_msg = "[quest] disconnected"
                    save_msg_until = t0 + 3.0
                elif _cmd == "quest_sim_reset":
                    if _quest_state is not None:
                        _quest_state.pending_commands.append({"cmd": "reset_sim"})
                        save_msg = "[quest] sim reset to home"
                        save_msg_until = t0 + 2.0
                elif _cmd == "quest_sim_align":
                    if _quest_state is not None:
                        _quest_state.pending_commands.append({"cmd": "align_to_actual"})
                        save_msg = "[quest] sim aligned to arm"
                        save_msg_until = t0 + 2.0
                key = None

            # Block teleop motor/recording keys while live replay owns the arm
            if live_replay.live and key in ("E", "I", "H", "X", "R", "Z", "M", "P",
                                            "PEDAL_A", "PEDAL_B", "PEDAL_C"):
                key = None

            # -----------------------------------------------------------------
            # M5 Joystick2 (on the OpenRB-150 leader, I2C 0x63 on D11/D12).
            # Read first so it takes precedence over the xbox gamepad and
            # the WASD path below — operator's hand-stick is the primary
            # drive when present.  Button-press edges are detected against
            # the leader thread's monotonic counter so quick clicks survive
            # the 30 Hz main-loop cadence.
            # -----------------------------------------------------------------
            with _lr_lock:
                m5 = dict(_lr_latest["joy"])
            _m5_active = False
            if m5["present"]:
                # Map axes to drive (matches xbox convention in _read_gamepad):
                #   Y → angular (forward/back, negated so stick-fwd = robot-fwd)
                #   X → linear  (turn)
                _m5_x = _apply_curve(_apply_deadzone(m5["x"], _m5_dz))
                _m5_y = _apply_curve(_apply_deadzone(m5["y"], _m5_dz))
                _m5_active = (abs(_m5_x) > 0.01 or abs(_m5_y) > 0.01)

                # Button-press edge → toggle recording, but only when it
                # would actually do something.  When idle / ready / engaging
                # (or while live replay owns the arm) we silently ignore the
                # click — matches user policy "do nothing if the arm is not
                # tracking".  The press counter is always consumed so a
                # silently-ignored edge can't fire later under different state.
                if m5["press_counter"] != prev_m5_press_counter:
                    can_toggle = (recording or teleop_state == State.TRACKING)
                    if can_toggle and not live_replay.live:
                        key = "R"
                    prev_m5_press_counter = m5["press_counter"]

            _stick_active = False
            if joystick is not None:
                gp = _read_gamepad(joystick, prev_gp_a, prev_gp_b, prev_gp_start,
                                   gp_cfg=_gp_cfg)
                prev_gp_a     = gp["raw_a"]
                prev_gp_b     = gp["raw_b"]
                prev_gp_start = gp["raw_start"]
                # Stick axes → drive targets (always apply, 0 when centered)
                _stick_active = (abs(gp["drive_linear"]) > 0.01
                                 or abs(gp["drive_angular"]) > 0.01)
                drive_linear_target  = gp["drive_linear"]
                drive_angular_target = gp["drive_angular"]
                if gp["enable"] and teleop_state in (State.READY, State.IDLE):
                    key = "E"
                if gp["hold"] and teleop_state in (State.TRACKING, State.HOLD,
                                                    State.IDLE, State.ENGAGING):
                    key = "H"
                if gp["shutdown"]:
                    key = "CANCEL_SHUTDOWN" if teleop_state == State.SHUTDOWN else "X"
                if gp["quit"]:
                    key = "Q"

            # M5 stick wins over xbox when deflected — operator's primary
            # input.  When the M5 is centred, fall back to whatever the
            # xbox stick set above (or to WASD below if neither stick is live).
            if _m5_active:
                drive_angular_target = -_m5_y
                drive_linear_target  =  _m5_x
                _stick_active        = True

            # -----------------------------------------------------------------
            # WASD drive input — pygame true key state (no repeat delay)
            # Matches teleop.py read_keyboard_pygame(): instant on/off.
            # -----------------------------------------------------------------
            if _pygame_available:
                # Pump events if no joystick did it already
                if joystick is None:
                    pygame.event.pump()
                _pkeys = pygame.key.get_pressed()
                # WORKAROUND: motor controller has linear/angular backwards
                # W/S → angular (forward/back), A/D → linear (turn)
                _kb_ang = 0.0
                _kb_lin = 0.0
                if _pkeys[pygame.K_w]:
                    _kb_ang = -1.0
                elif _pkeys[pygame.K_s]:
                    _kb_ang = 1.0
                if _pkeys[pygame.K_d]:
                    _kb_lin = 1.0
                elif _pkeys[pygame.K_a]:
                    _kb_lin = -1.0
                # Keyboard overrides only when stick is idle
                if not _stick_active:
                    drive_angular_target = _kb_ang
                    drive_linear_target  = _kb_lin
            else:
                # Fallback: terminal key with timeout (has OS repeat delay)
                if not _stick_active and key in ("W", "S", "A", "D"):
                    if key == "W":
                        drive_angular_target = -1.0
                        _last_w_time = t0
                    elif key == "S":
                        drive_angular_target = 1.0
                        _last_s_time = t0
                    elif key == "A":
                        drive_linear_target = -1.0
                        _last_a_time = t0
                    elif key == "D":
                        drive_linear_target = 1.0
                        _last_d_time = t0
                    key = None
                if not _stick_active:
                    if (t0 - _last_w_time > _wasd_timeout
                            and t0 - _last_s_time > _wasd_timeout):
                        drive_angular_target = 0.0
                    if (t0 - _last_a_time > _wasd_timeout
                            and t0 - _last_d_time > _wasd_timeout):
                        drive_linear_target = 0.0

            # Zero drive targets while live replay owns the rover
            if live_replay.live:
                drive_linear_target  = 0.0
                drive_angular_target = 0.0

            # Drive smoothing (fast accel, smooth decel)
            drive_linear  = _ramp_toward(drive_linear,  drive_linear_target,
                                         _drive_accel, _drive_decel, period)
            drive_angular = _ramp_toward(drive_angular, drive_angular_target,
                                         _drive_accel, _drive_decel, period)

            # -----------------------------------------------------------------
            # Foot pedal (3-button USB keyboard emitting A/B/C, captured by
            # the Qt GUI and sent through gui_cmd_queue as PEDAL_*).
            #   A → ENABLE     (delegate to "E" handler below)
            #   B → toggle: IDLE while active, RESUME while idle
            #   C → SHUTDOWN   (delegate to "X" handler below)
            # B is handled inline because the keyboard "I" handler only fires
            # from READY/IDLE, but the pedal must reach IDLE from TRACKING /
            # ENGAGING / HOLD too.
            # -----------------------------------------------------------------
            if key == "PEDAL_A":
                key = "E"
            elif key == "PEDAL_C":
                key = "X"
            elif key == "PEDAL_B":
                if teleop_state == State.IDLE:
                    key = "E"   # resume tracking
                elif teleop_state in (State.TRACKING, State.ENGAGING,
                                      State.HOLD):
                    if recording:
                        _finalize_recording(" (pedal idle)", t0)
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    ref = (q_actual.tolist() if q_actual is not None
                           else [0.0] * NUM_JOINTS)
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": ref,
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                    teleop_state = State.IDLE
                    key = None
                else:
                    key = None  # READY / SHUTDOWN — pedal B is a no-op

            # -----------------------------------------------------------------
            # Keyboard (command keys)
            # -----------------------------------------------------------------
            if key == "Q":
                break

            elif key == "I":
                if teleop_state in (State.READY, State.IDLE):
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    ref = q_actual.tolist() if q_actual is not None else [0.0] * NUM_JOINTS
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": ref,
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                    teleop_state = State.IDLE

            elif key == "E":
                if teleop_state in (State.READY, State.IDLE):
                    _send(cmd_sock, {"type": "enable",
                                     "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                    if leader is not None:
                        # Soft engage — seed the integrator from the current
                        # arm pose so ENGAGING ramps slowly to the leader
                        # instead of snapping at full PD authority.
                        engage_q_cmd  = (q_actual.copy().astype(np.float32)
                                         if q_actual is not None else None)
                        engage_warned = False
                        teleop_state  = State.ENGAGING
                    else:
                        held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                        teleop_state = State.HOLD

            elif key == "H":
                if teleop_state in (State.TRACKING, State.ENGAGING):
                    held_target  = q_actual.copy() if q_actual is not None else held_target
                    teleop_state = State.HOLD
                elif teleop_state == State.HOLD:
                    teleop_state = State.TRACKING if leader is not None else State.IDLE
                elif teleop_state == State.IDLE:
                    held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                    teleop_state = State.HOLD

            elif key == "R":
                if not recording:
                    if teleop_state == State.TRACKING:
                        recording      = True
                        _rec_flag.set()   # start background image decoder
                        qpos_buf       = []
                        qcmd_buf       = []
                        torque_buf     = []
                        gripper_buf    = []
                        scene_buf      = []
                        telem_ts_buf   = []
                        gripper_ts_buf = []
                        scene_ts_buf   = []
                        dropped_frames = 0
                        last_rec_time  = 0.0
                    else:
                        save_msg       = "[record blocked] enable tracking first (E)"
                        save_msg_until = t0 + 2.0
                else:
                    _finalize_recording("", t0)

            elif key == "Z" and leader is not None:
                if not getattr(leader, "SUPPORTS_ZEROING", True):
                    # Cartesian VR leader self-zeroes on every clutch engage;
                    # there's no persistent offset to capture, and running
                    # the zeroing math would corrupt the IK mapping.
                    zero_msg       = "[Z] n/a — VR leader self-zeroes on grip"
                    zero_msg_until = t0 + 2.0
                else:
                    with _lr_lock:
                        _z = _lr_latest["rad"]
                    if _z is not None:
                        zero_offsets = _z.copy()
                        leader.save_zero(zero_offsets)
                        # Republish to the cmd-thread mapping so its TRACKING
                        # compute picks up the new zero immediately.
                        with _lr_lock:
                            _lr_mapping["zero_offsets"] = zero_offsets
                        zero_msg       = "[Z] zeroed — saved"
                        zero_msg_until = t0 + 2.0

            elif key == "M" and leader is not None:
                if not getattr(leader, "SUPPORTS_ZEROING", True):
                    zero_msg       = "[M] n/a — VR leader self-zeroes on grip"
                    zero_msg_until = t0 + 2.0
                    _m = None
                else:
                    with _lr_lock:
                        _m = _lr_latest["rad"]
                if _m is not None and q_actual is not None:
                    # Mirror: pin zero so leader maps to current actual.
                    # ARM_JOINTS now includes swivel as joint 0, so a single
                    # loop covers all 7 joints.
                    # q_actual is now URDF frame; for physical leaders the
                    # mapped target is `directions * (leader - zero) * sign`
                    # → solve for zero such that target == q_actual:
                    #   zero = leader - q_actual * sign * direction
                    new_offsets = zero_offsets.copy()
                    for ai, si in enumerate(_so101_for_aizee):
                        s = _ALIGN_SIGNS[ai] if not _emits_urdf else 1.0
                        new_offsets[si] = _m[si] - directions[si] * s * q_actual[ai]
                    zero_offsets = new_offsets
                    leader.save_zero(zero_offsets)
                    with _lr_lock:
                        _lr_mapping["zero_offsets"] = zero_offsets
                    zero_msg       = "[M] mirrored — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "P":
                if q_actual is not None:
                    ready = {
                        "arm_joints": list(ARM_JOINTS),
                        "positions": q_actual.tolist(),
                    }
                    rp_path = Path(__file__).resolve().parent.parent.parent / "config" / "ready_pose.json"
                    rp_path.parent.mkdir(parents=True, exist_ok=True)
                    rp_path.write_text(json.dumps(ready, indent=2))
                    zero_msg       = f"[P] ready pose saved"
                    zero_msg_until = t0 + 3.0
                else:
                    zero_msg       = "[P] no telemetry — cannot save"
                    zero_msg_until = t0 + 2.0

            elif key == "K":
                # Hardware mechanical zero — sends Robstride CAN ZeroPos + SaveConfig
                # to every arm joint. Refused unless motors are fully disabled
                # (motor_control gates the same way), since a running motor may
                # produce a torque spike when zeroed.
                if teleop_state != State.READY:
                    zero_msg       = "[K] disable arm first (X), then press K"
                    zero_msg_until = t0 + 3.0
                else:
                    pre = (", ".join(f"{j}={q_actual[i]:+.3f}"
                                     for i, j in enumerate(ARM_JOINTS))
                           if q_actual is not None else "no telemetry")
                    _send(cmd_sock, {
                        "type": "mech_zero",
                        "motor_ids": list(ARM_JOINTS),
                        "save": True,
                    })
                    print(f"[K] mech_zero sent — pre: {pre}")
                    zero_msg       = "[K] mech zero sent — saved to flash"
                    zero_msg_until = t0 + 4.0

            elif key == "CANCEL_SHUTDOWN" and teleop_state == State.SHUTDOWN:
                teleop_state = State.HOLD
                held_target  = q_actual.copy() if q_actual is not None else held_target

            elif key == "X":
                if teleop_state in (State.TRACKING, State.HOLD, State.IDLE,
                                    State.ENGAGING):
                    shutdown_target    = (q_actual.copy() if q_actual is not None
                                          else held_target.copy() if held_target is not None
                                          else np.zeros(NUM_JOINTS))
                    shutdown_countdown  = 1.0
                    shutdown_zero_since = 0.0
                    teleop_state        = State.SHUTDOWN
                    if recording:
                        recording = False   # stop recording on shutdown
                        _rec_flag.clear()

            _prof.tick("input")

            # -----------------------------------------------------------------
            # Leader data
            # -----------------------------------------------------------------
            leader_rad:    Optional[np.ndarray] = None
            leader_vel:    Optional[np.ndarray] = None
            _clamped_live: Optional[list]       = None
            aizee_cmd:     Optional[np.ndarray] = None    # 7-DOF, swivel-first
            aizee_vel_ff:  Optional[np.ndarray] = None
            leader_age:    float                = 999.0

            if leader is not None:
                with _lr_lock:
                    leader_rad    = _lr_latest["rad"]
                    leader_vel    = _lr_latest["vel"]
                    _clamped_live = _lr_latest["clamped"]
                    _leader_t     = _lr_latest["time"]
                leader_age = t0 - _leader_t if _leader_t > 0 else 999.0
                if leader_rad is not None:
                    mapped = directions * (leader_rad - zero_offsets)
                    aizee_cmd = mapped[_so101_for_aizee]
                    # Convert motor-frame leader mapping → URDF frame for
                    # physical leaders; see _compute_tracking_bundle for
                    # the full explanation.  QuestLeader sets _emits_urdf
                    # so it skips this fold.
                    if not _emits_urdf:
                        aizee_cmd = aizee_cmd * _ALIGN_SIGNS
                if leader_vel is not None:
                    # Velocity has the same sign-flip mapping as position
                    # (zero_offset cancels under differentiation).
                    aizee_vel_ff = (directions * leader_vel)[_so101_for_aizee]
                    if not _emits_urdf:
                        aizee_vel_ff = aizee_vel_ff * _ALIGN_SIGNS

            # Determine target (7-DOF in ARM_JOINTS order; swivel is index 0).
            if live_replay.live:
                target = live_replay.current_target
            elif teleop_state == State.HOLD:
                target = held_target
            elif aizee_cmd is not None:
                target = aizee_cmd
            else:
                target = q_actual

            _prof.tick("leader")

            # -----------------------------------------------------------------
            # Hardware e-stop gate — skip ALL motor commands so watchdog
            # holds position (arm doesn't fall).
            # -----------------------------------------------------------------
            estop_hw_active = _estop_flag.is_set()
            if estop_hw_active and not prev_estop_hw:
                _finalize_recording(" (hw e-stop)", t0)
            prev_estop_hw = estop_hw_active

            # -----------------------------------------------------------------
            # Send motor commands
            # -----------------------------------------------------------------
            if estop_hw_active:
                pass  # watchdog holds position

            elif live_replay.live:
                # Live replay owns the arm — send whatever step() emits.
                # Clear the cmd-thread holder every tick so a stale bundle
                # left over from before replay_on can't keep getting
                # re-emitted at 100 Hz between our 30 Hz replay sends
                # (used to undermine ARMING with leftover kp=0 IDLE holds).
                _clear_bundle()
                for _c in live_replay.step(t0, q_actual, period):
                    _send(cmd_sock, _c)

            # Send drive command every tick (feeds watchdog, enables WASD/stick movement)
            elif teleop_state == State.READY:
                _clear_bundle()  # don't let cmd thread re-emit any prior teleop bundle

            elif teleop_state == State.SHUTDOWN:
                drive_zero = {"linear": 0.0, "angular": 0.0,
                              "kp": _drive_kp, "kd": _drive_kd}
                dt         = period
                max_change = 0.2 * dt   # 0.2 rad/s ramp
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    arm_payload = (
                        {"positions": shutdown_target.tolist(),
                         "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                         "torques": [0.0] * NUM_JOINTS}
                        if shutdown_target is not None else None
                    )
                    _post_bundle(arm=arm_payload, drive=drive_zero)
                else:
                    if shutdown_target is None:
                        shutdown_target = np.zeros(NUM_JOINTS)
                    ref     = q_actual if q_actual is not None else shutdown_target
                    new_tgt = shutdown_target.copy()
                    for i in range(len(new_tgt)):
                        new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                                      else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
                    shutdown_target = new_tgt
                    ramp_done = bool(np.all(np.abs(shutdown_target) < 0.01))
                    if ramp_done and shutdown_zero_since == 0.0:
                        shutdown_zero_since = t0
                    actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
                    timed_out    = (shutdown_zero_since > 0
                                    and t0 - shutdown_zero_since >= _SHUTDOWN_TIMEOUT)
                    if ramp_done and (actual_close or timed_out):
                        # Stop the cmd-sender thread from re-emitting the
                        # last shutdown bundle, then send disable.  We do
                        # NOT reuse the cmd thread for `disable` — it's a
                        # one-shot transition, not a periodic command.
                        _clear_bundle()
                        _send(cmd_sock, {"type": "disable",
                                         "motor_ids": _BASE_MOTORS + list(ARM_JOINTS)})
                        drive_linear = drive_angular = 0.0
                        drive_linear_target = drive_angular_target = 0.0
                        teleop_state = State.READY
                    else:
                        delta   = np.clip(shutdown_target - ref, -args.max_delta, args.max_delta)
                        q_cmd   = ref + delta
                        if arm_limits:
                            q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                        _post_bundle(
                            arm={"positions": q_cmd.tolist(),
                                 "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                                 "torques": [0.0] * NUM_JOINTS},
                            drive=drive_zero,
                        )

            elif teleop_state == State.IDLE:
                arm_payload: Optional[dict] = None
                if q_actual is not None:
                    arm_payload = {
                        "positions": q_actual.tolist(),
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )

            elif teleop_state == State.ENGAGING:
                # One-shot warning when engaging with a large gap to leader.
                if (not engage_warned and target is not None
                        and q_actual is not None):
                    gap = float(np.max(np.abs(target - q_actual)))
                    if gap > ENGAGE_WARN_THRESHOLD:
                        save_msg       = (f"[engaging] leader is {gap:.2f} rad "
                                          f"from arm — ramping slowly")
                        save_msg_until = t0 + 4.0
                    engage_warned = True

                arm_payload: Optional[dict] = None
                if target is not None:
                    # Integrate previous command (not q_actual) to keep the
                    # ramp smooth, then clamp the lead vs. q_actual so the
                    # commanded velocity is bounded by physical motion.
                    ref = (engage_q_cmd if engage_q_cmd is not None
                           else (q_actual if q_actual is not None else target))
                    if q_actual is not None:
                        lead = np.clip(ref - q_actual,
                                       -_engage_lead_arm, _engage_lead_arm)
                        ref  = q_actual + lead
                    delta = np.clip(target - ref, -ENGAGE_DELTA, ENGAGE_DELTA)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                    engage_q_cmd = q_cmd
                    latest_q_cmd = q_cmd.copy()
                    arm_payload = {
                        "positions": q_cmd.tolist(),
                        "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )
                # Promote to TRACKING once every joint is close (swivel is
                # joint 0 of the arm — no separate close-check anymore).
                arm_close = (q_actual is not None and target is not None
                             and np.max(np.abs(target - q_actual))
                                 < ENGAGE_DONE_THRESHOLD)
                if arm_close:
                    teleop_state  = State.TRACKING
                    engage_q_cmd  = None

            elif teleop_state == State.TRACKING:
                # Hand the leader→q_cmd computation off to the cmd-sender
                # thread.  It reads `_lr_latest` directly at 100 Hz and
                # computes q_cmd live with rate-limiting per real-time
                # elapsed — bounding leader→motor latency to ~10 ms instead
                # of the ~33 ms aliasing the 30 Hz main loop used to add.
                #
                # Quest gets tighter limits than the physical leaders: IK
                # near a singularity can throw a multi-radian jump in
                # leader_rad in a single sample, and the operator has no
                # haptic feedback to stop it.  Lower max_vel caps chase
                # speed; lower max_lead caps PD torque during the chase
                # (peak τ ≈ kp · max_lead = 30 · 0.20 = 6 N·m vs the
                # previous 18 N·m).  Engage mode's 0.45 rad/s smoothing
                # is what makes it feel safe — tracking now meets it
                # closer to half-way while staying responsive enough for
                # normal hand motion.
                if _leader_kind == "quest":
                    _tr_max_vel  = 3.0   # rad/s
                    _tr_max_lead = 0.20  # rad
                else:
                    _tr_max_vel  = args.max_delta * LOOP_HZ
                    _tr_max_lead = 2.0 * args.max_delta
                _post_tracking(
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                    kp=_kp, kd=_kd,
                    max_vel=_tr_max_vel,
                    max_lead=_tr_max_lead,
                    arm_limits=arm_limits,
                    vel_ff=True,
                )

            elif teleop_state == State.HOLD:
                # held_target is frozen — no benefit from running the rate
                # limit at 100 Hz, so this stays a static prebuilt bundle.
                arm_payload: Optional[dict] = None
                if target is not None:
                    if latest_q_cmd is not None:
                        ref = latest_q_cmd.copy()
                        if q_actual is not None:
                            _max_lead = 2.0 * args.max_delta
                            ref = q_actual + np.clip(ref - q_actual,
                                                     -_max_lead, _max_lead)
                    elif q_actual is not None:
                        ref = q_actual
                    else:
                        ref = target
                    delta = np.clip(target - ref, -args.max_delta, args.max_delta)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                    latest_q_cmd = q_cmd.copy()
                    arm_payload = {
                        "positions": q_cmd.tolist(),
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": _kp, "kd": _kd,
                        "torques": [0.0] * NUM_JOINTS,
                    }
                _post_bundle(
                    arm=arm_payload,
                    drive={"linear":  drive_linear  * _max_linear,
                           "angular": drive_angular * _max_angular,
                           "kp": _drive_kp, "kd": _drive_kd},
                )

            _prof.tick("motor")

            # -----------------------------------------------------------------
            # Telemetry — pulled from background-thread cache; only consume
            # the message when its timestamp advances so we don't re-parse
            # the same payload tick after tick.
            # -----------------------------------------------------------------
            with _telem_lock:
                _t_msg  = _telem_cache["msg"]
                _t_time = _telem_cache["time"]
            telem = _t_msg if _t_time > _telem_last_time else None
            if telem is not None:
                _telem_last_time = _t_time
            q_new = _qpos(telem)
            if q_new is not None:
                q_actual        = q_new
                robot_ok        = True
                last_telem_time = t0
            # Mirror to WebXR every tick — NOT gated on fresh Jetson telem.
            # Without this, the browser never loads the URDF mirror when no
            # arm is connected (q_new stays None), even though the operator
            # can still drive QuestLeader and see useful HUD state.
            if _quest_state is not None:
                # Use the real arm pose if we have it; otherwise the leader's
                # last commanded q; otherwise a zeros vector so the URDF
                # mirror still loads at home pose.
                _q_for_telem = q_actual
                _hud_ldr = _leader_box.get("leader")
                if _q_for_telem is None and _hud_ldr is not None:
                    _hud_qcmd = getattr(_hud_ldr, "_q_last", None)
                    if _hud_qcmd is not None:
                        _q_for_telem = np.asarray(_hud_qcmd, dtype=np.float32)
                if _q_for_telem is None:
                    _q_for_telem = np.zeros(NUM_JOINTS, dtype=np.float32)
                _telem_out = {
                    "ts":   t0,
                    "qpos": [float(x) for x in _q_for_telem],   # URDF control frame
                    # Per-joint visual offsets [rad].  Added BY THE BROWSER
                    # when rendering the URDF mesh — collect_demo keeps them
                    # OUT of the control loop on purpose (see _ALIGN_OFFSETS
                    # comment).  Forwarded here so hot-reload from /preview
                    # propagates to the VR mirror without a page reload.
                    "align_offsets": [float(x) for x in _ALIGN_OFFSETS],
                    # Camera freshness so the browser can hide the cam panel
                    # (avoiding a big black square) when no frames are flowing.
                    "cam_age": float(cam_age),
                }
                # Raw motor-frame snapshot for the /preview calibration UI.
                # Absent when we don't have arm telem (sim mode etc.); the
                # client falls back to displaying qpos directly in that case.
                _q_motor_now = _qpos_motor(telem) if telem is not None else None
                if _q_motor_now is not None:
                    _telem_out["qpos_motor"] = [float(x) for x in _q_motor_now]
                # Surface leader hud_snapshot (engaged / estop / workspace /
                # qcmd / fk_ee_actual) so the in-headset overlay can render
                # the workspace box, status pills, and the ghost URDF.
                _hud_get = getattr(_hud_ldr, "hud_snapshot", None)
                if _hud_get is not None:
                    try:
                        _hud = _hud_get()
                        # FK marker is computed from the COMMANDED q (what
                        # the main mirror shows) so the blue validation dot
                        # sits on the visible arm, not the static real one.
                        # Add _ALIGN_OFFSETS so the FK matches the visual
                        # mesh (which renders qcmd + offsets in scene.js).
                        _kin = getattr(_hud_ldr, "_kin", None)
                        _cmd_q = _hud.get("qcmd")
                        if _kin is not None and _cmd_q is not None:
                            try:
                                _q_vis = (np.asarray(_cmd_q[:6], dtype=np.float64)
                                          + _ALIGN_OFFSETS[:6].astype(np.float64))
                                _fk_pos, _ = _kin.fk_pose(_q_vis)
                                _hud["fk_ee_actual"] = _fk_pos.tolist()
                            except Exception:
                                pass
                        _telem_out["leader"] = _hud
                        if "qcmd" in _hud:
                            _telem_out["qcmd"] = _hud["qcmd"]
                    except Exception:
                        pass
                _quest_state.latest_telem = _telem_out
                _quest_state.latest_telem_seq += 1
            if telem and "motors" in telem:
                # Swivel is the first joint of ARM_JOINTS, so torque/temp
                # arrays already include it at index 0 — no separate extract.
                tq = _qtorque(telem)
                if tq is not None:
                    arm_torques = tq

                    # Force-feedback to OpenRB leader (gripper trigger only).
                    # Reflects follower gripper torque so the operator feels
                    # objects in the gripper / back-drive when the gripper
                    # hits a surface.  All other servos stay torque-OFF
                    # (SENTINEL) — they're free to backdrive as before.
                    if (args.ff_leader
                            and leader is not None
                            and hasattr(leader, "set_ff_currents")
                            and not estop_hw_active):
                        raw_t = float(tq[6])  # gripper is ARM_JOINTS[6]
                        if not math.isnan(raw_t):
                            _ff_gripper_lpf = (
                                _FF_LPF_ALPHA * raw_t
                                + (1.0 - _FF_LPF_ALPHA) * _ff_gripper_lpf
                            )
                            # Soft-knee deadband: anything below the floor
                            # gets zeroed out; above it the response is
                            # linear from zero (no step at the boundary).
                            eff = _ff_gripper_lpf
                            db  = float(args.ff_gripper_deadband)
                            if abs(eff) <= db:
                                eff = 0.0
                            else:
                                eff = eff - math.copysign(db, eff)
                            cur_ma = int(round(args.ff_gripper_gain * eff))
                            cur_ma = max(-FF_MAX_CURRENT_MA,
                                          min( FF_MAX_CURRENT_MA, cur_ma))
                            leader.set_ff_currents(
                                [FF_DISABLE_SENTINEL] * 6 + [cur_ma]
                            )
                            _ff_was_active = True
                    elif _ff_was_active:
                        # We were sending currents and now the conditions
                        # no longer apply (FF flag toggled, leader gone,
                        # e-stop fired).  Send one DISABLE_ALL frame so
                        # the trigger goes free immediately instead of
                        # waiting the firmware's 1 s watchdog timeout.
                        if leader is not None and hasattr(leader, "set_ff_currents"):
                            try:
                                leader.set_ff_currents([FF_DISABLE_SENTINEL] * 7)
                            except Exception:
                                pass
                        _ff_gripper_lpf = 0.0
                        _ff_was_active  = False
                te = _qtemp(telem)
                if te is not None:
                    arm_temps = te
                _arm_st = [
                    str(telem["motors"].get(j, {}).get("state", "?"))
                    for j in ARM_JOINTS
                ]
                if any(s != "?" for s in _arm_st):
                    arm_states = _arm_st
                # Wheel motor telemetry
                _ws: dict = {}
                for wn in _BASE_MOTORS:
                    wm = telem["motors"].get(wn)
                    if wm is not None:
                        _ws[wn] = {
                            "state":       wm.get("state", "?"),
                            "velocity":    wm.get("velocity"),
                            "torque":      wm.get("torque"),
                            "temperature": wm.get("temperature"),
                        }
                if _ws:
                    wheel_states = _ws
            if telem:
                ts = telem.get("timestamp")
                if ts is not None:
                    latest_telem_ts = float(ts)
                bv = telem.get("battery_voltage")
                if bv is not None:
                    battery_voltage = float(bv)
                estop_from_telem = bool(telem.get("emergency_stop", False))
            else:
                estop_from_telem = False
            estop_active = estop_from_telem or estop_hw_active

            with _cam_lock:
                _ups_msg = _cam_cache["ups"]
            if _ups_msg is not None and "ups" in _ups_msg:
                ups_data = _ups_msg["ups"]

            _prof.tick("telem")

            # Queue joint positions + leader commands to Rerun (every frame).
            # ARM_JOINTS includes swivel as joint 0 — q_actual / aizee_cmd
            # are both 7-element so a single loop covers everything.
            if _rr_event is not None:
                _jd: Optional[dict] = None
                _ld: Optional[dict] = None
                if q_actual is not None:
                    _jd = {jn: float(q_actual[i]) for i, jn in enumerate(ARM_JOINTS)}
                if aizee_cmd is not None:
                    _ld = {jn: float(aizee_cmd[i]) for i, jn in enumerate(ARM_JOINTS)}
                if _jd or _ld:
                    with _rr_lock:
                        _rr_holder["time"]   = t0
                        _rr_holder["joints"] = _jd
                        _rr_holder["leader"] = _ld
                    _rr_event.set()

            # -----------------------------------------------------------------
            # E-Stop detection
            # -----------------------------------------------------------------
            if telem and telem.get("emergency_stop"):
                if teleop_state != State.ESTOP:
                    _finalize_recording(" (e-stop)", t0)
                    teleop_state = State.ESTOP
            elif teleop_state == State.ESTOP:
                # E-stop cleared — return to READY, user must re-enable
                teleop_state = State.READY

            # -----------------------------------------------------------------
            # Recording (sub-sampled to REC_HZ)
            # -----------------------------------------------------------------
            if recording and t0 - last_rec_time >= 1.0 / REC_HZ:
                last_rec_time = t0
                # Pull image + its publisher capture-time atomically from
                # the decoder cache. Using the paired ts (not
                # latest_*_ts) is what keeps the recorded HDF5 actually
                # synchronized — latest_*_ts could be from a frame that
                # arrived AFTER the decoded image was committed,
                # producing an off-by-one ts in the file.
                with _dec_lock:
                    gripper_img    = _dec_cache["gripper"]
                    gripper_ts_rec = _dec_cache.get("gripper_ts_pub")
                    scene_img      = _dec_cache.get("scene")
                    scene_ts_rec   = _dec_cache.get("scene_ts_pub")
                cams_ok   = cam_age < _CAM_STALE
                # When scene cam is enabled it must also be fresh — a stale
                # scene frame would desync from the gripper / qpos timeline.
                # When disabled, scene_img is None and we skip the scene
                # append entirely (episode stays format_version=4).
                scene_ok  = (not _scene_cam_seen) or (
                    scene_img is not None and scene_age < _CAM_STALE)
                # In TRACKING the cmd thread owns the live commanded pose;
                # pull it from the holder so the recording captures the
                # exact value sent on the wire.  Falls back to the main
                # loop's `latest_q_cmd` (still authoritative for HOLD/
                # ENGAGING/SHUTDOWN) when the cmd thread hasn't run yet.
                with _cmd_lock:
                    _qcmd_live = _cmd_holder.get("last_q_cmd")
                if _qcmd_live is not None:
                    rec_q_cmd = np.asarray(_qcmd_live, dtype=np.float32).copy()
                elif latest_q_cmd is not None:
                    rec_q_cmd = latest_q_cmd.copy()
                else:
                    rec_q_cmd = None

                if (q_actual is not None and gripper_img is not None and cams_ok
                        and scene_ok):
                    # qpos_buf/qcmd_buf/torque_buf are 7-DOF (swivel-first)
                    # because q_actual / latest_q_cmd / arm_torques are.
                    qpos_buf.append(q_actual.copy())
                    qcmd_buf.append(rec_q_cmd if rec_q_cmd is not None else q_actual.copy())
                    torque_buf.append(arm_torques.copy() if arm_torques is not None else np.zeros(NUM_JOINTS, dtype=np.float32))
                    gripper_buf.append(gripper_img)
                    telem_ts_buf.append(latest_telem_ts if latest_telem_ts is not None else _nan)
                    gripper_ts_buf.append(
                        gripper_ts_rec if gripper_ts_rec is not None else _nan)
                    if _scene_cam_seen and scene_img is not None:
                        scene_buf.append(scene_img)
                        scene_ts_buf.append(
                            scene_ts_rec if scene_ts_rec is not None else _nan)
                else:
                    dropped_frames += 1

                if len(qpos_buf) >= args.max_steps:
                    _finalize_recording(" (max steps)", t0)

            # -----------------------------------------------------------------
            # Status strings
            # -----------------------------------------------------------------
            if teleop_state == State.READY:
                status = "[ ] ready — motors off"
                if leader is not None:
                    hint = "E=track · I=idle · Z=zero · M=mirror · K=mech-zero · Q=quit"
                else:
                    hint = "E=hold · I=idle · K=mech-zero · Q=quit"

            elif teleop_state == State.IDLE:
                status = "[I] idle — zero torque (arm free)"
                if leader is not None:
                    hint = "E=track · H=hold · R=record · X=shutdown · Q=quit"
                else:
                    hint = "H=hold · R=record · X=shutdown · Q=quit"

            elif teleop_state == State.ENGAGING:
                gap_str = ""
                if q_actual is not None and target is not None:
                    _g = float(np.max(np.abs(target - q_actual)))
                    gap_str = f"  (gap {_g:.2f} rad)"
                status = f"[~] engaging — slow ramp to leader{gap_str}"
                hint   = "X=shutdown · Q=quit"

            elif teleop_state == State.TRACKING:
                if leader_rad is None or leader_age > 0.5:
                    status = "[!] tracking — NO LEADER DATA"
                else:
                    status = "[*] tracking leader"
                hint = "H=hold · R=record · X=shutdown · Z=zero · M=mirror · Q=quit"

            elif teleop_state == State.HOLD:
                status = "[H] HOLD — target frozen"
                if leader is not None:
                    hint = "H=resume tracking · R=record · X=shutdown · Q=quit"
                else:
                    hint = "H=resume · R=record · X=shutdown · Q=quit"

            elif teleop_state == State.SHUTDOWN:
                status = (f"[X] shutdown  hold {shutdown_countdown:.1f}s"
                          if shutdown_countdown > 0 else "[X] returning to zero")
                hint   = "B=cancel (gamepad) · Q=quit"

            elif teleop_state == State.ESTOP:
                status = f"{_BG_RED} !! EMERGENCY STOP !! {_RST}"
                hint   = "release e-stop to clear · Q=quit"

            # Live replay overrides teleop status (shown while live mode active)
            if live_replay.live:
                _rs, _rh = live_replay.status_line()
                if _rs:
                    status = _rs
                if _rh:
                    hint = _rh

            # Flash messages override
            if t0 < zero_msg_until:
                status = zero_msg
            if t0 < save_msg_until:
                hint = save_msg

            # -----------------------------------------------------------------
            # Render — queue raw values to display thread (render + draw
            # both run off the main loop to avoid GIL contention)
            # -----------------------------------------------------------------
            # Display arrays match q_actual / target / arm_torques shape
            # directly — swivel is element 0 of each, no concat needed.
            _da = shutdown_target if teleop_state == State.SHUTDOWN else target

            # Mapped leader in the robot frame (Z/M-corrected); 7-DOF and
            # already includes swivel as element 0.  None when tracking is
            # inactive or no leader sample has arrived.
            leader_mapped = aizee_cmd if aizee_cmd is not None else None

            _disp_snapshot = dict(
                leader_rad=leader_rad,
                leader_mapped=leader_mapped,
                target=_da,
                actual=q_actual,
                status=status, hint=hint,
                robot_ok=robot_ok,
                telem_age=(t0 - last_telem_time if robot_ok else 999.0),
                ups_data=ups_data,
                clamped=(_clamped_live if leader_rad is not None else None),
                torque=arm_torques,
                temp=arm_temps,
                motor_states=list(arm_states),
                battery_voltage=battery_voltage,
                leader_connected=(leader is not None),
                leader_age=leader_age,
                cam_age=cam_age,
                rec_steps=len(qpos_buf),
                recording=recording,
                dropped=dropped_frames,
                estop_active=estop_active,
                wheel_states=wheel_states,
                wheels_enabled=teleop_state in (
                    State.IDLE, State.TRACKING, State.HOLD,
                    State.ENGAGING, State.SHUTDOWN),
                drive_linear=drive_linear * _max_linear,
                drive_angular=drive_angular * _max_angular,
                # M5 Joystick2 snapshot for the GUI's diagnostic panel.
                # `m5` was already read above under _lr_lock; passing it
                # through unchanged so the GUI sees the same view as the
                # drive / record-toggle logic.  None when no leader
                # installed yet (so the panel shows "NO LEADER").
                joy=(m5 if leader is not None else None),
                state=teleop_state.value,
                # Quest VR state for the GUI's leader panel.
                quest_available=_quest_available,
                quest_kind=_leader_kind,                       # "quest" | "openrb" | "so101" | None
                quest_server_running=_quest_server_started,
                quest_port=args.quest_port,
                quest_bind=args.quest_bind,
                save_msg=(save_msg if t0 < save_msg_until else None),
                action_msg=(zero_msg if t0 < zero_msg_until else None),
                last_saved_path=last_saved_path,
                task_tag=_meta["task_tag"],
                **live_replay.snapshot_fields(),
            )
            # Lock held only for reference swap (~µs)
            with _disp_lock:
                _disp_holder["args"] = _disp_snapshot
            if _disp_event is not None:
                _disp_event.set()

            _prof.tick("display")
            _prof.end()

            frame_counter += 1
            # Pick up edits to joint_align.json (made via /preview Save)
            # roughly once per second so calibration loops without a
            # collect_demo restart.  Stat-only; cheap enough at 1 Hz.
            if (frame_counter % LOOP_HZ) == 0:
                if _maybe_reload_joint_align() and leader is not None:
                    _push_visual_offsets_to_leader(leader)
            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        _hp_stop.set()
        if _hp_thread is not None:
            _hp_thread.join(timeout=2.0)
        _lr_stop.set()
        _lr_thread.join(timeout=1.0)
        if leader is not None:
            leader.close()
        # Stop the cmd-sender thread first so it can't re-emit a bundle
        # AFTER our explicit drive-zero + disable below.
        _clear_bundle()
        _cmd_tx_stop.set()
        _cmd_tx_thread.join(timeout=1.0)
        # Disable all motors before closing (prevents motors staying enabled after quit)
        _send(cmd_sock, {"type": "drive", "linear": 0.0, "angular": 0.0,
                         "kp": 0.0, "kd": 3.0})
        _send(cmd_sock, {"type": "disable", "motor_ids": _ALL_MOTORS})
        time.sleep(0.1)  # let ZMQ flush the disable command
        cmd_sock.close()
        _telem_stop.set()
        _telem_thread.join(timeout=1.0)
        _rec_flag.clear()
        _dec_stop.set()
        _dec_thread.join(timeout=1.0)
        _cam_stop.set()
        _cam_thread.join(timeout=2.0)
        _estop_stop.set()
        if _estop_thread is not None:
            _estop_thread.join(timeout=1.0)
        if _qt_renderer is not None:
            _qt_renderer.request_quit()
            _qt_renderer.join(timeout=2.0)
        else:
            _disp_stop.set()
            _disp_thread.join(timeout=1.0)
        if _rr_stop is not None:
            _rr_stop.set()
            _rr_thread.join(timeout=1.0)
        ctx.term()
        print("\nDone.")


if __name__ == "__main__":
    main()
