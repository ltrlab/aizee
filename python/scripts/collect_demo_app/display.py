"""ANSI terminal renderer + display / Rerun background threads (from collect_demo.py)."""
from __future__ import annotations

import sys
import threading
from typing import Optional

import numpy as np

try:
    import rerun as rr
    _rerun_available = True
except ImportError:
    _rerun_available = False

from .alignment import _SAT_TORQUE
from .runtime import _IW, _BASE_MOTORS, _LEADER_JOINTS, REC_HZ

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
