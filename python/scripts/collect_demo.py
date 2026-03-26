#!/usr/bin/env python3
"""collect_demo.py — Motor control + ACT demo recorder for AIZEE arm.

Combines SO-101 teleoperation with demonstration recording.  Optionally
drives the arm via the SO-101 leader arm (--port).  Without --port you
still get full motor control for setup.

Usage:
    python collect_demo.py --port COM4
    python collect_demo.py --port /dev/ttyACM0 \\
        --cmd   tcp://192.168.0.27:5555 \\
        --telem tcp://192.168.0.27:5556 \\
        --cam-left  tcp://192.168.0.27:5563 \\
        --cam-right tcp://192.168.0.27:5564

Controls:
    E    enable arm motors (align to leader if --port given)
    I    idle — enable with zero torque (see actual positions)
    H    hold — freeze target at current actual position
    R    toggle recording (IDLE / TRACKING / HOLD only)
    X    soft shutdown — hold 1 s, return to zero, disable
    Z    zero — capture current SO-101 pose as zero reference
    M    mirror — set zero so current leader maps to current actual
    Q    quit  (Ctrl-C also works)

Gamepad: A=enable  B=shutdown/cancel  Start=hold  Back=quit
"""

from __future__ import annotations

import argparse
import base64
import enum
import io
import json
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
    from so101_leader import So101Leader, CALIB_PATH as _CALIB_PATH
    _so101_available = True
except ImportError:
    _CALIB_PATH = Path("so101_calibration.json")

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import ARM_JOINTS, KP, KD, setup_keyboard, load_arm_limits, clamp_arm_positions

LOOP_HZ    = 30
REC_HZ     = 20
NUM_JOINTS = len(ARM_JOINTS)   # 6

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------

_W  = 76
_IW = _W - 2

_LEADER_JOINTS = ["swivel", "gantry_base", "gantry_mid", "gantry_end",
                  "wrist_pitch", "wrist_roll", "gripper"]

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
    cam_left_age:     float               = 999.0,
    cam_right_age:    float               = 999.0,
    rec_steps:        int                 = 0,
    recording:        bool                = False,
    dropped:          int                 = 0,
    estop_active:     bool                = False,
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

    l_s, l_v = _cam(cam_left_age,  "L")
    r_s, r_v = _cam(cam_right_age, "R")
    cam_part = f"cams  {l_s}  {r_s}"
    cam_vis  = 6 + l_v + 2 + r_v

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


def _read_gamepad(joystick, prev_a: bool, prev_b: bool, prev_start: bool) -> dict:
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


# ---------------------------------------------------------------------------
# Background camera / UPS receiver
# ---------------------------------------------------------------------------
# Runs in its own thread so that JSON-parsing large camera frames never
# blocks the motor-command loop.  Caches the latest message per source;
# the main loop reads cached values under a lock.

def _start_cam_receiver(
    ctx: zmq.Context,
    left_ep: str,
    right_ep: str,
    ups_ep: Optional[str],
) -> tuple[threading.Event, threading.Thread, threading.Lock, dict]:
    lock = threading.Lock()
    cache: dict = {
        "left": None, "left_time": 0.0, "left_ts": None,
        "right": None, "right_time": 0.0, "right_ts": None,
        "ups": None,
    }
    stop = threading.Event()

    def _run() -> None:
        left_sock = ctx.socket(zmq.SUB)
        left_sock.setsockopt(zmq.LINGER, 0)
        left_sock.setsockopt(zmq.CONFLATE, 1)
        left_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        left_sock.connect(left_ep)

        right_sock = ctx.socket(zmq.SUB)
        right_sock.setsockopt(zmq.LINGER, 0)
        right_sock.setsockopt(zmq.CONFLATE, 1)
        right_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        right_sock.connect(right_ep)

        ups_sock: Optional[zmq.Socket] = None
        if ups_ep:
            ups_sock = ctx.socket(zmq.SUB)
            ups_sock.setsockopt(zmq.LINGER, 0)
            ups_sock.setsockopt(zmq.CONFLATE, 1)
            ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
            ups_sock.connect(ups_ep)

        poller = zmq.Poller()
        poller.register(left_sock, zmq.POLLIN)
        poller.register(right_sock, zmq.POLLIN)
        if ups_sock:
            poller.register(ups_sock, zmq.POLLIN)

        try:
            while not stop.is_set():
                try:
                    events = dict(poller.poll(timeout=100))
                except zmq.ZMQError:
                    break
                now = time.time()

                if left_sock in events:
                    try:
                        msg = json.loads(left_sock.recv_string(zmq.NOBLOCK))
                        with lock:
                            cache["left"] = msg
                            cache["left_time"] = now
                            ts = msg.get("timestamp")
                            if ts is not None:
                                cache["left_ts"] = float(ts)
                    except Exception:
                        pass

                if right_sock in events:
                    try:
                        msg = json.loads(right_sock.recv_string(zmq.NOBLOCK))
                        with lock:
                            cache["right"] = msg
                            cache["right_time"] = now
                            ts = msg.get("timestamp")
                            if ts is not None:
                                cache["right_ts"] = float(ts)
                    except Exception:
                        pass

                if ups_sock and ups_sock in events:
                    try:
                        msg = json.loads(ups_sock.recv_string(zmq.NOBLOCK))
                        with lock:
                            cache["ups"] = msg
                    except Exception:
                        pass
        finally:
            left_sock.close()
            right_sock.close()
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
                    ser = _serial.Serial(port, 115200, timeout=1)
                    print(f"E-stop receiver connected on {port}")
                except _serial.SerialException:
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

def decode_image(msg: dict, target_size: tuple) -> Optional[np.ndarray]:
    """Decode a camera message to uint8 [H, W, 3]. target_size is (W, H)."""
    color    = msg.get("color", {})
    data_b64 = color.get("data")
    if data_b64 is None:
        return None
    img = Image.open(io.BytesIO(base64.b64decode(data_b64))).convert("RGB")
    img = img.resize(target_size, Image.LANCZOS)
    return np.array(img, dtype=np.uint8)


# ---------------------------------------------------------------------------
# Telemetry helpers
# ---------------------------------------------------------------------------

def _qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
    """Extract 6-element arm joint positions. Returns None if no arm data."""
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
    output_dir, qpos_buf, left_buf, right_buf,
    telem_ts_buf=None, left_ts_buf=None, right_ts_buf=None,
    swivel_buf=None, qcmd_buf=None, torque_buf=None,
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(output_dir.glob("episode_*.hdf5"))
    ep_num   = (int(existing[-1].stem.split("_")[1]) + 1) if existing else 0
    path     = output_dir / f"episode_{ep_num:04d}.hdf5"

    qpos_arr  = np.stack(qpos_buf,  axis=0)   # [T, 6]
    left_arr  = np.stack(left_buf,  axis=0)   # [T, H, W, 3]
    right_arr = np.stack(right_buf, axis=0)   # [T, H, W, 3]
    qcmd_arr  = np.stack(qcmd_buf, axis=0) if qcmd_buf and len(qcmd_buf) == len(qpos_buf) else None
    torque_arr = np.stack(torque_buf, axis=0) if torque_buf and len(torque_buf) == len(qpos_buf) else None
    # Actions derived from commanded positions (no sag) when available
    act_src   = qcmd_arr if qcmd_arr is not None else qpos_arr
    actions   = np.concatenate([act_src[1:], act_src[-1:]], axis=0)  # [T, 6]
    H, W      = left_arr.shape[1], left_arr.shape[2]

    with h5py.File(path, "w") as f:
        f.attrs["hz"]        = REC_HZ
        f.attrs["arm_joints"] = ",".join(ARM_JOINTS)
        obs  = f.create_group("observations")
        obs.create_dataset("qpos",   data=qpos_arr,  compression="gzip", compression_opts=4)
        if qcmd_arr is not None:
            obs.create_dataset("qcmd", data=qcmd_arr, compression="gzip", compression_opts=4)
        if torque_arr is not None:
            obs.create_dataset("torques", data=torque_arr, compression="gzip", compression_opts=4)
        imgs = obs.create_group("images")
        imgs.create_dataset("left",  data=left_arr,  compression="gzip", compression_opts=4, chunks=(1, H, W, 3))
        imgs.create_dataset("right", data=right_arr, compression="gzip", compression_opts=4, chunks=(1, H, W, 3))
        f.create_dataset("actions",  data=actions,   compression="gzip", compression_opts=4)
        if swivel_buf is not None and len(swivel_buf) == len(qpos_buf):
            obs.create_dataset("swivel", data=np.array(swivel_buf, dtype=np.float32),
                               compression="gzip", compression_opts=4)
        if telem_ts_buf is not None:
            ts = f.create_group("timestamps")
            ts.create_dataset("telem",        data=np.array(telem_ts_buf, dtype=np.float64))
            ts.create_dataset("camera_left",  data=np.array(left_ts_buf,  dtype=np.float64))
            ts.create_dataset("camera_right", data=np.array(right_ts_buf, dtype=np.float64))

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
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    _ep = _load_endpoints()
    ap  = argparse.ArgumentParser(
        description="SO-101 leader arm teleop + ACT demo recorder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",            default=None,
                    help="SO-101 serial port (optional — enables leader tracking)")
    ap.add_argument("--baud",            type=int, default=1_000_000)
    ap.add_argument("--calib",           default=str(_CALIB_PATH),
                    help="SO-101 calibration JSON")
    ap.add_argument("--cmd",             default=_ep.get("command",       "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem",           default=_ep.get("telemetry",     "tcp://192.168.0.27:5556"))
    ap.add_argument("--cam-left",        default="tcp://192.168.0.27:5563", dest="cam_left")
    ap.add_argument("--cam-right",       default="tcp://192.168.0.27:5564", dest="cam_right")
    ap.add_argument("--ups",             default=_ep.get("ups_telemetry", "tcp://192.168.0.27:5562"),
                    help="UPS telemetry address (empty to disable)")
    ap.add_argument("--output-dir",      default="episodes",              dest="output_dir")
    ap.add_argument("--max-steps",       type=int, default=10000,           dest="max_steps",
                    help="Max steps per episode (default: 10000 = 30 s at 20 Hz)")
    ap.add_argument("--image-size",      default="240x320",               dest="image_size",
                    help="Image size HxW (default: 240x320)")
    ap.add_argument("--dry-run",         action="store_true",             dest="dry_run")
    ap.add_argument("--max-delta",       type=float, default=0.3,         dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.3)")
    ap.add_argument("--robstride-calib", default=None,                    dest="robstride_calib")
    ap.add_argument("--no-rerun",       action="store_true",             dest="no_rerun",
                    help="Disable Rerun live camera preview")
    ap.add_argument("--estop-port",    default=None,                    dest="estop_port",
                    help="Serial port for ESP32 e-stop receiver (e.g. /dev/estop-receiver, COM10)")
    args = ap.parse_args()

    _ansi_on()

    h_s, w_s = args.image_size.split("x")
    img_size  = (int(w_s), int(h_s))   # PIL: (width, height)

    # -------------------------------------------------------------------------
    # SO-101 leader (optional)
    # -------------------------------------------------------------------------
    leader           = None
    _lr_lock         = threading.Lock()
    _lr_latest: dict = {"rad": None, "clamped": None, "time": 0.0}
    _lr_stop         = threading.Event()
    zero_offsets     = None
    directions       = None
    _so101_for_aizee: list[int] = []

    if args.port is not None:
        if not _so101_available:
            print("SO-101 support not available (missing so101_leader module)")
            sys.exit(1)
        leader = So101Leader(args.port, args.baud, calib=args.calib)
        if not leader.connect():
            sys.exit(1)
        print(f"SO-101 connected on {args.port}")
        zero_offsets     = leader.zero_offsets
        directions       = leader.directions
        _arm_joint_set   = set(ARM_JOINTS)
        _so101_for_aizee = [i for i, j in enumerate(leader.AIZEE_JOINTS) if j in _arm_joint_set]

        def _leader_reader(stop: threading.Event) -> None:
            while not stop.is_set():
                r = leader.poll()
                with _lr_lock:
                    if r is not None:
                        _lr_latest["rad"]     = r
                        _lr_latest["clamped"] = leader.clamped_joints
                        _lr_latest["time"]    = time.time()
                    elif _lr_latest["rad"] is None:
                        pass  # keep old rad so display doesn't flicker on momentary miss

        _lr_thread = threading.Thread(target=_leader_reader, args=(_lr_stop,), daemon=True)
        _lr_thread.start()

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    _yaml      = _load_teleop_yaml()
    _tcfg      = _yaml.get("gantry", {})
    _kp: list  = _tcfg.get("kp", KP)
    _kd: list  = _tcfg.get("kd", KD)
    _dcfg      = _yaml.get("drive", {})
    _swivel_kp = float(_dcfg.get("swivel_kp", 80.0))
    _swivel_kd = float(_dcfg.get("swivel_kd", 5.0))

    # -------------------------------------------------------------------------
    # ZMQ sockets
    # -------------------------------------------------------------------------
    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 2)
    cmd_sock.setsockopt(zmq.LINGER,  0)
    cmd_sock.connect(args.cmd)

    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.setsockopt(zmq.LINGER, 0)
    telem_sock.setsockopt(zmq.CONFLATE, 1)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sock.connect(args.telem)

    # Camera + UPS reception runs in a background thread so that
    # JSON-parsing large JPEG frames never delays motor commands.
    _cam_stop, _cam_thread, _cam_lock, _cam_cache = _start_cam_receiver(
        ctx, args.cam_left, args.cam_right, args.ups or None,
    )

    # -------------------------------------------------------------------------
    # Hardware e-stop (ESP32 serial)
    # -------------------------------------------------------------------------
    _estop_flag = threading.Event()   # set = e-stop active
    _estop_stop = threading.Event()
    _estop_thread: Optional[threading.Thread] = None
    if args.estop_port:
        _estop_thread = _start_estop_reader(args.estop_port, _estop_stop, _estop_flag)

    # -------------------------------------------------------------------------
    # Rerun live camera preview
    # -------------------------------------------------------------------------
    use_rerun = not args.no_rerun
    if use_rerun and not _rerun_available:
        print("WARNING: rerun not installed — live camera preview disabled")
        use_rerun = False
    if use_rerun:
        rr.init("aizee_collect", spawn=True)
        rr.send_blueprint(rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial2DView(name="Left", origin="cameras/left"),
                rrb.Spatial2DView(name="Right", origin="cameras/right"),
                column_shares=[1, 1],
            )
        ))

    get_key = setup_keyboard()

    # Seed q_actual from first telemetry packet
    q_actual: Optional[np.ndarray] = None
    for _ in range(40):
        telem = _drain(telem_sock)
        if telem:
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
        SHUTDOWN = "shutdown"
        ESTOP    = "estop"

    teleop_state                       = State.READY
    held_target:     Optional[np.ndarray] = None
    held_swivel:     Optional[float]   = None
    shutdown_countdown: float          = 0.0
    shutdown_target: Optional[np.ndarray] = None
    shutdown_swivel: Optional[float]   = None
    shutdown_zero_since: float         = 0.0   # when ramp first hit zero
    _SHUTDOWN_TIMEOUT                  = 3.0   # force-disable after this many seconds at zero
    swivel_actual:   Optional[float]   = None
    swivel_torque:   Optional[float]   = None
    swivel_temp:     Optional[float]   = None
    arm_torques:     Optional[np.ndarray] = None
    arm_temps:       Optional[np.ndarray] = None
    last_telem_time: float             = time.time() if q_actual is not None else 0.0
    ups_data:        Optional[dict]    = None
    battery_voltage: Optional[float]   = None
    robot_ok = q_actual is not None
    estop_active = False
    prev_estop_hw = False

    zero_msg       = ""
    zero_msg_until = 0.0
    save_msg       = ""
    save_msg_until = 0.0

    joystick           = _init_joystick() if _pygame_available else None
    prev_gp_a:   bool  = False
    prev_gp_b:   bool  = False
    prev_gp_start:bool = False

    # Recording state
    recording      = False
    qpos_buf:    list = []
    qcmd_buf:    list = []
    torque_buf:  list = []
    left_buf:    list = []
    right_buf:   list = []
    swivel_buf:  list = []
    telem_ts_buf:  list = []
    left_ts_buf:   list = []
    right_ts_buf:  list = []
    dropped_frames = 0
    last_rec_time  = 0.0

    # Camera state
    last_left_time   = 0.0
    last_right_time  = 0.0
    latest_left:  Optional[dict] = None
    latest_right: Optional[dict] = None
    latest_telem_ts: Optional[float] = None
    latest_left_ts:  Optional[float] = None
    latest_right_ts: Optional[float] = None
    latest_q_cmd: Optional[np.ndarray] = None  # last commanded position sent to motors

    status = "[ ] ready — motors off"
    hint   = ("E=hold · I=idle · Q=quit" if leader is None
              else "E=track · I=idle · Z=zero · M=mirror · Q=quit")

    _nan = float("nan")
    _init_actual = (np.concatenate([[_nan], q_actual]) if q_actual is not None else None)
    _draw(_render(None, None, _init_actual, status, hint,
                  robot_ok=robot_ok, leader_connected=(leader is not None)), first=True)

    frame_counter = 0
    period = 1.0 / LOOP_HZ

    _save_thread:        Optional[threading.Thread] = None
    _save_result_holder: list                       = [None]

    def _start_async_save(out_dir, qb, lb, rb, tb, ltb, rtb, swb, dur, drop_note, tag="", qcb=None, tqb=None):
        def _run():
            try:
                p, T = save_episode(out_dir, qb, lb, rb, tb, ltb, rtb, swivel_buf=swb, qcmd_buf=qcb, torque_buf=tqb)
                _save_result_holder[0] = f"[SAVED {p.name}  {T} steps  {dur:.1f}s{drop_note}]{tag}"
            except Exception as e:
                _save_result_holder[0] = f"[SAVE ERROR: {e}]"
        t = threading.Thread(target=_run, daemon=True)
        t.start()
        return t

    try:
        while True:
            t0 = time.time()

            # -----------------------------------------------------------------
            # Pick up completed background save
            # -----------------------------------------------------------------
            if _save_thread is not None and not _save_thread.is_alive():
                if _save_result_holder[0] is not None:
                    save_msg       = _save_result_holder[0]
                    save_msg_until = t0 + 5.0
                    _save_result_holder[0] = None
                _save_thread = None

            # -----------------------------------------------------------------
            # Read cached camera data (populated by background thread)
            # -----------------------------------------------------------------
            with _cam_lock:
                if _cam_cache["left"] is not None:
                    latest_left    = _cam_cache["left"]
                    last_left_time = _cam_cache["left_time"]
                    latest_left_ts = _cam_cache["left_ts"]
                if _cam_cache["right"] is not None:
                    latest_right    = _cam_cache["right"]
                    last_right_time = _cam_cache["right_time"]
                    latest_right_ts = _cam_cache["right_ts"]

            cam_left_age  = (t0 - last_left_time)  if last_left_time  > 0 else 999.0
            cam_right_age = (t0 - last_right_time) if last_right_time > 0 else 999.0

            # Log camera images to Rerun (~15 Hz = every other iteration)
            if use_rerun and (frame_counter % 2 == 0):
                rr.set_time("time", timestamp=t0)
                if latest_left is not None:
                    jpeg = latest_left.get("color", {}).get("data")
                    if jpeg:
                        rr.log("cameras/left", rr.EncodedImage(
                            contents=base64.b64decode(jpeg), media_type="image/jpeg"))
                if latest_right is not None:
                    jpeg = latest_right.get("color", {}).get("data")
                    if jpeg:
                        rr.log("cameras/right", rr.EncodedImage(
                            contents=base64.b64decode(jpeg), media_type="image/jpeg"))

            # -----------------------------------------------------------------
            # Gamepad
            # -----------------------------------------------------------------
            key = get_key()
            if joystick is not None:
                gp = _read_gamepad(joystick, prev_gp_a, prev_gp_b, prev_gp_start)
                prev_gp_a     = gp["raw_a"]
                prev_gp_b     = gp["raw_b"]
                prev_gp_start = gp["raw_start"]
                if gp["enable"] and teleop_state in (State.READY, State.IDLE):
                    key = "E"
                if gp["hold"] and teleop_state in (State.TRACKING, State.HOLD, State.IDLE):
                    key = "H"
                if gp["shutdown"]:
                    key = "CANCEL_SHUTDOWN" if teleop_state == State.SHUTDOWN else "X"
                if gp["quit"]:
                    key = "Q"

            # -----------------------------------------------------------------
            # Keyboard
            # -----------------------------------------------------------------
            if key == "Q":
                break

            elif key == "I":
                if teleop_state in (State.READY, State.IDLE):
                    _send(cmd_sock, {"type": "enable", "motor_ids": ["swivel"] + ARM_JOINTS})
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
                    _send(cmd_sock, {"type": "enable", "motor_ids": ["swivel"] + ARM_JOINTS})
                    if leader is not None:
                        teleop_state = State.TRACKING
                    else:
                        held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                        held_swivel  = swivel_actual
                        teleop_state = State.HOLD

            elif key == "H":
                if teleop_state == State.TRACKING:
                    held_target  = q_actual.copy() if q_actual is not None else held_target
                    held_swivel  = swivel_actual
                    teleop_state = State.HOLD
                elif teleop_state == State.HOLD:
                    teleop_state = State.TRACKING if leader is not None else State.IDLE
                elif teleop_state == State.IDLE:
                    held_target  = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                    held_swivel  = swivel_actual
                    teleop_state = State.HOLD

            elif key == "R":
                if not recording:
                    if teleop_state in (State.IDLE, State.TRACKING, State.HOLD):
                        recording      = True
                        qpos_buf       = []
                        qcmd_buf       = []
                        torque_buf     = []
                        left_buf       = []
                        right_buf      = []
                        swivel_buf     = []
                        telem_ts_buf   = []
                        left_ts_buf    = []
                        right_ts_buf   = []
                        dropped_frames = 0
                        last_rec_time  = 0.0
                else:
                    recording  = False
                    steps      = len(qpos_buf)
                    dur        = steps / REC_HZ
                    drop_note  = f"  drop:{dropped_frames}" if dropped_frames else ""
                    if steps == 0:
                        save_msg = "[STOPPED — 0 steps, nothing saved]"
                    elif args.dry_run:
                        save_msg = f"[DRY RUN] {steps} steps  {dur:.1f}s{drop_note}"
                    else:
                        save_msg       = f"[saving {steps} steps...]"
                        save_msg_until = t0 + 120.0
                        _save_result_holder[0] = None
                        _save_thread = _start_async_save(
                            args.output_dir, qpos_buf, left_buf, right_buf,
                            telem_ts_buf, left_ts_buf, right_ts_buf, swivel_buf,
                            dur, drop_note, qcb=qcmd_buf, tqb=torque_buf,
                        )
                    if steps == 0 or args.dry_run:
                        save_msg_until = t0 + 5.0

            elif key == "Z" and leader is not None:
                with _lr_lock:
                    _z = _lr_latest["rad"]
                if _z is not None:
                    zero_offsets   = _z.copy()
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[Z] zeroed — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "M" and leader is not None:
                with _lr_lock:
                    _m = _lr_latest["rad"]
                if _m is not None and q_actual is not None:
                    new_offsets = zero_offsets.copy()
                    for ai, si in enumerate(_so101_for_aizee):
                        new_offsets[si] = _m[si] - directions[si] * q_actual[ai]
                    if swivel_actual is not None:
                        new_offsets[0] = _m[0] - directions[0] * swivel_actual
                    zero_offsets   = new_offsets
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[M] mirrored — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "CANCEL_SHUTDOWN" and teleop_state == State.SHUTDOWN:
                teleop_state = State.HOLD
                held_target  = q_actual.copy() if q_actual is not None else held_target
                held_swivel  = swivel_actual

            elif key == "X":
                if teleop_state in (State.TRACKING, State.HOLD, State.IDLE):
                    shutdown_target    = (q_actual.copy() if q_actual is not None
                                          else held_target.copy() if held_target is not None
                                          else np.zeros(NUM_JOINTS))
                    shutdown_swivel    = (swivel_actual if swivel_actual is not None
                                          else held_swivel if held_swivel is not None else 0.0)
                    shutdown_countdown  = 1.0
                    shutdown_zero_since = 0.0
                    teleop_state        = State.SHUTDOWN
                    if recording:
                        recording = False   # stop recording on shutdown

            # -----------------------------------------------------------------
            # Leader data
            # -----------------------------------------------------------------
            leader_rad:    Optional[np.ndarray] = None
            _clamped_live: Optional[list]       = None
            aizee_cmd:     Optional[np.ndarray] = None
            swivel_cmd:    Optional[float]      = None
            leader_age:    float                = 999.0

            if leader is not None:
                with _lr_lock:
                    leader_rad    = _lr_latest["rad"]
                    _clamped_live = _lr_latest["clamped"]
                    _leader_t     = _lr_latest["time"]
                leader_age = t0 - _leader_t if _leader_t > 0 else 999.0
                if leader_rad is not None:
                    mapped = directions * (leader_rad - zero_offsets)
                    aizee_cmd  = mapped[_so101_for_aizee]
                    swivel_cmd = float(mapped[0])

            # Determine targets
            if teleop_state == State.HOLD:
                target     = held_target
                swivel_tgt = held_swivel
            elif aizee_cmd is not None:
                target     = aizee_cmd
                swivel_tgt = swivel_cmd
            else:
                target     = q_actual
                swivel_tgt = swivel_actual

            # -----------------------------------------------------------------
            # Hardware e-stop gate — skip ALL motor commands so watchdog
            # holds position (arm doesn't fall).
            # -----------------------------------------------------------------
            estop_hw_active = _estop_flag.is_set()
            if estop_hw_active and not prev_estop_hw:
                # Rising edge — auto-save recording
                if recording:
                    recording = False
                    steps     = len(qpos_buf)
                    dur       = steps / REC_HZ
                    drop_note = f"  drop:{dropped_frames}" if dropped_frames else ""
                    if steps > 0 and not args.dry_run:
                        save_msg       = f"[saving {steps} steps (hw e-stop)...]"
                        save_msg_until = t0 + 120.0
                        _save_result_holder[0] = None
                        _save_thread = _start_async_save(
                            args.output_dir, qpos_buf, left_buf, right_buf,
                            telem_ts_buf, left_ts_buf, right_ts_buf, swivel_buf,
                            dur, drop_note, tag=" (hw e-stop)", qcb=qcmd_buf, tqb=torque_buf,
                        )
                    elif steps > 0:
                        save_msg       = f"[DRY RUN] hw e-stop: {steps} steps  {dur:.1f}s"
                        save_msg_until = t0 + 5.0
            prev_estop_hw = estop_hw_active

            # -----------------------------------------------------------------
            # Send motor commands
            # -----------------------------------------------------------------
            if estop_hw_active:
                pass  # watchdog holds position
            elif teleop_state == State.SHUTDOWN:
                dt         = period
                max_change = 0.2 * dt   # 0.2 rad/s ramp
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    if shutdown_target is not None:
                        _send(cmd_sock, {
                            "type": "arm_joints", "positions": shutdown_target.tolist(),
                            "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                            "torques": [0.0] * NUM_JOINTS,
                        })
                    if shutdown_swivel is not None:
                        _send(cmd_sock, {"type": "swivel", "position": shutdown_swivel,
                                         "kp": _swivel_kp, "kd": _swivel_kd})
                else:
                    if shutdown_target is None:
                        shutdown_target = np.zeros(NUM_JOINTS)
                    ref     = q_actual if q_actual is not None else shutdown_target
                    new_tgt = shutdown_target.copy()
                    for i in range(len(new_tgt)):
                        new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                                      else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
                    shutdown_target = new_tgt
                    if shutdown_swivel is None:
                        shutdown_swivel = 0.0
                    shutdown_swivel = (0.0 if abs(shutdown_swivel) < max_change
                                       else shutdown_swivel - np.sign(shutdown_swivel) * max_change)
                    # Check completion BEFORE sending — the ZMQ PUSH socket
                    # has HWM=2 so sending arm+swivel+disable in one iteration
                    # would silently drop the disable command.
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
                    else:
                        delta   = np.clip(shutdown_target - ref, -args.max_delta, args.max_delta)
                        q_cmd   = ref + delta
                        if arm_limits:
                            q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                        _send(cmd_sock, {
                            "type": "arm_joints", "positions": q_cmd.tolist(),
                            "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                            "torques": [0.0] * NUM_JOINTS,
                        })
                        _send(cmd_sock, {"type": "swivel", "position": shutdown_swivel,
                                         "kp": _swivel_kp, "kd": _swivel_kd})

            elif teleop_state == State.IDLE:
                if q_actual is not None:
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": q_actual.tolist(),
                        "velocities": [0.0] * NUM_JOINTS,
                        "kp": [0.0] * NUM_JOINTS, "kd": [0.0] * NUM_JOINTS,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                if swivel_actual is not None:
                    _send(cmd_sock, {"type": "swivel", "position": swivel_actual,
                                     "kp": _swivel_kp, "kd": _swivel_kd})

            elif teleop_state in (State.TRACKING, State.HOLD):
                if target is not None:
                    ref   = q_actual if q_actual is not None else target
                    delta = np.clip(target - ref, -args.max_delta, args.max_delta)
                    q_cmd = ref + delta
                    if arm_limits:
                        q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                    latest_q_cmd = q_cmd.copy()
                    _send(cmd_sock, {
                        "type": "arm_joints", "positions": q_cmd.tolist(),
                        "velocities": [0.0] * NUM_JOINTS, "kp": _kp, "kd": _kd,
                        "torques": [0.0] * NUM_JOINTS,
                    })
                if swivel_tgt is not None:
                    _send(cmd_sock, {"type": "swivel", "position": swivel_tgt,
                                     "kp": _swivel_kp, "kd": _swivel_kd})

            # -----------------------------------------------------------------
            # Telemetry
            # -----------------------------------------------------------------
            telem = _drain(telem_sock)
            q_new = _qpos(telem)
            if q_new is not None:
                q_actual        = q_new
                robot_ok        = True
                last_telem_time = t0
            if telem and "motors" in telem:
                sw = telem["motors"].get("swivel")
                if sw is not None:
                    swivel_actual = float(sw.get("position",    0.0))
                    swivel_torque = float(sw.get("torque",      0.0))
                    swivel_temp   = float(sw.get("temperature", _nan))
                tq = _qtorque(telem)
                if tq is not None:
                    arm_torques = tq
                te = _qtemp(telem)
                if te is not None:
                    arm_temps = te
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

            # -----------------------------------------------------------------
            # E-Stop detection
            # -----------------------------------------------------------------
            if telem and telem.get("emergency_stop"):
                if teleop_state != State.ESTOP:
                    if recording:
                        recording = False
                        steps     = len(qpos_buf)
                        dur       = steps / REC_HZ
                        drop_note = f"  drop:{dropped_frames}" if dropped_frames else ""
                        if steps > 0 and not args.dry_run:
                            save_msg       = f"[saving {steps} steps (e-stop)...]"
                            save_msg_until = t0 + 120.0
                            _save_result_holder[0] = None
                            _save_thread = _start_async_save(
                                args.output_dir, qpos_buf, left_buf, right_buf,
                                telem_ts_buf, left_ts_buf, right_ts_buf, swivel_buf,
                                dur, drop_note, tag=" (e-stop)", qcb=qcmd_buf, tqb=torque_buf,
                            )
                        elif steps > 0:
                            save_msg       = f"[DRY RUN] e-stop: {steps} steps  {dur:.1f}s"
                            save_msg_until = t0 + 5.0
                    teleop_state = State.ESTOP
            elif teleop_state == State.ESTOP:
                # E-stop cleared — return to READY, user must re-enable
                teleop_state = State.READY

            # -----------------------------------------------------------------
            # Recording (sub-sampled to REC_HZ)
            # -----------------------------------------------------------------
            if recording and t0 - last_rec_time >= 1.0 / REC_HZ:
                last_rec_time = t0
                left_img  = decode_image(latest_left,  img_size) if latest_left  is not None else None
                right_img = decode_image(latest_right, img_size) if latest_right is not None else None
                cams_ok   = cam_left_age < _CAM_STALE and cam_right_age < _CAM_STALE
                if (q_actual is not None and left_img is not None
                        and right_img is not None and cams_ok):
                    qpos_buf.append(q_actual.copy())
                    qcmd_buf.append(latest_q_cmd.copy() if latest_q_cmd is not None else q_actual.copy())
                    torque_buf.append(arm_torques.copy() if arm_torques is not None else np.zeros(NUM_JOINTS, dtype=np.float32))
                    left_buf.append(left_img)
                    right_buf.append(right_img)
                    swivel_buf.append(swivel_actual if swivel_actual is not None else _nan)
                    telem_ts_buf.append(latest_telem_ts if latest_telem_ts is not None else _nan)
                    left_ts_buf.append(latest_left_ts   if latest_left_ts  is not None else _nan)
                    right_ts_buf.append(latest_right_ts if latest_right_ts is not None else _nan)
                else:
                    dropped_frames += 1

                if len(qpos_buf) >= args.max_steps:
                    recording = False
                    steps     = len(qpos_buf)
                    dur       = steps / REC_HZ
                    if args.dry_run:
                        save_msg = f"[DRY RUN] max steps: {steps}  {dur:.1f}s"
                    else:
                        save_msg       = f"[saving {steps} steps (max)...]"
                        save_msg_until = t0 + 120.0
                        _save_result_holder[0] = None
                        _save_thread = _start_async_save(
                            args.output_dir, qpos_buf, left_buf, right_buf,
                            telem_ts_buf, left_ts_buf, right_ts_buf, swivel_buf,
                            dur, drop_note="", tag=" (max steps)", qcb=qcmd_buf, tqb=torque_buf,
                        )
                    if args.dry_run:
                        save_msg_until = t0 + 5.0

            # -----------------------------------------------------------------
            # Status strings
            # -----------------------------------------------------------------
            if teleop_state == State.READY:
                status = "[ ] ready — motors off"
                if leader is not None:
                    hint = "E=track · I=idle · Z=zero · M=mirror · Q=quit"
                else:
                    hint = "E=hold · I=idle · Q=quit"

            elif teleop_state == State.IDLE:
                status = "[I] idle — zero torque (arm free)"
                if leader is not None:
                    hint = "E=track · H=hold · R=record · X=shutdown · Q=quit"
                else:
                    hint = "H=hold · R=record · X=shutdown · Q=quit"

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

            # Flash messages override
            if t0 < zero_msg_until:
                status = zero_msg
            if t0 < save_msg_until:
                hint = save_msg

            # -----------------------------------------------------------------
            # Render
            # -----------------------------------------------------------------
            if teleop_state == State.SHUTDOWN:
                disp_arm, disp_swivel = shutdown_target, shutdown_swivel
            else:
                disp_arm, disp_swivel = target, swivel_tgt

            disp_target = (np.concatenate([[disp_swivel if disp_swivel is not None else _nan], disp_arm])
                           if disp_arm is not None else None)
            disp_actual = (np.concatenate([[swivel_actual if swivel_actual is not None else _nan], q_actual])
                           if q_actual is not None else None)
            disp_torque = (np.concatenate([[swivel_torque if swivel_torque is not None else _nan], arm_torques])
                           if arm_torques is not None else None)
            disp_temp   = (np.concatenate([[swivel_temp if swivel_temp is not None else _nan], arm_temps])
                           if arm_temps is not None else None)

            telem_age = t0 - last_telem_time if robot_ok else 999.0

            _draw(_render(
                leader_rad, disp_target, disp_actual, status, hint,
                robot_ok, telem_age, ups_data,
                _clamped_live if leader_rad is not None else None,
                disp_torque, disp_temp, battery_voltage,
                leader_connected=(leader is not None),
                leader_age=leader_age,
                cam_left_age=cam_left_age,
                cam_right_age=cam_right_age,
                rec_steps=len(qpos_buf),
                recording=recording,
                dropped=dropped_frames,
                estop_active=estop_active,
            ))

            frame_counter += 1
            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        if leader is not None:
            _lr_stop.set()
            _lr_thread.join(timeout=1.0)
            leader.close()
        # Disable all motors before closing (prevents motors staying enabled after quit)
        _send(cmd_sock, {"type": "disable", "motor_ids": ["swivel"] + ARM_JOINTS})
        time.sleep(0.1)  # let ZMQ flush the disable command
        cmd_sock.close()
        telem_sock.close()
        _cam_stop.set()
        _cam_thread.join(timeout=2.0)
        _estop_stop.set()
        if _estop_thread is not None:
            _estop_thread.join(timeout=1.0)
        ctx.term()
        print("\nDone.")


if __name__ == "__main__":
    main()
