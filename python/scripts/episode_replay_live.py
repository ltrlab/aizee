#!/usr/bin/env python3
"""episode_replay_live.py — Live episode replay for AIZEE arm.

Loads a recorded episode (episode_XXXX.hdf5 or recording_XXXX.hdf5) and
replays it on the arm using the ZMQ motor control interface.

Usage:
    python episode_replay_live.py episodes/episode_0000.hdf5 [options]

Keys:
    SPACE / P   — play / pause / resume
    R           — restart from beginning
    X           — abort + shutdown (return arm to zero)
    Q           — quit
"""

from __future__ import annotations

import argparse
import enum
import json
import sys
import time
from pathlib import Path
from typing import Optional

import h5py
import numpy as np
import yaml
import zmq

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from record_replay import ARM_JOINTS, KP, KD, setup_keyboard, load_arm_limits, clamp_arm_positions
from control.gravity_comp import ArmGravityModel

NUM_JOINTS    = len(ARM_JOINTS)   # 6
LOOP_HZ       = 30
_REPLAY_JOINTS = ["swivel"] + ARM_JOINTS   # 7 joints for display

# ---------------------------------------------------------------------------
# Display constants
# ---------------------------------------------------------------------------

_W  = 76
_IW = _W - 2

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

_TEMP_WARN = 65.0
_TEMP_CRIT = 80.0
_VBUS_WARN = 20.0
_VBUS_CRIT = 18.0


# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


def _render(
    ep_name:         str,
    ep_frames:       int,
    ep_duration:     float,
    ep_hz:           float,
    target:          Optional[np.ndarray],   # [7] swivel + arm, or None
    actual:          Optional[np.ndarray],   # [7] swivel + arm, or None
    status:          str,
    hint:            str,
    robot_ok:        bool                 = False,
    telem_age:       float                = 999.0,
    ups_data:        Optional[dict]       = None,
    torque:          Optional[np.ndarray] = None,   # [7]
    temp:            Optional[np.ndarray] = None,   # [7]
    battery_voltage: Optional[float]      = None,
    frame_idx:       int                  = 0,
    speed:           float                = 1.0,
    dropped:         int                  = 0,
) -> list[str]:
    TOP = "\u2554" + "\u2550" * _IW + "\u2557"
    MID = "\u2560" + "\u2550" * _IW + "\u2563"
    BOT = "\u255a" + "\u2550" * _IW + "\u255d"
    SEP = "\u2551  " + "\u2500" * (_IW - 4) + "  \u2551"

    def _row(text: str, vis: int = -1) -> str:
        vlen = len(text) if vis < 0 else vis
        return "\u2551" + text + " " * max(0, _IW - vlen) + "\u2551"

    # Title row
    title_txt = "  AIZEE Episode Replay"
    title_vis = len(title_txt)
    gap = max(1, _IW - title_vis - len(status))
    title_line = _row(
        f"{title_txt}{' ' * gap}{status}",
        title_vis + gap + len(status),
    )

    # Episode info row
    ep_line = _row(f"  {ep_name}   {ep_frames} frames  {ep_duration:.1f}s  @ {ep_hz:.0f} Hz  speed: {speed:.1f}\u00d7")

    # Column header
    header_line = _row(
        f"  {'joint':<18} {'recorded':>9}  {'actual':>8}   {'err':>7}  {'torq':>5}  {'temp':>4}"
    )

    # Joint rows (7 joints: swivel + 6 arm)
    joint_lines = []
    for i, jname in enumerate(_REPLAY_JOINTS):
        t_ok = target is not None and i < len(target) and not np.isnan(float(target[i]))
        a_ok = actual is not None and i < len(actual) and not np.isnan(float(actual[i]))
        t_s  = f"{float(target[i]):>+9.3f}" if t_ok else "       --"
        a_s  = f"{float(actual[i]):>+8.3f}" if a_ok else "      --"
        e_s  = f"{float(target[i] - actual[i]):>+7.3f}" if (t_ok and a_ok) else "     --"

        tq_ok = torque is not None and i < len(torque) and not np.isnan(float(torque[i]))
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

        temp_ok = temp is not None and i < len(temp) and not np.isnan(float(temp[i]))
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

        row_text = f"  {jname:<18} {t_s}  {a_s}   {e_s}  {tq_s}  {temp_s}"
        row_vis  = 2 + 18 + 1 + 9 + 2 + 8 + 3 + 7 + 2 + 5 + 2 + 4
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

    # Progress line
    rec_elapsed = frame_idx / ep_hz if ep_hz > 0 else 0.0
    pct_done    = 100.0 * frame_idx / max(ep_frames, 1)
    prog_txt    = f"frame: {frame_idx:4d}/{ep_frames}   t={rec_elapsed:.1f}s/{ep_duration:.1f}s  ({pct_done:.0f}%)"
    if dropped:
        prog_txt += f"   drop:{dropped}"
    prog_line = _row(f"  {prog_txt}")

    ctrl_line = _row(f"  {hint}" if hint else "  Q quit")

    return [
        TOP, title_line, MID,
        ep_line, MID,
        header_line, SEP, *joint_lines, SEP,
        robot_line, prog_line,
        MID, ctrl_line, BOT,
    ]


_N = len(_render("episode_0000.hdf5", 200, 10.0, 20.0, None, None, "", ""))


def _draw(lines: list[str], first: bool = False) -> None:
    if not first:
        sys.stdout.write(f"\033[{_N}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()


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
# Telemetry helpers
# ---------------------------------------------------------------------------

def _qpos(telem: Optional[dict]) -> Optional[np.ndarray]:
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
# Episode loading
# ---------------------------------------------------------------------------

def load_episode(path: Path) -> tuple[np.ndarray, Optional[np.ndarray], float]:
    """Load episode HDF5 (format_version=2).

    format_version=2 stores the swivel as column 0 of qpos / qcmd. This
    loader peels column 0 back into a separate `swivel` array for the
    existing renderer / command path.

    Returns:
        qpos   : [T, 6] float32 — arm joint positions (commanded if available, else actual)
        swivel : [T]    float32 — swivel positions (column 0), or None if absent
        hz     : float  — recording rate
    """
    with h5py.File(path, "r") as f:
        if "observations" in f and "qpos" in f["observations"]:
            if "qcmd" in f["observations"]:
                raw = f["observations/qcmd"][:]
                print("  Using commanded positions (qcmd) for replay")
            else:
                raw = f["observations/qpos"][:]
                print("  Using actual positions (qpos) for replay — no qcmd in file")
            hz = float(f.attrs.get("hz", 20.0))
            fmt = int(f.attrs.get("format_version", 1))
        elif "qpos" in f:
            raw = f["qpos"][:]
            hz = float(f.attrs.get("hz", 20.0))
            fmt = int(f.attrs.get("format_version", 1))
        else:
            raise ValueError(f"Unrecognised HDF5 format: {path}")

    raw = raw.astype(np.float32)

    if raw.ndim != 2:
        raise ValueError(f"qpos must be 2D, got shape {raw.shape}")

    # format_version=2: 7 columns = [swivel, *ARM_JOINTS]
    if raw.shape[1] == NUM_JOINTS + 1:
        swivel = raw[:, 0].copy()
        qpos = raw[:, 1:].copy()
        return qpos, swivel, hz

    # Legacy: 6 columns, no swivel recorded
    if raw.shape[1] == NUM_JOINTS:
        if fmt >= 2:
            print(f"  [WARN] format_version={fmt} but qpos has only {NUM_JOINTS} columns")
        return raw, None, hz

    raise ValueError(
        f"qpos has {raw.shape[1]} columns; expected {NUM_JOINTS} or {NUM_JOINTS + 1} "
        f"(swivel-prefixed). Re-record this episode with the current collect_demo.py."
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    _ep = _load_endpoints()
    ap  = argparse.ArgumentParser(
        description="Live episode replay for AIZEE arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("file",                help="Episode HDF5 file to replay")
    ap.add_argument("--cmd",               default=_ep.get("command",       "tcp://192.168.0.27:5555"))
    ap.add_argument("--telem",             default=_ep.get("telemetry",     "tcp://192.168.0.27:5556"))
    ap.add_argument("--ups",               default=_ep.get("ups_telemetry", "tcp://192.168.0.27:5562"),
                    help="UPS telemetry address (empty to disable)")
    ap.add_argument("--speed",             type=float, default=1.0,  help="Playback speed multiplier (default 1.0)")
    ap.add_argument("--max-delta",         type=float, default=0.3,  dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.3)")
    ap.add_argument("--no-goto-start",     action="store_true",      dest="no_goto_start",
                    help="Skip slow ramp to episode start position before replay")
    ap.add_argument("--ramp-speed",        type=float, default=1.5,  dest="ramp_speed",
                    help="Approach speed to start position [rad/s] (default 1.5)")
    ap.add_argument("--loop",              action="store_true",      help="Loop episode indefinitely")
    ap.add_argument("--urdf",              default=None,             help="URDF file for gravity compensation")
    ap.add_argument("--gravity-comp",      action="store_true",      dest="gravity_comp",
                    help="Enable gravity compensation feedforward (off by default when replaying qcmd)")
    ap.add_argument("--gravity-scale",    type=float, default=1.0,  dest="gravity_scale",
                    help="Scale factor for gravity compensation torques (default 1.0)")
    ap.add_argument("--no-vel-ff",         action="store_true",      dest="no_vel_ff",
                    help="Disable velocity feedforward during replay")
    ap.add_argument("--dry-run",           action="store_true",      dest="dry_run",
                    help="Show UI and advance frames without sending motor commands")
    ap.add_argument("--robstride-calib",   default=None,             dest="robstride_calib")
    args = ap.parse_args()

    _ansi_on()

    # Load episode
    ep_path = Path(args.file)
    if not ep_path.exists():
        print(f"Error: file not found: {ep_path}", file=sys.stderr)
        sys.exit(1)

    ep_qpos, ep_swivel, ep_hz = load_episode(ep_path)
    ep_frames    = len(ep_qpos)
    ep_duration  = ep_frames / ep_hz if ep_hz > 0 else 0.0
    frame_period = 1.0 / (ep_hz * args.speed)   # wall-clock seconds per frame
    has_swivel   = ep_swivel is not None
    goto_start   = not args.no_goto_start
    _ramp_delta  = args.ramp_speed / LOOP_HZ   # rad per control step

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    _yaml  = _load_teleop_yaml()
    _tcfg  = _yaml.get("gantry", {})
    _dcfg  = _yaml.get("drive",  {})
    _kp: list      = _tcfg.get("kp", KP)[:NUM_JOINTS]
    _kd: list      = _tcfg.get("kd", KD)[:NUM_JOINTS]
    _swivel_kp     = float(_dcfg.get("swivel_kp", 80.0))
    _swivel_kd     = float(_dcfg.get("swivel_kd", 5.0))

    # -------------------------------------------------------------------------
    # Gravity compensation model
    # -------------------------------------------------------------------------
    _grav_model: Optional[ArmGravityModel] = None
    _grav_calib = Path(__file__).resolve().parent.parent.parent / "config" / "gravity_calibration.json"
    if args.gravity_comp:
        if args.urdf is not None:
            _grav_model = ArmGravityModel.from_urdf(args.urdf)
            print(f"Gravity comp: loaded from {args.urdf}")
        elif _grav_calib.exists():
            _grav_model = ArmGravityModel.from_calibration(_grav_calib)
            print(f"Gravity comp: loaded calibrated model from {_grav_calib.name}")
        else:
            _grav_model = ArmGravityModel()
            print("Gravity comp: using default model (placeholder masses)")
        _grav_model.print_model()

    # Gravity comp mask: only apply to joints that need it.
    # wrist_pitch disabled — physically perpendicular, doesn't bear gravity load.
    # gantry_base (Z-axis) and gripper (Z-axis) are always zero anyway.
    _GRAV_MASK = [1, 1, 1, 0, 1, 0]  # [base, mid, end, wrist_pitch, wrist_roll, gripper]
    if _grav_model is not None:
        print(f"Gravity comp: joints=[gantry_mid, gantry_end], scale={args.gravity_scale:.2f}")

    # -------------------------------------------------------------------------
    # Pre-compute velocity feedforward from trajectory
    # -------------------------------------------------------------------------
    # Finite-difference velocities: dq/dt between consecutive frames.
    # Smoothed with a ±1-frame moving average to reduce noise.
    ep_velocities: Optional[np.ndarray] = None
    if not args.no_vel_ff and ep_frames > 1:
        dt = 1.0 / ep_hz if ep_hz > 0 else 0.05
        dq = np.diff(ep_qpos, axis=0) / dt                  # [T-1, 6]
        # Pad last frame with zero velocity (decelerate to stop)
        dq = np.vstack([dq, np.zeros((1, NUM_JOINTS))])      # [T, 6]
        # Simple ±1 frame smoothing
        smoothed = dq.copy()
        for i in range(1, ep_frames - 1):
            smoothed[i] = (dq[i - 1] + dq[i] + dq[i + 1]) / 3.0
        smoothed[0] = (dq[0] + dq[1]) / 2.0 if ep_frames > 1 else dq[0]
        ep_velocities = smoothed.astype(np.float32)
        print(f"Velocity FF: pre-computed from {ep_frames} frames at {ep_hz} Hz")

    # -------------------------------------------------------------------------
    # ZMQ sockets
    # -------------------------------------------------------------------------
    ctx = zmq.Context()

    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 2)
    cmd_sock.setsockopt(zmq.LINGER,  0)
    if not args.dry_run:
        cmd_sock.connect(args.cmd)

    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.setsockopt(zmq.LINGER, 0)
    telem_sock.setsockopt(zmq.CONFLATE, 1)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    if not args.dry_run:
        telem_sock.connect(args.telem)

    ups_sock: Optional[zmq.Socket] = None
    if args.ups and not args.dry_run:
        ups_sock = ctx.socket(zmq.SUB)
        ups_sock.setsockopt(zmq.LINGER, 0)
        ups_sock.setsockopt(zmq.CONFLATE, 1)
        ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        ups_sock.connect(args.ups)

    get_key = setup_keyboard()

    # Seed q_actual from first telemetry packet
    q_actual:      Optional[np.ndarray] = None
    swivel_actual: Optional[float]      = None
    if not args.dry_run:
        for _ in range(40):
            telem = _drain(telem_sock)
            if telem:
                q = _qpos(telem)
                if q is not None:
                    q_actual = q
                    sw = telem.get("motors", {}).get("swivel")
                    if sw is not None:
                        swivel_actual = float(sw.get("position", 0.0))
                    break
            time.sleep(0.05)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------
    class State(enum.Enum):
        READY    = "ready"     # loaded, motors off, press SPACE to start
        ARMING   = "arming"    # enabling motors + moving to start position
        PLAYING  = "playing"   # replaying frames
        PAUSED   = "paused"    # mid-episode pause
        DONE     = "done"      # episode finished, holding last position
        SHUTDOWN = "shutdown"  # returning arm to zero

    _nan = float("nan")

    state               = State.READY
    frame_idx           = 0
    last_frame_wall     = 0.0
    # current_target / current_swivel_tgt: what we last commanded
    current_target:      Optional[np.ndarray] = ep_qpos[0] if ep_frames > 0 else None
    current_swivel_tgt:  Optional[float]      = float(ep_swivel[0]) if has_swivel and ep_frames > 0 else None
    arm_torques:         Optional[np.ndarray] = None
    arm_temps:           Optional[np.ndarray] = None
    swivel_torque:       Optional[float]      = None
    swivel_temp:         Optional[float]      = None
    last_telem_time:     float = time.time() if q_actual is not None else 0.0
    robot_ok:            bool  = q_actual is not None
    ups_data:            Optional[dict]  = None
    battery_voltage:     Optional[float] = None
    dropped              = 0
    shutdown_target:     Optional[np.ndarray] = None
    shutdown_swivel:     Optional[float]      = None
    shutdown_countdown:  float = 0.0
    shutdown_zero_since: float = 0.0
    shutdown_grav_ramp:  Optional[float] = None
    _SHUTDOWN_TIMEOUT          = 3.0

    status = "[ ] ready — motors off"
    hint   = "SPACE=start · Q=quit"
    if not has_swivel:
        hint += "  [no swivel in episode]"

    period = 1.0 / LOOP_HZ

    def _make_disp(arm: Optional[np.ndarray], swivel: Optional[float]) -> Optional[np.ndarray]:
        """Build 7-element [swivel, arm...] display array."""
        if arm is None:
            return None
        sv = swivel if swivel is not None else _nan
        return np.concatenate([[sv], arm]).astype(np.float32)

    _draw(_render(
        ep_path.name, ep_frames, ep_duration, ep_hz,
        _make_disp(current_target, current_swivel_tgt),
        _make_disp(q_actual, swivel_actual),
        status, hint, robot_ok=robot_ok,
        frame_idx=frame_idx, speed=args.speed,
    ), first=True)

    def _send_arm(
        q_cmd: np.ndarray,
        torques: Optional[list] = None,
        velocities: Optional[list] = None,
        grav_scale: Optional[float] = None,
    ) -> None:
        if args.dry_run:
            return
        # Compute gravity compensation if model is available
        ff_torques = torques
        if ff_torques is None and _grav_model is not None and q_actual is not None:
            gs = grav_scale if grav_scale is not None else args.gravity_scale
            raw = _grav_model.gravity_torques(q_actual)
            ff_torques = [float(raw[i]) * _GRAV_MASK[i] * gs for i in range(NUM_JOINTS)]
        _send(cmd_sock, {
            "type":       "arm_joints",
            "positions":  q_cmd.tolist(),
            "velocities": velocities if velocities is not None else [0.0] * NUM_JOINTS,
            "kp": _kp, "kd": _kd,
            "torques": ff_torques if ff_torques is not None else [0.0] * NUM_JOINTS,
        })

    def _send_swivel(pos: float) -> None:
        if args.dry_run:
            return
        _send(cmd_sock, {"type": "swivel", "position": pos,
                         "kp": _swivel_kp, "kd": _swivel_kd})

    def _safe_cmd(target_pos: np.ndarray, ref: Optional[np.ndarray]) -> np.ndarray:
        r = ref if ref is not None else target_pos
        q = r + np.clip(target_pos - r, -args.max_delta, args.max_delta)
        if arm_limits:
            q = np.array(clamp_arm_positions(q.tolist(), arm_limits))
        return q

    all_motor_ids = (["swivel"] + ARM_JOINTS) if has_swivel else ARM_JOINTS

    try:
        while True:
            t0 = time.time()

            # -----------------------------------------------------------------
            # Keyboard
            # -----------------------------------------------------------------
            key = get_key()

            if key == "Q":
                break

            elif key in (" ", "P"):  # SPACE or P = play/pause/resume
                if state == State.READY:
                    if not args.dry_run:
                        _send(cmd_sock, {"type": "enable", "motor_ids": all_motor_ids})
                    frame_idx = 0
                    dropped   = 0
                    if goto_start and q_actual is not None and not args.dry_run:
                        state = State.ARMING
                    else:
                        last_frame_wall = t0
                        state = State.PLAYING

                elif state == State.PLAYING:
                    state = State.PAUSED

                elif state == State.PAUSED:
                    last_frame_wall = t0
                    state = State.PLAYING

                elif state == State.DONE:
                    frame_idx = 0
                    dropped   = 0
                    if goto_start and not args.dry_run:
                        state = State.ARMING
                    else:
                        last_frame_wall = t0
                        state = State.PLAYING

            elif key == "R":
                if state in (State.PLAYING, State.PAUSED, State.DONE):
                    frame_idx = 0
                    dropped   = 0
                    if goto_start and not args.dry_run:
                        state = State.ARMING
                    else:
                        last_frame_wall = t0
                        state = State.PLAYING

            elif key == "X":
                if state in (State.PLAYING, State.PAUSED, State.DONE, State.ARMING):
                    shutdown_target    = q_actual.copy() if q_actual is not None else np.zeros(NUM_JOINTS)
                    shutdown_swivel    = swivel_actual if swivel_actual is not None else 0.0
                    shutdown_countdown  = 1.0
                    shutdown_zero_since = 0.0
                    shutdown_grav_ramp  = None
                    state = State.SHUTDOWN

            # -----------------------------------------------------------------
            # Per-state actions
            # -----------------------------------------------------------------
            if state == State.ARMING:
                tgt    = ep_qpos[0]
                sw_tgt = float(ep_swivel[0]) if has_swivel else None
                # Slow ramp to start position (ramp_speed rad/s, not max_delta)
                ref   = q_actual if q_actual is not None else tgt
                q_cmd = ref + np.clip(tgt - ref, -_ramp_delta, _ramp_delta)
                if arm_limits:
                    q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits))
                _send_arm(q_cmd)
                if sw_tgt is not None:
                    _send_swivel(sw_tgt)
                current_target     = tgt
                current_swivel_tgt = sw_tgt
                arm_ok  = q_actual is not None and np.all(np.abs(q_actual - tgt) < 0.03)
                swiv_ok = (not has_swivel or sw_tgt is None or swivel_actual is None
                           or abs(swivel_actual - sw_tgt) < 0.03)
                if arm_ok and swiv_ok:
                    last_frame_wall = t0
                    state = State.PLAYING

            elif state == State.PLAYING:
                if t0 - last_frame_wall >= frame_period and frame_idx < ep_frames:
                    last_frame_wall = t0
                    tgt    = ep_qpos[frame_idx]
                    sw_tgt = float(ep_swivel[frame_idx]) if has_swivel else None
                    vel_ff = (ep_velocities[frame_idx].tolist()
                              if ep_velocities is not None else None)
                    _send_arm(_safe_cmd(tgt, current_target), velocities=vel_ff)
                    if sw_tgt is not None:
                        _send_swivel(sw_tgt)
                    current_target     = tgt
                    current_swivel_tgt = sw_tgt
                    frame_idx += 1
                    if frame_idx >= ep_frames:
                        if args.loop:
                            frame_idx = 0
                        else:
                            state = State.DONE

            elif state in (State.PAUSED, State.DONE):
                if current_target is not None:
                    _send_arm(_safe_cmd(current_target, q_actual))
                if current_swivel_tgt is not None:
                    _send_swivel(current_swivel_tgt)

            elif state == State.SHUTDOWN:
                dt         = period
                max_change = 0.2 * dt   # 0.2 rad/s ramp
                if shutdown_countdown > 0:
                    shutdown_countdown -= dt
                    if shutdown_target is not None:
                        _send_arm(shutdown_target)
                    if shutdown_swivel is not None:
                        _send_swivel(shutdown_swivel)
                else:
                    if shutdown_target is None:
                        shutdown_target = np.zeros(NUM_JOINTS)
                    # Check completion BEFORE sending — the ZMQ PUSH socket
                    # has HWM=2 so sending arm+swivel+disable in one iteration
                    # would silently drop the disable command.
                    ramp_done = (np.all(np.abs(shutdown_target) < 0.01)
                                 and (not has_swivel
                                      or abs(shutdown_swivel if shutdown_swivel is not None else 0.0) < 0.01))
                    if ramp_done and shutdown_zero_since == 0.0:
                        shutdown_zero_since = t0
                    actual_close = (q_actual is None or np.all(np.abs(q_actual) < 0.05))
                    timed_out    = (shutdown_zero_since > 0
                                    and t0 - shutdown_zero_since >= _SHUTDOWN_TIMEOUT)
                    if shutdown_grav_ramp is not None:
                        # Gravity ramp-down phase: reduce feedforward before disable
                        if shutdown_grav_ramp > 0.01:
                            shutdown_grav_ramp = max(0.0, shutdown_grav_ramp - dt * 2.0)
                            _send_arm(np.zeros(NUM_JOINTS), grav_scale=shutdown_grav_ramp)
                            if has_swivel:
                                _send_swivel(0.0)
                        else:
                            if not args.dry_run:
                                _send(cmd_sock, {"type": "disable", "motor_ids": all_motor_ids})
                            state              = State.READY
                            frame_idx          = 0
                            current_target     = ep_qpos[0] if ep_frames > 0 else None
                            current_swivel_tgt = float(ep_swivel[0]) if has_swivel and ep_frames > 0 else None
                    elif ramp_done and (actual_close or timed_out):
                        # Position ramp complete — begin gravity comp ramp-down
                        shutdown_grav_ramp = args.gravity_scale if _grav_model is not None else 0.0
                    else:
                        # Still ramping — send position commands
                        ref     = q_actual if q_actual is not None else shutdown_target
                        new_tgt = shutdown_target.copy()
                        for i in range(len(new_tgt)):
                            new_tgt[i] = (0.0 if abs(new_tgt[i]) < max_change
                                          else new_tgt[i] - np.sign(new_tgt[i]) * max_change)
                        shutdown_target = new_tgt
                        _send_arm(_safe_cmd(shutdown_target, ref))
                        if shutdown_swivel is None:
                            shutdown_swivel = 0.0
                        shutdown_swivel = (0.0 if abs(shutdown_swivel) < max_change
                                           else shutdown_swivel - np.sign(shutdown_swivel) * max_change)
                        if has_swivel:
                            _send_swivel(shutdown_swivel)

            # -----------------------------------------------------------------
            # Telemetry
            # -----------------------------------------------------------------
            telem = _drain(telem_sock) if not args.dry_run else None
            q_new = _qpos(telem)
            if q_new is not None:
                q_actual        = q_new
                robot_ok        = True
                last_telem_time = t0
            if telem and "motors" in telem:
                sw = telem["motors"].get("swivel")
                if sw is not None:
                    swivel_actual = float(sw.get("position",    0.0))
                    swivel_torque = float(sw.get("torque",      _nan))
                    swivel_temp   = float(sw.get("temperature", _nan))
                tq = _qtorque(telem)
                if tq is not None:
                    arm_torques = tq
                te = _qtemp(telem)
                if te is not None:
                    arm_temps = te
                bv = telem.get("battery_voltage")
                if bv is not None:
                    battery_voltage = float(bv)

            if ups_sock is not None:
                um = _drain(ups_sock)
                if um and "ups" in um:
                    ups_data = um["ups"]

            # -----------------------------------------------------------------
            # Status strings
            # -----------------------------------------------------------------
            if state == State.READY:
                status = "[ ] ready — motors off"
                hint   = "SPACE=start · Q=quit"

            elif state == State.ARMING:
                if q_actual is not None:
                    err = float(np.max(np.abs(q_actual - ep_qpos[0])))
                    status = f"[~] moving to start  err {err:.3f} rad"
                else:
                    status = "[~] moving to start position"
                hint = "X=abort · Q=quit"

            elif state == State.PLAYING:
                pct    = 100.0 * frame_idx / max(ep_frames, 1)
                lp_tag = "  LOOP" if args.loop else ""
                status = f"[>] PLAYING  {pct:.0f}%{lp_tag}"
                hint   = "SPACE=pause · R=restart · X=shutdown · Q=quit"

            elif state == State.PAUSED:
                pct    = 100.0 * frame_idx / max(ep_frames, 1)
                status = f"[||] PAUSED  {pct:.0f}%"
                hint   = "SPACE=resume · R=restart · X=shutdown · Q=quit"

            elif state == State.DONE:
                status = "[done] episode complete"
                hint   = "SPACE=replay · R=restart · X=shutdown · Q=quit"

            elif state == State.SHUTDOWN:
                if shutdown_countdown > 0:
                    status = f"[X] shutdown  hold {shutdown_countdown:.1f}s"
                elif shutdown_grav_ramp is not None:
                    status = f"[X] ramping down gravity comp ({shutdown_grav_ramp:.2f})"
                else:
                    status = "[X] returning to zero"
                hint   = "Q=quit"

            # -----------------------------------------------------------------
            # Render
            # -----------------------------------------------------------------
            if state == State.SHUTDOWN:
                disp_arm    = shutdown_target
                disp_swivel = shutdown_swivel
            else:
                disp_arm    = current_target
                disp_swivel = current_swivel_tgt

            disp_torque = (_make_disp(arm_torques, swivel_torque)
                           if arm_torques is not None else None)
            disp_temp   = (_make_disp(arm_temps,   swivel_temp)
                           if arm_temps is not None else None)
            telem_age   = t0 - last_telem_time if robot_ok else 999.0

            _draw(_render(
                ep_path.name, ep_frames, ep_duration, ep_hz,
                _make_disp(disp_arm, disp_swivel),
                _make_disp(q_actual, swivel_actual),
                status, hint,
                robot_ok, telem_age, ups_data,
                disp_torque, disp_temp, battery_voltage,
                frame_idx, args.speed, dropped,
            ))

            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        if not args.dry_run:
            # Disable all motors before closing (prevents motors staying enabled after quit)
            _send(cmd_sock, {"type": "disable", "motor_ids": all_motor_ids})
            time.sleep(0.1)  # let ZMQ flush the disable command
            cmd_sock.close()
            telem_sock.close()
        if ups_sock is not None:
            ups_sock.close()
        ctx.term()
        print("\nDone.")


if __name__ == "__main__":
    main()
