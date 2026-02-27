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
    H    hold — freeze target at current actual position
    Q    quit  (Ctrl-C also works)
"""

from __future__ import annotations

import argparse
import enum
import json
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import zmq

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from so101_leader import So101Leader, CALIB_PATH

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import ARM_JOINTS, RECORD_HZ, KP, KD, setup_keyboard, load_arm_limits, clamp_arm_positions

# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_W = 68
_LEADER_JOINTS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_yaw",     "wrist_roll",    "gripper",
]

# UPS voltage thresholds (V)
_UPS_OK   = 11.7
_UPS_WARN = 10.8
_UPS_CRIT = 10.0

# ANSI color codes (enabled on Windows via _ansi_on())
_GRN = "\033[1;32m"
_YEL = "\033[1;33m"
_RED = "\033[1;31m"
_RST = "\033[0m"


def _render(
    leader_rad: Optional[np.ndarray],
    target:     Optional[np.ndarray],
    actual:     Optional[np.ndarray],
    status:     str,
    hint:       str,
    robot_ok:   bool  = False,
    telem_age:  float = 999.0,
    ups_data:   Optional[dict] = None,
) -> list[str]:
    BAR = "=" * _W
    SEP = "-" * (_W - 2)   # inner separator indented 2 spaces = same total width as BAR

    # Robot status (with color; pad by visible text length to keep UPS aligned)
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

    # UPS status
    if ups_data:
        v   = float(ups_data.get("voltage",    0.0))
        c   = float(ups_data.get("current",    0.0))
        p   = float(ups_data.get("power",      0.0))
        pct = float(ups_data.get("percentage", 0.0))
        if   v >= _UPS_OK:   col, ups_st = _GRN, "OK"
        elif v >= _UPS_WARN: col, ups_st = _YEL, "WARN"
        elif v >= _UPS_CRIT: col, ups_st = _RED, "CRIT"
        else:                col, ups_st = _RED, "SHUTDOWN"
        ups_line = f"UPS  {v:.2f}V  {c:.2f}A  {p:.1f}W  ({pct:.0f}%)  {col}[{ups_st}]{_RST}"
    else:
        ups_line = "UPS  --"

    lines = [
        BAR,
        f"  SO-101 \u2192 AIZEE Teleop{' ' * max(1, _W - 23 - len(status))}{status}",
        BAR,
        f"  {'so101 joint':<18} {'leader':>8}  {'target':>8}  {'actual':>8}   {'err':>7}",
        f"  {SEP}",
    ]
    for i, so101j in enumerate(_LEADER_JOINTS):
        l_s = f"{float(leader_rad[i]):>+8.3f}" if leader_rad is not None else "      --"
        t_ok = target is not None and not np.isnan(target[i])
        a_ok = actual is not None and not np.isnan(actual[i])
        t_s = f"{float(target[i]):>+8.3f}" if t_ok else "      --"
        a_s = f"{float(actual[i]):>+8.3f}" if a_ok else "      --"
        e_s = f"{float(target[i] - actual[i]):>+7.3f}" if (t_ok and a_ok) else "     --"
        lines.append(f"  {so101j:<18} {l_s}  {t_s}  {a_s}   {e_s}")
    lines += [
        f"  {SEP}",
        f"  {robot_display}{robot_pad}{ups_line}",
        f"  {hint}",
        BAR,
    ]
    return lines


_N = len(_render(None, None, None, "", ""))


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

def main() -> None:
    ap = argparse.ArgumentParser(
        description="SO-101 leader arm teleop for the AIZEE arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",      required=True,                          help="SO-101 serial port")
    ap.add_argument("--baud",      type=int,  default=1_000_000)
    ap.add_argument("--calib",     default=str(CALIB_PATH),               help="Calibration JSON")
    ap.add_argument("--cmd",       default="tcp://localhost:5555")
    ap.add_argument("--telem",     default="tcp://localhost:5556")
    ap.add_argument("--ups",       default="tcp://localhost:5562",
                    help="UPS telemetry address (empty string to disable)")
    ap.add_argument("--max-delta",     type=float, default=0.05, dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.05)")
    ap.add_argument("--robstride-calib", default=None, dest="robstride_calib",
                    help="Path to robstride_calibration.json (default: auto-discover)")
    ap.add_argument("--align-margin",  type=float, default=0.05, dest="align_margin",
                    help="Max per-joint error [rad] to be considered aligned (default 0.05)")
    ap.add_argument("--align-time",    type=float, default=3.0,  dest="align_time",
                    help="Seconds to hold within margin before tracking begins (default 3.0)")
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

    # Indices into the 7-elem leader array that map to ARM_JOINTS (skips wrist_yaw).
    # e.g. [0, 1, 2, 3, 5, 6] — index 4 (wrist_yaw) has no AIZEE motor.
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
    telem_sock.connect(args.telem)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    ups_sock: Optional[zmq.Socket] = None
    if args.ups:
        ups_sock = ctx.socket(zmq.SUB)
        ups_sock.setsockopt(zmq.LINGER, 0)
        ups_sock.setsockopt_string(zmq.SUBSCRIBE, "")
        ups_sock.connect(args.ups)

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
        ALIGNING = "aligning"   # enabled, slowly moving arm to match leader
        TRACKING = "tracking"   # following leader in real time
        HOLD     = "hold"       # target frozen at last actual

    teleop_state                   = State.READY
    converge_start: Optional[float] = None   # when arm first entered margin
    held_target:    Optional[np.ndarray] = None
    zero_msg:       str   = ""               # status text for zero-capture flash
    zero_msg_until: float = 0.0              # show zero_msg until this time
    last_telem_time: float = time.time() if q_actual is not None else 0.0
    ups_data:       Optional[dict] = None
    robot_ok = q_actual is not None

    status = "[ ] ready"
    hint   = "E=enable  Z=zero  M=mirror  Q=quit"

    # Initial draw
    _init_actual = None
    if q_actual is not None:
        _init_actual = np.full(7, np.nan, dtype=np.float32)
        _init_actual[_so101_for_aizee] = q_actual
    _draw(_render(None, None, _init_actual, status, hint, robot_ok, 999.0, None), first=True)

    period = 1.0 / RECORD_HZ

    try:
        while True:
            t0 = time.time()

            # --- Keyboard ---
            key = get_key()
            if key == "Q":
                break

            elif key == "E":
                # Enable arm motors and enter alignment phase
                try:
                    cmd_sock.send_string(json.dumps({"type": "enable", "motor_ids": ARM_JOINTS}), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                teleop_state   = State.ALIGNING
                converge_start = None

            elif key == "H":
                if teleop_state in (State.TRACKING, State.ALIGNING):
                    # Freeze target at current actual position
                    if q_actual is not None:
                        held_target = q_actual.copy()
                    teleop_state = State.HOLD
                elif teleop_state == State.HOLD:
                    # Return to alignment before tracking resumes
                    teleop_state   = State.ALIGNING
                    converge_start = None

            elif key == "Z":
                # Capture current SO-101 positions as new zero reference.
                # leader_rad may not be available yet; read one poll directly.
                _z = leader.poll()
                if _z is not None:
                    zero_offsets = _z.copy()
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[Z] zeroed — saved"
                    zero_msg_until = t0 + 2.0

            elif key == "M":
                # Mirror: set zero so current SO-101 pose maps to current AIZEE actual.
                # q_actual is 6-elem (AIZEE); zero_offsets/directions are 7-elem (SO-101).
                # For each AIZEE motor j: zero_offsets[s] = _m[s] - directions[s] * q_actual[j]
                # where s = _so101_for_aizee[j].  wrist_yaw offset (index 4) is unchanged.
                _m = leader.poll()
                if _m is not None and q_actual is not None:
                    new_offsets = zero_offsets.copy()
                    for aizee_j, so101_i in enumerate(_so101_for_aizee):
                        new_offsets[so101_i] = _m[so101_i] - directions[so101_i] * q_actual[aizee_j]
                    zero_offsets = new_offsets
                    leader.save_zero(zero_offsets)
                    zero_msg       = "[M] mirrored — saved"
                    zero_msg_until = t0 + 2.0

            # --- Read SO-101 ---
            leader_rad = leader.poll()

            # --- Apply per-joint zero offset + direction ---
            # mapped_rad (7-elem, AIZEE space) = directions * (leader_rad - zero_offsets)
            mapped_rad: Optional[np.ndarray] = (
                directions * (leader_rad - zero_offsets)
                if leader_rad is not None else None
            )
            # aizee_cmd: 6-elem command for Rust (skips wrist_yaw at index 4)
            aizee_cmd: Optional[np.ndarray] = (
                mapped_rad[_so101_for_aizee] if mapped_rad is not None else None
            )

            # --- Determine target (6-elem, AIZEE command space) ---
            if teleop_state == State.HOLD:
                target = held_target
            elif aizee_cmd is not None:
                target = aizee_cmd
            else:
                target = q_actual   # no leader data — hold current actual

            # --- Send arm command (all states except READY) ---
            if target is not None and teleop_state != State.READY:
                ref   = q_actual if q_actual is not None else target
                delta = np.clip(target - ref, -args.max_delta, args.max_delta)
                q_cmd = ref + delta
                if arm_limits:
                    q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits), dtype=np.float32)
                try:
                    cmd_sock.send_string(json.dumps({
                        "type":       "arm_joints",
                        "positions":  q_cmd.tolist(),
                        "velocities": [0.0] * len(ARM_JOINTS),
                        "kp":         KP,
                        "kd":         KD,
                    }), zmq.NOBLOCK)
                except zmq.Again:
                    pass

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

            if ups_sock is not None:
                ups_msg = _drain(ups_sock)
                if ups_msg and "ups" in ups_msg:
                    ups_data = ups_msg["ups"]

            # --- Build status + hint ---
            if teleop_state == State.READY:
                status = "[ ] ready"
                hint   = "E=enable  Z=zero  M=mirror  Q=quit"

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
                hint = "H=hold  Z/M=zero  E=align  Q=quit"

            elif teleop_state == State.TRACKING:
                status = "[*] tracking" if leader_rad is not None else "[!] no leader data"
                hint   = "H=hold  Z/M=zero  E=align  Q=quit"

            elif teleop_state == State.HOLD:
                status = "[H] HOLD"
                hint   = "H=resume  Z/M=zero  Q=quit"

            # Zero capture flash overrides status for 2 s
            if t0 < zero_msg_until:
                status = zero_msg

            # --- Render ---
            # Expand 6-elem AIZEE arrays to 7-elem display arrays (NaN for wrist_yaw)
            if target is not None:
                target_display = np.full(7, np.nan, dtype=np.float32)
                target_display[_so101_for_aizee] = target
            else:
                target_display = None
            if q_actual is not None:
                actual_display = np.full(7, np.nan, dtype=np.float32)
                actual_display[_so101_for_aizee] = q_actual
            else:
                actual_display = None
            telem_age = t0 - last_telem_time if robot_ok else 999.0
            _draw(_render(leader_rad, target_display, actual_display, status, hint,
                          robot_ok, telem_age, ups_data))

            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        print("\nQuit.")
        leader.close()
        cmd_sock.close()
        telem_sock.close()
        if ups_sock is not None:
            ups_sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
