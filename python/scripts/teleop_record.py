#!/usr/bin/env python3
"""teleop_record.py — Combined arm teleop + trajectory recording.

Single-terminal UI: drive the arm with keys, press R to record/stop.
Recordings saved to recordings/recording_XXXX.hdf5 (record_replay.py format).

Key bindings:
  1/2   gantry_base    +/- 0.02 rad
  3/4   gantry_mid     +/- 0.02 rad
  5/6   gantry_end     +/- 0.02 rad
  7/8   wrist_pitch    +/- 0.02 rad
  [/]   wrist_roll     +/- 0.02 rad
  -/=   gripper        +/- 0.02 rad
  E     enable all arm joints
  H     re-seed target from current actual position (safe re-home)
  R     start / stop recording
  SPC   emergency stop
  Q     quit  (Ctrl-C also works)

Usage:
    python teleop_record.py
    python teleop_record.py --cmd tcp://192.168.0.27:5555 \\
                            --telem tcp://192.168.0.27:5556
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_msg
from typing import Optional

import numpy as np
import zmq

# ---------------------------------------------------------------------------
# Import shared helpers from record_replay (co-deployed in the same directory)
# ---------------------------------------------------------------------------
sys.path.insert(0, str(Path(__file__).parent))
from record_replay import (
    ARM_JOINTS, RECORD_HZ, KP, KD,
    save_recording, _next_recording_path,
    setup_keyboard, load_arm_limits, clamp_arm_positions,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
INCREMENT = 0.02  # rad per key-press (matches teleop.yaml increment)

# (joint_index, delta) — indices match ARM_JOINTS (swivel-first 7-DOF).
# Z/C jog the swivel; the original gantry bindings (1..8, [/], -/=) shift up
# by one index to account for swivel landing at position 0.
KEY_BINDINGS: dict[str, tuple[int, float]] = {
    "Z": (0, +INCREMENT), "C": (0, -INCREMENT),  # swivel
    "1": (1, +INCREMENT), "2": (1, -INCREMENT),  # gantry_base
    "3": (2, +INCREMENT), "4": (2, -INCREMENT),  # gantry_mid
    "5": (3, +INCREMENT), "6": (3, -INCREMENT),  # gantry_end
    "7": (4, +INCREMENT), "8": (4, -INCREMENT),  # wrist_pitch
    "[": (5, +INCREMENT), "]": (5, -INCREMENT),  # wrist_roll
    "-": (6, +INCREMENT), "=": (6, -INCREMENT),  # gripper
}

# ---------------------------------------------------------------------------
# ZMQ helpers (small enough to copy inline)
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


def _qpos(telem: dict) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    out = []
    for j in ARM_JOINTS:
        m = motors.get(j)
        if m is None:
            return None
        out.append(float(m.get("position", 0.0)))
    return np.array(out, dtype=np.float32)


def _vels(telem: dict) -> Optional[np.ndarray]:
    if not telem or "motors" not in telem:
        return None
    motors = telem["motors"]
    out = []
    for j in ARM_JOINTS:
        m = motors.get(j)
        if m is None:
            return None
        out.append(float(m.get("velocity", 0.0)))
    return np.array(out, dtype=np.float32)

# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _enable_ansi_windows() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_KEY_LABELS = ["1 / 2", "3 / 4", "5 / 6", "7 / 8", "[ / ]", "- / ="]
_W = 58  # base display width (extended when limits shown)


def _render(
    q_target: np.ndarray,
    q_actual: Optional[np.ndarray],
    rec_line: str,
    hint_line: str,
    arm_limits: Optional[dict] = None,
) -> list[str]:
    show_lim = arm_limits is not None
    w = (_W + 20) if show_lim else _W
    sep = "-" * w
    hdr_suffix = f"  {'lo':>8} {'hi':>8}" if show_lim else ""
    lines = [
        "=" * w,
        "  AIZEE  TELEOP + RECORD",
        "=" * w,
        f"  {'joint':<16} {'keys':<7} {'target':>8}  {'actual':>8}{hdr_suffix}",
        f"  {sep}",
    ]
    for i, (joint, kl) in enumerate(zip(ARM_JOINTS, _KEY_LABELS)):
        t = float(q_target[i])
        if q_actual is not None:
            a_str = f"{float(q_actual[i]):>+8.3f}"
        else:
            a_str = "      --"
        if show_lim and joint in arm_limits:
            lo, hi = arm_limits[joint]
            at_lo = t <= lo + 1e-4
            at_hi = t >= hi - 1e-4
            marker = "!" if (at_lo or at_hi) else " "
            lim_str = f"  {marker}{lo:>+7.3f} {hi:>+8.3f}"
        else:
            lim_str = ""
        lines.append(f"  {joint:<16} {kl:<7} {t:>+8.3f}  {a_str}{lim_str}")
    lines += [
        f"  {sep}",
        f"  {rec_line}",
        f"  {hint_line}",
        "=" * w,
    ]
    return lines


_N_LINES = len(_render(np.zeros(len(ARM_JOINTS)), None, "", ""))  # line count unchanged by limits


def _draw(lines: list[str], first: bool = False) -> None:
    if not first:
        sys.stdout.write(f"\033[{_N_LINES}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(
        description="Combined arm teleop + recording",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--cmd",        default="tcp://localhost:5555")
    ap.add_argument("--telem",      default="tcp://localhost:5556")
    ap.add_argument("--output-dir", default="recordings")
    ap.add_argument(
        "--max-delta", type=float, default=0.05, dest="max_delta",
        help="Per-step safety clamp radius (rad). Default 0.05.",
    )
    ap.add_argument(
        "--robstride-calib", default=None, dest="robstride_calib",
        help="Path to robstride_calibration.json (default: auto-discover)",
    )
    args = ap.parse_args()

    _enable_ansi_windows()

    arm_limits = load_arm_limits(Path(args.robstride_calib) if args.robstride_calib else None)
    if arm_limits:
        print(f"Arm limits loaded ({len(arm_limits)} joints)")

    # --- ZMQ ---
    ctx        = zmq.Context()
    cmd_sock   = ctx.socket(zmq.PUSH)
    cmd_sock.connect(args.cmd)
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.connect(args.telem)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")

    get_key = setup_keyboard()

    # Seed target from first telemetry packet (avoids jump on startup)
    q_target: np.ndarray        = np.zeros(len(ARM_JOINTS), dtype=np.float32)
    q_actual: Optional[np.ndarray] = None
    print(f"Connecting to {args.telem} ...")
    for _ in range(40):          # up to 2 s
        telem = _drain(telem_sock)
        if telem:
            q = _qpos(telem)
            if q is not None:
                q_target = q.copy()
                q_actual = q.copy()
                break
        time.sleep(0.05)
    if q_actual is None:
        print("No telemetry yet — target seeded at zero. Press E to enable motors.")
        time.sleep(1.0)

    # --- Recording state ---
    recording:  bool              = False
    qpos_buf:   list[np.ndarray]  = []
    vel_buf:    list[np.ndarray]  = []
    ts_buf:     list[float]       = []
    rec_path:   Optional[Path]    = None
    last_saved: Optional[Path]    = None

    recordings_dir = Path(args.output_dir)
    recordings_dir.mkdir(parents=True, exist_ok=True)

    # --- Initial draw ---
    rec_line  = "[ ] IDLE"
    hint_line = "E=enable  H=home  SPC=estop  R=rec  Q=quit"
    _draw(_render(q_target, q_actual, rec_line, hint_line, arm_limits), first=True)

    period = 1.0 / RECORD_HZ

    try:
        while True:
            t0 = time.time()

            # ---- Key handling ----------------------------------------
            key = get_key()

            if key == "Q":
                break

            elif key == " ":
                cmd_sock.send(pack_msg({"type": "emergency_stop"}))
                if recording:
                    _save(qpos_buf, vel_buf, ts_buf, rec_path)
                    last_saved, recording = rec_path, False
                rec_line  = "[!] ESTOP sent"
                hint_line = "E=enable  H=home  SPC=estop  R=rec  Q=quit"

            elif key == "E":
                cmd_sock.send(pack_msg({
                    "type": "enable", "motor_ids": ARM_JOINTS,
                }))
                hint_line = "E=enable  H=home  SPC=estop  R=rec  Q=quit"

            elif key == "H":
                if q_actual is not None:
                    q_target  = q_actual.copy()
                    hint_line = "Homed (target = actual)  R=rec  Q=quit"

            elif key == "R":
                if not recording:
                    rec_path  = _next_recording_path(recordings_dir)
                    recording = True
                    qpos_buf.clear(); vel_buf.clear(); ts_buf.clear()
                    hint_line = "R=stop  Q=quit"
                else:
                    _save(qpos_buf, vel_buf, ts_buf, rec_path)
                    last_saved, recording = rec_path, False
                    rec_path  = None
                    hint_line = "E=enable  H=home  SPC=estop  R=rec  Q=quit"

            elif key in KEY_BINDINGS:
                joint_i, delta = KEY_BINDINGS[key]
                q_target[joint_i] = q_target[joint_i] + delta
                # Clamp target so it can't wind up past calibration limits.
                if arm_limits:
                    q_target = np.array(
                        clamp_arm_positions(q_target.tolist(), arm_limits),
                        dtype=np.float32,
                    )

            # ---- Safety-clamped command --------------------------------
            q_cmd = q_target.copy()
            if q_actual is not None:
                delta = np.clip(q_cmd - q_actual, -args.max_delta, args.max_delta)
                q_cmd = q_actual + delta
            if arm_limits:
                q_cmd = np.array(clamp_arm_positions(q_cmd.tolist(), arm_limits), dtype=np.float32)

            cmd_sock.send(pack_msg({
                "type":       "arm_joints",
                "positions":  q_cmd.tolist(),
                "velocities": [0.0] * len(q_cmd),
                "kp":         KP,
                "kd":         KD,
            }))

            # ---- Telemetry ---------------------------------------------
            telem = _drain(telem_sock)
            if telem:
                q = _qpos(telem)
                v = _vels(telem)
                if q is not None:
                    q_actual = q
                if recording and q is not None and v is not None:
                    qpos_buf.append(q.copy())
                    vel_buf.append(v.copy())
                    ts_buf.append(time.time())

            # ---- Status line -------------------------------------------
            if recording:
                elapsed  = len(ts_buf) / RECORD_HZ
                rec_line = (
                    f"[*] REC  {len(qpos_buf):4d} steps  {elapsed:5.1f}s"
                    f"  ->  {rec_path.name}"
                )
            elif last_saved:
                rec_line = f"[ ] IDLE  saved: {last_saved.name}"
            else:
                rec_line = "[ ] IDLE"

            # ---- Render ------------------------------------------------
            _draw(_render(q_target, q_actual, rec_line, hint_line, arm_limits))

            sleep_t = period - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        pass
    finally:
        if recording and qpos_buf:
            _save(qpos_buf, vel_buf, ts_buf, rec_path)
            last_saved = rec_path
        # Move cursor below the display block
        sys.stdout.write(f"\n")
        print("Quit.")
        if last_saved:
            print(f"Last recording: {last_saved}")
        cmd_sock.close()
        telem_sock.close()
        ctx.term()


def _save(
    qpos_buf: list[np.ndarray],
    vel_buf:  list[np.ndarray],
    ts_buf:   list[float],
    path:     Optional[Path],
) -> None:
    if not qpos_buf or path is None:
        return
    save_recording(path, np.stack(qpos_buf), np.stack(vel_buf), np.array(ts_buf))


if __name__ == "__main__":
    main()
