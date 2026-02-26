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
from record_replay import ARM_JOINTS, RECORD_HZ, KP, KD, setup_keyboard

# ---------------------------------------------------------------------------
# ANSI display
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_W = 62
_LEADER_JOINTS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper",
]


def _render(
    leader_rad: Optional[np.ndarray],
    target:     Optional[np.ndarray],
    actual:     Optional[np.ndarray],
    status:     str,
    hint:       str,
    robot_ok:   bool = False,
) -> list[str]:
    sep = "-" * _W
    robot_line = "  robot: connected" if robot_ok else "  robot: offline (no telemetry)"
    lines = [
        "=" * _W,
        "  SO-101 -> AIZEE Teleop",
        "=" * _W,
        f"  {'so101 joint':<18} {'leader':>8}  {'target':>8}  {'actual':>8}",
        f"  {sep}",
    ]
    for i, (so101j, aizeej) in enumerate(zip(_LEADER_JOINTS, ARM_JOINTS)):
        l_s = f"{float(leader_rad[i]):>+8.3f}" if leader_rad is not None else "      --"
        t_s = f"{float(target[i]):>+8.3f}"     if target     is not None else "      --"
        a_s = f"{float(actual[i]):>+8.3f}"     if actual     is not None else "      --"
        lines.append(f"  {so101j:<18} {l_s}  {t_s}  {a_s}")
    lines += [
        f"  {sep}",
        robot_line,
        f"  {status}",
        f"  {hint}",
        "=" * _W,
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
    ap.add_argument("--max-delta", type=float, default=0.05, dest="max_delta",
                    help="Per-step safety clamp [rad] (default 0.05)")
    args = ap.parse_args()

    _ansi_on()

    # --- SO-101 leader arm ---
    leader = So101Leader(args.port, args.baud, calib=args.calib)
    if not leader.connect():
        sys.exit(1)

    calib_present = Path(args.calib).exists()
    print(f"SO-101 connected on {args.port}")
    print(f"Calibration: {'loaded from ' + args.calib if calib_present else 'NONE — raw ticks->rad (run so101_calibrate.py first)'}")

    # --- ZMQ ---
    ctx        = zmq.Context()
    cmd_sock   = ctx.socket(zmq.PUSH)
    cmd_sock.setsockopt(zmq.SNDHWM, 2)   # never block — drop stale commands
    cmd_sock.setsockopt(zmq.LINGER,  0)  # don't wait on close
    cmd_sock.connect(args.cmd)
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.connect(args.telem)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")

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

    hold        = False
    held_target: Optional[np.ndarray] = None
    status      = "[ ] ready"
    hint        = "E=enable  H=hold  Q=quit"
    robot_ok    = q_actual is not None

    # Initial draw
    _draw(_render(None, None, q_actual, status, hint, robot_ok), first=True)

    period = 1.0 / RECORD_HZ

    try:
        while True:
            t0 = time.time()

            # --- Keyboard ---
            key = get_key()
            if key == "Q":
                break
            elif key == "E":
                try:
                    cmd_sock.send_string(json.dumps({"type": "enable", "motor_ids": ARM_JOINTS}), zmq.NOBLOCK)
                except zmq.Again:
                    pass
                hint   = "E=enable  H=hold  Q=quit"
                status = "[ ] enabled"
            elif key == "H":
                hold = not hold
                if hold and q_actual is not None:
                    held_target = q_actual.copy()
                    status = "[H] HOLD — target frozen at actual"
                else:
                    hold   = False
                    status = "[ ] tracking leader"

            # --- Read SO-101 ---
            leader_rad = leader.poll()

            if hold and held_target is not None:
                target = held_target
            elif leader_rad is not None:
                target = leader_rad
            else:
                target = q_actual  # stale — no command if no leader read

            # --- Safety clamp and send ---
            if target is not None:
                ref = q_actual if q_actual is not None else target
                delta  = np.clip(target - ref, -args.max_delta, args.max_delta)
                q_cmd  = ref + delta
                try:
                    cmd_sock.send_string(json.dumps({
                        "type":       "arm_joints",
                        "positions":  q_cmd.tolist(),
                        "velocities": [0.0] * 6,
                        "kp":         KP,
                        "kd":         KD,
                    }), zmq.NOBLOCK)
                except zmq.Again:
                    pass

            # --- Telemetry ---
            telem  = _drain(telem_sock)
            q_new  = _qpos(telem)
            if q_new is not None:
                q_actual = q_new
                robot_ok = True

            # --- Status ---
            if not hold and leader_rad is not None:
                status = "[*] tracking"
            elif leader_rad is None:
                status = "[!] no leader data"

            # --- Render ---
            _draw(_render(leader_rad, target, q_actual, status, hint, robot_ok))

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
        ctx.term()


if __name__ == "__main__":
    main()
