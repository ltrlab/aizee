#!/usr/bin/env python3
"""robstride_calibrate.py — RobStride arm position monitor and calibration wizard.

Phase 1  MONITOR    Shows live joint positions (radians) from telemetry.
                    Press E to enable motors, Ctrl-C to quit.

Phase 2  ENABLE     Sends enable command, starts zero-impedance keepalive.
                    Motors enter free-move mode (Kp=0, Kd=0).

Phase 3  CALIBRATE  Guided per-joint capture of MIN and MAX positions.
                    Move the joint, press SPACE to capture, then repeat.

Output:  config/robstride_calibration.json

Usage:
    python robstride_calibrate.py
    python robstride_calibrate.py --endpoint tcp://192.168.0.27:5555 --telemetry tcp://192.168.0.27:5556
    python robstride_calibrate.py --output config/robstride_calibration.json
    python robstride_calibrate.py --joints gantry_base gantry_mid
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import setup_keyboard

# ---------------------------------------------------------------------------
# Joint definitions (matches hardware_jetson_rover.yaml arm section)
# ---------------------------------------------------------------------------

ALL_JOINTS = [
    "gantry_base",
    "gantry_mid",
    "gantry_end",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
]

JOINT_META = {
    "gantry_base": {"can_id": 5,  "model": "ROBSTRIDE04"},
    "gantry_mid":  {"can_id": 6,  "model": "ROBSTRIDE03"},
    "gantry_end":  {"can_id": 7,  "model": "ROBSTRIDE02"},
    "wrist_pitch": {"can_id": 8,  "model": "ROBSTRIDE02"},
    "wrist_roll":  {"can_id": 9,  "model": "ROBSTRIDE00"},
    "gripper":     {"can_id": 10, "model": "ROBSTRIDE00"},
}

DEFAULT_ENDPOINT  = "tcp://192.168.0.27:5555"
DEFAULT_TELEMETRY = "tcp://192.168.0.27:5556"
DEFAULT_OUTPUT    = "config/robstride_calibration.json"

# ---------------------------------------------------------------------------
# ANSI helpers
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_W = 62


def _bar_rad(pos_rad: float, width: int = 20) -> str:
    """ASCII progress bar normalized to ±π."""
    import math
    clamped = max(-math.pi, min(math.pi, pos_rad))
    frac = (clamped + math.pi) / (2 * math.pi)  # 0.0 … 1.0
    filled = int(frac * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"


# ---------------------------------------------------------------------------
# Telemetry thread
# ---------------------------------------------------------------------------

class TelemetryReader:
    """Background thread that subscribes to ZMQ telemetry and stores latest positions."""

    def __init__(self, address: str, joints: list[str]) -> None:
        self._address = address
        self._joints = joints
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_positions(self) -> Optional[dict[str, float]]:
        """Return latest {joint: rad} dict, or None if no data received yet."""
        with self._lock:
            if not self._positions:
                return None
            return dict(self._positions)

    def _run(self) -> None:
        import zmq
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.connect(self._address)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")

        try:
            while not self._stop.is_set():
                # Drain queue, keep only the latest message
                latest: Optional[str] = None
                while True:
                    try:
                        latest = sock.recv_string(zmq.NOBLOCK)
                    except zmq.Again:
                        break

                if latest:
                    try:
                        msg = json.loads(latest)
                        motors = msg.get("motors", {})
                        new_pos = {}
                        for joint in self._joints:
                            if joint in motors:
                                new_pos[joint] = float(motors[joint]["position"])
                        if new_pos:
                            with self._lock:
                                self._positions.update(new_pos)
                    except Exception:
                        pass

                time.sleep(0.02)
        finally:
            sock.close()
            ctx.term()


# ---------------------------------------------------------------------------
# Keepalive thread
# ---------------------------------------------------------------------------

class KeepaliveThread:
    """Sends arm_joints every 50ms with Kp=0, Kd=0 to keep motors in free-move.

    This satisfies the 100ms watchdog and keeps each motor target tracking its
    current position so no torque is applied when gains are non-zero.
    """

    def __init__(self, endpoint: str, joints: list[str], telem: TelemetryReader) -> None:
        self._endpoint = endpoint
        self._joints = joints
        self._telem = telem
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        import zmq
        ctx = zmq.Context()
        sock = ctx.socket(zmq.PUSH)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self._endpoint)

        n = len(self._joints)
        last_positions = [0.0] * n

        try:
            while not self._stop.is_set():
                positions = self._telem.get_positions()
                if positions:
                    last_positions = [
                        positions.get(j, last_positions[i])
                        for i, j in enumerate(self._joints)
                    ]

                cmd = {
                    "type": "arm_joints",
                    "positions": list(last_positions),
                    "velocities": [0.0] * n,
                    "kp": [0.0] * n,
                    "kd": [0.0] * n,
                }
                try:
                    sock.send_string(json.dumps(cmd), zmq.NOBLOCK)
                except Exception:
                    pass

                time.sleep(0.05)
        finally:
            sock.close()
            ctx.term()


# ---------------------------------------------------------------------------
# One-shot ZMQ command helper
# ---------------------------------------------------------------------------

def _send_command(endpoint: str, cmd: dict) -> None:
    """Send a single ZMQ PUSH command and clean up."""
    import zmq
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.LINGER, 2000)
    sock.connect(endpoint)
    time.sleep(0.05)  # allow connection to establish before sending
    sock.send_string(json.dumps(cmd))
    sock.close()
    ctx.term()


# ---------------------------------------------------------------------------
# Display helpers
# ---------------------------------------------------------------------------

def _render_monitor(
    positions: Optional[dict[str, float]],
    joints: list[str],
    endpoint: str,
    prompt: str,
) -> list[str]:
    lines = [
        "=" * _W,
        f"  RobStride Monitor        {endpoint}",
        "=" * _W,
        f"  {'joint':<16} {'radians':>9}  bar",
        "  " + "-" * (_W - 2),
    ]
    for joint in joints:
        if positions and joint in positions:
            r = positions[joint]
            lines.append(f"  {joint:<16} {r:>+9.4f}  {_bar_rad(r, 18)}")
        else:
            lines.append(f"  {joint:<16}      ---")
    lines += [
        "  " + "-" * (_W - 2),
        f"  {prompt}",
        "=" * _W,
    ]
    return lines


def _draw(lines: list[str], n_prev: int = 0, first: bool = False) -> int:
    if not first and n_prev:
        sys.stdout.write(f"\033[{n_prev}A")
    for line in lines:
        sys.stdout.write(f"\r{line}\033[K\n")
    sys.stdout.flush()
    return len(lines)


# ---------------------------------------------------------------------------
# Phase 1 — MONITOR (pre-enable)
# ---------------------------------------------------------------------------

def run_monitor_pre(
    telem: TelemetryReader,
    joints: list[str],
    endpoint: str,
    get_key,
) -> bool:
    """Show live positions until user presses E.  Returns True to proceed to enable."""
    lines = _render_monitor(None, joints, endpoint,
                            "E = enable motors    Ctrl-C = quit")
    n = _draw(lines, first=True)

    while True:
        key = get_key()
        if key == "E":
            return True

        positions = telem.get_positions()
        lines = _render_monitor(positions, joints, endpoint,
                                "E = enable motors    Ctrl-C = quit")
        n = _draw(lines, n_prev=n)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Phase 2 — ENABLE
# ---------------------------------------------------------------------------

def enable_motors(
    endpoint: str,
    joints: list[str],
    telem: TelemetryReader,
) -> KeepaliveThread:
    """Enable arm motors, then start zero-impedance keepalive thread."""
    _send_command(endpoint, {"type": "enable", "motor_ids": joints})

    print("\nEnabling motors", end="", flush=True)

    # Wait up to 10s for all joints to appear in telemetry
    deadline = time.time() + 10.0
    while time.time() < deadline:
        positions = telem.get_positions()
        if positions and all(j in positions for j in joints):
            break
        print(".", end="", flush=True)
        time.sleep(0.2)
    print()

    keepalive = KeepaliveThread(endpoint, joints, telem)
    keepalive.start()

    print("Motors enabled in zero-impedance mode — move freely\n")
    return keepalive


# ---------------------------------------------------------------------------
# Phase 3 — MONITOR (post-enable, live)
# ---------------------------------------------------------------------------

def run_monitor_enabled(
    telem: TelemetryReader,
    joints: list[str],
    endpoint: str,
    get_key,
) -> bool:
    """Show live positions until user presses C.  Returns True to proceed to calibrate."""
    lines = _render_monitor(telem.get_positions(), joints, endpoint,
                            "C = start calibration    Ctrl-C = quit")
    n = _draw(lines, first=True)

    while True:
        key = get_key()
        if key == "C":
            return True

        positions = telem.get_positions()
        lines = _render_monitor(positions, joints, endpoint,
                                "C = start calibration    Ctrl-C = quit")
        n = _draw(lines, n_prev=n)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Phase 4 — CALIBRATE (per joint)
# ---------------------------------------------------------------------------

def run_calibration(
    telem: TelemetryReader,
    joints: list[str],
    get_key,
) -> Optional[dict]:
    """Walk through each joint collecting min/max radian positions.

    For each joint:
      Step A: operator moves joint to MIN, presses SPACE → capture min_rad
      Step B: operator moves joint to MAX, presses SPACE → capture max_rad

    Returns dict of {joint: {min_rad, max_rad}} or None if aborted.
    """
    n_joints = len(joints)
    results: dict[str, dict] = {}

    print("\n")

    for idx, joint in enumerate(joints):
        for step, label in enumerate(("MINIMUM", "MAXIMUM")):
            _print_calib_header(idx, n_joints, joint, step, results)
            print(f"  >> Move  {joint}  to its {label} position, then press SPACE.")
            print(f"     (Ctrl-C to abort)\n")

            captured: Optional[float] = None
            while captured is None:
                key = get_key()
                if key == " ":
                    positions = telem.get_positions()
                    if positions and joint in positions:
                        captured = positions[joint]
                        break

                positions = telem.get_positions()
                if positions and joint in positions:
                    r = positions[joint]
                    line = f"\r  current: rad={r:>+9.4f}  {_bar_rad(r, 20)}\033[K"
                else:
                    line = "\r  current: ---\033[K"
                sys.stdout.write(line)
                sys.stdout.flush()
                time.sleep(0.05)

            sys.stdout.write("\r\033[K")
            print(f"  Captured {label}: rad={captured:+.4f}\n")

            if step == 0:
                results.setdefault(joint, {})["min_rad"] = captured
            else:
                results[joint]["max_rad"] = captured

    return results


def _print_calib_header(
    idx: int,
    total: int,
    joint: str,
    step: int,
    done: dict,
) -> None:
    print("=" * _W)
    print(f"  CALIBRATION  [{idx + 1}/{total}]  {joint}")
    print("=" * _W)
    for j, data in done.items():
        mn = data.get("min_rad")
        mx = data.get("max_rad")
        mn_s = f"{mn:+.4f}" if mn is not None else "?"
        mx_s = f"{mx:+.4f}" if mx is not None else "?"
        print(f"  [done] {j:<18}  min={mn_s}  max={mx_s}")
    if idx > 0:
        print()
    step_str = "A: move to MIN" if step == 0 else "B: move to MAX"
    print(f"  Step {step_str}")
    print()


# ---------------------------------------------------------------------------
# Phase 5 — SAVE
# ---------------------------------------------------------------------------

def save_calibration(
    results: dict,
    joints: list[str],
    endpoint: str,
    path: Path,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    calib: dict = {
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "endpoint": endpoint,
        "joints": {},
    }
    for joint in joints:
        r = results.get(joint, {})
        meta = JOINT_META[joint]
        calib["joints"][joint] = {
            "can_id":  meta["can_id"],
            "model":   meta["model"],
            "min_rad": r.get("min_rad", 0.0),
            "max_rad": r.get("max_rad", 0.0),
        }
    with open(path, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"\nSaved -> {path}")
    print(json.dumps(calib, indent=2))


# ---------------------------------------------------------------------------
# Disable helper
# ---------------------------------------------------------------------------

def disable_motors(endpoint: str, joints: list[str]) -> None:
    try:
        _send_command(endpoint, {"type": "disable", "motor_ids": joints})
        print("Motors disabled.")
    except Exception as exc:
        print(f"Warning: could not send disable command: {exc}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="RobStride arm position monitor and calibration wizard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument(
        "--endpoint",
        default=DEFAULT_ENDPOINT,
        help=f"ZMQ command endpoint (default: {DEFAULT_ENDPOINT})",
    )
    ap.add_argument(
        "--telemetry",
        default=DEFAULT_TELEMETRY,
        help=f"ZMQ telemetry endpoint (default: {DEFAULT_TELEMETRY})",
    )
    ap.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help=f"Output JSON path (default: {DEFAULT_OUTPUT})",
    )
    ap.add_argument(
        "--joints",
        nargs="+",
        default=list(ALL_JOINTS),
        choices=ALL_JOINTS,
        metavar="JOINT",
        help=f"Joints to calibrate (default: all). Choices: {ALL_JOINTS}",
    )
    args = ap.parse_args()

    joints: list[str] = args.joints
    endpoint = args.endpoint
    telem_addr = args.telemetry
    out_path = Path(args.output)

    _ansi_on()

    telem = TelemetryReader(telem_addr, joints)
    telem.start()

    get_key = setup_keyboard()
    keepalive: Optional[KeepaliveThread] = None

    try:
        print(f"RobStride Calibration")
        print(f"  cmd:   {endpoint}")
        print(f"  telem: {telem_addr}\n")

        # Phase 1 — MONITOR (pre-enable)
        if not run_monitor_pre(telem, joints, endpoint, get_key):
            return

        # Phase 2 — ENABLE
        keepalive = enable_motors(endpoint, joints, telem)
        time.sleep(0.3)

        # Phase 3 — MONITOR (post-enable)
        if not run_monitor_enabled(telem, joints, endpoint, get_key):
            return

        # Phase 4 — CALIBRATE
        print("\nStarting calibration wizard...\n")
        time.sleep(0.3)

        results = run_calibration(telem, joints, get_key)
        if results is None:
            print("Calibration aborted.")
            return

        # Phase 5 — SAVE
        save_calibration(results, joints, endpoint, out_path)

    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        if keepalive is not None:
            keepalive.stop()
        disable_motors(endpoint, joints)
        telem.stop()


if __name__ == "__main__":
    main()
