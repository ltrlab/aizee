#!/usr/bin/env python3
"""minerva_calibrate.py — dual-arm RobStride min/max calibration for Minerva.

Records every arm joint's physical travel (min/max radians) by moving it under
zero-impedance and capturing — the same idea as AIZEE's robstride_calibrate.py,
but for BOTH Minerva arms and over the **msgpack** wire format the live
motor_control node actually speaks. (robstride_calibrate.py still uses the old
JSON send_string/recv_string format and no longer talks to the node.)

Topology (Path A: one motor_control instance per arm, one CAN bus per arm):

    left  arm  ->  cmd tcp://HOST:5555   telem tcp://HOST:5556
    right arm  ->  cmd tcp://HOST:5557   telem tcp://HOST:5558

Each instance's "arm group" is swivel(idx0) + arm[] = 7 joints, in this order
(must match config/hardware_minerva_{left,right}.yaml):

    j1(shoulder,id4)  j2(id5)  j3(id6)  j4(id7)  j5(id8)  j6(id9)  gripper(id10)

Per-arm flow:
    MONITOR  ->  (E) enable free-move  ->  (C) capture each joint's MIN then MAX

Output: config/minerva_calibration.json  (flat joints map keyed by canonical
Minerva joint name, covering whichever arms were calibrated).

Usage:
    python python/scripts/minerva_calibrate.py --host 192.168.0.27
    python python/scripts/minerva_calibrate.py --arm left
    python python/scripts/minerva_calibrate.py \
        --left-cmd tcp://10.0.0.5:5555 --left-telem tcp://10.0.0.5:5556

Controls:  E = enable   C = start capture   SPACE = capture position   Ctrl-C = quit
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import zmq

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # python/ -> common.*
from common.arm_constants import setup_keyboard
from common.wire import pack_msg, unpack_msg
from collect_minerva_app.config import resolve_jetson_host

# ---------------------------------------------------------------------------
# Minerva arm layout — MUST match hardware_minerva_{left,right}.yaml arm group
# order (swivel index 0, then arm[]).  (suffix, can_id, model)
# ---------------------------------------------------------------------------
JOINT_SPECS = [
    ("j1",      0x04, "ROBSTRIDE03"),   # shoulder  — model ASSUMED, confirm
    ("j2",      0x05, "ROBSTRIDE04"),
    ("j3",      0x06, "ROBSTRIDE03"),
    ("j4",      0x07, "ROBSTRIDE02"),
    ("j5",      0x08, "ROBSTRIDE02"),
    ("j6",      0x09, "ROBSTRIDE00"),
    ("gripper", 0x0A, "ROBSTRIDE00"),
]

DEFAULT_HOST = "192.168.0.27"
# right arm on 5575/5576 (5557-5560 are taken by aizee-camera-relay on the Jetson).
DEFAULT_PORTS = {"left": (5555, 5556), "right": (5575, 5576)}
DEFAULT_OUTPUT = "config/minerva_calibration.json"


def joint_name(side: str, suffix: str) -> str:
    """Canonical Minerva joint name (matches minerva_constants.MINERVA_JOINTS)."""
    return f"{side}_gripper" if suffix == "gripper" else f"{side}_arm_{suffix}"


def arm_joint_names(side: str) -> List[str]:
    return [joint_name(side, suffix) for suffix, _, _ in JOINT_SPECS]


# ---------------------------------------------------------------------------
# Telemetry subscriber (msgpack PUB -> SUB)
# ---------------------------------------------------------------------------
class TelemetryReader:
    """Background SUB thread; caches the latest {joint: position_rad}."""

    def __init__(self, ctx: zmq.Context, address: str, joints: List[str]) -> None:
        self._address = address
        self._joints = joints
        self._lock = threading.Lock()
        self._positions: Dict[str, float] = {}
        self._sock = ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.connect(address)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sock.close()

    def get(self) -> Optional[Dict[str, float]]:
        with self._lock:
            return dict(self._positions) if self._positions else None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                buf = self._sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.01)
                continue
            except Exception:
                break
            try:
                msg = unpack_msg(buf)
                motors = msg.get("motors", {})
                new = {j: float(motors[j]["position"]) for j in self._joints if j in motors}
                if new:
                    with self._lock:
                        self._positions.update(new)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Command helpers (msgpack PUSH -> PULL)
# ---------------------------------------------------------------------------
def send_command(ctx: zmq.Context, address: str, cmd: dict) -> None:
    """One-shot PUSH send (enable/disable). Fresh socket, linger to flush."""
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.LINGER, 1000)
    sock.connect(address)
    time.sleep(0.1)  # let PUSH connect before sending, else the frame is dropped
    sock.send(pack_msg(cmd))
    sock.close()


class Keepalive:
    """Sends arm_joints @ 20 Hz with kp=kd=0 so motors free-backdrive while the
    operator moves them, and the 500 ms watchdog never fires."""

    def __init__(self, ctx: zmq.Context, address: str, joints: List[str],
                 telem: TelemetryReader) -> None:
        self._joints = joints
        self._telem = telem
        self._n = len(joints)
        self._sock = ctx.socket(zmq.PUSH)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(address)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._sock.close()

    def _run(self) -> None:
        last = [0.0] * self._n
        while not self._stop.is_set():
            pos = self._telem.get()
            if pos:
                last = [pos.get(j, last[i]) for i, j in enumerate(self._joints)]
            cmd = {
                "type": "arm_joints",
                "positions": list(last),
                "velocities": [0.0] * self._n,
                "kp": [0.0] * self._n,
                "kd": [0.0] * self._n,
                "torques": [0.0] * self._n,
            }
            try:
                self._sock.send(pack_msg(cmd), zmq.NOBLOCK)
            except Exception:
                pass
            time.sleep(0.05)


# ---------------------------------------------------------------------------
# Terminal UI
# ---------------------------------------------------------------------------
def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


def _bar(rad: float, width: int = 22) -> str:
    import math
    frac = (max(-math.pi, min(math.pi, rad)) + math.pi) / (2 * math.pi)
    filled = int(frac * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def monitor(telem: TelemetryReader, joints: List[str], get_key,
            wait_key: str, prompt: str) -> bool:
    """Live-print positions until *wait_key* is pressed. False on Ctrl-C/quit."""
    n_lines = 0
    while True:
        key = get_key()
        if key == wait_key:
            return True
        if key == "Q":
            return False
        pos = telem.get()
        lines = [f"  {'joint':<16}{'radians':>10}   bar"]
        for j in joints:
            if pos and j in pos:
                lines.append(f"  {j:<16}{pos[j]:>+10.4f}   {_bar(pos[j])}")
            else:
                lines.append(f"  {j:<16}{'---':>10}")
        lines.append(f"  {prompt}")
        if n_lines:
            sys.stdout.write(f"\033[{n_lines}A")
        for ln in lines:
            sys.stdout.write(f"\r{ln}\033[K\n")
        sys.stdout.flush()
        n_lines = len(lines)
        time.sleep(0.05)


def capture(telem: TelemetryReader, joints: List[str], get_key) -> Optional[Dict[str, dict]]:
    """Walk each joint: capture MIN then MAX on SPACE. None on abort."""
    results: Dict[str, dict] = {}
    for idx, j in enumerate(joints):
        for step, label in enumerate(("MINIMUM", "MAXIMUM")):
            print("=" * 52)
            print(f"  [{idx + 1}/{len(joints)}]  {j}   ->  move to {label}, press SPACE")
            print("=" * 52)
            captured: Optional[float] = None
            while captured is None:
                key = get_key()
                if key == "Q":
                    return None
                pos = telem.get()
                if key == " " and pos and j in pos:
                    captured = pos[j]
                    break
                cur = f"{pos[j]:+.4f}" if pos and j in pos else "---"
                sys.stdout.write(f"\r  current: {cur:>12}  {_bar(pos[j]) if pos and j in pos else ''}\033[K")
                sys.stdout.flush()
                time.sleep(0.05)
            print(f"\r  captured {label}: {captured:+.4f}\033[K\n")
            results.setdefault(j, {})["min_rad" if step == 0 else "max_rad"] = captured
    return results


# ---------------------------------------------------------------------------
# Per-arm calibration
# ---------------------------------------------------------------------------
def calibrate_arm(ctx: zmq.Context, side: str, cmd_addr: str, telem_addr: str,
                  get_key) -> Optional[Dict[str, dict]]:
    joints = arm_joint_names(side)
    print("\n" + "#" * 52)
    print(f"#  {side.upper()} ARM   cmd={cmd_addr}  telem={telem_addr}")
    print("#" * 52)

    telem = TelemetryReader(ctx, telem_addr, joints)
    telem.start()
    keepalive: Optional[Keepalive] = None
    try:
        if not monitor(telem, joints, get_key, "E",
                       "E = enable free-move   Q = quit"):
            return None

        print(f"\nEnabling {side} motors ...")
        send_command(ctx, cmd_addr, {"type": "enable", "motor_ids": joints})
        deadline = time.time() + 10.0
        while time.time() < deadline:
            pos = telem.get()
            if pos and all(j in pos for j in joints):
                break
            time.sleep(0.2)
        else:
            print(f"[warn] not all {side} joints reported telemetry — check the "
                  f"instance, CAN bus, and motor models before moving on.")

        keepalive = Keepalive(ctx, cmd_addr, joints, telem)
        keepalive.start()
        print("Motors free (kp=kd=0). Move them by hand.\n")

        if not monitor(telem, joints, get_key, "C",
                       "C = start min/max capture   Q = quit"):
            return None
        return capture(telem, joints, get_key)
    finally:
        if keepalive is not None:
            keepalive.stop()
        try:
            send_command(ctx, cmd_addr, {"type": "disable", "motor_ids": joints})
            print(f"{side} motors disabled.")
        except Exception as exc:
            print(f"[warn] could not disable {side}: {exc}")
        telem.stop()


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------
def save(results_by_side: Dict[str, Dict[str, dict]], endpoints: dict,
         path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    out: dict = {
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "wire": "msgpack",
        "arms": endpoints,
        "joints": {},
    }
    for side, results in results_by_side.items():
        for suffix, can_id, model in JOINT_SPECS:
            name = joint_name(side, suffix)
            r = results.get(name, {})
            out["joints"][name] = {
                "arm": side,
                "can_id": can_id,
                "model": model,
                "min_rad": r.get("min_rad", 0.0),
                "max_rad": r.get("max_rad", 0.0),
            }
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved -> {path}")
    print(json.dumps(out, indent=2))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["left", "right", "both"], default="both")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help=f"Jetson host for default endpoints (default {DEFAULT_HOST})")
    for side in ("left", "right"):
        c, t = DEFAULT_PORTS[side]
        ap.add_argument(f"--{side}-cmd", default=None, help=f"{side} command endpoint")
        ap.add_argument(f"--{side}-telem", default=None, help=f"{side} telemetry endpoint")
    ap.add_argument("--output", default=DEFAULT_OUTPUT)
    args = ap.parse_args()
    host = resolve_jetson_host(args.host)

    def ep(side: str, kind_idx: int, override: Optional[str]) -> str:
        if override:
            return override
        return f"tcp://{host}:{DEFAULT_PORTS[side][kind_idx]}"

    sides = ["left", "right"] if args.arm == "both" else [args.arm]
    endpoints = {
        s: {"cmd": ep(s, 0, getattr(args, f"{s}_cmd")),
            "telem": ep(s, 1, getattr(args, f"{s}_telem"))}
        for s in sides
    }

    _ansi_on()
    get_key = setup_keyboard()
    ctx = zmq.Context()
    results_by_side: Dict[str, Dict[str, dict]] = {}
    try:
        for s in sides:
            res = calibrate_arm(ctx, s, endpoints[s]["cmd"], endpoints[s]["telem"], get_key)
            if res is None:
                print(f"\n{s} arm aborted — nothing saved for it.")
                continue
            results_by_side[s] = res
        if results_by_side:
            save(results_by_side, endpoints, Path(args.output))
        else:
            print("\nNo arms calibrated.")
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        ctx.term()


if __name__ == "__main__":
    main()
