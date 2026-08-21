#!/usr/bin/env python3
"""minerva_motor_probe.py — non-interactive idle/telemetry probe for the Minerva arms.

Safe first-power-on check over the msgpack wire. For each arm it:
  1. reads telemetry for a moment WITHOUT energizing (pure CAN-comms check),
  2. optionally sends `enable` and holds ZERO-IMPEDANCE (kp=kd=torque=0) so the
     motors are energized but limp/backdrivable — never commands a pose,
  3. prints live per-joint position / temperature / state and flags faults,
  4. sends `disable`.

Runs on the laptop against the Jetson's motor_control instances (Path A):
    left  arm -> cmd :5555  telem :5556
    right arm -> cmd :5575  telem :5576

The Jetson host is auto-detected (LAN -> USB-C direct -> WiFi AP); --host only sets
which address to try first.

Usage:
    python python/scripts/minerva_motor_probe.py --no-enable   # telemetry only
    python python/scripts/minerva_motor_probe.py               # + zero-impedance enable
    python python/scripts/minerva_motor_probe.py --arm left --seconds 10
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import zmq

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.wire import pack_msg, unpack_msg
from minerva_calibrate import arm_joint_names, DEFAULT_HOST, DEFAULT_PORTS
from collect_minerva_app.config import resolve_jetson_host


def _recv_latest(sub) -> Optional[dict]:
    latest = None
    while True:
        try:
            latest = unpack_msg(sub.recv(zmq.NOBLOCK))
        except zmq.Again:
            break
        except Exception:
            break
    return latest


def _fmt_motors(msg: Optional[dict], joints: List[str]) -> str:
    if not msg or not isinstance(msg.get("motors"), dict):
        return "   (no telemetry)"
    motors = msg["motors"]
    lines = []
    for j in joints:
        m = motors.get(j)
        if not isinstance(m, dict):
            lines.append(f"   {j:<15} --- (absent)")
            continue
        pos = m.get("position", 0.0)
        temp = m.get("temperature", 0.0)
        st = m.get("state", "?")
        err = m.get("error")
        flag = f"  !!{err}" if err else (f"  HOT {temp:.0f}C" if temp and temp > 70 else "")
        lines.append(f"   {j:<15} pos={pos:+8.4f}  {temp:4.0f}C  {st}{flag}")
    return "\n".join(lines)


def _summary(msg: Optional[dict], joints: List[str]) -> Dict[str, object]:
    present, faults, hot = [], [], []
    motors = (msg or {}).get("motors", {}) if msg else {}
    for j in joints:
        m = motors.get(j) if isinstance(motors, dict) else None
        if isinstance(m, dict):
            present.append(j)
            if m.get("error"):
                faults.append(f"{j}:{m['error']}")
            if (m.get("temperature") or 0) > 70:
                hot.append(f"{j}:{m['temperature']:.0f}C")
    return {"present": present, "faults": faults, "hot": hot}


def probe_arm(ctx: zmq.Context, side: str, cmd_addr: str, telem_addr: str,
              seconds: float, do_enable: bool) -> bool:
    joints = arm_joint_names(side)
    print(f"\n{'='*60}\n  {side.upper()} ARM   cmd={cmd_addr}  telem={telem_addr}\n{'='*60}")

    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.connect(telem_addr)
    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.LINGER, 500)
    push.connect(cmd_addr)
    time.sleep(0.3)

    # 1. Passive telemetry (no energize)
    print("[1] Passive telemetry (motors NOT energized) ...")
    msg = None
    t0 = time.time()
    while time.time() - t0 < 1.5:
        m = _recv_latest(sub)
        if m is not None:
            msg = m
        time.sleep(0.1)
    if msg is None:
        print("   [FAIL] no telemetry at all — is aizee-minerva-%s running and the "
              "bus up? (journalctl -u aizee-minerva-%s)" % (side, side))
        sub.close(); push.close()
        return False
    print(_fmt_motors(msg, joints))
    s0 = _summary(msg, joints)
    if not s0["present"]:
        print(f"   -> 0/{len(joints)} reporting (normal before enable — RobStride "
              f"MIT motors reply only to command frames)")
    else:
        print(f"   -> {len(s0['present'])}/{len(joints)} joints reporting"
              + (f"; FAULTS {s0['faults']}" if s0['faults'] else ""))

    # PASS is judged on the ENABLED read when energizing (motors are silent while
    # disabled); only in --no-enable mode does the passive read decide.
    ok = len(s0["present"]) == len(joints) and not s0["faults"]

    # 2. Optional zero-impedance enable
    if do_enable:
        print("\n[2] Enabling ZERO-IMPEDANCE (kp=kd=0, limp) — backdrive to see position change ...")
        push.send(pack_msg({"type": "enable", "motor_ids": joints}))
        last = None
        t0 = time.time()
        next_print = 0.0
        n = len(joints)
        while time.time() - t0 < seconds:
            push.send(pack_msg({
                "type": "arm_joints",
                "positions": [0.0] * n, "velocities": [0.0] * n,
                "kp": [0.0] * n, "kd": [0.0] * n, "torques": [0.0] * n,
            }))
            m = _recv_latest(sub)
            if m is not None:
                last = m
            now = time.time() - t0
            if now >= next_print:
                next_print = now + 1.0
                sys.stdout.write("\033[2J\033[H")   # clear for a stable live view
                print(f"  {side.upper()} idle  t={now:4.1f}/{seconds:.0f}s  (Ctrl-C to stop early)")
                print(_fmt_motors(last, joints))
            time.sleep(0.05)
        s1 = _summary(last, joints)
        print(f"\n   -> enabled read: {len(s1['present'])}/{len(joints)} joints"
              + (f"; FAULTS {s1['faults']}" if s1['faults'] else "")
              + (f"; HOT {s1['hot']}" if s1['hot'] else ""))
        ok = len(s1["present"]) == len(joints) and not s1["faults"]

        print("[3] Disabling ...")
        push.send(pack_msg({"type": "disable", "motor_ids": joints}))
        time.sleep(0.3)

    sub.close(); push.close()
    print(f"   {side.upper()} ARM: {'PASS' if ok else 'CHECK'}")
    return ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["left", "right", "both"], default="both")
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="preferred Jetson host; auto-falls back LAN -> USB-C -> AP")
    ap.add_argument("--seconds", type=float, default=8.0, help="zero-impedance hold duration")
    ap.add_argument("--no-enable", dest="enable", action="store_false",
                    help="telemetry only — never energize the motors")
    for side in ("left", "right"):
        ap.add_argument(f"--{side}-cmd", default=None)
        ap.add_argument(f"--{side}-telem", default=None)
    args = ap.parse_args()

    host = resolve_jetson_host(args.host)
    sides = ["left", "right"] if args.arm == "both" else [args.arm]
    ctx = zmq.Context()
    results = {}
    try:
        for s in sides:
            cmd = getattr(args, f"{s}_cmd") or f"tcp://{host}:{DEFAULT_PORTS[s][0]}"
            tel = getattr(args, f"{s}_telem") or f"tcp://{host}:{DEFAULT_PORTS[s][1]}"
            results[s] = probe_arm(ctx, s, cmd, tel, args.seconds, args.enable)
    except KeyboardInterrupt:
        print("\nInterrupted — sending disable to all probed arms ...")
        for s in sides:
            cmd = getattr(args, f"{s}_cmd") or f"tcp://{host}:{DEFAULT_PORTS[s][0]}"
            p = ctx.socket(zmq.PUSH); p.setsockopt(zmq.LINGER, 500); p.connect(cmd)
            time.sleep(0.2)
            p.send(pack_msg({"type": "disable", "motor_ids": arm_joint_names(s)}))
            p.close()
    finally:
        ctx.term()
    print("\n=== SUMMARY ===")
    for s, ok in results.items():
        print(f"  {s:<5}: {'PASS' if ok else 'CHECK'}")


if __name__ == "__main__":
    main()
