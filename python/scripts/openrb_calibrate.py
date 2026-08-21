#!/usr/bin/env python3
"""openrb_calibrate.py — OpenRB-150 + XL330 leader-arm SWEEP calibration wizard.

For each joint you SWEEP it fully min<->max (a couple of passes), then press SPACE.
Unlike the old two-endpoint capture, a continuous sweep measures the joint's TRUE
travelled arc — so a joint whose range is a wide (>180°) descending sweep, or one that
genuinely wraps across the encoder's 0/4096 seam, is captured unambiguously instead of
guessed. The mapping is then centre-anchored (ref_raw / lo_off / hi_off), which has no
wrap heuristic and never pins a joint into a dead zone.

Output schema (per joint):
    ref_raw   physical encoder tick (0-4095) at the CENTRE of the swept arc
    lo_off    signed tick offset from centre to one end   (<= 0)
    hi_off    signed tick offset from centre to other end (>= 0)
    rad_min/rad_max   0 and 1 — the leader emits a NORMALIZED position; the collector
                      maps [0,1] onto the follower joint's calibrated range (range-to-range)
    direction         +1; flip to -1 if a joint drives its arm backwards (flip_leader_dir.py)

Usage:
    python python/scripts/openrb_calibrate.py --port COM4 --output config/openrb_left.json
    python python/scripts/openrb_calibrate.py --port COM7 --output config/openrb_right.json
    python python/scripts/openrb_calibrate.py --port COM4 --output config/openrb_left.json \
        --joints elbow_flex,wrist_flex     # re-capture only these; others preserved

Run python/scripts/openrb_setup_arm.py first if your servos still have factory IDs.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from openrb_leader import OpenRBLeader, find_openrb_port, CALIB_PATH, AIZEE_DEFAULTS, _TICKS

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from so101_calibrate import run_monitor, _ansi_on   # read-only monitor + ANSI setup
from common.arm_constants import setup_keyboard

_RAD_PER_TICK = 2.0 * math.pi / _TICKS


def run_sweep_calibration(arm, get_key, only_joints: Optional[set] = None) -> Optional[dict]:
    """Per joint: sweep fully min<->max, then SPACE. Tracks the continuous unwrapped
    range so the TRUE arc (and any wrap) is measured, not guessed. Returns a
    {joint: {ref_raw, lo_off, hi_off}} dict, or None if aborted."""
    results: dict[str, dict] = {}
    print("\nSWEEP each joint fully min<->max (a couple of passes), then press SPACE.")
    print("Q aborts.\n")
    for i, joint in enumerate(arm.JOINTS):
        aizee = arm.AIZEE_JOINTS[i]
        if only_joints is not None and joint not in only_joints and aizee not in only_joints:
            print(f"  [skip] {joint:<14} -> {aizee}  (not in --joints)")
            continue
        print("=" * 56)
        print(f"  [{i + 1}/{len(arm.JOINTS)}]  {joint} -> {aizee}:  sweep FULLY, then SPACE")
        u_min: Optional[int] = None
        u_max: Optional[int] = None
        while True:
            key = get_key()
            unw = arm.read_unwrapped()
            if unw is not None:
                u = unw[joint]
                u_min = u if u_min is None else min(u_min, u)
                u_max = u if u_max is None else max(u_max, u)
            if key == "Q":
                return None
            if key == " ":
                if u_min is None or (u_max - u_min) < 20:
                    sys.stdout.write("\r  (barely moved — sweep the FULL range, then SPACE)\033[K")
                    sys.stdout.flush()
                    continue
                break
            if u_min is not None:
                sys.stdout.write(f"\r  swept {u_max - u_min:>5} ticks "
                                 f"({(u_max - u_min) * _RAD_PER_TICK:>4.2f} rad)  "
                                 f"cur={u % _TICKS:>4}\033[K")
                sys.stdout.flush()
            time.sleep(0.03)
        center = (u_min + u_max) / 2.0
        results[joint] = {
            "ref_raw": int(round(center)) % _TICKS,
            "lo_off": round(u_min - center, 1),
            "hi_off": round(u_max - center, 1),
        }
        print(f"\r  captured: span={u_max - u_min} ticks "
              f"({(u_max - u_min) * _RAD_PER_TICK:.2f} rad), ref_raw={results[joint]['ref_raw']}\033[K\n")
    return results


def save_sweep_calibration(results: dict, joints: list, aizee_joints: list, path: Path) -> None:
    existing: dict = {}
    if path.exists():
        try:
            existing = json.loads(path.read_text()).get("joints", {})
        except Exception:
            pass
    calib = {
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "baud": 1_000_000,
        "mapping": "center-anchored-sweep",
        "joints": {},
    }
    for i, (joint, aizee) in enumerate(zip(joints, aizee_joints)):
        r = results.get(joint)
        old = existing.get(joint, {})
        if r is None:                      # not re-captured this run — preserve prior
            calib["joints"][joint] = old or {
                "id": i + 1, "aizee": aizee, "ref_raw": _TICKS // 2,
                "lo_off": -(_TICKS // 2), "hi_off": _TICKS // 2,
                "rad_min": AIZEE_DEFAULTS[i][0], "rad_max": AIZEE_DEFAULTS[i][1],
                "zero_offset": 0.0, "direction": 1,
            }
            continue
        calib["joints"][joint] = {
            "id": i + 1,
            "aizee": aizee,
            "ref_raw": r["ref_raw"],
            "lo_off": r["lo_off"],
            "hi_off": r["hi_off"],
            # Leader outputs a NORMALIZED position [0,1] across its sweep; the collector
            # maps that onto the follower joint's calibrated range (range-to-range teleop).
            "rad_min": 0.0,
            "rad_max": 1.0,
            "zero_offset": 0.0,
            # +1 / -1 chooses which follower end frac=0 maps to (flip a backwards joint
            # with flip_leader_dir.py). Preserved across re-runs.
            "direction": old.get("direction", 1),
        }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"\nSaved -> {path}")
    print(json.dumps(calib, indent=2))


def main() -> None:
    ap = argparse.ArgumentParser(
        description="OpenRB-150 leader-arm SWEEP calibration wizard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", default=None, help="OpenRB-150 serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--output", default=str(CALIB_PATH), help="Output JSON path")
    ap.add_argument("--joints", default=None,
                    help="Comma-separated joints to (re)capture; others preserved. OpenRB or "
                         "AIZEE names (e.g. --joints elbow_flex,wrist_flex).")
    args = ap.parse_args()
    only_joints = ({j.strip() for j in args.joints.split(",") if j.strip()}
                   if args.joints else None)

    _ansi_on()
    port = args.port
    if port is None:
        print("Searching for OpenRB-150...")
        port = find_openrb_port(verbose=True)
        if port is None:
            print("OpenRB-150 not found — pass --port explicitly.", file=sys.stderr)
            sys.exit(1)
        print(f"Found OpenRB-150 on {port}")

    arm = OpenRBLeader(port, args.baud, calib=args.output)
    if not arm.connect():
        sys.exit(1)
    get_key = setup_keyboard()
    try:
        print(f"Connected to OpenRB-150 on {port} at {args.baud} baud\n")
        if not run_monitor(arm, get_key):     # Phase 1: live monitor, C to proceed
            return
        print("\nStarting SWEEP calibration...\n")
        time.sleep(0.5)
        results = run_sweep_calibration(arm, get_key, only_joints=only_joints)
        if results is None:
            print("Calibration aborted.")
            return
        save_sweep_calibration(results, arm.JOINTS, arm.AIZEE_JOINTS, Path(args.output))
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
