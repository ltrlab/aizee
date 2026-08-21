#!/usr/bin/env python3
"""leader_diag.py — live per-joint leader mapping diagnostic.

Shows, for each leader joint as you sweep it: the physical raw (0-4095), the
UNWRAPPED value, the calibrated window, the wrap/desc/asc mode the mapper picked,
the resulting frac, and whether it's CLAMPED. Use it to see exactly which joints
hit a dead zone (frac pinned at 0 or 1 while you're still moving them).

Quit the collector first (it holds the serial port).

    python python/scripts/leader_diag.py --port COM4 --calib config/openrb_left.json
    python python/scripts/leader_diag.py --port COM7 --calib config/openrb_right.json
"""
from __future__ import annotations
import argparse, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "teleop"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from openrb_leader import OpenRBLeader, _frac_from_calib

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("--calib", required=True)
args = ap.parse_args()

arm = OpenRBLeader(args.port, calib=args.calib)
if not arm.connect():
    sys.exit(f"connect failed on {args.port}")
calib = (arm._calib or {}).get("joints", {})

print("Sweep each joint FULLY min<->max. 'CLAMP' = mapper pinned it (dead zone).")
print("Ctrl-C to stop.\n")
# Track the raw range actually swept, per joint, to compare against the calib window.
lo_seen = {j: 9999 for j in arm.JOINTS}
hi_seen = {j: -1 for j in arm.JOINTS}
try:
    while True:
        unw = arm.read_unwrapped()
        if not unw:
            time.sleep(0.05); continue
        rows = []
        for i, j in enumerate(arm.JOINTS):
            u = unw[j]; raw = u % 4096
            lo_seen[j] = min(lo_seen[j], raw); hi_seen[j] = max(hi_seen[j], raw)
            jc = calib.get(j, {})
            frac = _frac_from_calib(jc, u)     # same math the collector uses
            if "ref_raw" in jc:
                win = f"ctr={jc['ref_raw']:>4}[{jc.get('lo_off',0):>+5.0f}..{jc.get('hi_off',0):>+5.0f}]"
            else:
                win = f"win[{jc.get('min_raw',0):>4}..{jc.get('max_raw',4095):>4}]"
            clamp = "CLAMP" if (frac < 0.0 or frac > 1.0) else ""
            rows.append(f"  {arm.AIZEE_JOINTS[i]:<12} raw={raw:>4} {win} "
                        f"frac={frac:>+5.2f} swept[{lo_seen[j]:>4}..{hi_seen[j]:>4}] {clamp}")
        sys.stdout.write("\033[H\033[J")
        print(f"port={args.port}  calib={args.calib}")
        print("\n".join(rows))
        time.sleep(0.08)
except KeyboardInterrupt:
    pass
finally:
    arm.close()
