#!/usr/bin/env python3
"""openrb_calibrate.py — OpenRB-150 + XL330 leader-arm calibration wizard.

Same two-phase wizard as so101_calibrate.py (live monitor, then per-joint
min/max capture), but talks to the OpenRB-150 USB-CDC bridge instead of the
Feetech bus.  Output: config/openrb_calibration.json (same schema as the
SO-101 calibration so downstream code is interchangeable).

Usage:
    python python/scripts/openrb_calibrate.py                       # auto-detect port, all joints
    python python/scripts/openrb_calibrate.py --port COM5
    python python/scripts/openrb_calibrate.py --joints swivel,wrist_pitch
        # only re-capture those joints; existing min_raw/max_raw on the
        # others is preserved.  Direction and zero_offset are always
        # preserved across re-runs regardless of --joints.

Run python/scripts/openrb_setup_arm.py first if your servos still have
factory IDs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from openrb_leader import OpenRBLeader, find_openrb_port, CALIB_PATH

sys.path.insert(0, str(Path(__file__).parent))
# Reuse the SO-101 wizard verbatim — it operates on the duck-typed leader
# interface (arm.JOINTS / arm.AIZEE_JOINTS / arm.read_unwrapped) so it works
# unchanged with OpenRBLeader.
from so101_calibrate import (
    run_monitor, run_calibration, save_calibration, _ansi_on,
)
from record_replay import setup_keyboard


def main() -> None:
    ap = argparse.ArgumentParser(
        description="OpenRB-150 leader-arm position monitor and calibration wizard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",   default=None, help="OpenRB-150 serial port (auto-detected if omitted)")
    ap.add_argument("--baud",   type=int, default=1_000_000)
    ap.add_argument("--output", default=str(CALIB_PATH), help="Output JSON path")
    ap.add_argument(
        "--joints",
        default=None,
        help=("Comma-separated list of joints to (re)capture; all others are "
              "skipped and their existing min_raw/max_raw are preserved. "
              "Accepts OpenRB names (shoulder_pan, shoulder_lift, elbow_flex, "
              "wrist_flex, wrist_yaw, wrist_roll, gripper) or AIZEE names "
              "(swivel, gantry_base, gantry_mid, gantry_end, wrist_pitch, "
              "wrist_roll, gripper).  Example: --joints swivel,wrist_pitch"),
    )
    args = ap.parse_args()
    only_joints = (
        {j.strip() for j in args.joints.split(",") if j.strip()}
        if args.joints else None
    )

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
        # Phase 1 — monitor
        print(f"Connected to OpenRB-150 on {port} at {args.baud} baud\n")
        if not run_monitor(arm, get_key):
            return

        # Phase 2 — calibration
        print("\nStarting calibration wizard...\n")
        import time as _time
        _time.sleep(0.5)

        results = run_calibration(arm, get_key, only_joints=only_joints)
        if results is None:
            print("Calibration aborted.")
            return

        save_calibration(results, arm.JOINTS, arm.AIZEE_JOINTS, Path(args.output))

    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
