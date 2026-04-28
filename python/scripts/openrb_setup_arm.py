#!/usr/bin/env python3
"""openrb_setup_arm.py — first-time servo ID assignment for the OpenRB-150 leader arm.

Walks you through plugging in each Dynamixel XL330-M077-T servo one at a time
and assigns IDs 1..7 in the same joint order the SO-101 leader uses, so the
new arm is a drop-in replacement for the existing calibration / pipeline.

The script talks to the OpenRB-150 firmware (firmware/openrb_leader/) via the
SCAN and REID commands.  For each joint it:

  1. Scans the bus to confirm exactly one servo is currently connected.
  2. Sends REID with the target ID (1..7).  The firmware sweeps every
     supported baud rate, finds the single responder, sets its ID and
     bumps it to 1 Mbps.
  3. Verifies the assignment with a follow-up scan.

Usage:
    python python/scripts/openrb_setup_arm.py            # auto-detect port
    python python/scripts/openrb_setup_arm.py --port COM5
    python python/scripts/openrb_setup_arm.py --start-at 3   # resume mid-arm

Hardware steps (printed by the script, repeated here for reference):
  - Power the OpenRB-150 from USB only at first; servos draw their bus power
    from the OpenRB's TTL header.
  - For each prompt, plug in EXACTLY one servo to the OpenRB Dynamixel bus,
    confirm, then unplug it before moving on.
  - At the end, plug all 7 in together and the script verifies the layout.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError:
    print("pyserial not installed — run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from openrb_leader import (
    OpenRBLeader, find_openrb_port,
    bus_scan, reassign_id,
    BAUD_CODES, REID_OK, REID_NOT_FOUND, REID_AMBIGUOUS, REID_STATUS_NAMES,
    _BAUD as USB_BAUD,
)


# Joint order — IDs 1..7 must match So101Leader.JOINTS.
JOINTS = OpenRBLeader.JOINTS
AIZEE  = OpenRBLeader.AIZEE_JOINTS


def _open_port(port: str) -> serial.Serial:
    ser = serial.Serial(port, USB_BAUD, timeout=0.5)
    # OpenRB-150 USB-CDC resets the MCU on open.
    time.sleep(0.6)
    ser.reset_input_buffer()
    return ser


def _format_scan(scan: list[tuple[int, int]]) -> str:
    if not scan:
        return "(empty bus)"
    parts = []
    for sid, code in scan:
        baud = BAUD_CODES.get(code, code)
        parts.append(f"ID={sid}@{baud}")
    return ", ".join(parts)


def _wait_for_single_servo(ser, prompt: str, allow_skip: bool = False) -> Optional[list[tuple[int, int]]]:
    """Scan repeatedly until the bus has exactly one responder, or user skips."""
    print(prompt)
    print("  (press ENTER to scan, S to skip, Q to abort)")
    while True:
        try:
            key = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if key == "q":
            return None
        if allow_skip and key == "s":
            return []
        scan = bus_scan(ser)
        if scan is None:
            print("  scan failed — is the OpenRB-150 connected?")
            continue
        print(f"  bus: {_format_scan(scan)}")
        if len(scan) == 1:
            return scan
        if len(scan) == 0:
            print("  no servo detected — check the cable and power.")
        else:
            print(f"  {len(scan)} servos on the bus — unplug all but one.")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Assign Dynamixel XL330 servo IDs 1..7 in SO-101 joint order",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",     default=None, help="OpenRB-150 USB-CDC port (auto-detected if omitted)")
    ap.add_argument("--start-at", type=int, default=1,
                    help="Joint ID to start at (1..7).  Use to resume mid-setup.")
    ap.add_argument("--no-verify", action="store_true",
                    help="Skip the final all-7-plugged-in verification scan.")
    args = ap.parse_args()

    if args.start_at < 1 or args.start_at > len(JOINTS):
        ap.error(f"--start-at must be 1..{len(JOINTS)}")

    # Locate the OpenRB-150.
    port = args.port
    if port is None:
        print("Searching for OpenRB-150...")
        port = find_openrb_port(verbose=True)
        if port is None:
            print("OpenRB-150 not found — pass --port explicitly.", file=sys.stderr)
            sys.exit(1)
        print(f"Found OpenRB-150 on {port}")
    ser = _open_port(port)

    print()
    print("=" * 60)
    print("  OpenRB-150 leader arm — servo ID setup")
    print("=" * 60)
    print()
    print("  Plug in ONE servo at a time when prompted.")
    print("  Each servo will be assigned its joint ID at 1 Mbps.")
    print()

    # Start with a clean-bus check unless resuming.
    if args.start_at == 1:
        print("Step 0: confirm the bus is empty.")
        print("  Disconnect ALL servos from the OpenRB-150 now.")
        print("  Press ENTER once nothing is plugged into the Dynamixel bus.")
        try:
            input("> ")
        except (EOFError, KeyboardInterrupt):
            print("Aborted.")
            return
        scan = bus_scan(ser)
        if scan is None:
            print("Initial scan failed — aborting.", file=sys.stderr)
            sys.exit(1)
        if scan:
            print(f"WARNING: bus is not empty: {_format_scan(scan)}")
            print("Continuing anyway, but expect AMBIGUOUS errors below.")

    # Per-joint assignment.
    skipped: list[int] = []
    for idx in range(args.start_at - 1, len(JOINTS)):
        target_id  = idx + 1
        joint_name = JOINTS[idx]
        aizee_name = AIZEE[idx]
        print()
        print("-" * 60)
        print(f"  Step {target_id}: assign servo for")
        print(f"    {joint_name:<14}  ->  {aizee_name}    (ID={target_id})")
        print("-" * 60)
        prompt = f"  Plug in the servo for '{joint_name}' now."
        scan = _wait_for_single_servo(ser, prompt, allow_skip=True)
        if scan is None:
            print("Aborted.")
            return
        if scan == []:
            print(f"  Skipped joint {joint_name} (ID={target_id}).")
            skipped.append(target_id)
            continue

        sid_old, code_old = scan[0]
        baud_old = BAUD_CODES.get(code_old, "?")
        print(f"  Found servo: ID={sid_old} @ {baud_old} baud  ->  reassigning to ID={target_id} @ 1Mbps")

        status, found_id, found_code = reassign_id(ser, target_id)
        if status == REID_OK:
            print(f"  OK — servo is now ID={target_id} at 1Mbps")
        else:
            print(f"  FAILED: {REID_STATUS_NAMES.get(status, hex(status))}")
            print("  You can re-run with --start-at to retry this joint.")
            print("  Aborting.")
            sys.exit(1)

        print("  Now UNPLUG this servo before plugging in the next one.")
        print("  Press ENTER once unplugged.")
        try:
            input("> ")
        except (EOFError, KeyboardInterrupt):
            print("Aborted.")
            return

    # Verification: plug all 7 in together and confirm the bus.
    if args.no_verify:
        print("\nDone (verification skipped).")
        return

    print()
    print("=" * 60)
    print("  Verification")
    print("=" * 60)
    print()
    print("  Plug ALL 7 servos in together now.")
    print("  Press ENTER to scan.")
    try:
        input("> ")
    except (EOFError, KeyboardInterrupt):
        print("Aborted before verification.")
        return

    scan = bus_scan(ser)
    if scan is None:
        print("Verification scan failed.", file=sys.stderr)
        sys.exit(1)

    found_ids = sorted(sid for sid, _ in scan)
    expected  = sorted(set(range(1, len(JOINTS) + 1)) - set(skipped))
    print(f"  bus: {_format_scan(scan)}")
    print(f"  expected IDs: {expected}")
    print(f"  found IDs   : {found_ids}")

    missing  = [i for i in expected if i not in found_ids]
    extra    = [i for i in found_ids if i not in expected]
    wrong_baud = [(sid, BAUD_CODES.get(code, code)) for sid, code in scan if code != 0]

    ok = not missing and not extra and not wrong_baud
    print()
    if ok:
        print("ALL GOOD — every expected servo answered at 1 Mbps with the correct ID.")
        print("You can now run the calibration wizard:")
        print("  python python/scripts/openrb_calibrate.py")
    else:
        if missing:
            print(f"  MISSING IDs: {missing}")
        if extra:
            print(f"  UNEXPECTED IDs: {extra}")
        if wrong_baud:
            print(f"  WRONG BAUD: {wrong_baud}  (should all be 1Mbps)")
        if skipped:
            print(f"  (joints you skipped: {skipped})")
        sys.exit(1)


if __name__ == "__main__":
    main()
