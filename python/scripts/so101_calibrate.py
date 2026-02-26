#!/usr/bin/env python3
"""so101_calibrate.py — SO-101 position monitor and calibration wizard.

Phase 1  MONITOR  Shows live servo positions (ticks + radians).
                  Press C to advance to calibration, Ctrl-C to quit.

Phase 2  CALIBRATE  Guided per-joint capture of MIN and MAX positions.
                    Move the joint, press SPACE to capture, then repeat.

Output:  config/so101_calibration.json

Usage:
    python so101_calibrate.py --port /dev/ttyACM0
    python so101_calibrate.py --port COM4
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent.parent / "teleop"))
from so101_leader import So101Leader, ticks_to_rad, AIZEE_DEFAULTS, CALIB_PATH

sys.path.insert(0, str(Path(__file__).parent))
from record_replay import setup_keyboard

# ---------------------------------------------------------------------------
# ANSI helpers
# ---------------------------------------------------------------------------

def _ansi_on() -> None:
    if sys.platform == "win32":
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.SetConsoleMode(k32.GetStdHandle(-11), 7)


_W = 60


def _bar(ticks: int, width: int = 20) -> str:
    """ASCII progress bar showing position within 0-4095."""
    filled = int(ticks / 4095 * width)
    return "[" + "#" * filled + "-" * (width - filled) + "]"


# ---------------------------------------------------------------------------
# Phase 1 — live monitor
# ---------------------------------------------------------------------------

_MON_LINES = 12


def _render_monitor(
    raw: Optional[dict[str, int]],
    joints: list[str],
    port: str,
) -> list[str]:
    lines = [
        "=" * _W,
        f"  SO-101 Monitor          port: {port}",
        "=" * _W,
        f"  {'joint':<16} {'ticks':>6}  {'radians':>8}  bar",
        "  " + "-" * (_W - 2),
    ]
    for joint in joints:
        if raw:
            t = raw[joint]
            r = ticks_to_rad(t)
            lines.append(f"  {joint:<16} {t:>6}   {r:>+7.3f}  {_bar(t, 16)}")
        else:
            lines.append(f"  {joint:<16}    ---      ---")
    lines += [
        "  " + "-" * (_W - 2),
        "  C = start calibration    Ctrl-C = quit",
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


def run_monitor(arm: So101Leader, get_key) -> bool:
    """Show live positions until user presses C.  Returns True to proceed."""
    lines = _render_monitor(None, arm.JOINTS, arm.port)
    n = _draw(lines, first=True)

    while True:
        key = get_key()
        if key == "C":
            return True

        raw = arm.read_raw()
        lines = _render_monitor(raw, arm.JOINTS, arm.port)
        n = _draw(lines, n_prev=n)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Phase 2 — calibration wizard
# ---------------------------------------------------------------------------

def run_calibration(arm: So101Leader, get_key) -> Optional[dict]:
    """Walk through each joint collecting min/max raw ticks.

    For each joint:
      Step A: user moves joint to MIN, presses SPACE
      Step B: user moves joint to MAX, presses SPACE

    Returns calibration dict or None if aborted.
    """
    n_joints = len(arm.JOINTS)
    results: dict[str, dict] = {}

    # Clear display area from monitor phase
    print("\n")

    for idx, joint in enumerate(arm.JOINTS):
        aizee_joint = arm.AIZEE_JOINTS[idx]
        rad_min_default, rad_max_default = AIZEE_DEFAULTS[idx]

        for step, label in enumerate(("MINIMUM", "MAXIMUM")):
            _print_calib_header(idx, n_joints, joint, aizee_joint, step, results)
            print(f"  >> Move  {joint}  to its {label} position, then press SPACE.")
            print(f"     (Ctrl-C to abort)\n")

            captured: Optional[int] = None
            last_n = 0
            while captured is None:
                key = get_key()
                if key == " ":
                    raw = arm.read_raw()
                    if raw:
                        captured = raw[joint]
                        break

                raw = arm.read_raw()
                cur = raw[joint] if raw else None
                cur_r = f"{ticks_to_rad(cur):+.3f}" if cur is not None else "---"
                line = f"\r  current: ticks={cur or '---':>5}  rad={cur_r}   {_bar(cur or 0, 20)}\033[K"
                sys.stdout.write(line)
                sys.stdout.flush()
                time.sleep(0.05)

            sys.stdout.write("\r\033[K")
            print(f"  Captured {label}: ticks={captured}  rad={ticks_to_rad(captured):+.3f}\n")

            if step == 0:
                results.setdefault(joint, {})["min_raw"] = captured
            else:
                results[joint]["max_raw"] = captured

        # Set AIZEE target range for this joint (use defaults)
        results[joint]["rad_min"] = rad_min_default
        results[joint]["rad_max"] = rad_max_default

    return results


def _print_calib_header(
    idx: int,
    total: int,
    joint: str,
    aizee_joint: str,
    step: int,
    done: dict,
) -> None:
    print("=" * _W)
    print(f"  CALIBRATION  [{idx + 1}/{total}]  {joint}  ->  {aizee_joint}")
    print("=" * _W)
    # Show progress for completed joints
    for j, data in done.items():
        mn = data.get("min_raw", "?")
        mx = data.get("max_raw", "?")
        print(f"  [done] {j:<18}  min={mn}  max={mx}")
    if idx > 0:
        print()
    step_str = "A: move to MIN" if step == 0 else "B: move to MAX"
    print(f"  Step {step_str}")
    print()


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------

def save_calibration(results: dict, joints: list[str], aizee_joints: list[str], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    calib = {
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "baud": 1_000_000,
        "joints": {},
    }
    for i, (joint, aizee_joint) in enumerate(zip(joints, aizee_joints)):
        r = results.get(joint, {})
        calib["joints"][joint] = {
            "id":        i + 1,
            "aizee":     aizee_joint,
            "min_raw":   r.get("min_raw", 0),
            "max_raw":   r.get("max_raw", 4095),
            "rad_min":   r.get("rad_min", AIZEE_DEFAULTS[i][0]),
            "rad_max":   r.get("rad_max", AIZEE_DEFAULTS[i][1]),
        }
    with open(path, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"\nSaved -> {path}")
    print(json.dumps(calib, indent=2))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(
        description="SO-101 position monitor and calibration wizard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port",  required=True, help="Serial port, e.g. /dev/ttyACM0 or COM4")
    ap.add_argument("--baud",  type=int, default=1_000_000)
    ap.add_argument("--output", default=str(CALIB_PATH), help="Output JSON path")
    args = ap.parse_args()

    _ansi_on()

    arm = So101Leader(args.port, args.baud, calib=args.output)
    if not arm.connect():
        sys.exit(1)

    get_key = setup_keyboard()

    try:
        # Phase 1 — monitor
        print(f"Connected to SO-101 on {args.port} at {args.baud} baud\n")
        proceed = run_monitor(arm, get_key)
        if not proceed:
            return

        # Phase 2 — calibration
        print("\nStarting calibration wizard...\n")
        time.sleep(0.5)

        results = run_calibration(arm, get_key)
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
