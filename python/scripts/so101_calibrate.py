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
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from common.arm_constants import setup_keyboard

ROBSTRIDE_CALIB_PATH = Path(__file__).parent.parent.parent / "config" / "robstride_calibration.json"

# ---------------------------------------------------------------------------
# AIZEE limits
# ---------------------------------------------------------------------------

def _load_aizee_limits(path: Path = ROBSTRIDE_CALIB_PATH) -> dict[str, float]:
    """Return {aizee_joint: span_rad} from robstride_calibration.json, or {} if absent."""
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text())
        result = {}
        for joint, jd in data.get("joints", {}).items():
            mn = float(jd.get("min_rad", 0.0))
            mx = float(jd.get("max_rad", 0.0))
            result[joint] = abs(mx - mn)
        return result
    except Exception:
        return {}


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
    unwrapped: Optional[dict[str, int]],
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
        if raw and unwrapped:
            t = raw[joint]                  # physical 0-4095 for bar and ticks column
            r = ticks_to_rad(unwrapped[joint])  # continuous unwrapped for radians
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
    """Show live positions until user presses C.  Returns True to proceed.

    Uses read_unwrapped() so the unwrap state is seeded before calibration
    starts — critical for joints that cross the 0/4095 boundary.
    """
    lines = _render_monitor(None, None, arm.JOINTS, arm.port)
    n = _draw(lines, first=True)

    while True:
        key = get_key()
        if key == "C":
            return True

        unwrapped = arm.read_unwrapped()
        # ticks column and bar use physical 0-4095; radians use unwrapped for continuity
        display = {j: v % 4096 for j, v in unwrapped.items()} if unwrapped else None
        lines = _render_monitor(display, unwrapped, arm.JOINTS, arm.port)
        n = _draw(lines, n_prev=n)
        time.sleep(0.05)


# ---------------------------------------------------------------------------
# Phase 2 — calibration wizard
# ---------------------------------------------------------------------------

def run_calibration(
    arm: So101Leader,
    get_key,
    only_joints: Optional[set] = None,
) -> Optional[dict]:
    """Walk through each joint collecting min/max raw ticks.

    For each joint:
      Step A: user moves joint to MIN, presses SPACE
      Step B: user moves joint to MAX, presses SPACE

    `only_joints` (optional) is a set of joint names to capture; any joint
    not in the set is skipped and `save_calibration` will preserve its
    existing min_raw/max_raw from the on-disk JSON.

    Returns calibration dict or None if aborted.
    """
    n_joints = len(arm.JOINTS)
    results: dict[str, dict] = {}

    # Clear display area from monitor phase
    print("\n")

    for idx, joint in enumerate(arm.JOINTS):
        aizee_joint = arm.AIZEE_JOINTS[idx]
        rad_min_default, rad_max_default = AIZEE_DEFAULTS[idx]

        if only_joints is not None and joint not in only_joints and aizee_joint not in only_joints:
            print(f"  [skip] {joint:<18}  ->  {aizee_joint}  (not in --joints filter)")
            continue

        for step, label in enumerate(("MINIMUM", "MAXIMUM")):
            _print_calib_header(idx, n_joints, joint, aizee_joint, step, results)
            print(f"  >> Move  {joint}  to its {label} position, then press SPACE.")
            print(f"     (Ctrl-C to abort)\n")

            captured: Optional[int] = None
            while captured is None:
                key = get_key()
                if key == " ":
                    unwrapped = arm.read_unwrapped()
                    if unwrapped:
                        captured = unwrapped[joint] % 4096  # store physical 0-4095
                        break

                unwrapped = arm.read_unwrapped()
                if unwrapped:
                    cur_u = unwrapped[joint]          # continuous unwrapped
                    cur_p = cur_u % 4096              # physical 0-4095 for bar
                    cur_r = ticks_to_rad(cur_u)       # continuous radians for display
                    wrap_note = f"  (wraps)" if cur_u != cur_p else ""
                    line = (f"\r  current: ticks={cur_p:>5}  rad={cur_r:>+7.3f}"
                            f"  {_bar(cur_p, 18)}{wrap_note}\033[K")
                else:
                    line = "\r  current: ---\033[K"
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
        if isinstance(mn, int) and isinstance(mx, int) and mn > mx:
            note = "  [wrap]" if (mn - mx) > 2048 else "  [inverted]"
        else:
            note = ""
        print(f"  [done] {j:<18}  min={mn}  max={mx}{note}")
    if idx > 0:
        print()
    step_str = "A: move to MIN" if step == 0 else "B: move to MAX"
    print(f"  Step {step_str}")
    print()


# ---------------------------------------------------------------------------
# Save
# ---------------------------------------------------------------------------

def save_calibration(results: dict, joints: list[str], aizee_joints: list[str], path: Path) -> None:
    # Load existing calibration to preserve manually-set fields (direction, zero_offset).
    existing: dict = {}
    if path.exists():
        try:
            import json as _json
            existing = _json.loads(path.read_text()).get("joints", {})
        except Exception:
            pass

    # Load AIZEE physical spans so SO-101 full range maps to AIZEE full range.
    aizee_spans = _load_aizee_limits()

    path.parent.mkdir(parents=True, exist_ok=True)
    calib = {
        "calibrated_at": datetime.now(timezone.utc).isoformat(),
        "baud": 1_000_000,
        "joints": {},
    }
    for i, (joint, aizee_joint) in enumerate(zip(joints, aizee_joints)):
        r   = results.get(joint, {})
        old = existing.get(joint, {})
        if aizee_joint in aizee_spans:
            # Map SO-101 full physical range → [0, aizee_span].
            # zero_offset must be re-set with M key after recalibration.
            rad_min     = 0.0
            rad_max     = round(aizee_spans[aizee_joint], 4)
            zero_offset = 0.0
        else:
            rad_min     = r.get("rad_min", AIZEE_DEFAULTS[i][0])
            rad_max     = r.get("rad_max", AIZEE_DEFAULTS[i][1])
            zero_offset = old.get("zero_offset", 0.0)
        # If this joint wasn't re-captured this run (e.g. --joints filter
        # excluded it), preserve the existing min_raw/max_raw rather than
        # falling back to (0, 4095) which would wipe the prior calibration.
        calib["joints"][joint] = {
            "id":          i + 1,
            "aizee":       aizee_joint,
            "min_raw":     r.get("min_raw", old.get("min_raw", 0)),
            "max_raw":     r.get("max_raw", old.get("max_raw", 4095)),
            "rad_min":     rad_min,
            "rad_max":     rad_max,
            "zero_offset": zero_offset,
            "direction":   old.get("direction", 1),
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
