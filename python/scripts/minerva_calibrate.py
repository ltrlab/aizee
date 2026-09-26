#!/usr/bin/env python3
"""Compatibility entry point for shared stopped-only Minerva calibration.

Default: live CAD preview with local flip/align/accept commands.
Use scripts/calibrate_minerva.ps1 -Mode Directions for the guided sequence.
The old legacy enable/zero-impedance min/max workflow is retired.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from robot_platform.control.joint_calibration import main

if __name__ == "__main__":
    raise SystemExit(main([
        "--profile", str(ROOT / "config/devices/minerva-joint-calibration.json"),
        "--output", str(ROOT / "calibration_runs/minerva-production-01-directions"),
        "--resume", "--live", "--seed",
        str(ROOT / "docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json"),
        *sys.argv[1:],
    ]))
