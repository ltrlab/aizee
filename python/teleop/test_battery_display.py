#!/usr/bin/env python3
"""
Test battery voltage display with different voltage levels.
Shows how battery status will appear in teleop UI.
"""

import sys

def test_battery_status(voltage, config):
    """Test battery status calculation and display."""
    bat_cfg = config["battery"]

    # Determine status
    if voltage >= bat_cfg["voltage_nominal"]:
        status = "OK"
    elif voltage >= bat_cfg["voltage_warning"]:
        status = "GOOD"
    elif voltage >= bat_cfg["voltage_critical"]:
        status = "WARN"
    else:
        status = "CRIT"

    # Calculate percentage
    v_range = bat_cfg["voltage_full"] - bat_cfg["voltage_min"]
    v_current = voltage - bat_cfg["voltage_min"]
    percent = max(0, min(100, (v_current / v_range) * 100))

    print(f"  Battery: {voltage:.2f}V ({percent:.0f}%) [{status}]  "
          f"({bat_cfg['cells']}S {bat_cfg['cell_type'].upper()})")

# 6S LiPo config
config = {
    "battery": {
        "cells": 6,
        "cell_type": "lipo",
        "voltage_full": 25.2,
        "voltage_nominal": 22.2,
        "voltage_warning": 21.0,
        "voltage_critical": 20.0,
        "voltage_min": 18.0,
    }
}

print("="*70)
print("Battery Voltage Display Test (6S LiPo)")
print("="*70)
print()

test_cases = [
    (25.2, "Fully charged"),
    (24.0, "Good charge"),
    (22.2, "Nominal (50%)"),
    (21.5, "Getting low"),
    (21.0, "Warning threshold"),
    (20.5, "Warning range"),
    (20.0, "Critical threshold"),
    (19.5, "Critical - land immediately"),
    (18.0, "Minimum safe"),
]

for voltage, description in test_cases:
    print(f"{description}:")
    test_battery_status(voltage, config)
    print()

print("="*70)
print("Status Legend:")
print("  [OK]   - Above nominal (>22.2V)")
print("  [GOOD] - Above warning (>21.0V)")
print("  [WARN] - Above critical (>20.0V) - LAND SOON")
print("  [CRIT] - Below critical (<20.0V) - LAND IMMEDIATELY")
print("="*70)
