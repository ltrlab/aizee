#!/usr/bin/env python3
"""
Read Feetech STS/SMS servo positions once and print them.

Usage
-----
$ python3 read_servo_positions.py               # defaults
$ python3 read_servo_positions.py /dev/ttyUSB1 2000000 5:12

Arguments
---------
1. serial port path          (default: /dev/ttyUSB0)
2. baud rate                 (default: 1_000_000)
3. scan range  start:end     (default: 1:16)
"""

import math, sys
from typing import Dict, List

# ───── scservo-sdk -----------------------------------------------------------
try:
    from scservo_sdk import (
        PortHandler, PacketHandler,
        COMM_SUCCESS
    )
except ImportError:
    sys.exit("Install SDK first:  pip install feetech-servo-sdk")

# Feetech control-table addresses (Protocol-2 SMS/STS)
ADDR_PRESENT_POSITION = 56   # 2 bytes

# Helpers: pulse ↔ rad
_RAD2DEG = 180.0 / math.pi
_DEG2RAD = math.pi / 180.0
_PULSE_MAX, _RANGE_DEG = 4095, 240.0
_SCALE = _PULSE_MAX / _RANGE_DEG

def pulse_to_rad(pulse: int) -> float:
    deg = (pulse / _SCALE) - _RANGE_DEG / 2
    return deg * _DEG2RAD

# ───── main ------------------------------------------------------------------
def main():
    # -------- CLI args -------------------------------------------------------
    port_path  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud       = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
    start, end = (sys.argv[3] if len(sys.argv) > 3 else "1:17").split(":")
    start, end = int(start), int(end)

    # -------- open port ------------------------------------------------------
    port = PortHandler(port_path)
    if not port.openPort():
        sys.exit(f"Cannot open {port_path}")
    if not port.setBaudRate(baud):
        sys.exit(f"Cannot set {baud=} on {port_path}")
    pkt  = PacketHandler(0)         # protocol 2.0
    print(f"Opened {port_path} @ {baud} baud")

    # -------- scan & read ----------------------------------------------------
    print(f"\nScanning IDs {start}–{end} …")
    table: List[Dict[str, float]] = []
    for sid in range(start, end + 1):
        pulse, res, err = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        if res == COMM_SUCCESS and err == 0:
            table.append({"id": sid,
                          "pulse": pulse,
                          "rad": pulse_to_rad(pulse)})
    if not table:
        print("No servos replied"); port.closePort(); return

    # -------- pretty print ---------------------------------------------------
    print("\nID   Pulse   Position(rad)   Position(deg)")
    print("--   -----   -------------   -------------")
    for row in table:
        deg = row["rad"] * _RAD2DEG
        print(f"{row['id']:>2}   {row['pulse']:>5}      {row['rad']:>8.3f}        {deg:8.1f}")
    print()

    port.closePort()

if __name__ == "__main__":
    main()
