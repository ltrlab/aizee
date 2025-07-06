#!/usr/bin/env python3
"""
RobStride04 one-turn round-trip test
===================================

* Assumes:
  – socketCAN `can0` UP at 1 M bit s-¹
  – Motor CAN-ID = 1
* Steps:
  1. enable, set Position mode
  2. record current mechPos as zero
  3. target = zero – 2 π  (one CCW turn)
  4. back to zero
"""

import math, time, can, robstride

CAN_IF   = 'socketcan'
CHANNEL  = 'can0'
BITRATE  = 1_000_000
MOTOR_ID = 1

MECH_POS = 0x7019      # read-only, rad
LOC_REF  = 0x7016      # write, rad

bus = can.Bus(interface=CAN_IF, channel=CHANNEL, bitrate=BITRATE)
rs  = robstride.Client(bus)

def wait(seconds=2.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        time.sleep(0.05)

try:
    print("→ enable")
    rs.enable(MOTOR_ID)

    print("→ Position mode")
    rs.write_param(MOTOR_ID, 'run_mode', robstride.RunMode.Position)

    zero = rs.read_param(MOTOR_ID, MECH_POS)
    print(f"zero set at {zero:.4f} rad")

    target = zero + 2*math.pi
    print(f"→ move to {target:.4f} rad  (-360°)")
    rs.write_param(MOTOR_ID, LOC_REF, target)
    wait(3)

    print("→ back to zero")
    rs.write_param(MOTOR_ID, LOC_REF, zero)
    wait(3)

    final = rs.read_param(MOTOR_ID, MECH_POS)
    print(f"final position {final:.4f} rad")

finally:
    print("→ disable and shutdown")
    try:
        rs.disable(MOTOR_ID)
    except Exception:
        pass
    bus.shutdown()
