#!/usr/bin/env python3
"""
detect_servo_directions.py  –  interactively discover sign (+ / -)
for each Feetech servo axis.

Usage
-----
python3 detect_servo_directions.py               # defaults
python3 detect_servo_directions.py /dev/ttyUSB1 2000000
"""

import sys, time
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ------------- edit once ----------------------------------------------------
JOINT_MAP = {
     1: "right_shoulder_pitch_joint",
     2: "right_upper_arm_joint",
     3: "right_upper_elbow_joint",
     4: "right_lower_elbow_joint",
     5: "right_forearm_joint",
     6: "right_wrist_pitch_joint",
     7: "right_wrist_tool_joint",
    11: "left_shoulder_pitch_joint",
    12: "left_upper_arm_joint",
    13: "left_upper_elbow_joint",
    14: "left_lower_elbow_joint",
    15: "left_forearm_joint",
    16: "left_wrist_pitch_joint",
    17: "left_wrist_tool_joint",
}
SCAN_IDS = JOINT_MAP.keys()
PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000
ADDR_PRESENT_POSITION = 56        # 2 bytes, protocol-2

# ---------------------------------------------------------------------------
port = PortHandler(PORT)
if not port.openPort() or not port.setBaudRate(BAUD):
    sys.exit(f"Cannot open {PORT} @ {BAUD}")

pkt = PacketHandler(0)
direction = {}

print("\nServo direction wizard\n"
      "For each joint I’ll ask you to move it a little in the *positive* "
      "direction (as defined in the robot model) and hit <Enter>.\n"
      "Press Ctrl-C at any time to quit.\n")

try:
    for sid in SCAN_IDS:
        name = JOINT_MAP.get(sid, f"servo_{sid}")
        inp   = input(f">>> {name}: make sure it is stationary and press Enter ")
        pulse0, res, err = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS or err:
            print("   ⚠️  read failed, skipping")
            continue

        input(f"   Now rotate {name} a *few degrees POSITIVE* and hit Enter ")
        time.sleep(0.1)
        pulse1, res, err = pkt.read2ByteTxRx(port, sid, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS or err:
            print("   ⚠️  read failed, skipping")
            continue

        direction[name] =  1 if pulse1 > pulse0 else -1
        sign = "POSITIVE pulse increases" if direction[name] == 1 else "POSITIVE pulse decreases"
        print(f"   ✅ {sign}")

except KeyboardInterrupt:
    print("\nInterrupted – printing what we have.")

finally:
    port.closePort()

# -------- result -----------------------------------------------------------
print("\nCopy this into your URDF / YAML:")
for name, sgn in direction.items():
    print(f"  - {{ name: {name}, direction: {sgn} }}")

