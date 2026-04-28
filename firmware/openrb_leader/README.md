# OpenRB-150 Leader Arm Firmware

Reads 7× Dynamixel XL330-M077-T servos and exposes them to the host over
USB-CDC for use as the AIZEE leader arm. See
[../../docs/subsystems/OPENRB_LEADER.md](../../docs/subsystems/OPENRB_LEADER.md)
for full documentation including wire protocol and setup procedure.

## Build / upload

```bash
pip install --user platformio
pio run -e openrb150 -t upload --upload-port COM4 -d firmware/openrb_leader
```

The `openrb150_bsp/` folder vendors the Robotis OpenRB-150 SAMD Arduino
core (LGPL-2.1) so the build is self-contained — no Arduino IDE required.

## Layout

```
firmware/openrb_leader/
├── platformio.ini              # PlatformIO build config (overrides framework with the vendored BSP)
├── boards/openrb150.json       # Custom PIO board definition for OpenRB-150
├── src/main.cpp                # Sketch — wire protocol, sync-read, setup commands
├── openrb150_bsp/              # Vendored Robotis OpenRB-150 Arduino SAMD core
└── README.md                   # This file
```

## Host-side scripts

| Script | Purpose |
|---|---|
| `python/scripts/openrb_setup_arm.py` | First-time servo ID assignment (1..7) |
| `python/scripts/openrb_calibrate.py` | Per-joint min/max calibration |
| `python/scripts/leader_monitor.py`   | Live position + limits monitor (also drives `C` key for sequential centering) |
