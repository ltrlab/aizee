# OpenRB-150 Leader Arm Firmware

Reads 7 Dynamixel XL330-M077-T servos and exposes them to the host over USB-CDC. It is the firmware of Minerva's two
GELLO leader arms and of AIZEE's optional OpenRB leader. See
[../../docs/robots/OPENRB_LEADER.md](../../docs/robots/OPENRB_LEADER.md) for the wire protocol, the firmware history
and force feedback, and
[../../docs/getting-started/INSTALL_OPERATOR_STATION.md](../../docs/getting-started/INSTALL_OPERATOR_STATION.md#3-the-leaders)
for setting a leader up on the operator station.

Two hosts use it:

- **Minerva:** the guarded shared input driver, `src/robot_platform/teleop/openrb.py`. It uses the 50-byte `0xA6`
  freshness reply (full-read counter, sample and reply times, validity), and pins each board's USB serial through a
  device profile (`config/devices/gello-dual-viewer.json`). Only a complete, error-free seven-servo read refreshes the
  source counter and time. Startup disables responding servos; reading inputs never enables them. Robot joint mapping
  is configured separately from this firmware.
- **AIZEE:** the legacy driver `python/teleop/openrb_leader.py`, which uses the 37-byte `0xA5` reply.

The firmware version is `LEADER_FIRMWARE_VERSION` in `src/main.cpp` (3). Hosts need firmware 2 or later for the
freshness guarantee; `python scripts/openrb_leader_setup.py info` prints a board's version.

## Build / upload

```bash
pip install --user platformio
pio run -e openrb150 -t upload --upload-port COM4 -d firmware/openrb_leader
```

Use the board's own port in place of `COM4`. The `openrb150_bsp/` folder vendors the Robotis OpenRB-150 SAMD Arduino
core (LGPL-2.1) so the build is self-contained, with no Arduino IDE required.

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
| `scripts/openrb_leader_setup.py` | Minerva and any board running firmware 2+: `info` (board and servo diagnostics, reset cause) and `return-delay` (the servos' Return Delay Time), opening boards by their pinned USB serial |
| `python/scripts/openrb_setup_arm.py` | First-time servo ID assignment (1..7) |
| `python/scripts/openrb_calibrate.py` | Per-joint min/max calibration (AIZEE; Minerva's ranges are swept in Helm's Leaders tab) |
| `python/scripts/leader_monitor.py`   | Live position + limits monitor (also drives `C` key for sequential centering) |
| `scripts/probe_leader_timing.py`, `scripts/probe_leader_reopen.py` | Timing and USB-reopen probes for the bench |
