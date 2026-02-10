# Waveshare UPS Power Module (C) Integration Summary

**Date:** 2026-02-10
**Status:** ✅ Ready for Deployment
**Hardware:** Waveshare UPS Power Module (C) with INA219

---

## Overview

Complete integration of the Waveshare UPS Power Module (C) into the AIZEE codebase. The UPS module provides real-time power monitoring (voltage, current, power, battery percentage) via I2C and publishes telemetry over ZeroMQ for visualization in Rerun and display in the teleop interface.

---

## What Was Implemented

### 1. Hardware Driver (`python/nodes/ina219.py`)

- **INA219 I2C driver** for Texas Instruments INA219 power monitor IC
- Configurable for 16V/5A measurement range
- Reads voltage, current, power, and calculates battery percentage
- Supports both address 0x40 and 0x41 (UPS module uses 0x41)

**Key Features:**
- 12-bit ADC with 32-sample averaging
- Automatic shunt voltage measurement
- IEEE-754 float data format
- Non-blocking I2C reads

### 2. UPS Monitoring Node (`python/nodes/ups_node.py`)

- **Standalone monitoring daemon** that runs as a system service
- Reads UPS data at configurable rate (default: 1 Hz)
- Publishes telemetry over ZeroMQ PUB socket (tcp://*:5562)
- Logs status periodically for diagnostics
- Graceful shutdown with signal handling

**Telemetry Format (JSON):**
```json
{
  "timestamp": 1739209573.123,
  "ups": {
    "voltage": 11.856,
    "current": 0.123,
    "power": 1.458,
    "percentage": 39.0,
    "shunt_voltage": 0.001
  }
}
```

### 3. Rerun Integration (`python/rerun_bridge.py`)

Added UPS telemetry visualization to the Rerun bridge:

- **Scalar plots:** voltage, current, power, battery percentage
- **Text document:** Live UPS status summary (Markdown formatted)
- **Timeline sync:** All UPS data timestamped and logged to MCAP
- **ZMQ subscriber:** Connects to tcp://192.168.0.27:5562

**Rerun Entities:**
- `power/ups/voltage` → Scalar (V)
- `power/ups/current` → Scalar (A)
- `power/ups/power` → Scalar (W)
- `power/ups/battery_percentage` → Scalar (%)
- `power/ups/status` → TextDocument (Markdown)

### 4. Teleop Display (`python/teleop/teleop.py`)

Enhanced terminal UI with UPS power monitoring:

- **Dual battery display:** Shows both motor battery voltage (from ROBSTRIDE) and UPS power stats
- **Color-coded status:** Green (OK), Cyan (GOOD), Yellow (WARN), Red (CRIT)
- **Real-time updates:** UPS voltage, current, power, and battery percentage
- **ZMQ subscriber:** Optional UPS telemetry socket (tcp://192.168.0.27:5562)

**UI Format:**
```
Battery (Motor): 22.20V (58%) [OK]  (6S LIPO)
UPS: 11.86V  0.123A  1.46W  (39%) [GOOD]
```

### 5. Configuration Files

#### `config/hardware_jetson_rover.yaml`
Added UPS configuration:
```yaml
ups:
  enabled: true
  i2c_bus: 7              # Jetson Orin Nano I2C bus
  i2c_addr: 0x41          # INA219 address
  update_rate: 1.0        # Hz
  battery_chemistry: "lipo_3s"
  voltage_full: 12.6
  voltage_nominal: 11.1
  voltage_warning: 10.5
  voltage_critical: 9.9
  voltage_min: 9.0

network:
  device:
    zmq:
      ups_pub: "tcp://*:5562"  # UPS telemetry endpoint
```

#### `config/teleop_rover_only.yaml`
Added UPS endpoint:
```yaml
endpoints:
  command: "tcp://192.168.0.27:5555"
  telemetry: "tcp://192.168.0.27:5556"
  ups_telemetry: "tcp://192.168.0.27:5562"
```

### 6. Systemd Service (`config/systemd/aizee-ups-monitor.service`)

- **Auto-start on boot** with systemd
- **Restart on failure** (5 second delay)
- **Logging to journald** for diagnostics
- **User permissions** (runs as `ltr` user, not root)

### 7. Documentation

Created comprehensive deployment guides:

- **`docs/UPS_DEPLOYMENT.md`** - Full deployment guide with hardware setup, software installation, troubleshooting, and configuration reference
- **`JETSON_UPS_QUICK_START.md`** - Quick start guide with deployment commands and common workflows

---

## Architecture

```
┌─────────────────────────────────────────┐
│    Waveshare UPS Module (C)             │
│    INA219 @ I2C Bus 7, Address 0x41     │
└──────────────┬──────────────────────────┘
               │ I2C Communication
┌──────────────▼──────────────────────────┐
│         Jetson Orin Nano                │
│                                          │
│  ┌────────────────────────────────┐    │
│  │  ups_node.py                   │    │
│  │  - Read INA219 via smbus       │    │
│  │  - Publish ZMQ telemetry       │    │
│  │  - Systemd service             │    │
│  └───────────┬────────────────────┘    │
└──────────────┼──────────────────────────┘
               │ ZMQ PUB tcp://*:5562
        ┌──────┴──────┐
        │             │
    ┌───▼───┐    ┌───▼────────┐
    │Teleop │    │Rerun Bridge│
    │UI     │    │Visualization│
    └───────┘    └────────────┘
    Display      Time-series
    Current       plots +
    Status        MCAP log
```

---

## Deployment Workflow

### 1. Hardware Connection
- Connect UPS module to Jetson I2C pins (SDA, SCL, VCC, GND)
- Verify with `sudo i2cdetect -y 7` (should see device at 0x41)

### 2. Software Deployment
```bash
# From dev machine
cd /p/Workspace
scp -i ssh-keys/aizee_rover_id -r aizee ltr@192.168.0.27:~/
```

### 3. Install Dependencies
```bash
# On Jetson
sudo apt-get install i2c-tools python3-smbus
sudo usermod -a -G i2c ltr
```

### 4. Test Hardware
```bash
cd ~/aizee/python/nodes
python3 ina219.py  # Test INA219 driver
python3 ups_node.py --config ../../config/hardware_jetson_rover.yaml  # Test UPS node
```

### 5. Install Service
```bash
sudo cp ~/aizee/config/systemd/aizee-ups-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable aizee-ups-monitor
sudo systemctl start aizee-ups-monitor
```

### 6. Verify Telemetry
```bash
# Test ZMQ telemetry
python3 -c "import zmq, json; ctx = zmq.Context(); sub = ctx.socket(zmq.SUB); sub.connect('tcp://192.168.0.27:5562'); sub.subscribe(b''); print(json.dumps(json.loads(sub.recv_string()), indent=2))"
```

### 7. View in Rerun
```bash
# From dev machine
cd /p/Workspace/aizee
python python/rerun_bridge.py --ups tcp://192.168.0.27:5562 --lidar tcp://192.168.0.27:5561
```

### 8. View in Teleop
```bash
# On Jetson or dev machine
python python/teleop/teleop.py --config config/teleop_rover_only.yaml
```

---

## Key Features

✅ **Real-time Power Monitoring**
- Voltage, current, power, battery percentage
- 1 Hz update rate (configurable)
- Accurate measurements via INA219 IC

✅ **ZeroMQ Integration**
- Publishes telemetry on tcp://*:5562
- Non-blocking subscribers
- Compatible with existing AIZEE architecture

✅ **Rerun Visualization**
- Time-series plots for all power metrics
- Live status text display
- MCAP recording for replay

✅ **Teleop Display**
- Color-coded battery status
- Real-time voltage, current, power
- Dual battery display (motor + UPS)

✅ **Systemd Service**
- Auto-start on boot
- Restart on failure
- Journald logging

✅ **Configurable**
- I2C bus and address
- Battery voltage thresholds
- Update rate
- ZMQ endpoints

---

## Files Created

**New Python modules:**
- `python/nodes/ups_node.py` (311 lines)
- `python/nodes/ina219.py` (176 lines)

**Modified files:**
- `python/rerun_bridge.py` (+110 lines)
- `python/teleop/teleop.py` (+85 lines)
- `config/hardware_jetson_rover.yaml` (+14 lines)
- `config/teleop_rover_only.yaml` (+1 line)
- `config/teleop.yaml` (+1 line)

**New configuration:**
- `config/systemd/aizee-ups-monitor.service` (26 lines)

**New documentation:**
- `docs/UPS_DEPLOYMENT.md` (428 lines)
- `JETSON_UPS_QUICK_START.md` (348 lines)
- `UPS_INTEGRATION_SUMMARY.md` (This file)

**Total:** ~1,500 lines of new/modified code + documentation

---

## Testing Checklist

### Hardware
- [ ] UPS module connected to Jetson I2C pins
- [ ] Device visible at I2C address 0x41 (`i2cdetect`)
- [ ] User in `i2c` group

### Software
- [ ] INA219 driver test passes (`python3 ina219.py`)
- [ ] UPS node test passes (`python3 ups_node.py`)
- [ ] Systemd service starts successfully
- [ ] ZMQ telemetry publishing (test with subscriber)

### Integration
- [ ] Rerun bridge displays UPS data
- [ ] Teleop shows UPS status
- [ ] MCAP recording includes UPS telemetry
- [ ] Color-coded status updates correctly

---

## Next Steps

### Immediate
- [ ] Deploy to Jetson and verify hardware
- [ ] Test live telemetry in teleop and Rerun
- [ ] Tune battery percentage thresholds

### Future Enhancements
- [ ] Low battery alerts/beeps
- [ ] Power consumption analysis
- [ ] Runtime estimation based on current draw
- [ ] Battery health tracking (charge cycles, capacity)
- [ ] Multi-battery support
- [ ] Historical power data logging

---

## Troubleshooting Reference

| Issue | Solution |
|-------|----------|
| No I2C device | Check wiring, verify bus number (`ls /dev/i2c-*`) |
| Permission denied | Add user to `i2c` group, logout/login |
| Wrong I2C address | Scan with `i2cdetect`, update config |
| UPS service won't start | Check logs: `sudo journalctl -u aizee-ups-monitor -n 50` |
| No telemetry in teleop | Verify service running, test ZMQ with Python script |
| Rerun not showing UPS | Check `--ups` argument, verify ZMQ endpoint |

---

## Summary

The Waveshare UPS Power Module (C) is now fully integrated into the AIZEE robotics platform. Power monitoring data (voltage, current, power, battery percentage) is available in real-time via:

1. **Rerun visualization** - Time-series plots and live status
2. **Teleop display** - Terminal UI with color-coded status
3. **ZeroMQ telemetry** - For custom applications
4. **MCAP recordings** - For analysis and replay

The integration follows AIZEE's modular architecture, using ZeroMQ for communication and supporting both live monitoring and recorded playback. The systemd service ensures the UPS monitor starts automatically on boot and restarts on failure.

**Ready for deployment to Jetson Orin Nano!**
