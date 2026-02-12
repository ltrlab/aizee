# Jetson UPS Power Module - Quick Start Guide

**Hardware:** Waveshare UPS Power Module (C)
**Last Updated:** 2026-02-10
**Jetson IP:** 192.168.0.27

---

## Quick Deployment

### 1. Deploy Files to Jetson

From your development machine (`P:/Workspace`):

```bash
# Deploy entire aizee directory
scp -i ssh-keys/aizee_rover_id -r aizee ltr@192.168.0.27:~/

# Or deploy only UPS files
scp -i ssh-keys/aizee_rover_id aizee/python/nodes/ups_node.py ltr@192.168.0.27:~/aizee/python/nodes/
scp -i ssh-keys/aizee_rover_id aizee/python/nodes/ina219.py ltr@192.168.0.27:~/aizee/python/nodes/
scp -i ssh-keys/aizee_rover_id aizee/config/hardware_jetson_rover.yaml ltr@192.168.0.27:~/aizee/config/
scp -i ssh-keys/aizee_rover_id aizee/config/systemd/aizee-ups-monitor.service ltr@192.168.0.27:~/aizee/config/systemd/
scp -i ssh-keys/aizee_rover_id aizee/python/teleop/teleop.py ltr@192.168.0.27:~/aizee/python/teleop/
scp -i ssh-keys/aizee_rover_id aizee/config/teleop_rover_only.yaml ltr@192.168.0.27:~/aizee/config/
```

### 2. SSH into Jetson

```bash
ssh -i ssh-keys/aizee_rover_id ltr@192.168.0.27
```

### 3. Install Dependencies

```bash
# Install I2C tools and Python SMBus library
sudo apt-get update
sudo apt-get install i2c-tools python3-smbus

# Add user to i2c group
sudo usermod -a -G i2c ltr

# Logout and login for group changes
exit
# SSH back in
```

### 4. Verify Hardware Connection

```bash
# Scan I2C bus 7 for devices
sudo i2cdetect -y 7

# You should see device at address 0x41:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 40: -- 41 -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

If you see `UU` instead of `41`, the device is already in use (which is fine if UPS monitor is running).

If you don't see anything:
- Check I2C wiring (SDA, SCL, VCC, GND)
- Try a different I2C bus: `ls /dev/i2c-*` and scan each one
- Verify UPS module has power

### 5. Test UPS Module

```bash
cd ~/aizee/python/nodes

# Test INA219 driver directly
python3 ina219.py

# Expected output:
# Load Voltage:  11.856 V
# Current:        0.123000 A
# Power:          1.458000 W
# Percentage:    39.00 %
```

If errors occur, check hardware connections and I2C permissions.

### 6. Test UPS Node

```bash
# Test UPS monitoring node
python3 ups_node.py --config ../../config/hardware_jetson_rover.yaml

# Expected output:
# 2026-02-10 10:30:45 - INFO - Initializing INA219 on I2C bus 7, address 0x41
# 2026-02-10 10:30:45 - INFO - Initial voltage reading: 11.86V
# 2026-02-10 10:30:45 - INFO - ZeroMQ publisher initialized
# ============================================================
# UPS Power Monitor Ready
# I2C: Bus 7, Address 0x41
# Publishing to: tcp://*:5562
# ============================================================
# 2026-02-10 10:30:50 - INFO - UPS Status: 11.86V, 0.123A, 1.46W, 39% (Publishing at 1.0 Hz)
```

Press `Ctrl+C` to stop the test.

### 7. Install Systemd Service

```bash
# Copy service file
sudo cp ~/aizee/config/systemd/aizee-ups-monitor.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service (start on boot)
sudo systemctl enable aizee-ups-monitor

# Start service now
sudo systemctl start aizee-ups-monitor

# Check status
sudo systemctl status aizee-ups-monitor
```

View logs in real-time:
```bash
sudo journalctl -u aizee-ups-monitor -f
```

---

## Using UPS Data

### View in Teleop (on Jetson or Remote)

From your dev machine:
```bash
ssh -i ssh-keys/aizee_rover_id ltr@192.168.0.27
cd ~/aizee/python/teleop
python3 teleop.py --config ../../config/teleop_rover_only.yaml
```

You'll see two battery lines in the UI:
```
Battery (Motor): 22.20V (58%) [OK]  (6S LIPO)
UPS: 11.86V  0.123A  1.46W  (39%) [GOOD]
```

### View in Rerun (Visualization)

From your dev machine:
```bash
cd /p/Workspace/aizee

# Start Rerun bridge with UPS telemetry
python python/rerun_bridge.py \
  --ups tcp://192.168.0.27:5562 \
  --lidar tcp://192.168.0.27:5561
```

In the Rerun viewer, you'll see:
- `power/ups/voltage` - Voltage plot
- `power/ups/current` - Current plot
- `power/ups/power` - Power plot
- `power/ups/battery_percentage` - Battery % plot
- `power/ups/status` - Text summary

### Record to MCAP

```bash
python python/rerun_bridge.py \
  --ups tcp://192.168.0.27:5562 \
  --lidar tcp://192.168.0.27:5561 \
  --save logs/session_$(date +%Y%m%d_%H%M%S).mcap
```

---

## Configuration

### UPS Settings (`config/hardware_jetson_rover.yaml`)

```yaml
ups:
  enabled: true
  i2c_bus: 7              # I2C bus number
  i2c_addr: 0x41          # INA219 address
  update_rate: 1.0        # Hz
  battery_chemistry: "lipo_3s"
  voltage_full: 12.6      # 3 × 4.2V
  voltage_nominal: 11.1   # 3 × 3.7V
  voltage_warning: 10.5   # 3 × 3.5V
  voltage_critical: 9.9   # 3 × 3.3V
  voltage_min: 9.0        # 3 × 3.0V
```

Adjust voltages based on your battery chemistry (3S/4S LiPo, NiMH, etc.).

### Teleop Settings (`config/teleop_rover_only.yaml`)

```yaml
endpoints:
  command: "tcp://192.168.0.27:5555"
  telemetry: "tcp://192.168.0.27:5556"
  ups_telemetry: "tcp://192.168.0.27:5562"  # UPS endpoint
```

---

## Troubleshooting

### UPS service won't start

```bash
# Check service status
sudo systemctl status aizee-ups-monitor

# View full logs
sudo journalctl -u aizee-ups-monitor -n 100
```

Common issues:
- **I2C permission denied:** User not in `i2c` group, logout/login required
- **No I2C device:** Check wiring, verify with `sudo i2cdetect -y 7`
- **Wrong I2C bus:** Try bus 1 instead: edit config to use `i2c_bus: 1`

### No UPS data in teleop

**Check UPS service is running:**
```bash
sudo systemctl status aizee-ups-monitor
```

**Test ZMQ connection:**
```bash
python3 -c "
import zmq, json, time
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('tcp://192.168.0.27:5562')
sub.subscribe(b'')
sub.setsockopt(zmq.RCVTIMEO, 5000)
try:
    msg = sub.recv_json()
    print('UPS Data:', json.dumps(msg, indent=2))
except zmq.Again:
    print('Timeout - no UPS messages')
"
```

### Wrong I2C bus

If `i2cdetect -y 7` shows nothing, try other buses:
```bash
# List all I2C buses
ls /dev/i2c-*

# Scan each one
sudo i2cdetect -y 0
sudo i2cdetect -y 1
```

Update `config/hardware_jetson_rover.yaml` with the correct bus number.

---

## Service Management

```bash
# Start UPS monitor
sudo systemctl start aizee-ups-monitor

# Stop UPS monitor
sudo systemctl stop aizee-ups-monitor

# Restart UPS monitor
sudo systemctl restart aizee-ups-monitor

# Enable auto-start on boot
sudo systemctl enable aizee-ups-monitor

# Disable auto-start
sudo systemctl disable aizee-ups-monitor

# View status
sudo systemctl status aizee-ups-monitor

# View logs (real-time)
sudo journalctl -u aizee-ups-monitor -f

# View logs (last 50 lines)
sudo journalctl -u aizee-ups-monitor -n 50
```

---

## Architecture Summary

```
┌─────────────────────────────────────────┐
│        Waveshare UPS Module (C)         │
│         INA219 @ I2C 0x41               │
└──────────────┬──────────────────────────┘
               │ I2C Bus 7
┌──────────────▼──────────────────────────┐
│         Jetson Orin Nano                │
│  ┌──────────────────────────────────┐  │
│  │  ups_node.py                     │  │
│  │  - Reads INA219 via I2C          │  │
│  │  - Publishes ZMQ tcp://*:5562    │  │
│  └──────────┬───────────────────────┘  │
└─────────────┼───────────────────────────┘
              │ ZMQ tcp://192.168.0.27:5562
      ┌───────┴───────┐
      ▼               ▼
┌───────────┐   ┌───────────┐
│  Teleop   │   │  Rerun    │
│  (Python) │   │  Bridge   │
└───────────┘   └───────────┘
   Display         Visualize
   UPS stats       Time-series
```

---

## Files Modified/Created

**New files:**
- `python/nodes/ups_node.py` - UPS monitoring daemon
- `python/nodes/ina219.py` - INA219 I2C driver
- `config/systemd/aizee-ups-monitor.service` - Systemd service
- `docs/UPS_DEPLOYMENT.md` - Full deployment guide
- `JETSON_UPS_QUICK_START.md` - This quick start

**Modified files:**
- `python/teleop/teleop.py` - Added UPS display
- `python/rerun_bridge.py` - Added UPS visualization
- `config/hardware_jetson_rover.yaml` - Added UPS config
- `config/teleop_rover_only.yaml` - Added UPS endpoint
- `config/teleop.yaml` - Added UPS endpoint

---

## Next Steps

- [ ] Test with live telemetry
- [ ] Tune battery percentage calculation
- [ ] Add low battery alerts/warnings
- [ ] Power consumption analysis over time
- [ ] Runtime estimation based on current draw

---

## References

- **Full Deployment Guide:** `docs/UPS_DEPLOYMENT.md`
- **Waveshare Wiki:** https://www.waveshare.com/wiki/UPS_Power_Module_(C)
- **INA219 Datasheet:** https://www.ti.com/lit/ds/symlink/ina219.pdf
