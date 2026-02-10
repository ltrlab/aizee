# Waveshare UPS Power Module (C) Deployment Guide

**Hardware:** Waveshare UPS Power Module (C) with INA219
**Communication:** I2C bus
**Integration:** ZeroMQ telemetry + Rerun visualization
**Last Updated:** 2026-02-10

---

## Overview

The Waveshare UPS Power Module (C) provides battery backup and real-time power monitoring via the INA219 current/power monitor IC. This guide covers hardware setup, software deployment, and integration with the AIZEE system.

---

## Hardware Setup

### UPS Module Specifications

- **Power IC:** INA219 (current/power monitor)
- **I2C Address:** 0x41 (default for UPS module)
- **I2C Bus:** Bus 7 (Jetson Orin Nano)
- **Measurements:**
  - Voltage: 0-16V (configurable)
  - Current: ±5A continuous
  - Power: Calculated from V×I
  - Battery percentage: Calculated

### Physical Connection

1. **Connect UPS to Jetson Orin Nano:**
   - VCC → 5V pin (or system power)
   - GND → Ground
   - SDA → I2C SDA (Pin 3 on 40-pin header)
   - SCL → I2C SCL (Pin 5 on 40-pin header)

2. **Verify I2C Connection:**
   ```bash
   # Install i2c-tools if needed
   sudo apt-get install i2c-tools

   # Scan I2C bus 7 for devices
   sudo i2cdetect -y 7

   # You should see device at address 0x41:
   #      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
   # 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
   # 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   # 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   # 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
   # 40: -- 41 -- -- -- -- -- -- -- -- -- -- -- -- -- --
   ```

3. **Set I2C Permissions:**
   ```bash
   # Add user to i2c group
   sudo usermod -a -G i2c ltr

   # Logout and login for group changes to take effect
   ```

---

## Software Deployment

### 1. Deploy Code to Jetson

From your development machine:

```bash
# Deploy entire aizee directory
cd /p/Workspace
scp -i ssh-keys/aizee_rover_id -r aizee ltr@192.168.0.27:~/

# Or deploy specific UPS files
scp -i ssh-keys/aizee_rover_id python/nodes/ups_node.py ltr@192.168.0.27:~/aizee/python/nodes/
scp -i ssh-keys/aizee_rover_id python/nodes/ina219.py ltr@192.168.0.27:~/aizee/python/nodes/
scp -i ssh-keys/aizee_rover_id config/hardware_jetson_rover.yaml ltr@192.168.0.27:~/aizee/config/
```

### 2. Install Python Dependencies

SSH into Jetson:

```bash
ssh -i ssh-keys/aizee_rover_id ltr@192.168.0.27

# Install smbus for I2C communication
sudo apt-get update
sudo apt-get install python3-smbus

# Install other dependencies if needed
pip3 install pyzmq pyyaml
```

### 3. Test UPS Module

Test the INA219 library directly:

```bash
cd ~/aizee/python/nodes
python3 ina219.py
```

Expected output:
```
INA219 Test - Waveshare UPS Power Module (C)
==================================================
Load Voltage:   11.856 V
Current:         0.123000 A
Power:           1.458000 W
Percentage:     39.00 %
--------------------------------------------------
```

If you see errors:
- Check I2C wiring
- Verify I2C address with `sudo i2cdetect -y 7`
- Check I2C permissions (user in i2c group)

### 4. Test UPS Node

Test the full UPS monitoring node:

```bash
cd ~/aizee/python/nodes
python3 ups_node.py --config ../../config/hardware_jetson_rover.yaml
```

Expected output:
```
2026-02-10 10:30:45,123 - __main__ - INFO - Initializing INA219 on I2C bus 7, address 0x41
2026-02-10 10:30:45,234 - __main__ - INFO - INA219 initialized successfully
2026-02-10 10:30:45,235 - __main__ - INFO - Initial voltage reading: 11.86V
2026-02-10 10:30:45,345 - __main__ - INFO - Initializing ZeroMQ publisher at tcp://*:5562
2026-02-10 10:30:45,856 - __main__ - INFO - ZeroMQ publisher initialized
============================================================
UPS Power Monitor Ready
I2C: Bus 7, Address 0x41
Publishing to: tcp://*:5562
============================================================
2026-02-10 10:30:50,856 - __main__ - INFO - UPS Status: 11.86V, 0.123A, 1.46W, 39% (Publishing at 1.0 Hz)
```

### 5. Install Systemd Service

Deploy as a system service for automatic startup:

```bash
# Copy service file
sudo cp ~/aizee/config/systemd/aizee-ups-monitor.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable service to start on boot
sudo systemctl enable aizee-ups-monitor

# Start service now
sudo systemctl start aizee-ups-monitor

# Check status
sudo systemctl status aizee-ups-monitor
```

View logs:
```bash
# Real-time logs
sudo journalctl -u aizee-ups-monitor -f

# Last 50 lines
sudo journalctl -u aizee-ups-monitor -n 50
```

---

## Integration with Rerun

### Viewing UPS Data in Rerun

From your development machine, start the Rerun bridge with UPS endpoint:

```bash
cd /p/Workspace/aizee
python python/rerun_bridge.py \
  --ups tcp://192.168.0.27:5562 \
  --lidar tcp://192.168.0.27:5561
```

In the Rerun viewer, you'll see:
- **power/ups/voltage** - Voltage scalar plot
- **power/ups/current** - Current scalar plot
- **power/ups/power** - Power scalar plot
- **power/ups/battery_percentage** - Battery % scalar plot
- **power/ups/status** - Text box with current stats

### Recording to MCAP

Save UPS telemetry to MCAP files for replay:

```bash
python python/rerun_bridge.py \
  --ups tcp://192.168.0.27:5562 \
  --lidar tcp://192.168.0.27:5561 \
  --save logs/session_$(date +%Y%m%d_%H%M%S).mcap
```

---

## Telemetry Format

### ZeroMQ Message (JSON)

Published on `tcp://*:5562`:

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

### Rerun Visualization

The bridge logs UPS data to these Rerun entities:

- `power/ups/voltage` → `Scalar` (Volts)
- `power/ups/current` → `Scalar` (Amperes)
- `power/ups/power` → `Scalar` (Watts)
- `power/ups/battery_percentage` → `Scalar` (%)
- `power/ups/status` → `TextDocument` (Markdown summary)

---

## Configuration

### Hardware Config (`config/hardware_jetson_rover.yaml`)

```yaml
ups:
  enabled: true
  i2c_bus: 7              # I2C bus number
  i2c_addr: 0x41          # INA219 I2C address
  update_rate: 1.0        # Hz - publishing rate
  battery_chemistry: "lipo_3s"  # 3S LiPo
  voltage_full: 12.6      # Fully charged (3 × 4.2V)
  voltage_nominal: 11.1   # Nominal (3 × 3.7V)
  voltage_warning: 10.5   # Warning threshold
  voltage_critical: 9.9   # Critical threshold
  voltage_min: 9.0        # Absolute minimum

network:
  device:
    zmq:
      ups_pub: "tcp://*:5562"  # UPS telemetry endpoint
```

### Battery Thresholds

Adjust voltage thresholds based on your battery:

**3S LiPo (11.1V nominal):**
- Full: 12.6V (3 × 4.2V)
- Nominal: 11.1V (3 × 3.7V)
- Warning: 10.5V (3 × 3.5V)
- Critical: 9.9V (3 × 3.3V)
- Minimum: 9.0V (3 × 3.0V)

**4S LiPo (14.8V nominal):**
- Full: 16.8V (4 × 4.2V)
- Nominal: 14.8V (4 × 3.7V)
- Warning: 14.0V (4 × 3.5V)
- Critical: 13.2V (4 × 3.3V)
- Minimum: 12.0V (4 × 3.0V)

---

## Troubleshooting

### Issue: "No such file or directory: '/dev/i2c-7'"

**Cause:** I2C bus 7 not enabled on Jetson

**Solution:**
```bash
# Check available I2C buses
ls /dev/i2c-*

# If only /dev/i2c-0 and /dev/i2c-1 exist, you may need to use bus 1
# Update config to use i2c_bus: 1

# Or enable additional I2C buses in device tree (advanced)
```

### Issue: "[Errno 13] Permission denied"

**Cause:** User doesn't have I2C permissions

**Solution:**
```bash
sudo usermod -a -G i2c ltr
# Logout and login
```

### Issue: "Remote I/O error"

**Cause:** Wrong I2C address or hardware not connected

**Solution:**
```bash
# Scan for I2C devices
sudo i2cdetect -y 7

# Check physical connections (SDA, SCL, power, ground)
# Verify address in config matches detected address
```

### Issue: No telemetry in Rerun

**Cause:** UPS node not running or ZeroMQ endpoint mismatch

**Solution:**
```bash
# Check UPS service status
sudo systemctl status aizee-ups-monitor

# Check ZeroMQ connectivity
python3 -c "
import zmq, json, time
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('tcp://192.168.0.27:5562')
sub.subscribe(b'')
sub.setsockopt(zmq.RCVTIMEO, 5000)
try:
    msg = sub.recv_json()
    print('Received:', json.dumps(msg, indent=2))
except zmq.Again:
    print('Timeout - no messages received')
"
```

---

## Next Steps

- [x] Hardware connection and I2C verification
- [x] Software deployment and testing
- [x] Systemd service installation
- [x] Rerun visualization integration
- [ ] Teleop UI battery display (in progress)
- [ ] Low battery alerts
- [ ] Power consumption analysis
- [ ] Runtime estimation

---

## References

- **Waveshare Wiki:** https://www.waveshare.com/wiki/UPS_Power_Module_(C)
- **INA219 Datasheet:** https://www.ti.com/lit/ds/symlink/ina219.pdf
- **Jetson I2C Guide:** https://developer.nvidia.com/embedded/learn/tutorials/jetson-gpio-i2c
- **AIZEE Documentation:** `docs/`

---

## Files Modified/Created

1. `python/nodes/ups_node.py` - UPS monitoring node
2. `python/nodes/ina219.py` - INA219 driver library
3. `python/rerun_bridge.py` - Added UPS telemetry support
4. `config/hardware_jetson_rover.yaml` - Added UPS configuration
5. `config/systemd/aizee-ups-monitor.service` - Systemd service
6. `docs/UPS_DEPLOYMENT.md` - This deployment guide
