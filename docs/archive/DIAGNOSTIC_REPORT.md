# AIZEE Telemetry and Motor Diagnostics Report
**Date**: 2026-02-09
**Test Duration**: 5 minutes

---

## Executive Summary

✓ **ZeroMQ Communication**: WORKING
✓ **Motor Control Process**: RUNNING
✓ **Telemetry Streaming**: WORKING (10.6 Hz)
✗ **Motor Detection**: **FAILED - 0 motors detected**
✗ **Arm Module**: NOT RESPONDING

---

## Detailed Findings

### 1. Rover Module (192.168.0.27) - PARTIAL SUCCESS

#### Communication Layer ✓
- ZeroMQ command socket: `tcp://192.168.0.27:5555` - Connected
- ZeroMQ telemetry socket: `tcp://192.168.0.27:5556` - Connected
- Telemetry rate: **10.6 Hz** (53 samples in 5 seconds)
- Commands sent successfully (zero velocity keepalive confirmed)

#### Motor Control Process ✓
- Service: `aizee-motor-control-rover.service` - **RUNNING**
- PID: 1337
- User: ltr
- Uptime: ~34 minutes
- Config: `/home/ltr/aizee/config/hardware_jetson_rover.yaml`

#### CAN Interface ✗ **ISSUE DETECTED**
```
can0: <NO-CARRIER,NOARP,UP,ECHO> mtu 16 qdisc pfifo_fast state DOWN
can1: <NOARP,ECHO> mtu 16 qdisc noop state DOWN
```

**Problem**: `can0` is UP but shows `NO-CARRIER`
**Meaning**: CAN interface configured but **no physical devices detected**

#### Expected Motors (from config)
1. **left_wheel** (CAN ID 0x02, ROBSTRIDE04)
2. **right_wheel** (CAN ID 0x04, ROBSTRIDE04)
3. **swivel** (CAN ID 0x03, ROBSTRIDE03)

#### Actual Motors Detected
**NONE** - Empty motors dictionary in all telemetry samples

#### Telemetry Sample
```json
{
  "timestamp": 1770699005.0395098,
  "motors": {}
}
```

---

### 2. Arm Module (192.168.0.28) - FAILURE

**Status**: NOT RESPONDING
- Connection timeout on both command and telemetry sockets
- Likely causes:
  - Motor control process not running on RPi4
  - Network connectivity issue
  - Service not started/enabled

---

## Root Cause Analysis

### Rover Module - No Motor Detection

**Most Likely Causes** (in order of probability):

1. **Motors Not Powered** ⚡
   - ROBSTRIDE motors require external 24-48V power
   - Check power supply to motors
   - Verify power LED on motors (if equipped)

2. **CAN Bus Not Connected** 🔌
   - Physical CAN wiring disconnected from `can0`
   - CAN-H and CAN-L wires swapped
   - Motors connected to wrong CAN interface (`can1` instead of `can0`)

3. **CAN Termination Issue** 🔧
   - Missing 120Ω termination resistors
   - CAN bus requires termination at both ends
   - Check for 60Ω resistance between CAN-H and CAN-L

4. **Motor Communication Fault** ❌
   - Motors in fault/error state
   - Motors need manual reset/power cycle
   - CAN IDs misconfigured on motors

---

## Diagnostic Commands Run

### Network Tests
```bash
# Connectivity test - Rover
python test_connectivity.py --module rover --timeout 3000
Result: PASS (telemetry received, 0 motors)

# Connectivity test - Arm
python test_connectivity.py --module arm --timeout 3000
Result: FAIL (timeout)

# Detailed telemetry monitor
python detailed_motor_test.py --module rover --duration 5
Result: 53 samples @ 10.6Hz, motors: {}
```

### Jetson Status Checks
```bash
# Process status
ps aux | grep motor_control
Result: PID 1337, running as ltr

# CAN interface status
ip link show can0
Result: UP, NO-CARRIER

ip link show can1
Result: DOWN

# Service status
systemctl list-units | grep aizee
Result: aizee-motor-control-rover.service RUNNING
```

---

## Recommended Actions

### Immediate (Critical) ⚠️

1. **Check Motor Power Supply**
   ```bash
   # On Jetson, verify motors are powered
   # Look for LEDs on ROBSTRIDE motors (red = power, green = CAN)
   ```

2. **Verify CAN Physical Connection**
   ```bash
   ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

   # Check if motors connected to can1 instead of can0
   # If so, either:
   # A) Move wiring to can0, OR
   # B) Update config to use can1
   ```

3. **Test CAN Bus Health**
   ```bash
   ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

   # Monitor for ANY CAN traffic
   candump can0
   # Should see periodic messages if motors are alive
   ```

### Short-term (High Priority) 📋

4. **Scan for Motors on Both Interfaces**
   ```bash
   # Run motor scanner on can0 and can1
   ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
   cd ~/aizee
   python3 ~/scan_all_motors.py can0  # Scan CAN IDs 1-127
   python3 ~/scan_all_motors.py can1  # Check alternate interface
   ```

5. **Fix Arm Module**
   ```bash
   # SSH to arm module and check status
   ssh -i P:/Workspace/ssh-keys/aizee_arm_id user@192.168.0.28

   # Check if motor control service exists/running
   systemctl status aizee-motor-control-arm

   # If not deployed, deploy arm module
   # (deploy script may need to be created)
   ```

### Configuration Updates (If Needed) 🔧

6. **If Motors on can1 Instead of can0**

   Edit `P:/Workspace/aizee/config/hardware_jetson_rover.yaml`:
   ```yaml
   can:
     interface: can1  # Change from can0
   ```

   Then redeploy and restart service:
   ```bash
   scp -i P:/Workspace/ssh-keys/aizee_rover_id \
       P:/Workspace/aizee/config/hardware_jetson_rover.yaml \
       ltr@192.168.0.27:~/aizee/config/

   ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
       sudo systemctl restart aizee-motor-control-rover
   ```

   Also update systemd service to configure can1:
   Edit `/etc/systemd/system/aizee-motor-control-rover.service`
   ```ini
   ExecStartPre=-/usr/bin/sudo /usr/sbin/ip link set can1 down
   ExecStartPre=/usr/bin/sudo /usr/sbin/ip link set can1 type can bitrate 1000000
   ExecStartPre=/usr/bin/sudo /usr/sbin/ip link set can1 up
   ```

---

## Test Scripts Created

1. **`python/teleop/test_connectivity.py`**
   - Tests ZeroMQ connectivity to rover and arm modules
   - Verifies telemetry streaming
   - Sends test drive commands
   - Usage: `python test_connectivity.py --module all`

2. **`python/teleop/detailed_motor_test.py`**
   - Detailed telemetry monitoring
   - Shows full motor data in real-time
   - Sends periodic keepalive commands
   - Usage: `python detailed_motor_test.py --module rover --duration 10`

---

## System Architecture (Current State)

```
Dev Machine (P:/Workspace/aizee)
│
├─ Teleop (teleop.py) ← Your controller
│  └─ ZeroMQ Client ✓ WORKING
│
Ethernet (192.168.0.x)
│
├─ Jetson (192.168.0.27) - Rover Module
│  ├─ Service: aizee-motor-control-rover ✓ RUNNING
│  ├─ ZMQ: :5555/:5556 ✓ ACTIVE
│  ├─ CAN: can0 ⚠ UP but NO-CARRIER
│  │         can1 ✗ DOWN
│  └─ Motors: ✗ 0/3 detected
│     ├─ left_wheel (0x02) - MISSING
│     ├─ right_wheel (0x04) - MISSING
│     └─ swivel (0x03) - MISSING
│
└─ RPi4 (192.168.0.28) - Arm Module
   └─ Service: ✗ NOT RESPONDING
```

---

## Next Steps

**Priority 1**: Get motors detected on Jetson rover module
- [ ] Check motor power supply
- [ ] Verify CAN physical wiring
- [ ] Run candump to check for traffic
- [ ] Run motor scanner on both can0/can1

**Priority 2**: Bring up arm module
- [ ] SSH to 192.168.0.28
- [ ] Check motor_control service status
- [ ] Deploy if needed

**Priority 3**: Full system test
- [ ] Enable motors via teleop (press E key or A button)
- [ ] Test drive commands
- [ ] Verify all 6 motors responding

---

## Contact Points

- Jetson SSH: `ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27`
- Config: `P:/Workspace/aizee/config/hardware_jetson_rover.yaml`
- Service: `sudo systemctl status aizee-motor-control-rover`
- Logs: `sudo journalctl -u aizee-motor-control-rover -f`

---

**Report Generated**: 2026-02-09 23:51 UTC
**Test Tools**: test_connectivity.py, detailed_motor_test.py
**Status**: Motors not detected - requires physical hardware check
