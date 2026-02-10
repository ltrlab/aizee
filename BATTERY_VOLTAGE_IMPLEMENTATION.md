# Battery Voltage Monitoring Implementation

**Date:** 2026-02-10
**Status:** ✅ Complete and Working
**Battery Type:** 6S LiPo (22.2V nominal, 25.2V full, 18.0V minimum)

---

## Overview

Real-time battery voltage monitoring integrated into the teleop interface with color-coded status indicators. Battery voltage is read directly from ROBSTRIDE motors via CAN bus (no external sensor required).

---

## Implementation Details

### 1. ROBSTRIDE CAN Protocol

**Register Address:** `0x701C` (VBUS)
**Communication Type:** 17 (Single parameter read)
**Data Format:** IEEE-754 float (4 bytes, little-endian)
**Unit:** Volts (V)
**Update Rate:** 100ms (10 Hz)

**CAN Frame Structure:**
```
Request:  0x1100AA02 [8] 1C 70 00 00 00 00 00 00
Response: 0x110002AA [8] 1C 70 00 00 66 66 A9 41
                                     └─ Float: 21.3V
```

### 2. Rust Motor Controller Changes

**File:** `rust/motor_control/src/robstride.rs`

Added VBUS register constant:
```rust
pub const VBUS: u16 = 0x701C;  // Battery voltage (float, in Volts)
```

Added parameter response parser:
```rust
pub fn parse_read_param_response(frame: &CanFrame) -> Result<(u16, f32)> {
    let data = frame.data();
    let param_id = u16::from_le_bytes([data[0], data[1]]);
    let value = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    Ok((param_id, value))
}
```

**File:** `rust/motor_control/src/main.rs`

Added battery voltage tracking:
```rust
struct ControlSystem {
    // ... existing fields ...
    battery_voltage: Option<f32>,
    last_vbus_request: Instant,
}
```

VBUS request logic (every 100ms):
```rust
fn send_control_commands(&mut self) -> Result<()> {
    // ... motor control ...

    // Request battery voltage periodically (10Hz)
    if self.last_vbus_request.elapsed() >= Duration::from_millis(100) {
        if let Some(motor) = self.base_group.motors.first() {
            let frame = robstride::build_read_param_frame(
                motor.config.can_id,
                robstride::params::VBUS
            );
            self.can_socket.write_frame(&frame)?;
            self.last_vbus_request = Instant::now();
        }
    }
    Ok(())
}
```

Response parsing in `read_feedback()`:
```rust
if msg_type == 17 {  // ReadParam response
    match robstride::parse_read_param_response(&frame) {
        Ok((param_id, value)) => {
            if param_id == robstride::params::VBUS {
                self.battery_voltage = Some(value);
            }
        }
        Err(e) => warn!("Failed to parse param response: {}", e),
    }
    continue;
}
```

Include in telemetry:
```rust
fn publish_telemetry(&mut self) -> Result<()> {
    let mut msg = TelemetryMessage::new();
    // ... add motor data ...
    msg.battery_voltage = self.battery_voltage;
    self.telemetry_pub.publish(&msg)?;
    Ok(())
}
```

### 3. Telemetry Message Structure

**File:** `rust/comms/src/messages.rs`

Added battery_voltage field:
```rust
pub struct TelemetryMessage {
    pub timestamp: f64,
    pub motors: HashMap<String, MotorTelemetry>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub battery_voltage: Option<f32>,
}
```

**JSON Format:**
```json
{
  "timestamp": 1739209573.123,
  "motors": { ... },
  "battery_voltage": 21.18
}
```

### 4. Teleop Display

**File:** `python/teleop/teleop.py`

Color initialization:
```python
curses.start_color()
curses.use_default_colors()
curses.init_pair(1, curses.COLOR_GREEN, -1)   # OK
curses.init_pair(2, curses.COLOR_CYAN, -1)    # GOOD
curses.init_pair(3, curses.COLOR_YELLOW, -1)  # WARN
curses.init_pair(4, curses.COLOR_RED, -1)     # CRIT
```

Battery display with color coding:
```python
if voltage >= bat_cfg["voltage_nominal"]:
    status = "OK"
    attr = curses.color_pair(1) | curses.A_BOLD  # Green
elif voltage >= bat_cfg["voltage_warning"]:
    status = "GOOD"
    attr = curses.color_pair(2)  # Cyan
elif voltage >= bat_cfg["voltage_critical"]:
    status = "WARN"
    attr = curses.color_pair(3) | curses.A_BOLD  # Yellow
else:
    status = "CRIT"
    attr = curses.color_pair(4) | curses.A_BOLD | curses.A_REVERSE  # Red

safe_addstr(stdscr, row, 0,
    f"  Battery: {voltage:.2f}V ({percent:.0f}%) [{status}]  "
    f"({bat_cfg['cells']}S {bat_cfg['cell_type'].upper()})",
    attr, clear_line=True)
```

### 5. Configuration

**Files:** `config/teleop.yaml`, `config/teleop_rover_only.yaml`

```yaml
battery:
  cells: 6                    # 6S LiPo
  cell_type: "lipo"
  voltage_full: 25.2          # 6 × 4.2V (fully charged)
  voltage_nominal: 22.2       # 6 × 3.7V (nominal)
  voltage_warning: 21.0       # 6 × 3.5V (land soon)
  voltage_critical: 20.0      # 6 × 3.33V (land immediately)
  voltage_min: 18.0           # 6 × 3.0V (damage threshold)
```

---

## Status Thresholds

| Status | Voltage Range | Color | Display | Meaning |
|--------|---------------|-------|---------|---------|
| **OK** | ≥22.2V | 🟢 Green (bold) | Normal | Above nominal, battery healthy |
| **GOOD** | 21.0-22.2V | 🔵 Cyan | Normal | Above warning, safe to operate |
| **WARN** | 20.0-21.0V | 🟡 Yellow (bold) | Alert | Low battery, land soon |
| **CRIT** | <20.0V | 🔴 Red (bold+reverse) | Alert | Critical, land immediately |

---

## Display Format

```
Battery: 21.18V (44%) [GOOD]  (6S LIPO)
         ^^^^^^  ^^^  ^^^^^^   ^^^^^^^^^
         voltage  %   status   cell info
```

**Color coding:**
- Entire line colored based on battery status
- Makes critical warnings immediately visible
- No additional hardware required (uses motor's built-in voltage sensor)

---

## Testing

**Test Results (2026-02-10):**
- ✅ Battery voltage: 21.18V (stable)
- ✅ Status display: [GOOD] (cyan)
- ✅ Update rate: 10 Hz (100ms intervals)
- ✅ CAN communication: Working
- ✅ Color display: Working
- ✅ Real-time updates: Working

**Test Commands:**
```bash
# Test battery display formatting
python python/teleop/test_battery_display.py

# Run teleop with battery monitoring
python python/teleop/teleop.py --config config/teleop_rover_only.yaml

# Monitor CAN bus traffic
candump can1  # On Jetson
```

---

## Troubleshooting

### Issue: Battery shows "(no data)"

**Causes:**
1. Motor controller not running
2. Motors not connected/powered
3. Wrong VBUS register address
4. Telemetry field missing

**Solution:**
```bash
# Check motor controller service
sudo systemctl status aizee-motor-control-rover

# Check CAN bus traffic
candump can1 | grep "1C 70"  # Look for VBUS requests/responses

# Verify telemetry
python3 -c "import zmq, json
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.connect('tcp://192.168.0.27:5556')
sub.setsockopt(zmq.SUBSCRIBE, b'')
msg = sub.recv_json()
print('battery_voltage' in msg, msg.get('battery_voltage'))"
```

### Issue: CAN interface won't come up

**Solution:**
```bash
# Reset USB CAN adapter
echo "1-2.3.1:1.0" | sudo tee /sys/bus/usb/drivers/gs_usb/unbind
sleep 1
echo "1-2.3.1:1.0" | sudo tee /sys/bus/usb/drivers/gs_usb/bind
sleep 2

# Configure and bring up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

---

## Hardware Requirements

- **ROBSTRIDE Motors:** Model 03 or 04 with built-in VBUS sensing
- **CAN Interface:** USB CAN adapter (gs_usb compatible)
- **Battery:** 6S LiPo (or adjust config for different cell count)

**No external voltage sensor required!** Battery voltage is read directly from motor controllers.

---

## Safety Notes

1. **Never discharge below 18.0V** (3.0V/cell) - causes permanent battery damage
2. **Land immediately at 20.0V** (3.33V/cell) - critical threshold
3. **Plan to land at 21.0V** (3.5V/cell) - safe warning margin
4. **Storage voltage: 22.8V** (3.8V/cell) when not in use

---

## Future Enhancements

- [ ] Battery remaining time estimate based on current draw
- [ ] Low battery alarm/beep
- [ ] Battery health tracking (charge cycles, capacity degradation)
- [ ] Multiple battery support (if using multiple power sources)
- [ ] Automatic logging of battery voltage history

---

## Files Modified

1. `rust/comms/src/messages.rs` - Added battery_voltage field
2. `rust/motor_control/src/robstride.rs` - Added VBUS register and parser
3. `rust/motor_control/src/main.rs` - Added VBUS reading logic
4. `python/teleop/teleop.py` - Added color-coded battery display
5. `config/teleop.yaml` - Added battery configuration
6. `config/teleop_rover_only.yaml` - Added battery configuration
7. `python/teleop/test_battery_display.py` - Test script for battery display

---

## References

- ROBSTRIDE RS-03 User Manual (docs/RS03User Manual260112.pdf)
- ROBSTRIDE RS-04 User Manual (docs/RS04User Manual260112.pdf)
- Register 0x701C: VBUS (Battery voltage, float, Volts)
- CAN Communication Type 17: Single parameter read
