# Battery Voltage Monitoring

**Added:** 2026-02-10
**Battery Type:** 6S LiPo (6 × 3.7V nominal = 22.2V)

---

## Feature Overview

The teleop now displays real-time battery voltage with configurable warning levels and visual status indicators.

---

## Display Format

```
Battery: 22.20V (58%) [OK]  (6S LIPO)
         ^^^^^^  ^^^  ^^^^   ^^^^^^^
         voltage  %  status  cell info
```

### Status Levels

| Status | Voltage Range | Meaning | Display |
|--------|---------------|---------|---------|
| **OK** | ≥22.2V | Above nominal | Normal |
| **GOOD** | 21.0-22.2V | Above warning | Normal |
| **WARN** | 20.0-21.0V | Low battery | **Reverse video** |
| **CRIT** | <20.0V | Critical | **Reverse + Bold** |

---

## Configuration

Battery parameters are configurable in `config/teleop.yaml` and `config/teleop_rover_only.yaml`:

```yaml
battery:
  cells: 6                  # Number of cells
  cell_type: "lipo"         # Battery chemistry
  voltage_full: 25.2        # 6 × 4.2V (fully charged)
  voltage_nominal: 22.2     # 6 × 3.7V (nominal)
  voltage_warning: 21.0     # 6 × 3.5V (land soon)
  voltage_critical: 20.0    # 6 × 3.33V (land immediately)
  voltage_min: 18.0         # 6 × 3.0V (damage threshold)
```

### Customization

You can adjust the thresholds based on your use case:
- **Conservative:** Increase `voltage_warning` to 21.5V for longer battery life
- **Aggressive:** Decrease to 20.5V for more flight time (not recommended)
- **Different cell count:** Adjust all voltages proportionally

---

## Motor Controller Integration

### Required Telemetry Field

The motor controller must include `battery_voltage` in its telemetry JSON:

```json
{
  "timestamp": 1234567890.123,
  "battery_voltage": 22.4,
  "motors": {
    ...
  }
}
```

### Implementation (Rust Motor Controller)

Add battery voltage reading to `rust/motor_control/src/main.rs`:

```rust
// Example implementation (pseudocode)
let battery_voltage = read_battery_voltage(); // Read from ADC/sensor

let telemetry = json!({
    "timestamp": get_timestamp(),
    "battery_voltage": battery_voltage,
    "motors": motor_telemetry,
});

telemetry_publisher.send(&telemetry)?;
```

### Hardware Connection

✅ **Battery voltage is provided by ROBSTRIDE motors!**

The ROBSTRIDE motor controllers have built-in battery voltage sensing. The voltage
is available in the motor feedback data and should be read via CAN bus.

**Implementation:**
1. Read battery voltage from any ROBSTRIDE motor via CAN
2. Average readings from multiple motors if desired
3. Include in telemetry JSON as `battery_voltage`

No external voltage divider or ADC needed!

---

## Testing

Run the battery display test:

```bash
cd python/teleop
python test_battery_display.py
```

This shows how the display will appear at different voltage levels.

---

## Current Status

✅ **Teleop UI:** Battery display implemented
✅ **Configuration:** Warning levels configured for 6S LiPo
⏳ **Motor Controller:** Needs to send `battery_voltage` in telemetry
⏳ **Hardware:** Battery voltage sensor needs to be connected

---

## Safety Notes

1. **Never discharge below 18.0V** (3.0V per cell) - causes permanent damage
2. **Land immediately at 20.0V** (3.33V per cell) - critical threshold
3. **Plan to land at 21.0V** (3.5V per cell) - safe margin
4. **Storage voltage: 22.8V** (3.8V per cell) when not flying

---

## Example Display States

```
Fully charged:
  Battery: 25.20V (100%) [OK]  (6S LIPO)

Normal operation:
  Battery: 22.20V (58%) [OK]  (6S LIPO)

Getting low:
  Battery: 21.50V (49%) [GOOD]  (6S LIPO)

Warning - land soon:
  Battery: 20.50V (35%) [WARN]  (6S LIPO)  ← Reverse video

Critical - land now:
  Battery: 19.50V (21%) [CRIT]  (6S LIPO)  ← Reverse + Bold
```

---

## Next Steps

1. ✅ Add battery config to YAML files
2. ✅ Add battery display to teleop UI
3. ✅ Create test script
4. ⏳ Add voltage sensor hardware
5. ⏳ Implement battery voltage reading in Rust motor controller
6. ⏳ Test with live battery
