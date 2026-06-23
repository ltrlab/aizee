# Waveshare UPS Power Module (INA219)

**Hardware:** Waveshare UPS Power Module (C) with an INA219 current/power monitor
**Bus:** I2C bus 7, address `0x41` (Jetson Orin Nano)
**Node:** `python/nodes/ups_node.py` → ZMQ PUB `tcp://*:5562` at ~1 Hz
**Service:** `aizee-ups-monitor`

The module provides battery backup and real-time power telemetry. `ups_node.py`
reads the INA219 over I2C and publishes JSON telemetry over ZeroMQ for the Tufty
status display, the Rerun bridge, and any other subscriber.

## Hardware

| Property | Value |
|---|---|
| Monitor IC | INA219 |
| I2C bus | 7 |
| I2C address | `0x41` |
| Measurements | bus voltage, shunt voltage, current, power, derived battery % |

### I2C wiring and verification

SDA → pin 3, SCL → pin 5 on the 40-pin header; VCC/GND to system power and ground.

```bash
sudo apt-get install -y i2c-tools
sudo i2cdetect -y 7          # expect a device at 0x41
sudo usermod -a -G i2c ltr   # log out/in for group change to take effect
```

## Configuration

`ups:` section of `config/hardware_jetson_rover.yaml`:

```yaml
ups:
  enabled: true
  i2c_bus: 7              # I2C bus number (Jetson Orin Nano default)
  i2c_addr: 0x41          # INA219 I2C address
  update_rate: 1.0        # Hz - publishing rate
  battery_chemistry: "lipo_3s"  # 3S LiPo (11.1V nominal)
  voltage_full: 12.6      # Fully charged (3 × 4.2V)
  voltage_nominal: 11.1   # Nominal (3 × 3.7V)
  voltage_warning: 10.5   # Warning threshold (3 × 3.5V)
  voltage_critical: 9.9   # Critical threshold (3 × 3.3V)
  voltage_min: 9.0        # Absolute minimum (3 × 3.0V)
```

The publish endpoint comes from `network.device.zmq.ups_pub` (`tcp://*:5562`).

> Note: `ups_node.py` reads `i2c_bus`, `i2c_addr`, and the publish endpoint from
> config. Battery percentage is currently computed in code from a fixed 3S range
> (9.0 V = 0 %, 12.6 V = 100 %); the `voltage_*` thresholds document the chemistry
> but are not yet wired into the node.

## Running

The node takes config from the YAML, with CLI overrides:

```bash
# Via config (how the service runs)
python3 python/nodes/ups_node.py --config config/hardware_jetson_rover.yaml

# Manual override
python3 python/nodes/ups_node.py --i2c-bus 7 --i2c-addr 0x41 \
    --publish tcp://*:5562 --rate 1.0
```

### Systemd service (`aizee-ups-monitor`)

```bash
sudo cp config/systemd/aizee-ups-monitor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now aizee-ups-monitor
sudo systemctl status aizee-ups-monitor
sudo journalctl -u aizee-ups-monitor -f
```

The unit runs `ups_node.py --config .../hardware_jetson_rover.yaml` as user `ltr`
from `/home/ltr/aizee`, with `Restart=on-failure`.

## Telemetry format

One JSON message per update on `tcp://*:5562` (msgpack-framed via
`common.wire.pack_msg`):

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

| Field | Unit | Notes |
|---|---|---|
| `voltage` | V | INA219 bus voltage |
| `current` | A | converted from mA |
| `power` | W | INA219 power register |
| `percentage` | % | clamped 0–100, derived from voltage |
| `shunt_voltage` | V | for diagnostics |

The Tufty display node subscribes to this stream (`up` / `ub` fields). See
[TUFTY2040.md](TUFTY2040.md).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No such file: /dev/i2c-7` | Bus 7 not present — check `ls /dev/i2c-*`, adjust `i2c_bus` |
| `[Errno 13] Permission denied` | `ltr` not in `i2c` group — `sudo usermod -a -G i2c ltr`, re-login |
| `Remote I/O error` | Wrong address or bad wiring — re-run `sudo i2cdetect -y 7` |
| No telemetry to subscribers | Check `systemctl status aizee-ups-monitor`; confirm subscriber connects to `tcp://<jetson>:5562` |

## Related files

| File | Purpose |
|---|---|
| `python/nodes/ups_node.py` | UPS monitoring node (I2C → ZMQ) |
| `python/nodes/ina219.py` | INA219 driver |
| `config/hardware_jetson_rover.yaml` | `ups:` config + ZMQ endpoint |
| `config/systemd/aizee-ups-monitor.service` | systemd unit |
