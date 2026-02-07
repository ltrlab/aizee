# Quick Start: Multi-Device Deployment

Fast setup guide for deploying AIZEE arm module to Raspberry Pi 4.

## Prerequisites

- ✅ Raspberry Pi 4 (4GB+ RAM)
- ✅ USB CAN adapter (CANable, PEAK, etc.)
- ✅ 3× ROBSTRIDE motors (arm) with CAN IDs 0x05, 0x06, 0x07
- ✅ Network connection to same subnet as Jetson (192.168.0.x)
- ✅ Raspberry Pi OS Lite 64-bit installed

## Quick Deploy (10 minutes)

### 1. Configure RPi4 Network
```bash
# Set static IP: 192.168.0.28
ssh pi@aizee-arm  # or pi@192.168.0.28
sudo nmcli con mod "Wired connection 1" ipv4.addresses 192.168.0.28/24
sudo nmcli con mod "Wired connection 1" ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

### 2. Install Dependencies
```bash
# One-liner install
sudo apt update && sudo apt install -y git build-essential pkg-config libzmq3-dev can-utils && \
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
source ~/.cargo/env
```

### 3. Deploy from Dev Machine
```bash
# From P:/Workspace/aizee
./scripts/deploy_rpi4_arm.sh pi@192.168.0.28
```

### 4. Start Service
```bash
ssh pi@192.168.0.28
sudo systemctl start aizee-motor-control-arm
sudo systemctl status aizee-motor-control-arm
```

### 5. Test
```bash
# From dev machine
python scripts/test_arm_module.py --host 192.168.0.28
```

## Verify Deployment

```bash
# Check service running
ssh pi@192.168.0.28 sudo systemctl status aizee-motor-control-arm

# Check telemetry
python3 -c "import zmq, json; ctx = zmq.Context(); s = ctx.socket(zmq.SUB); s.connect('tcp://192.168.0.28:5558'); s.setsockopt(zmq.SUBSCRIBE, b''); print(json.loads(s.recv_string()))"

# Check CAN interface
ssh pi@192.168.0.28 ip link show can0
```

## Run Unified Teleop

```bash
cd P:/Workspace/aizee
python python/teleop/teleop.py --config config/teleop.yaml
```

Now you can control both rover and arm from a single interface!

## Common Commands

```bash
# View logs
ssh pi@192.168.0.28 sudo journalctl -u aizee-motor-control-arm -f

# Restart service
ssh pi@192.168.0.28 sudo systemctl restart aizee-motor-control-arm

# Stop service
ssh pi@192.168.0.28 sudo systemctl stop aizee-motor-control-arm

# Rebuild after code changes
ssh pi@192.168.0.28 "cd ~/aizee/rust/motor_control && cargo build --release && sudo systemctl restart aizee-motor-control-arm"
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No telemetry | Check service: `sudo systemctl status aizee-motor-control-arm` |
| CAN error | Check interface: `ip link show can0` should show UP |
| Permission denied | Add to group: `sudo usermod -a -G dialout pi` |
| Motors not responding | Check CAN wiring and motor power |

## Network Topology

```
Dev Machine (Windows) ─────┐
                           │
Jetson (192.168.0.27)      ├─── POE Switch
:5555/:5556 (rover)        │
                           │
RPi4 (192.168.0.28)        ┘
:5557/:5558 (arm)
```

## Motor Assignments

**Rover Module (Jetson)**:
- 0x02: left_wheel
- 0x03: swivel
- 0x04: right_wheel

**Arm Module (RPi4)**:
- 0x05: shoulder_pitch
- 0x06: elbow
- 0x07: wrist

## Next Steps

- [ ] Configure GitHub SSH for git pull on RPi4
- [ ] Add arm control mapping to teleop (right stick)
- [ ] Test simultaneous rover + arm control
- [ ] Deploy torso module (future)

See `docs/MULTI_DEVICE_DEPLOYMENT.md` for detailed documentation.
