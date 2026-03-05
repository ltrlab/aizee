# Quick Start: RPi4 Arm Module Deployment

> **Note**: In the current production setup, **all 9 motors run on the Jetson** via `config/hardware_jetson_rover.yaml` and `can1`. This guide covers the *alternative* configuration where the 6 arm motors run on a separate Raspberry Pi 4 at 192.168.0.28.

Fast setup guide for deploying the AIZEE arm module to a standalone Raspberry Pi 4.

## Prerequisites

- ✅ Raspberry Pi 4 (4GB+ RAM)
- ✅ USB CAN adapter (CANable, PEAK, etc.)
- ✅ 6× ROBSTRIDE motors (arm) with CAN IDs 0x05–0x0A
- ✅ Network connection to same subnet as Jetson (192.168.0.x)
- ✅ Raspberry Pi OS Lite 64-bit installed

## Quick Deploy (10 minutes)

### 1. Configure RPi4 Network
```bash
# Set static IP: 192.168.0.28
ssh ltr@aizee-arm  # or ltr@192.168.0.28
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
./scripts/deploy_rpi4_arm.sh ltr@192.168.0.28
```

### 4. Start Service
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28
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
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28 \
    sudo systemctl status aizee-motor-control-arm

# Check telemetry
python3 -c "import zmq, json; ctx = zmq.Context(); s = ctx.socket(zmq.SUB); s.connect('tcp://192.168.0.28:5558'); s.setsockopt(zmq.SUBSCRIBE, b''); print(json.loads(s.recv_string()))"

# Check CAN interface
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28 ip link show can0
```

## Run Unified Teleop

```bash
cd P:/Workspace/aizee
python python/teleop/teleop.py --config config/teleop.yaml
```

Now you can control both rover (Jetson) and arm (RPi4) from a single interface.

## Common Commands

```bash
# View logs
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28 \
    sudo journalctl -u aizee-motor-control-arm -f

# Restart service
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28 \
    sudo systemctl restart aizee-motor-control-arm

# Rebuild after code changes
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.28 \
    "cd ~/aizee/rust/motor_control && cargo build --release && sudo systemctl restart aizee-motor-control-arm"
```

## Network Topology

```
Dev Machine (Windows) ─────┐
                           │
Jetson (192.168.0.27)      ├─── POE Switch
:5555/:5556 (rover base)   │
                           │
RPi4 (192.168.0.28)        ┘
:5557/:5558 (arm)
```

## Motor Assignments (RPi4 Arm Module)

**Rover Module (Jetson, can1):**
| Motor | CAN ID | Model |
|---|---|---|
| left_wheel | 0x02 | ROBSTRIDE04 |
| swivel | 0x03 | ROBSTRIDE03 |
| right_wheel | 0x04 | ROBSTRIDE04 |

**Arm Module (RPi4, can0):**
| Motor | CAN ID | Model |
|---|---|---|
| gantry_base | 0x05 | ROBSTRIDE04 |
| gantry_mid | 0x06 | ROBSTRIDE03 |
| gantry_end | 0x07 | ROBSTRIDE02 |
| wrist_pitch | 0x08 | ROBSTRIDE02 |
| wrist_roll | 0x09 | ROBSTRIDE00 |
| gripper | 0x0A | ROBSTRIDE00 |

See `docs/deployment/MULTI_DEVICE_DEPLOYMENT.md` for detailed documentation.
