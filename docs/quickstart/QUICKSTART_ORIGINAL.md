# AIZEE Motor Control - Quick Start Guide

This guide will help you get the motor control system running on your Jetson Orin Nano with ROBSTRIDE actuators.

## Prerequisites

### Hardware
- NVIDIA Jetson Orin Nano with JetPack 6.x
- CANable USB-CAN adapter (or compatible slcan device)
- ROBSTRIDE motors (02/03/04 series)
- 40V power supply for motors
- CAN bus wiring with 120Ω termination resistors

### Software
- Rust toolchain (stable)
- Python 3.10+
- can-utils package
- ZeroMQ library

## Installation

### 1. Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

### 2. Install System Dependencies

```bash
# On Jetson/Ubuntu
sudo apt update
sudo apt install -y \
    can-utils \
    libzmq3-dev \
    python3-pip \
    python3-zmq
```

### 3. Install Python Dependencies

```bash
cd aizee
pip3 install -r requirements.txt
```

### 4. Build Motor Control

```bash
cd rust/motor_control
cargo build --release
```

This will take several minutes on first build. The binary will be at `target/release/motor_control`.

## Hardware Setup

### 1. Connect CAN Adapter

1. Plug CANable USB adapter into Jetson
2. Verify it appears as `/dev/ttyACM0` or `/dev/ttyUSB0`:
   ```bash
   ls /dev/ttyACM*
   ```

### 2. Wire CAN Bus

```
[CANable]  ──CAN-H──┬──[Motor 1]──┬──[Motor 2]──...──┬──[Motor N]
              CAN-L──┤             │                  │
                     └── 120Ω ────┘         120Ω ────┘
                     (termination)          (termination)
```

**Important**:
- Use twisted pair for CAN-H and CAN-L
- Add 120Ω termination resistors at both ends of bus
- Keep total bus length under 40 meters at 1 Mbps

### 3. Power Motors

1. Connect 40V power supply to all motors
2. Verify green power LEDs are lit on motors

## Configuration

### 1. Update Motor CAN IDs

Edit `config/hardware.yaml` to match your actual motor CAN IDs:

```yaml
motors:
  wheels:
    - id: left_wheel
      can_id: 0x01  # ← Change these to match your motors
      type: ROBSTRIDE04
      # ...
```

To find motor IDs, use the Python script:
```bash
# This will scan for motors on the bus
python3 python/nodes/find_motors.py
```

### 2. Verify Network Endpoints

Check ZeroMQ endpoints in `config/hardware.yaml`:

```yaml
network:
  jetson:
    zmq:
      command_sub: "tcp://*:5555"   # Commands from teleop
      telemetry_pub: "tcp://*:5556" # Telemetry to clients
```

## First Run

### 1. Setup CAN Interface

```bash
sudo ./scripts/setup_can.sh
```

Select option 1 (CANable/slcan) and follow prompts.

### 2. Test CAN Communication

In one terminal, monitor CAN traffic:
```bash
candump can0
```

In another terminal, send a test frame:
```bash
cansend can0 001#0000000000000000
```

You should see the frame in candump output. If not, check wiring.

### 3. Launch Motor Control

```bash
# Make script executable
chmod +x scripts/launch_motor_control.sh

# Launch motor controller
./scripts/launch_motor_control.sh
```

You should see:
```
====================================
 AIZEE Motor Control Launcher
====================================

✓ CAN interface ready
✓ Config loaded: config/hardware.yaml

Starting motor control system...
  - Arm control: 1 kHz
  - Base control: 100 Hz
  - Telemetry: 50 Hz

Commands:     tcp://*:5555
Telemetry:    tcp://*:5556
```

### 4. Test with Python Script

In a new terminal:

```bash
# Interactive mode
python3 python/teleop/simple_test.py
```

You'll see a prompt:
```
>
```

Try these commands:

```bash
# Enable all motors
> e left_wheel right_wheel shoulder_pitch elbow wrist_gripper

# Read telemetry
> t

# Zero arm positions
> z shoulder_pitch elbow wrist_gripper

# Move arm to test position
> arm 0.0 0.5 0.0

# Drive forward slowly
> drive 0.2 0.0

# Stop
> drive 0.0 0.0

# Disable motors
> d left_wheel right_wheel shoulder_pitch elbow wrist_gripper

# Quit
> q
```

## Automated Test

Run the automated test sequence:

```bash
python3 python/teleop/simple_test.py auto
```

This will:
1. Enable all motors
2. Zero arm positions
3. Move arm through test positions
4. Drive base forward
5. Stop and disable motors

## Troubleshooting

### "CAN interface not found"

```bash
# Check USB connection
lsusb | grep -i can

# Check serial devices
ls /dev/ttyACM* /dev/ttyUSB*

# Re-run setup
sudo ./scripts/setup_can.sh
```

### "Permission denied" on /dev/ttyACM0

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, or:
newgrp dialout
```

### Motors not responding

1. **Check power**: Green LED on motor should be lit
2. **Check wiring**: Verify CAN-H and CAN-L not reversed
3. **Check termination**: 120Ω resistors at both ends of bus
4. **Check CAN ID**: Use `candump can0` to see if motors are transmitting
5. **Test single motor**: Disconnect all but one motor, enable it
6. **Verify motor firmware**: Motors must be configured for CAN bus mode

### "No messages received" in Python test

```bash
# Verify motor controller is running
ps aux | grep motor_control

# Check ZeroMQ ports are open
sudo netstat -tulpn | grep 555

# Try connecting directly
python3 -c "import zmq; ctx=zmq.Context(); s=ctx.socket(zmq.SUB); s.connect('tcp://localhost:5556'); s.subscribe(b''); print('Connected')"
```

### High CPU usage

The 1kHz arm control loop is CPU-intensive:

```bash
# Set performance governor
sudo cpufreq-set -g performance

# Check CPU frequency
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq

# Monitor CPU usage
htop
```

### Motor errors in telemetry

Check error field in telemetry output:

- `undervoltage`: Power supply voltage too low (< 36V)
- `overcurrent`: Motor drawing too much current
- `overtemp`: Motor temperature too high (> 80°C)
- `magnetic_encoding_fault`: Encoder error
- `uncalibrated`: Motor needs calibration (run zero_position)

## Next Steps

Once basic motor control is working:

1. **Tune control gains**: Adjust `kp` and `kd` values for arm control
2. **Implement teleop**: Create joystick/keyboard control interface
3. **Add camera nodes**: Set up RPi camera streaming
4. **Integrate Rerun**: Add visualization and logging
5. **Develop autonomous behaviors**: Path planning, obstacle avoidance

See [PHASES.md](PHASES.md) for the full implementation roadmap.

## Safety Notes

⚠️ **IMPORTANT SAFETY WARNINGS**:

1. **Emergency Stop**: Always know how to quickly kill power to motors
2. **Soft Limits**: Configure position limits in `hardware.yaml` to prevent collisions
3. **Workspace**: Keep clear area around robot during testing
4. **Power**: Use proper 40V supply with current limiting
5. **Watchdog**: System will stop motors if no command received for 100ms
6. **Test Incrementally**: Start with single motor at low speeds

## Getting Help

- **Documentation**: See `rust/motor_control/README.md` for API details
- **Issues**: Report bugs at https://github.com/ltrlab/aizee/issues
- **Community**: Join discussions in GitHub Discussions

## License

MIT License - See LICENSE file for details.
