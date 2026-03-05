# Quick Start After Jetson Reboot

This guide helps you get the AIZEE rover operational after a reboot.

## What Happens After Reboot

When the Jetson restarts, you need to ensure:
1. **CAN interface is configured** (usually automatic if service is enabled)
2. **Motor control service is running** (automatic if enabled)
3. **Telemetry is flowing** (verify connectivity)

## Quick Check: Is Everything Running?

```bash
cd /p/Workspace/aizee
./scripts/check_rover_status.sh
```

This will show you:
- ✓ Network connectivity
- ✓ CAN interface status
- ✓ Motor control service status
- ✓ Telemetry availability

## First-Time Setup (If Not Done Yet)

If you haven't deployed the systemd service to the Jetson yet:

### 1. Deploy Rover Module to Jetson

```bash
cd /p/Workspace/aizee
./scripts/deploy_jetson_rover.sh ltr@192.168.0.27
```

This will:
- Sync the codebase to the Jetson
- Build the motor_control binary
- Install the systemd service

### 2. Enable and Start Service on Jetson

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# On Jetson:
sudo systemctl enable aizee-motor-control-rover  # Auto-start on boot
sudo systemctl start aizee-motor-control-rover    # Start now
sudo systemctl status aizee-motor-control-rover   # Check status
```

### 3. Verify Rover is Running

```bash
# Back on dev machine:
cd /p/Workspace/aizee
./scripts/check_rover_status.sh
```

### 4. Start Teleop

```bash
cd /p/Workspace/aizee
python python/teleop/teleop.py --config config/teleop_rover_only.yaml
```

## If Service Is Already Enabled (Normal Usage)

If the service is enabled to start on boot, after a reboot you should only need to:

```bash
cd /p/Workspace/aizee
./scripts/check_rover_status.sh  # Verify everything started
python python/teleop/teleop.py --config config/teleop_rover_only.yaml
```

## Troubleshooting After Reboot

### CAN Interface Not Up

**Symptom**: `check_rover_status.sh` shows "CAN Interface: DOWN"

**Solution**:
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

**Permanent fix** (done by systemd service):
The systemd service automatically configures CAN on startup. If it's not working, check:
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
sudo journalctl -u aizee-motor-control-rover | grep "ExecStartPre"
```

### Motor Control Service Not Running

**Symptom**: `check_rover_status.sh` shows "Motor Control Service: Not Running"

**Solution**:
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
sudo systemctl start aizee-motor-control-rover
sudo journalctl -u aizee-motor-control-rover -f
```

**Check for errors**:
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27
sudo systemctl status aizee-motor-control-rover
sudo journalctl -u aizee-motor-control-rover -n 50
```

Common issues:
- CAN interface failed to come up (check ExecStartPre logs)
- Config file not found (check AIZEE_CONFIG path)
- Binary not built (run cargo build --release)

### No Telemetry in Teleop

**Symptom**: Teleop shows "waiting for telemetry..."

**Test telemetry directly**:
```bash
python3 -c "import zmq, json; ctx = zmq.Context(); s = ctx.socket(zmq.SUB); s.connect('tcp://192.168.0.27:5556'); s.setsockopt(zmq.SUBSCRIBE, b''); s.setsockopt(zmq.RCVTIMEO, 5000); print(json.loads(s.recv_string()))"
```

If this fails:
1. Check service is running: `./scripts/check_rover_status.sh`
2. Check firewall: `ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo ufw status`
3. Check port binding: `ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo netstat -tulpn | grep 5556`

### Motors Not Responding

1. **Check power**: Ensure motors are powered on
2. **Check CAN wiring**: Verify CAN bus connections are secure
3. **Check CAN interface**: Should show bitrate 1000000
   ```bash
   ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 ip link show can1
   ```
4. **Monitor CAN traffic**:
   ```bash
   ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 candump can1
   ```
5. **Enable motors**: In teleop, press **A button** (or **E key**) to enable all motors

## Service Management Commands

### Check Status
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo systemctl status aizee-motor-control-rover
```

### View Logs (Live)
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo journalctl -u aizee-motor-control-rover -f
```

### View Recent Logs
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo journalctl -u aizee-motor-control-rover -n 100
```

### Restart Service
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo systemctl restart aizee-motor-control-rover
```

### Stop Service
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo systemctl stop aizee-motor-control-rover
```

### Disable Auto-Start on Boot
```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 sudo systemctl disable aizee-motor-control-rover
```

## Manual CAN Setup (Without Service)

If you want to run motor control manually without the service:

```bash
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Setup CAN interface
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# Run motor control
cd ~/aizee
AIZEE_CONFIG=config/hardware_jetson_rover.yaml RUST_LOG=info ./rust/target/release/motor_control
```

## System Architecture

```
Dev Machine (Windows P:/Workspace/aizee)
│
├─ python/teleop/teleop.py (Controller)
│
Ethernet (192.168.0.x)
│
└─ Jetson (192.168.0.27) - Rover Module
   ├─ Service: aizee-motor-control-rover
   ├─ Config: hardware_jetson_rover.yaml
   ├─ ZMQ: :5555 (cmd), :5556 (telem)
   ├─ CAN: can1 @ 1Mbps
   └─ Motors:
      ├─ left_wheel (CAN ID 0x02, ROBSTRIDE04)
      ├─ right_wheel (CAN ID 0x04, ROBSTRIDE04)
      └─ swivel (CAN ID 0x03, ROBSTRIDE03)
```

## Common Workflows

**Daily startup** (if service enabled on boot):
```bash
cd /p/Workspace/aizee
./scripts/check_rover_status.sh                                   # Verify running
python python/teleop/teleop.py --config config/teleop_rover_only.yaml  # Start teleop
```

**After code changes**:
```bash
cd /p/Workspace/aizee
./scripts/deploy_jetson_rover.sh                                  # Deploy + rebuild
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    sudo systemctl restart aizee-motor-control-rover              # Restart service
./scripts/check_rover_status.sh                                   # Verify
python python/teleop/teleop.py --config config/teleop_rover_only.yaml  # Test
```

**Debugging**:
```bash
# Check status
./scripts/check_rover_status.sh

# View live logs
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 \
    sudo journalctl -u aizee-motor-control-rover -f

# Monitor CAN traffic
ssh -i /p/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27 candump can1

# Test telemetry directly
python3 -c "import zmq, json; ctx = zmq.Context(); s = ctx.socket(zmq.SUB); s.connect('tcp://192.168.0.27:5556'); s.setsockopt(zmq.SUBSCRIBE, b''); s.setsockopt(zmq.RCVTIMEO, 5000); print(json.dumps(json.loads(s.recv_string()), indent=2))"
```
