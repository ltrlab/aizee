# Fix CAN1 Configuration - Action Required

## What I Fixed

✅ Updated `config/hardware_jetson_rover.yaml` - Changed `can0` → `can1`
✅ Updated `config/systemd/aizee-motor-control-rover.service` - Now configures can1
✅ Updated `CLAUDE.md` - Documented that motors are on can1
✅ Deployed files to Jetson

## What You Need to Do (Requires Sudo Password)

SSH into the Jetson and run the update script:

```bash
ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# On Jetson, run:
bash ~/fix_can1_on_jetson.sh
```

This will:
1. Stop the motor control service
2. Install updated service file (configures can1 instead of can0)
3. Reload systemd
4. Start service with new config
5. Verify can1 is UP
6. Check for CAN traffic

**OR** manually run these commands:

```bash
ssh -i P:/Workspace/ssh-keys/aizee_rover_id ltr@192.168.0.27

# Stop service
sudo systemctl stop aizee-motor-control-rover

# Install updated service file
sudo cp ~/aizee-motor-control-rover.service /etc/systemd/system/aizee-motor-control-rover.service

# Reload and restart
sudo systemctl daemon-reload
sudo systemctl start aizee-motor-control-rover

# Verify
sudo systemctl status aizee-motor-control-rover
ip link show can1
```

## After Service Restart

Run the connectivity test again from your dev machine:

```bash
cd P:/Workspace/aizee
python python/teleop/test_connectivity.py --module rover --timeout 3000
```

You should now see **3 motors detected** instead of 0!

Expected output:
```
Motors found: 3
  left_wheel: Enabled/Disabled pos=... vel=... T=...C
  right_wheel: Enabled/Disabled pos=... vel=... T=...C
  swivel: Enabled/Disabled pos=... vel=... T=...C
```

## Files Changed

- `config/hardware_jetson_rover.yaml` - CAN interface: can0 → can1
- `config/systemd/aizee-motor-control-rover.service` - ExecStartPre now brings up can1
- `CLAUDE.md` - Added documentation about can1
- `scripts/fix_can1_on_jetson.sh` - Helper script for updating service
