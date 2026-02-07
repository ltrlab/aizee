#!/usr/bin/env python3
"""
Motor control test with proper cleanup for safe testing
PRODUCTION NOTE: In real robots, motors should stay enabled to prevent 
dropping loads. This version is for testing only.
"""
import can
import math
import time
import signal
import sys

HOST_CAN_ID = 0xAA
MSG_ENABLE = 3
MSG_DISABLE = 4
MSG_CONTROL = 1

# Global variables for signal handler
bus = None
motor_id = 2
cleanup_done = False

def signal_handler(signum, frame):
    """Handle signals to ensure motor is always disabled on exit"""
    global cleanup_done
    if cleanup_done:
        return
    cleanup_done = True
    
    print(f"\n\nReceived signal {signum}, disabling motor...")
    disable_motor()
    if bus:
        bus.shutdown()
    print("Motor safely disabled. Exiting.")
    sys.exit(0)

def build_can_id(motor_id, msg_type):
    return motor_id | (HOST_CAN_ID << 8) | (msg_type << 24)

def encode_control(position, velocity, kp, kd, torque):
    pos_enc = int(((position + 4.0 * math.pi) / (8.0 * math.pi) * 65535.0))
    pos_enc = max(0, min(65535, pos_enc))
    vel_enc = int(((velocity + 30.0) / 60.0 * 4095.0))
    vel_enc = max(0, min(4095, vel_enc))
    kp_enc = int((kp / 500.0 * 4095.0))
    kp_enc = max(0, min(4095, kp_enc))
    kd_enc = int((kd / 5.0 * 4095.0))
    kd_enc = max(0, min(4095, kd_enc))
    torque_enc = int(((torque + 18.0) / 36.0 * 4095.0))
    torque_enc = max(0, min(4095, torque_enc))
    
    data = bytearray(8)
    data[0] = (pos_enc >> 8) & 0xFF
    data[1] = pos_enc & 0xFF
    data[2] = (vel_enc >> 4) & 0xFF
    data[3] = ((vel_enc & 0x0F) << 4) | ((kp_enc >> 8) & 0x0F)
    data[4] = kp_enc & 0xFF
    data[5] = (kd_enc >> 4) & 0xFF
    data[6] = ((kd_enc & 0x0F) << 4) | ((torque_enc >> 8) & 0x0F)
    data[7] = torque_enc & 0xFF
    return bytes(data)

def send_cmd(msg_type, data=None):
    global bus, motor_id
    can_id = build_can_id(motor_id, msg_type)
    if data is None:
        data = bytes([0] * 8)
    msg = can.Message(arbitration_id=can_id, is_extended_id=True, data=data)
    bus.send(msg)

def disable_motor():
    """Safely disable the motor"""
    global bus, motor_id
    if bus:
        try:
            # Send zero torque first, then disable
            control_data = encode_control(0.0, 0.0, 3.0, 0.3, 0.0)
            send_cmd(MSG_CONTROL, control_data)
            time.sleep(0.05)
            send_cmd(MSG_DISABLE)
        except Exception as e:
            print(f"Warning: Error during motor disable: {e}")

def main():
    global bus, motor_id
    
    # Register signal handlers for safe cleanup
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # timeout/kill
    
    # Tuned gains - smooth motion, no vibrations
    kp = 3.0
    kd = 0.3
    
    print(f"=== Motor Control Test (Safe Cleanup) ===")
    print(f"Tuned gains: Kp={kp}, Kd={kd}")
    print(f"NOTE: Motor will auto-disable on script exit\n")
    
    bus = can.Bus(interface="socketcan", channel="can1", bitrate=1000000)
    
    try:
        # Enable motor
        print("Enabling motor...")
        send_cmd(MSG_ENABLE)
        time.sleep(0.5)
        
        print("Running smooth sine wave motion...")
        print("Press Ctrl+C to stop\n")
        
        start_time = time.time()
        loop_freq = 50  # Hz
        dt = 1.0 / loop_freq
        
        while True:
            t = time.time() - start_time
            
            # Smooth sine wave: 0.3 rad amplitude, 0.3 Hz frequency
            target_pos = 0.3 * math.sin(2 * math.pi * 0.3 * t)
            target_vel = 0.3 * 2 * math.pi * 0.3 * math.cos(2 * math.pi * 0.3 * t)
            
            # Send control command
            control_data = encode_control(target_pos, target_vel, kp, kd, 0.0)
            send_cmd(MSG_CONTROL, control_data)
            
            if int(t) != int(t - dt):  # Print every 1s
                print(f"t={t:5.1f}s: pos={target_pos:6.3f} rad")
            
            time.sleep(dt)
            
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Ensure cleanup always happens
        signal_handler(0, None)

if __name__ == "__main__":
    main()
