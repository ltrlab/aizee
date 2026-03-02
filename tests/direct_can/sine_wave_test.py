#!/usr/bin/env python3
"""
Sine wave test for two ROBSTRIDE motors via direct CAN.
  - ROBSTRIDE03, CAN ID 0x03 (on can1)
  - ROBSTRIDE04, CAN ID 0x02 (on can1)

Usage on Jetson:
  python3 sine_wave_single.py
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

MOTORS = [
    {"name": "RS03", "can_id": 3, "kp": 3.0, "kd": 0.3},
    {"name": "RS04", "can_id": 2, "kp": 3.0, "kd": 0.3},
]
CAN_CHANNEL = "can1"

# Sine wave parameters
AMPLITUDE = 0.3      # radians
FREQUENCY = 0.3      # Hz
LOOP_HZ = 50
DURATION = 20.0      # seconds
RAMP_TIME = 1.0      # seconds to ramp up/down (eliminates jerk)

bus = None
cleanup_done = False


def signal_handler(signum, frame):
    global cleanup_done
    if cleanup_done:
        return
    cleanup_done = True
    print(f"\nSignal {signum}, disabling motors...")
    if bus:
        try:
            for m in MOTORS:
                send_control(m["can_id"], 0.0, 0.0, m["kp"], m["kd"], 0.0)
            time.sleep(0.05)
            for m in MOTORS:
                send_cmd(m["can_id"], MSG_DISABLE)
        except Exception as e:
            print(f"Cleanup error: {e}")
        bus.shutdown()
    print("Motors disabled. Exiting.")
    sys.exit(0)


def build_can_id(motor_id, msg_type):
    return motor_id | (HOST_CAN_ID << 8) | (msg_type << 24)


def send_cmd(motor_id, msg_type, data=None):
    cid = build_can_id(motor_id, msg_type)
    if data is None:
        data = bytes(8)
    bus.send(can.Message(arbitration_id=cid, is_extended_id=True, data=data))


def encode_control(position, velocity, kp, kd, torque):
    pos_enc = max(0, min(65535, int((position + 4*math.pi) / (8*math.pi) * 65535)))
    vel_enc = max(0, min(4095, int((velocity + 30.0) / 60.0 * 4095)))
    kp_enc = max(0, min(4095, int(kp / 500.0 * 4095)))
    kd_enc = max(0, min(4095, int(kd / 5.0 * 4095)))
    torque_enc = max(0, min(4095, int((torque + 18.0) / 36.0 * 4095)))

    d = bytearray(8)
    d[0] = (pos_enc >> 8) & 0xFF
    d[1] = pos_enc & 0xFF
    d[2] = (vel_enc >> 4) & 0xFF
    d[3] = ((vel_enc & 0x0F) << 4) | ((kp_enc >> 8) & 0x0F)
    d[4] = kp_enc & 0xFF
    d[5] = (kd_enc >> 4) & 0xFF
    d[6] = ((kd_enc & 0x0F) << 4) | ((torque_enc >> 8) & 0x0F)
    d[7] = torque_enc & 0xFF
    return bytes(d)


def send_control(motor_id, pos, vel, kp, kd, torque):
    send_cmd(motor_id, MSG_CONTROL, encode_control(pos, vel, kp, kd, torque))


def smooth_ramp(t, duration, ramp_time):
    """Returns 0→1 over ramp_time at start, 1→0 over ramp_time at end."""
    if t < ramp_time:
        return 0.5 * (1.0 - math.cos(math.pi * t / ramp_time))
    elif t > duration - ramp_time:
        remaining = duration - t
        return 0.5 * (1.0 - math.cos(math.pi * remaining / ramp_time))
    return 1.0


def main():
    global bus, cleanup_done

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    motor_names = ", ".join(f"{m['name']} (ID {m['can_id']})" for m in MOTORS)
    print(f"=== Sine Wave Test: {motor_names} ===")
    print(f"Amplitude={AMPLITUDE} rad, Freq={FREQUENCY} Hz, Duration={DURATION}s")
    print(f"CAN: {CAN_CHANNEL} @ 1Mbps, Loop: {LOOP_HZ} Hz, Ramp: {RAMP_TIME}s\n")

    bus = can.Bus(interface="socketcan", channel=CAN_CHANNEL, bitrate=1000000)

    try:
        # Enable all motors
        for m in MOTORS:
            print(f"Enabling {m['name']} (ID {m['can_id']})...")
            send_cmd(m["can_id"], MSG_ENABLE)
            time.sleep(0.1)

        # Wait and check for responses
        time.sleep(0.5)
        while True:
            r = bus.recv(timeout=0.1)
            if r is None:
                break
            if r.is_rx:
                mid = (r.arbitration_id >> 8) & 0xFF
                print(f"  Response from motor {mid}: 0x{r.arbitration_id:08X} {r.data.hex()}")

        print(f"\nRunning sine wave for {DURATION}s (Ctrl+C to stop)...\n")

        dt = 1.0 / LOOP_HZ
        start = time.time()

        while True:
            t = time.time() - start
            if t >= DURATION:
                print(f"\n  Duration complete ({DURATION}s).")
                break

            envelope = smooth_ramp(t, DURATION, RAMP_TIME)
            pos = envelope * AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)
            vel = envelope * AMPLITUDE * 2 * math.pi * FREQUENCY * math.cos(2 * math.pi * FREQUENCY * t)

            for m in MOTORS:
                send_control(m["can_id"], pos, vel, m["kp"], m["kd"], 0.0)

            if int(t) != int(t - dt) and int(t) % 2 == 0:
                print(f"  t={t:5.1f}s  pos={pos:+.3f} rad  vel={vel:+.3f} rad/s  env={envelope:.2f}")

            time.sleep(dt)

        # Ramp already brings us to zero, send a few hold commands then disable
        print("\nStopping motors...")
        for _ in range(10):
            for m in MOTORS:
                send_control(m["can_id"], 0.0, 0.0, m["kp"], m["kd"], 0.0)
            time.sleep(0.02)

        time.sleep(0.3)
        for m in MOTORS:
            send_cmd(m["can_id"], MSG_DISABLE)
            print(f"  {m['name']} disabled.")
        time.sleep(0.2)

        cleanup_done = True
        print("\nTest complete!")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if not cleanup_done:
            signal_handler(0, None)
        else:
            bus.shutdown()


if __name__ == "__main__":
    main()
