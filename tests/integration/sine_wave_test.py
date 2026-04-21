#!/usr/bin/env python3
"""
Sine wave test for two ROBSTRIDE motors via Rust motor_control + ZeroMQ.

Requires the motor_control binary running on the Jetson:
  AIZEE_CONFIG=~/aizee/config/hardware_two_motors.yaml \
  ~/aizee/rust/target/release/motor_control

Then run this script (on Jetson or remotely):
  python3 sine_wave_test.py [--endpoint tcp://localhost:5555]

Motors:
  - left_wheel:  ROBSTRIDE04, CAN ID 0x02
  - right_wheel: ROBSTRIDE03, CAN ID 0x03
"""
import zmq
import json
import math
import time
import signal
import sys
import argparse
import threading

# Defaults
CMD_ENDPOINT = "tcp://localhost:5555"
TEL_ENDPOINT = "tcp://localhost:5556"

# Sine wave parameters
AMPLITUDE = 2.0      # rad/s peak velocity (clearly visible motion)
FREQUENCY = 0.25     # Hz - slow gentle oscillation
COMMAND_RATE = 50     # Hz - how fast we send commands
DURATION = 20.0       # seconds to run
RAMP_TIME = 1.0       # seconds to ramp up/down (eliminates jerk)

cleanup_done = False
running = True


def signal_handler(signum, frame):
    global running
    print(f"\nReceived signal {signum}, stopping...")
    running = False


def telemetry_listener(tel_endpoint):
    """Background thread to print telemetry."""
    global running
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(tel_endpoint)
    sub.subscribe(b"")
    sub.setsockopt(zmq.RCVTIMEO, 500)

    last_print = 0
    while running:
        try:
            msg = sub.recv_string()
            data = json.loads(msg)
            now = time.time()
            if now - last_print >= 1.0:
                motors = data.get("motors", {})
                parts = []
                for name, info in sorted(motors.items()):
                    pos = info.get("position", 0)
                    vel = info.get("velocity", 0)
                    temp = info.get("temperature", 0)
                    state = info.get("state", "?")
                    mode = info.get("mode", "?")
                    err = info.get("error")
                    s = f"{name}: pos={pos:+.3f} vel={vel:+.3f} T={temp:.0f}C [{state}/{mode}]"
                    if err:
                        s += f" ERR={err}"
                    parts.append(s)
                if parts:
                    print(f"  [telemetry] {' | '.join(parts)}")
                last_print = now
        except zmq.Again:
            continue
        except Exception as e:
            if running:
                print(f"  [telemetry error] {e}")
            break

    sub.close()
    ctx.term()


def send_command(socket, cmd):
    """Send a JSON command via PUSH socket."""
    socket.send_string(json.dumps(cmd))


def main():
    global running, cleanup_done

    parser = argparse.ArgumentParser(description="Sine wave motor test")
    parser.add_argument("--cmd-endpoint", default=CMD_ENDPOINT,
                        help=f"ZeroMQ command endpoint (default: {CMD_ENDPOINT})")
    parser.add_argument("--tel-endpoint", default=TEL_ENDPOINT,
                        help=f"ZeroMQ telemetry endpoint (default: {TEL_ENDPOINT})")
    parser.add_argument("--amplitude", type=float, default=AMPLITUDE,
                        help=f"Velocity amplitude in rad/s (default: {AMPLITUDE})")
    parser.add_argument("--frequency", type=float, default=FREQUENCY,
                        help=f"Sine wave frequency in Hz (default: {FREQUENCY})")
    parser.add_argument("--duration", type=float, default=DURATION,
                        help=f"Test duration in seconds (default: {DURATION})")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("=" * 50)
    print("  AIZEE Sine Wave Motor Test")
    print("=" * 50)
    print(f"  Command endpoint: {args.cmd_endpoint}")
    print(f"  Telemetry endpoint: {args.tel_endpoint}")
    print(f"  Amplitude: {args.amplitude} rad/s")
    print(f"  Frequency: {args.frequency} Hz")
    print(f"  Duration: {args.duration}s")
    print(f"  Command rate: {COMMAND_RATE} Hz")
    print()

    # Connect ZeroMQ PUSH socket (motor_control binds PULL)
    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PUSH)
    cmd_socket.connect(args.cmd_endpoint)
    print(f"Connected PUSH socket to {args.cmd_endpoint}")

    # Allow connection to establish
    time.sleep(0.5)

    # Start telemetry listener in background
    tel_thread = threading.Thread(
        target=telemetry_listener,
        args=(args.tel_endpoint,),
        daemon=True,
    )
    tel_thread.start()

    try:
        # Step 1: Enable both motors
        print("\n[1/4] Enabling motors...")
        send_command(cmd_socket, {
            "type": "enable",
            "motor_ids": ["left_wheel", "right_wheel"]
        })
        time.sleep(1.0)
        print("  Motors enabled.")

        # Step 2: Zero positions
        print("[2/4] Zeroing motor positions...")
        send_command(cmd_socket, {
            "type": "zero_position",
            "motor_ids": ["left_wheel", "right_wheel"]
        })
        time.sleep(0.5)
        print("  Positions zeroed.")

        # Step 3: Run sine wave
        print(f"[3/4] Running sine wave for {args.duration}s...")
        print("  Press Ctrl+C to stop early.\n")

        dt = 1.0 / COMMAND_RATE
        start_time = time.time()
        last_status = 0

        while running:
            t = time.time() - start_time
            if t >= args.duration:
                print(f"\n  Duration complete ({args.duration}s).")
                break

            # Smooth ramp envelope (cosine taper)
            if t < RAMP_TIME:
                envelope = 0.5 * (1.0 - math.cos(math.pi * t / RAMP_TIME))
            elif t > args.duration - RAMP_TIME:
                envelope = 0.5 * (1.0 - math.cos(math.pi * (args.duration - t) / RAMP_TIME))
            else:
                envelope = 1.0

            # Sinusoidal velocity command with smooth ramp
            velocity = envelope * args.amplitude * math.sin(2 * math.pi * args.frequency * t)

            # Send as drive command (both motors get same velocity when angular=0)
            send_command(cmd_socket, {
                "type": "drive",
                "linear": velocity,
                "angular": 0.0
            })

            # Status printout every 2 seconds
            if int(t / 2) != last_status:
                last_status = int(t / 2)
                print(f"  t={t:5.1f}s  vel={velocity:+.3f} rad/s  ramp={envelope:.2f}")

            time.sleep(dt)

        # Step 4: Stop sine wave
        print("\n[4/6] Stopping sine wave...")

        # Send zero velocity
        for _ in range(10):
            send_command(cmd_socket, {
                "type": "drive",
                "linear": 0.0,
                "angular": 0.0
            })
            time.sleep(0.02)

        time.sleep(1.0)

        # Step 5: Fault trigger/clear test
        print("[5/6] Fault detection test...")
        print("  Triggering simulated fault on right_wheel...")
        send_command(cmd_socket, {
            "type": "trigger_fault",
            "motor_ids": ["right_wheel"]
        })
        time.sleep(2.0)
        print("  (check telemetry above - right_wheel should show state=error)")

        print("  Clearing fault on right_wheel...")
        send_command(cmd_socket, {
            "type": "clear_fault",
            "motor_ids": ["right_wheel"]
        })
        time.sleep(2.0)
        print("  (check telemetry above - right_wheel should show state=enabled)")

        # Step 6: Disable motors
        print("[6/6] Disabling motors...")

        send_command(cmd_socket, {
            "type": "disable",
            "motor_ids": ["left_wheel", "right_wheel"]
        })
        time.sleep(0.5)
        print("  Motors disabled.")

        cleanup_done = True
        print("\nTest complete!")

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if not cleanup_done:
            print("\nEmergency cleanup...")
            try:
                send_command(cmd_socket, {"type": "emergency_stop"})
                time.sleep(0.2)
                send_command(cmd_socket, {
                    "type": "disable",
                    "motor_ids": ["left_wheel", "right_wheel"]
                })
            except Exception:
                pass

        running = False
        cmd_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
