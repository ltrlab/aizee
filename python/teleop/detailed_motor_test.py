#!/usr/bin/env python3
"""
Detailed motor diagnostic - shows full telemetry and allows sending test commands.
"""

import json
import sys
import time
import zmq
import argparse
from pathlib import Path

def monitor_telemetry(cmd_addr, telem_addr, duration_sec=10):
    """Monitor telemetry and send periodic status commands."""

    print(f"\n{'='*70}")
    print(f"MOTOR TELEMETRY MONITOR")
    print(f"  Command:   {cmd_addr}")
    print(f"  Telemetry: {telem_addr}")
    print(f"  Duration:  {duration_sec}s")
    print(f"{'='*70}\n")

    ctx = zmq.Context()

    # Setup sockets
    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.connect(cmd_addr)

    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.connect(telem_addr)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sock.setsockopt(zmq.RCVTIMEO, 100)

    print("[OK] Connected to ZeroMQ sockets\n")

    start_time = time.time()
    sample_count = 0
    last_sample_time = 0

    try:
        while (time.time() - start_time) < duration_sec:
            # Receive telemetry
            try:
                raw = telem_sock.recv_string(zmq.NOBLOCK)
                telem = json.loads(raw)
                sample_count += 1

                now = time.time()
                elapsed = now - start_time
                dt = now - last_sample_time if last_sample_time > 0 else 0
                last_sample_time = now

                print(f"\n--- Sample #{sample_count} (t={elapsed:.1f}s, dt={dt*1000:.1f}ms) ---")
                print(f"Timestamp: {telem.get('timestamp', 'N/A')}")

                if 'motors' in telem and telem['motors']:
                    print(f"Motors: {len(telem['motors'])}")
                    for motor_id, data in telem['motors'].items():
                        state = data.get('state', '?')
                        pos = data.get('position', 0.0)
                        vel = data.get('velocity', 0.0)
                        torque = data.get('torque', 0.0)
                        temp = data.get('temperature', 0.0)
                        error = data.get('error', None)

                        status_line = (
                            f"  [{motor_id:15s}] {state:10s} | "
                            f"pos={pos:+8.3f} vel={vel:+7.3f} "
                            f"torque={torque:+6.2f} T={temp:4.0f}C"
                        )
                        if error:
                            status_line += f" | ERROR: {error}"
                        print(status_line)
                else:
                    print("Motors: NONE (empty dict or missing)")
                    print(f"Raw telemetry keys: {list(telem.keys())}")
                    print(f"Full data: {json.dumps(telem, indent=2)}")

            except zmq.Again:
                pass  # No telemetry available
            except json.JSONDecodeError as e:
                print(f"[ERROR] Failed to parse JSON: {e}")

            # Send periodic zero-velocity command (keepalive for watchdog)
            if sample_count % 10 == 1:  # Every 10th sample
                cmd = {"type": "drive", "linear": 0.0, "angular": 0.0, "swivel": 0.0}
                cmd_sock.send_string(json.dumps(cmd))
                print(f"  [CMD] Sent keepalive: {cmd}")

            time.sleep(0.05)  # 20Hz polling

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        # Send final zero command
        cmd_sock.send_string(json.dumps({"type": "drive", "linear": 0.0, "angular": 0.0, "swivel": 0.0}))

        cmd_sock.close()
        telem_sock.close()
        ctx.term()

    print(f"\n{'='*70}")
    print(f"SUMMARY: Received {sample_count} telemetry samples in {duration_sec}s")
    if sample_count > 0:
        print(f"Average rate: {sample_count/duration_sec:.1f} Hz")
    print(f"{'='*70}\n")

def main():
    parser = argparse.ArgumentParser(description="Monitor motor telemetry in detail")
    parser.add_argument(
        "--module",
        choices=["rover", "arm"],
        default="rover",
        help="Which module to monitor (default: rover)"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=10,
        help="How long to monitor in seconds (default: 10)"
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to teleop.yaml config file"
    )
    args = parser.parse_args()

    # Load config
    if args.config is None:
        script_dir = Path(__file__).parent
        config_path = script_dir / ".." / ".." / "config" / "teleop.yaml"
        if not config_path.exists():
            config_path = Path("config/teleop.yaml")
        if not config_path.exists():
            print("Error: Cannot find config/teleop.yaml")
            return 1
    else:
        config_path = Path(args.config)

    import yaml
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    if "modules" not in cfg or args.module not in cfg["modules"]:
        print(f"Error: Module '{args.module}' not found in config")
        return 1

    module_cfg = cfg["modules"][args.module]
    monitor_telemetry(
        module_cfg["command"],
        module_cfg["telemetry"],
        args.duration
    )

    return 0

if __name__ == "__main__":
    sys.exit(main())
