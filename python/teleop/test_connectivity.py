#!/usr/bin/env python3
"""
Test script to verify ZeroMQ telemetry and motor control connectivity.
Tests both rover and arm modules independently.
"""

import json
import sys
import time
import zmq
import argparse
from pathlib import Path

def test_module(module_name, cmd_addr, telem_addr, timeout_ms=2000):
    """Test connectivity to a single module."""
    print(f"\n{'='*60}")
    print(f"Testing {module_name.upper()} module")
    print(f"  Command:   {cmd_addr}")
    print(f"  Telemetry: {telem_addr}")
    print(f"{'='*60}\n")

    ctx = zmq.Context()

    # Setup command socket (PUSH)
    cmd_sock = ctx.socket(zmq.PUSH)
    cmd_sock.connect(cmd_addr)
    print(f"[OK] Connected to command socket")

    # Setup telemetry socket (SUB)
    telem_sock = ctx.socket(zmq.SUB)
    telem_sock.connect(telem_addr)
    telem_sock.setsockopt_string(zmq.SUBSCRIBE, "")
    telem_sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    print(f"[OK] Connected to telemetry socket")

    # Test 1: Listen for telemetry
    print(f"\n[1/3] Listening for telemetry (timeout: {timeout_ms}ms)...")
    try:
        raw = telem_sock.recv_string()
        telem = json.loads(raw)
        print(f"[OK] Received telemetry!")
        print(f"  Timestamp: {telem.get('timestamp', 'N/A')}")
        if 'motors' in telem:
            print(f"  Motors found: {len(telem['motors'])}")
            for motor_id, data in list(telem['motors'].items())[:3]:  # Show first 3
                state = data.get('state', 'unknown')
                pos = data.get('position', 0.0)
                vel = data.get('velocity', 0.0)
                temp = data.get('temperature', 0.0)
                print(f"    {motor_id}: {state} pos={pos:+.3f} vel={vel:+.3f} T={temp:.0f}C")
        else:
            print(f"  Warning: No 'motors' field in telemetry")
            print(f"  Raw data: {json.dumps(telem, indent=2)}")
    except zmq.Again:
        print(f"[FAIL] TIMEOUT - No telemetry received")
        print(f"  Possible issues:")
        print(f"    - Motor control process not running on {cmd_addr.split('//')[1].split(':')[0]}")
        print(f"    - Network connectivity problem")
        print(f"    - Wrong IP address or port")
        cmd_sock.close()
        telem_sock.close()
        ctx.term()
        return False
    except json.JSONDecodeError as e:
        print(f"[FAIL] Invalid JSON received: {e}")
        cmd_sock.close()
        telem_sock.close()
        ctx.term()
        return False

    # Test 2: Send a safe command (zero velocity drive)
    print(f"\n[2/3] Sending test command (zero velocity drive)...")
    test_cmd = {"type": "drive", "linear": 0.0, "angular": 0.0, "swivel": 0.0}
    cmd_sock.send_string(json.dumps(test_cmd))
    print(f"[OK] Command sent: {test_cmd}")

    # Test 3: Verify telemetry is still flowing
    print(f"\n[3/3] Verifying continuous telemetry (3 samples)...")
    samples = 0
    for i in range(3):
        try:
            raw = telem_sock.recv_string()
            telem = json.loads(raw)
            samples += 1
            timestamp = telem.get('timestamp', 0)
            motor_count = len(telem.get('motors', {}))
            print(f"  Sample {i+1}: timestamp={timestamp:.3f}, motors={motor_count}")
            time.sleep(0.05)  # Small delay between samples
        except zmq.Again:
            print(f"  Sample {i+1}: TIMEOUT")
            break

    if samples == 3:
        print(f"[OK] Telemetry flowing continuously")
    else:
        print(f"[WARN] Only received {samples}/3 samples")

    # Cleanup
    cmd_sock.close()
    telem_sock.close()
    ctx.term()

    print(f"\n{'='*60}")
    print(f"{module_name.upper()} module test: {'PASS' if samples > 0 else 'FAIL'}")
    print(f"{'='*60}\n")

    return samples > 0

def main():
    parser = argparse.ArgumentParser(description="Test AIZEE motor control connectivity")
    parser.add_argument(
        "--config",
        default=None,
        help="Path to teleop.yaml config file"
    )
    parser.add_argument(
        "--module",
        choices=["rover", "arm", "all"],
        default="all",
        help="Which module to test (default: all)"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=2000,
        help="Telemetry timeout in ms (default: 2000)"
    )
    args = parser.parse_args()

    # Load config
    if args.config is None:
        # Try to find config relative to script
        script_dir = Path(__file__).parent
        config_path = script_dir / ".." / ".." / "config" / "teleop.yaml"
        if not config_path.exists():
            config_path = Path("config/teleop.yaml")
        if not config_path.exists():
            print("Error: Cannot find config/teleop.yaml")
            print("Use --config to specify path")
            return 1
    else:
        config_path = Path(args.config)

    print(f"Loading config from: {config_path}")

    import yaml
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    # Check for multi-module config
    if "modules" not in cfg:
        print("Error: Config does not have 'modules' section")
        print("This test requires multi-module configuration")
        return 1

    modules = cfg["modules"]

    # Test requested modules
    results = {}

    if args.module in ["rover", "all"]:
        if "rover" in modules:
            results["rover"] = test_module(
                "rover",
                modules["rover"]["command"],
                modules["rover"]["telemetry"],
                args.timeout
            )
        else:
            print("Warning: 'rover' module not found in config")

    if args.module in ["arm", "all"]:
        if "arm" in modules:
            results["arm"] = test_module(
                "arm",
                modules["arm"]["command"],
                modules["arm"]["telemetry"],
                args.timeout
            )
        else:
            print("Warning: 'arm' module not found in config")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for module, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {module.capitalize()}: {status}")
    print("="*60)

    # Return 0 if all tests passed, 1 otherwise
    return 0 if all(results.values()) else 1

if __name__ == "__main__":
    sys.exit(main())
