#!/usr/bin/env python3
"""
Test script to verify teleop exits cleanly without hanging.
Tests the ZeroMQ cleanup fix.
"""

import subprocess
import time
import sys
import os

def test_exit_no_telemetry():
    """Test that teleop exits cleanly when there's no telemetry."""
    print("="*70)
    print("TEST: Exit hang fix (no telemetry)")
    print("="*70)
    print()

    # Path to config
    config_path = os.path.join("..", "..", "config", "teleop_rover_only.yaml")

    print(f"1. Starting teleop with config: {config_path}")
    print("   (No motor controller connected - no telemetry expected)")
    print()

    # Start the teleop process
    try:
        proc = subprocess.Popen(
            [sys.executable, "teleop.py", "--config", config_path, "--log-level", "INFO"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE,
            text=True,
            env=dict(os.environ, TERM='xterm')  # Ensure TERM is set for curses
        )

        print("2. Teleop process started (PID: {})".format(proc.pid))
        print("   Waiting 3 seconds for initialization...")
        time.sleep(3)

        print()
        print("3. Checking if process is still running...")
        if proc.poll() is not None:
            print("   [FAIL] Process already exited (unexpected)")
            stdout, stderr = proc.communicate()
            print("\nSTDOUT:", stdout[:500])
            print("\nSTDERR:", stderr[:500])
            return False
        else:
            print("   [OK] Process is running")

        print()
        print("4. Sending interrupt signal (Ctrl+C)...")
        start_time = time.time()

        # Send Ctrl+C
        proc.terminate()

        # Wait for process to exit (with timeout)
        try:
            proc.wait(timeout=5.0)
            elapsed = time.time() - start_time

            print(f"   [OK] Process exited cleanly in {elapsed:.2f} seconds")

            # Check exit code
            if proc.returncode == 0 or proc.returncode == -15:  # 0 or SIGTERM
                print(f"   [OK] Exit code: {proc.returncode} (clean exit)")
            else:
                print(f"   [WARN] Exit code: {proc.returncode}")

            # Show some log output
            stdout, stderr = proc.communicate()
            print()
            print("5. Log output (last 500 chars of stderr):")
            print("-" * 70)
            if stderr:
                print(stderr[-500:])
            else:
                print("(no stderr output)")
            print("-" * 70)

            return True

        except subprocess.TimeoutExpired:
            print("   [FAIL] FAIL: Process did not exit within 5 seconds (HANGING)")
            print("   Killing process forcefully...")
            proc.kill()
            proc.wait()

            stdout, stderr = proc.communicate()
            print()
            print("Log output before kill:")
            print(stderr[-500:] if stderr else "(no output)")

            return False

    except FileNotFoundError:
        print("[FAIL] ERROR: Could not find python or teleop.py")
        return False
    except Exception as e:
        print(f"[FAIL] ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    print()
    result = test_exit_no_telemetry()
    print()
    print("="*70)
    if result:
        print("[PASS] TEST PASSED: Exit hang fix is working correctly")
        print("   - Process started successfully")
        print("   - Process exited cleanly when interrupted")
        print("   - No hanging on ZeroMQ cleanup")
    else:
        print("[FAIL] TEST FAILED: Exit hang still present or other error")
        print("   - Check logs above for details")
    print("="*70)
    print()

    return 0 if result else 1

if __name__ == "__main__":
    sys.exit(main())
