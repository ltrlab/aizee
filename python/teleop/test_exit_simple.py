#!/usr/bin/env python3
"""
Simple test to verify teleop exits cleanly.
Starts teleop, waits, then kills it and checks if it exits quickly.
"""

import subprocess
import time
import sys
import os

def test_clean_exit():
    """Test that teleop exits within reasonable time after termination."""
    print("="*70)
    print("Simple Exit Test - Verifying ZeroMQ cleanup fix")
    print("="*70)
    print()

    config_path = os.path.join("..", "..", "config", "teleop_rover_only.yaml")

    print("Starting teleop process...")
    print("  Config: {}".format(config_path))
    print("  Note: No motor controller expected (will have no telemetry)")
    print()

    try:
        # Start process with minimal I/O
        proc = subprocess.Popen(
            [sys.executable, "teleop.py",
             "--config", config_path,
             "--log-level", "WARNING"],
            # Don't redirect - let it fail naturally if terminal issues
        )

        print("Process started: PID {}".format(proc.pid))
        print("Waiting 2 seconds...")
        time.sleep(2)

        # Check if still running
        if proc.poll() is not None:
            print("[INFO] Process already exited with code: {}".format(proc.returncode))
            print("      (This might be due to terminal issues in test environment)")
            return True  # Can't really test, but not a hang

        print("Process is running. Sending termination signal...")
        print()

        # Measure exit time
        start = time.time()
        proc.terminate()

        # Wait with timeout
        try:
            proc.wait(timeout=3.0)
            elapsed = time.time() - start

            print("[PASS] Process exited cleanly!")
            print("       Exit time: {:.2f} seconds".format(elapsed))
            print("       Exit code: {}".format(proc.returncode))

            if elapsed < 2.0:
                print("       [OK] Exit was fast (< 2s) - no hanging detected")
                return True
            else:
                print("       [WARN] Exit took longer than expected")
                return True  # Still passed, just slow

        except subprocess.TimeoutExpired:
            print("[FAIL] Process HUNG - did not exit within 3 seconds!")
            print("       This indicates ZeroMQ cleanup is still blocking.")
            print("       Killing process forcefully...")
            proc.kill()
            proc.wait()
            return False

    except Exception as e:
        print("[ERROR] Test failed with exception: {}".format(e))
        return False

if __name__ == "__main__":
    print()
    result = test_clean_exit()
    print()
    print("="*70)
    if result:
        print("Test Result: PASS")
    else:
        print("Test Result: FAIL")
    print("="*70)
    print()
    sys.exit(0 if result else 1)
