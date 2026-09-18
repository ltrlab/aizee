#!/usr/bin/env python3
"""Exercise real simulator processes over loopback; never opens robot devices.

Run from an installed checkout: python scripts/validate_simulation.py --output
simulation_runs/validation-001. Output must be a new directory. Two independent
robot processes run concurrently; each has its own dynamically allocated ports.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from robot_platform.control import RobotSession, ThreadedRobotClient
from robot_platform.control.session import PartialSendError
from robot_platform.data.traces import read_events
from robot_platform.robots import load_robot
from robot_platform.settings import load_settings
from robot_platform.simulation.server import SCENARIOS
from robot_platform.transport.wire import unpack_camera, unpack_msg

ROOT = Path(__file__).resolve().parents[1]


def run_case(output: Path, robot: str, scenario: str) -> dict:
    import zmq
    from PIL import Image

    case = output / f"{robot}-{scenario}"
    duration = 15 if scenario == "nominal" else 6
    result = {"robot": robot, "scenario": scenario, "requested_duration_s": duration,
              "checks": {}, "passed": False, "run_directory": str(case)}
    checks = result["checks"]
    client = context = process = None
    cameras = {}
    pause_start = resume_at = None
    enabled = stopped = cleared = False
    initial_ready = False
    last_ready = {}
    command_times = {}
    samples = {}
    frame_counts = {}
    last_camera = {}
    first_image = {}
    previous_frames = {}
    camera_ordered = True
    unexpected_errors = []
    backpressure_events = 0
    previous_error = None

    def inspect_error(error, elapsed):
        nonlocal previous_error, backpressure_events
        if error is None or error is previous_error:
            return
        previous_error = error
        if "first_client_error" not in result:
            result["first_client_error"] = {"elapsed_s": elapsed, "error": repr(error),
                                            "cause": repr(error.__cause__)}
        # SNDHWM=1 deliberately bounds the queue. EAGAIN is a recoverable
        # nonblocking-send result; the worker retains diagnostics even after
        # recovery. Service delivery deadline failures remain fatal here.
        if isinstance(error, PartialSendError) and isinstance(error.__cause__, zmq.Again):
            backpressure_events += 1
        else:
            unexpected_errors.append({"elapsed_s": elapsed, "error": repr(error)})
    try:
        with (output / f"{robot}-{scenario}.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                [sys.executable, "-u", "-m", "robot_platform", "sim", "run", "--robot", robot,
                 "--output", str(case), "--duration", str(duration), "--scenario", scenario, "--seed", "12"],
                cwd=ROOT, env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                stdout=log, stderr=subprocess.STDOUT)
            setup_deadline = time.monotonic() + 30
            while not (case / "settings.json").is_file():
                if process.poll() is not None or time.monotonic() > setup_deadline:
                    raise RuntimeError("simulator did not publish settings before startup deadline")
                time.sleep(0.02)
            spec = load_robot(robot)
            # The manifest is written immediately after settings; parsing may
            # race the final file write, so wait for the first complete JSON.
            while True:
                try:
                    settings = load_settings(spec, [case / "settings.json"])
                    break
                except ValueError:
                    if time.monotonic() > setup_deadline:
                        raise
                    time.sleep(0.02)
            session = RobotSession(spec, settings)
            client = ThreadedRobotClient(session).start()
            context = zmq.Context()
            for role, endpoint in settings["endpoints"]["cameras"].items():
                sock = context.socket(zmq.SUB)
                sock.setsockopt(zmq.SUBSCRIBE, b"")
                sock.setsockopt(zmq.RCVHWM, 10)
                sock.setsockopt(zmq.LINGER, 0)
                sock.connect(endpoint)
                cameras[role] = sock
                frame_counts[role] = 0
            arm_names = list(session.arms)
            first_arm = arm_names[0]
            started = time.monotonic()
            while process.poll() is None and time.monotonic() - started < duration + 5:
                now = time.monotonic()
                elapsed = now - started
                observation = client.snapshot()
                last_ready = dict(client.arm_ready())
                if observation.ready and not enabled:
                    initial_ready = True
                    initial = [client.telemetry(arm) for arm in arm_names]
                    checks["starts_disabled"] = all(
                        message and all(motor["state"] == "disabled" for motor in message["motors"].values())
                        for message in initial)
                    client.submit({arm: {"type": "enable", "motor_ids": list(ep.joints)}
                                   for arm, ep in session.arms.items()})
                    enabled = True
                inspect_error(client.error, elapsed)
                # Stop renewals while feedback remains healthy, then explicitly
                # submit a new target. The trace checks actual received frames.
                paused = scenario == "nominal" and 3 <= elapsed < 4.5
                if paused and pause_start is None:
                    pause_start = now
                if pause_start is not None and elapsed >= 4.5 and resume_at is None:
                    resume_at = now
                if scenario == "nominal" and elapsed >= 10 and not stopped:
                    client.submit({arm: {"type": "emergency_stop"} for arm in arm_names})
                    stopped = True
                if scenario == "nominal" and elapsed >= 11 and not cleared:
                    client.submit({arm: {"type": "clear_emergency_stop"} for arm in arm_names})
                    cleared = True
                if enabled and not paused:
                    target = [0.12 * math.sin(elapsed * 0.7) * (-1 if joint.name.startswith("right") else 1)
                              if joint.active else 0.0 for joint in spec.joints]
                    client.set_target(target, kp=[3.0 if j.active else 0 for j in spec.joints],
                                      kd=[0.5 if j.active else 0 for j in spec.joints])
                for role, sock in cameras.items():
                    for _ in range(10):
                        try:
                            message = unpack_camera(sock.recv_multipart(zmq.NOBLOCK))
                        except zmq.Again:
                            break
                        number = message["frame_number"]
                        camera_ordered &= number > previous_frames.get(role, -1)
                        previous_frames[role] = number
                        frame_counts[role] += 1
                        last_camera[role] = elapsed
                        if role not in first_image:
                            image = Image.open(io.BytesIO(message["color"]["data_bytes"]))
                            image.load()
                            first_image[role] = list(image.size)
                time.sleep(0.02)
            if process.poll() is None:
                raise RuntimeError("simulator exceeded its bounded duration")
            checks["clean_process_exit"] = process.returncode == 0
        checks["initial_telemetry_ready"] = initial_ready
        inspect_error(client.error, time.monotonic() - started)
        checks["no_unexpected_transport_error"] = not unexpected_errors
        result["client_error"] = repr(client.error) if client.error is not None else None
        result["observed_backpressure_events"] = backpressure_events
        result["unexpected_transport_errors"] = unexpected_errors
        checks["camera_streams_decode"] = len(first_image) == len(cameras) and all(size == [640, 480] for size in first_image.values())
        checks["camera_frame_order"] = camera_ordered
        if scenario == "camera-loss":
            checks["camera_outage_detected"] = all(last_camera.get(role, duration) < 1.5 for role in cameras)
        else:
            checks["camera_streams_continue"] = all(last_camera.get(role, 0) > duration - 1 for role in cameras)
        expected = {arm: (arm != first_arm if scenario in ("feedback-loss", "frozen-feedback", "malformed-feedback") else True)
                    for arm in arm_names}
        checks["arm_freshness_matches_scenario"] = last_ready == expected
        manifest = json.loads((case / "manifest.json").read_text(encoding="utf-8"))
        checks["complete_manifest"] = manifest["status"] == "completed"
        checks["no_trace_drops"] = manifest["trace_drops"] == 0
        checks["no_rejected_commands"] = manifest["rejected_commands"] == 0
        final_plant = None
        motion_max = {name: 0.0 for name in spec.active_joint_names}
        for event in read_events(case / "trace.h5"):
            payload = event["payload"]
            if event["kind"] == "command_receive":
                message = unpack_msg(bytes.fromhex(payload["raw_hex"]))
                if message["type"] == "arm_joints":
                    command_times.setdefault(payload["arm"], []).append(event["time_s"])
            elif event["kind"] == "telemetry_publish":
                samples.setdefault(payload["arm"], []).append(event["time_s"])
            elif event["kind"] == "plant_state":
                final_plant = payload
                for name, joint in payload["joints"].items():
                    motion_max[name] = max(motion_max[name], abs(joint["q"]))
        checks["all_active_joints_moved"] = all(value > 0.001 for value in motion_max.values())
        checks["commands_delivered_after_startup_backpressure"] = all(
            any(stamp > started + 0.15 for stamp in command_times.get(arm, [])) for arm in arm_names)
        if scenario == "nominal":
            checks["producer_lease_expires"] = pause_start is not None and resume_at is not None and not any(
                pause_start + settings["control"]["target_lease_s"] + 0.1 < stamp < resume_at
                for stamps in command_times.values() for stamp in stamps)
            checks["explicit_renewal_resumes_commands"] = resume_at is not None and all(
                any(resume_at < stamp < resume_at + 1 for stamp in stamps) for stamps in command_times.values())
            checks["estop_clear_does_not_enable"] = cleared and final_plant is not None and all(
                not joint["enabled"] and joint["tau_applied"] == 0 for joint in final_plant["joints"].values())
        elif scenario == "emergency-stop":
            checks["estop_disables_all_joints"] = final_plant is not None and all(
                not joint["enabled"] and joint["tau_applied"] == 0 for joint in final_plant["joints"].values())
        elif scenario in ("feedback-loss", "frozen-feedback", "malformed-feedback"):
            checks["stale_arm_motion_stops"] = not any(stamp > started + 2.0 for stamp in command_times.get(first_arm, []))
            if len(arm_names) > 1:
                checks["healthy_peer_continues"] = any(stamp > started + 3 for stamp in command_times.get(arm_names[1], []))
        def rates(stamps):
            if len(stamps) < 2:
                return {"samples": len(stamps)}
            gaps = sorted((b-a)*1000 for a, b in zip(stamps, stamps[1:]))
            return {"samples": len(stamps), "mean_hz": (len(stamps)-1)/(stamps[-1]-stamps[0]),
                    "p99_gap_ms": gaps[min(len(gaps)-1, int(0.99*len(gaps)))], "max_gap_ms": max(gaps)}
        result.update(telemetry={arm: rates(stamps) for arm, stamps in samples.items()},
                      camera_frames=frame_counts, cycles=manifest["cycles"],
                      deadline_overruns=manifest["deadline_overruns"], trace_drops=manifest["trace_drops"],
                      final_arm_ready=last_ready, max_joint_excursion_rad=motion_max,
                      passed=all(checks.values()))
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                result["close_error"] = str(exc)
                result["passed"] = False
        for sock in cameras.values():
            sock.close(linger=0)
        if context is not None:
            context.term()
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
    (output / f"{robot}-{scenario}-result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"robot": robot, "scenario": scenario, "passed": result["passed"],
                      "failed_checks": [name for name, passed in checks.items() if not passed],
                      "error": result.get("error")}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--scenario", choices=SCENARIOS, action="append", help="select scenarios; defaults to all")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    # Bound CPU/load and process count; no virtual or physical CAN is involved.
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(run_case, output, robot, scenario)
                   for scenario in (args.scenario or SCENARIOS) for robot in ("minerva", "aizee")]
        results = [future.result() for future in futures]
    report = {"schema_version": 1, "evidence": "synthetic software validation", "hardware_access": False,
              "passed": all(r["passed"] for r in results), "results": results}
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
