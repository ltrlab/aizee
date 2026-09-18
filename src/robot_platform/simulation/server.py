"""Loopback-only protocol server; all endpoints are bound before publication."""
from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import platform
import subprocess
import time
import uuid

from robot_platform.robots import load_robot
from robot_platform.settings import load_settings, validate_settings
from robot_platform.transport.wire import pack_msg, unpack_msg, pack_camera
from robot_platform.data.traces import TraceWriter
from .plant import SimulatedRobot


SCENARIOS = ("nominal", "feedback-loss", "frozen-feedback", "malformed-feedback", "camera-loss", "emergency-stop")


def source_identity():
    root = Path(__file__).resolve().parents[3]
    result = {"revision": None, "source_sha256": None}
    if (root / "Cargo.toml").is_file():
        try:
            result["revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
            names = subprocess.check_output(["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=root).decode().split("\0")
            digest = hashlib.sha256()
            for name in sorted(set(names) - {""}):
                path = root / name
                if path.is_file():
                    digest.update(name.encode() + b"\0" + hashlib.sha256(path.read_bytes()).digest())
            result["source_sha256"] = digest.hexdigest()
        except (OSError, subprocess.SubprocessError):
            pass
    return result


class ProtocolServer:
    def __init__(self, robot, output, *, seed=0, scenario="nominal", cameras=True):
        if scenario not in SCENARIOS:
            raise ValueError("unknown scenario")
        self.spec = load_robot(robot)
        self.output = Path(output).resolve()
        # Never overwrite a run or mix simulated episodes into an existing run.
        self.output.mkdir(parents=True, exist_ok=False)
        self.context = None
        self.sockets = []
        self.trace = None
        self.scenario = scenario
        self.cameras_enabled = cameras
        self.scenario_applied = False
        self.count = 0
        self.rejected = 0
        self.overruns = 0
        self.seed = seed

    def __enter__(self):
        import zmq
        self.zmq = zmq
        self.context = zmq.Context()
        try:
            settings = load_settings(self.spec, overrides={"host": "127.0.0.1"})
            def socket(kind):
                sock = self.context.socket(kind)
                self.sockets.append(sock)
                sock.setsockopt(zmq.LINGER, 0)
                sock.setsockopt(zmq.SNDHWM, 10)
                sock.setsockopt(zmq.RCVHWM, 100)
                port = sock.bind_to_random_port("tcp://127.0.0.1")
                return sock, f"tcp://127.0.0.1:{port}"
            self.arms = {}
            for arm, endpoint in settings["endpoints"]["arms"].items():
                pull, endpoint["command"] = socket(zmq.PULL)
                pub, endpoint["telemetry"] = socket(zmq.PUB)
                self.arms[arm] = (pull, pub)
            self.cameras = {}
            for role in settings["endpoints"]["cameras"]:
                pub, settings["endpoints"]["cameras"][role] = socket(zmq.PUB)
                self.cameras[role] = pub
            for service in settings["endpoints"]["services"]:
                _, settings["endpoints"]["services"][service] = socket(zmq.PUB)
            settings["control"].update(require_source_freshness=True, target_lease_s=0.5)
            settings["applications"]["collect"]["output_dir"] = str(self.output / "episodes")
            settings["applications"]["train"]["data_dir"] = str(self.output / "episodes")
            settings["applications"]["train"]["output_dir"] = str(self.output / "checkpoints")
            validate_settings(self.spec, settings)
            self.settings = settings
            self.run_id = str(uuid.uuid4())
            settings["simulation"] = {"run_id": self.run_id, "evidence": "synthetic"}
            validate_settings(self.spec, settings)
            self.plant = SimulatedRobot(self.spec, settings, seed=self.seed, producer_id=self.run_id)
            self.trace = TraceWriter(self.output / "trace.h5", clock_domain=f"host-monotonic:{self.run_id}")
            if self.cameras_enabled:
                from PIL import Image, ImageDraw
                self.Image, self.ImageDraw = Image, ImageDraw
            self.manifest = {"schema_version": 1, "run_id": self.run_id, "robot": self.spec.to_dict(),
                             "settings": settings, "scenario": self.scenario, "seed": self.seed,
                             "evidence": "synthetic", "physical_torque_validated": False,
                             "parameters": self.plant.parameters, "platform": platform.platform(),
                             "python": platform.python_version(), "source": source_identity(),
                             "clock": "host-monotonic plus explicit simulation time", "status": "running"}
            (self.output / "settings.json").write_text(json.dumps(settings, indent=2), encoding="utf-8")
            (self.output / "manifest.json").write_text(json.dumps(self.manifest, indent=2), encoding="utf-8")
            self.trace.record("run_start", self.manifest)
            return self
        except Exception:
            self.close(status="failed")
            raise

    def cycle(self, dt=0.02):
        for arm, (pull, _) in self.arms.items():
            # Bounded draining: a busy producer cannot starve telemetry or peers.
            for _ in range(64):
                try:
                    raw = pull.recv(self.zmq.NOBLOCK)
                except self.zmq.Again:
                    break
                self.trace.record("command_receive", {"arm": arm, "raw_hex": raw.hex()})
                try:
                    self.plant.command(arm, unpack_msg(raw))
                except (ValueError, TypeError, KeyError) as exc:
                    self.rejected += 1
                    self.trace.record("command_rejected", {"arm": arm, "reason": str(exc)})
        if self.plant.t >= 1.0 and not self.scenario_applied:
            first = next(iter(self.arms))
            if self.scenario == "feedback-loss":
                self.plant.drop_arms.add(first)
            elif self.scenario == "frozen-feedback":
                self.plant.joints[self.plant.session.arms[first].joints[0]].frozen = True
            elif self.scenario == "malformed-feedback":
                self.plant.malformed_arms.add(first)
            elif self.scenario == "camera-loss":
                self.cameras_enabled = False
            elif self.scenario == "emergency-stop":
                for arm in self.arms:
                    self.plant.command(arm, {"type": "emergency_stop"})
            self.scenario_applied = True
            self.trace.record("scenario", {"name": self.scenario, "simulation_time_s": self.plant.t})
        self.plant.step(dt)
        for arm, (_, pub) in self.arms.items():
            msg = self.plant.telemetry(arm)
            if msg is not None:
                msg.setdefault("simulation", {})["time_s"] = self.plant.t
                msg["timestamp"] = time.time()  # retain epoch timestamps for existing apps
                pub.send(pack_msg(msg), self.zmq.NOBLOCK)
                self.trace.record("telemetry_publish", {"arm": arm, "message": msg})
        self.trace.record("plant_state", self.plant.snapshot())
        if self.cameras_enabled and self.count % 2 == 0:
            for role, pub in self.cameras.items():
                frame = self.Image.new("RGB", (640, 480), (24, 36, (self.count * 7) % 255))
                self.ImageDraw.Draw(frame).text((16, 16), f"SIMULATION {self.spec.name} {role} frame {self.count}", fill="white")
                buf = io.BytesIO()
                frame.save(buf, format="JPEG", quality=80)
                msg = {"camera_id": role, "timestamp": time.time(), "frame_number": self.count,
                       "simulation": True, "color": {"format": "jpeg", "width": 640, "height": 480}}
                pub.send_multipart(pack_camera(msg, buf.getvalue()), self.zmq.NOBLOCK)
        self.count += 1

    def run(self, duration=0.0):
        if not isinstance(duration, (int, float)) or not 0 <= duration < float("inf"):
            raise ValueError("duration must be finite and nonnegative")
        started = previous = time.monotonic()
        deadline = started
        while not duration or time.monotonic() - started < duration:
            now = time.monotonic()
            elapsed = now - previous
            if elapsed > 1.0:
                raise RuntimeError("simulation fell more than one second behind wall time")
            if elapsed > 0.04:
                self.overruns += 1
            self.cycle(max(0.000001, elapsed))
            previous = now
            deadline += 0.02
            time.sleep(max(0, deadline - time.monotonic()))

    def close(self, status="completed"):
        try:
            if self.trace is not None:
                self.trace.close()
        finally:
            for sock in self.sockets:
                sock.close(linger=0)
            self.sockets.clear()
            if self.context is not None:
                self.context.term()
                self.context = None
        if hasattr(self, "manifest"):
            self.manifest.update(status=status, cycles=self.count, rejected_commands=self.rejected,
                                 deadline_overruns=self.overruns, trace_drops=self.trace.dropped)
            (self.output / "manifest.json").write_text(json.dumps(self.manifest, indent=2), encoding="utf-8")

    def __exit__(self, kind, *_):
        self.close("failed" if kind and kind is not KeyboardInterrupt else "completed")
