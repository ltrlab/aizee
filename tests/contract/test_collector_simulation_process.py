"""Start the real application main loops against loopback simulation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from robot_platform.apps.launcher import build_launch_plan
from robot_platform.simulation.server import ProtocolServer

REPO = Path(__file__).resolve().parents[2]
# Interrupt after receiving live data so the applications exercise their normal
# finally/shutdown paths. An independent timeout also covers startup failures.
WRAPPER = r'''
import _thread, json, os, runpy, sys, threading
from robot_platform.control import RobotSession
import serial
def forbidden(*args, **kwargs):
    print("HARDWARE_OPEN_ATTEMPT", flush=True)
    raise SystemExit("simulation attempted serial I/O")
serial.Serial = forbidden
observed = threading.Event()
original = RobotSession.observation
timers = []
def observe(self, *args, **kwargs):
    result = original(self, *args, **kwargs)
    if result.ready and not observed.is_set():
        observed.set()
        print("SIMULATION_OBSERVATION_READY", flush=True)
        timer = threading.Timer(2, _thread.interrupt_main)
        timers.append(timer)
        timer.start()
    return result
RobotSession.observation = observe
sys.argv = json.loads(os.environ["SMOKE_ARGV"])
timeout = threading.Timer(20, _thread.interrupt_main)
timeout.start()
try:
    runpy.run_path(sys.argv[0], run_name="__main__")
finally:
    timeout.cancel()
    for timer in timers:
        timer.cancel()
'''


@pytest.mark.integration
@pytest.mark.parametrize("robot", ["minerva", "aizee"])
def test_real_collector_connects_without_hardware_and_shuts_down(robot, tmp_path, monkeypatch):
    for dependency in ("cv2", "serial", "zmq", "PIL", "h5py", "yaml"):
        pytest.importorskip(dependency)
    monkeypatch.setattr("robot_platform.simulation.server.source_identity", lambda: {"test": True})
    with ProtocolServer(robot, tmp_path / robot) as server:
        server.settings["applications"]["collect"]["gui"] = False
        plan = build_launch_plan(server.spec, server.settings, "collect", repo=REPO)
        env = {**os.environ, "ROBOT_PLATFORM_SETTINGS_JSON": plan.settings_json,
               "SMOKE_ARGV": json.dumps(plan.argv[1:]), "PYTHONIOENCODING": "utf-8",
               "SDL_VIDEODRIVER": "dummy", "SDL_AUDIODRIVER": "dummy"}
        path = tmp_path / "collector.log"
        with path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen([sys.executable, "-u", "-c", WRAPPER], cwd=REPO, env=env,
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                started = previous = time.monotonic()
                while process.poll() is None and time.monotonic() - started < 30:
                    now = time.monotonic()
                    server.cycle(max(0.000001, now - previous))
                    previous = now
                    time.sleep(0.02)
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait(timeout=5)
        output = path.read_text(encoding="utf-8")
        assert process.returncode == 0, output[-6000:]
        assert "SIMULATION_OBSERVATION_READY" in output, output[-6000:]
        assert "HARDWARE_OPEN_ATTEMPT" not in output
        assert server.rejected == 0
