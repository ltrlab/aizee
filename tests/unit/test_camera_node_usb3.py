"""The scene camera stays on USB 3 (python/nodes/camera_node.py, Usb3Keeper): a camera found at USB 2 is reset, never
streamed (on Minerva the USB 2 side is the CAN adapters' hub), with backoff while resets do not hold and none while a
live receiver runs; a camera that is gone never holds up the exit; systemd starts the node again. No SDK or device:
a fake SDK and clock (Minerva's D435 dropped its USB 3 link six times on 2026-10-24/25)."""

import configparser
import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
INFO = SimpleNamespace(usb_type_descriptor='usb', serial_number='serial')


class Camera:
    def __init__(self, bus, usb, serial='919122072056'):
        self.bus, self.usb, self.serial, self.resets = bus, usb, serial, 0

    def get_info(self, key):
        return {'usb': self.usb, 'serial': self.serial}[key]

    def hardware_reset(self):
        self.resets += 1
        self.bus.reset(self)


class Bus:
    """What the SDK enumerates; a reset detaches the camera, which re-attaches at `reattach_usb` after `delay_s`."""

    def __init__(self, clock, reattach_usb='3.2', delay_s=2.0):
        self.clock, self.reattach_usb, self.delay_s = clock, reattach_usb, delay_s
        self.cameras, self.pending = [], None

    def query(self):
        if self.pending is not None and self.clock() >= self.pending[0]:
            self.cameras, self.pending = [self.pending[1]], None
        return list(self.cameras)

    def reset(self, camera):
        self.cameras = []
        self.pending = (self.clock() + self.delay_s, Camera(self, self.reattach_usb, camera.serial))


class Clock:
    def __init__(self):
        self.now, self.slept = 1000.0, 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.slept += seconds


def load_node(monkeypatch, bus):
    # The SDK is stubbed before loading the publisher: nothing here opens a device, service or socket.
    import sys

    monkeypatch.setitem(sys.modules, 'pyrealsense2', SimpleNamespace())
    monkeypatch.syspath_prepend(str(ROOT / 'python'))
    spec = importlib.util.spec_from_file_location('camera_usb3_under_test', ROOT / 'python/nodes/camera_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pipelines = []
    module.rs = SimpleNamespace(
        context=lambda: SimpleNamespace(query_devices=bus.query),
        camera_info=INFO,
        pipeline=lambda: pipelines.append(1) or pytest.fail('a camera at USB 2 was streamed'),
    )
    return module


@pytest.fixture
def rig(monkeypatch, tmp_path):
    clock = Clock()
    bus = Bus(clock)
    module = load_node(monkeypatch, bus)
    live = []

    def node(**kwargs):
        keeper = module.Usb3Keeper(
            tmp_path / 'usb3.json', clock=clock, sleep=clock.sleep, live=lambda: bool(live) and live.pop(0)
        )
        return module.CameraNode(camera_id='scene_cam', usb3_keeper=keeper, **kwargs)

    return SimpleNamespace(module=module, clock=clock, bus=bus, node=node, live=live, state=tmp_path / 'usb3.json')


def test_a_camera_at_usb3_streams_from_that_camera(rig):
    rig.bus.cameras = [Camera(rig.bus, '3.2')]
    node = rig.node()
    assert node.ensure_usb3() is True and node.device_serial == '919122072056' and rig.clock.slept == 0


def test_a_camera_that_cannot_report_its_link_still_streams(rig):
    camera = Camera(rig.bus, '3.2')

    def get_info(key):
        if key == 'usb':
            raise RuntimeError('usb_type_descriptor is not supported')
        return camera.serial

    camera.get_info = get_info
    rig.bus.cameras = [camera]
    assert rig.node().ensure_usb3() is True and camera.resets == 0


def test_a_camera_at_usb2_is_reset_never_streamed_and_comes_back_at_usb3(rig):
    camera = Camera(rig.bus, '2.1')
    rig.bus.cameras = [camera]
    node = rig.node()
    assert node.run() == rig.module.RETRAIN_EXIT  # Reset sent; systemd starts a fresh process.
    assert camera.resets == 1 and node.pipeline is None
    # The next process finds the camera re-attaching, waits for it, and streams from it at USB 3.
    again = rig.node()
    assert again.ensure_usb3() is True and again.device_serial == '919122072056'
    assert rig.clock.slept == pytest.approx(2.0, abs=rig.module.POLL_S)
    assert '"failures": 0' in rig.state.read_text()  # The reset held.


def test_resets_that_do_not_hold_back_off(rig):
    rig.bus.reattach_usb = '2.1'  # The link will not retrain: the camera keeps coming back at USB 2.
    rig.bus.cameras = [Camera(rig.bus, '2.1')]
    resets = []
    for _ in range(5):
        assert rig.node().ensure_usb3() is False
        resets.append(rig.clock.now)
    gaps = [round(b - a) for a, b in zip(resets, resets[1:])]
    assert gaps == [30, 120, 600, 600]  # RETRAIN_BACKOFF_S, then its cap.


def test_a_camera_replugged_at_usb3_during_the_backoff_streams_at_once(rig):
    # 12 resets on the old cable left a 10-minute backoff; the head camera went onto a new cable 30 s into it and
    # stayed dark until the backoff ran out (2026-10-29).
    rig.state.write_text(json.dumps({'failures': 12, 'last_reset': rig.clock.now - 11.0}))
    rig.bus.cameras = [old := Camera(rig.bus, '2.1')]
    replugged = rig.clock.now + 30.0
    rig.bus.pending = (replugged, Camera(rig.bus, '3.2'))
    node = rig.node()
    assert node.ensure_usb3() is True and old.resets == 0
    assert rig.clock.now - replugged <= rig.module.LOOK_S
    assert '"failures": 0' in rig.state.read_text()


def test_a_camera_unplugged_during_the_backoff_is_looked_for_again_never_reset(rig):
    rig.state.write_text(json.dumps({'failures': 3, 'last_reset': rig.clock.now - 11.0}))
    old, new = Camera(rig.bus, '2.1'), Camera(rig.bus, '3.2')
    start = rig.clock.now
    timeline = ((start + 20.0, []), (start + 50.0, [new]))

    def query():
        cameras = [old]
        for at, present in timeline:
            if rig.clock.now >= at:
                cameras = present
        return list(cameras)

    rig.bus.query = query
    node = rig.node()
    assert node.ensure_usb3() is True and old.resets == 0 and node.device_serial == '919122072056'
    assert rig.clock.now - (start + 50.0) <= rig.module.POLL_S


def test_a_camera_back_at_usb3_while_a_live_receiver_runs_is_not_reset(rig):
    rig.bus.cameras = [old := Camera(rig.bus, '2.1')]
    rig.live.extend([True] * 10)
    rig.bus.pending = (rig.clock.now + 7.0, Camera(rig.bus, '3.2'))
    assert rig.node().ensure_usb3() is True and old.resets == 0


def test_no_reset_while_a_live_receiver_runs(rig):
    rig.bus.cameras = [camera := Camera(rig.bus, '2.1')]
    rig.live.extend([True, True, True])  # A live arm session for three polls.
    assert rig.node().ensure_usb3() is False
    assert camera.resets == 1 and rig.clock.slept == pytest.approx(2 * rig.module.LIVE_POLL_S)


def test_with_no_camera_the_node_waits_then_fails_for_systemd_to_retry(rig):
    node = rig.node()
    assert node.run() == 1 and rig.clock.slept == pytest.approx(rig.module.NO_CAMERA_WAIT_S, abs=rig.module.POLL_S)


def test_the_live_receiver_check_reads_process_names(tmp_path, monkeypatch):
    module = load_node(monkeypatch, Bus(Clock()))
    for pid, argv0 in ((12, b'/usr/bin/python3'), (34, b'/home/ltr/minerva/current/bin/guarded_controller'), (56, b'')):
        (tmp_path / str(pid)).mkdir()
        (tmp_path / str(pid) / 'cmdline').write_bytes(argv0 + b'\0--config\0x')
    (tmp_path / 'self').mkdir()
    assert module._live_receiver_running(tmp_path) is True
    (tmp_path / '34' / 'cmdline').write_bytes(b'/usr/bin/control_harness\0')  # A virtual session's harness.
    assert module._live_receiver_running(tmp_path) is False
    assert module._live_receiver_running(tmp_path / 'missing') is False


def test_a_camera_gone_mid_stream_cannot_hold_up_the_exit(rig, monkeypatch):
    rig.bus.cameras = [Camera(rig.bus, '3.2')]
    monkeypatch.setattr(rig.module, 'STOP_TIMEOUT_S', 0.2)
    hung = threading.Event()
    node = rig.node()
    node.ensure_usb3()

    def lost(**_kwargs):
        raise RuntimeError("Frame didn't arrive within 5000")

    node.pipeline = SimpleNamespace(wait_for_frames=lost, stop=lambda: hung.wait(30))
    node.initialize_camera = node.initialize_zmq = lambda: None
    monkeypatch.setattr(rig.module.time, 'sleep', lambda _s: None)
    started = time.monotonic()
    assert node.run() == 1  # A capture error: systemd starts a fresh process.
    assert time.monotonic() - started < 2.0
    hung.set()


def test_config_and_unit(monkeypatch):
    module = load_node(monkeypatch, Bus(Clock()))
    assert module._kwargs_from_config({})['require_usb3'] is True
    assert module._kwargs_from_config({'camera': {'require_usb3': False}})['require_usb3'] is False
    unit = configparser.ConfigParser(strict=False, interpolation=None)
    unit.read(ROOT / 'config/systemd/aizee-scene-cam.service', encoding='utf-8')
    assert 'BindsTo' not in unit['Unit'] and unit['Unit']['StartLimitIntervalSec'] == '0'
    assert unit['Service']['Restart'] == 'always'
    # A stop outlasts the node's frame wait (5 s) and its bounded pipeline stop.
    assert float(unit['Service']['TimeoutStopSec']) > 5 + module.STOP_TIMEOUT_S
