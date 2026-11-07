"""Minerva's UPS in Helm: the robot-side watch script, the watcher's states, and Helm's part in a shutdown."""

import asyncio
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

import pytest

from robot_platform.helm import ups as helm_ups
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.ups import SILENT_S, STALE_S, UpsWatch

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'

ON_POWER = {
    'version': 1,
    'sensor': 'ok',
    'source': 'external',
    'charge': 'charging',
    'level': 'ok',
    'percent': 41,
    'voltage_v': 11.62,
    'current_a': 1.21,
    'power_w': 14.06,
    'cell_v': 3.873,
    'shutdown': None,
    'thresholds': {'low_percent': 25.0, 'shutdown_percent': 10.0, 'floor_cell_v': 3.35},
}
SHUTTING = dict(
    ON_POWER,
    source='battery',
    charge='discharging',
    level='critical',
    percent=9,
    current_a=-1.4,
    shutdown={
        'stage': 'pending',
        'reason': 'about 9% charge, at or under 10%',
        'since': 1.8e9,
        'poweroff_by': 1.8e9 + 90,
    },
)


def until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(0.01)


def test_the_robot_script_prints_the_state_with_its_age_by_the_robots_clock(tmp_path):
    path = tmp_path / 'state.json'
    now = time.time()
    path.write_text(
        json.dumps(dict(SHUTTING, updated_at=now - 3, shutdown=dict(SHUTTING['shutdown'], poweroff_by=now + 40)))
    )
    script = subprocess.Popen(
        [sys.executable, '-u', '-c', helm_ups.WATCH, str(path)], stdout=subprocess.PIPE, text=True
    )
    try:
        record = json.loads(script.stdout.readline())
        assert record['age_s'] == pytest.approx(3, abs=1) and record['poweroff_in_s'] == pytest.approx(40, abs=1)
        assert record['state']['shutdown']['stage'] == 'pending' and record['missing'] is False
        path.unlink()
        record = json.loads(script.stdout.readline())
        assert record == {'missing': True}
    finally:
        script.kill()
        script.wait(5)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_the_watcher_tells_ok_missing_stale_and_unreachable_apart():
    clock = Clock()
    watch = UpsWatch(lambda address: None, lambda: 'tether', clock=clock)
    assert watch.snapshot()['status'] == 'starting'
    watch.take({'state': dict(ON_POWER, updated_at=1.0), 'age_s': 1.2, 'missing': False})
    view = watch.snapshot()
    assert view['status'] == 'ok' and view['state']['percent'] == 41 and view['age_s'] == 1.2
    watch.take({'missing': True})
    assert watch.snapshot()['status'] == 'missing'
    watch.take({'state': dict(ON_POWER, updated_at=1.0), 'age_s': STALE_S + 1, 'missing': False})
    assert watch.snapshot()['status'] == 'stale' and watch.snapshot()['state'] is None
    watch.take({'state': dict(ON_POWER, source='battery', percent=4, updated_at=1.0), 'age_s': 0.5, 'missing': False})
    clock.now += SILENT_S + 1
    offline = watch.snapshot()
    assert offline['status'] == 'unreachable' and offline['state'] is None
    assert offline['last']['source'] == 'battery' and offline['last']['seen_s_ago'] == round(SILENT_S + 1)


def test_starts_are_refused_only_while_a_fresh_reading_announces_a_shutdown():
    clock = Clock()
    watch = UpsWatch(lambda address: None, lambda: 'tether', clock=clock)
    assert watch.refusal() is None
    watch.take({'state': dict(ON_POWER, updated_at=1.0), 'age_s': 1.0, 'missing': False})
    assert watch.refusal() is None
    watch.take({'state': SHUTTING, 'age_s': 1.0, 'missing': False})
    assert 'shutting down' in watch.refusal()
    watch.take({'state': dict(SHUTTING, shutdown=None), 'age_s': 1.0, 'missing': False})
    assert 'shutting down' in watch.refusal()  # Critical on battery: the announcement is a second away.
    clock.now += SILENT_S + 1
    assert watch.refusal() is None  # No fresh word from the robot: the robot itself refuses then.


def test_each_announced_shutdown_is_acted_on_once():
    calls = []
    watch = UpsWatch(lambda address: None, lambda: 'tether', on_shutdown=calls.append)
    for _ in range(3):
        watch.take({'state': SHUTTING, 'age_s': 1.0, 'missing': False})
    until(lambda: len(calls) == 1)
    watch.take({'state': dict(SHUTTING, updated_at=1.0), 'age_s': STALE_S + 5, 'missing': False})
    again = dict(SHUTTING, shutdown=dict(SHUTTING['shutdown'], since=1.8e9 + 600))
    watch.take({'state': again, 'age_s': 1.0, 'missing': False})
    until(lambda: len(calls) == 2)
    time.sleep(0.05)
    assert len(calls) == 2 and calls[1]['state']['shutdown']['since'] == 1.8e9 + 600


class Stream:
    """A robot line stream fed by the test."""

    def __init__(self):
        self.lines, self.closed = queue.Queue(), False

    def readline(self):
        line = self.lines.get(timeout=5)
        if line is None:
            return ''
        return json.dumps(line) + '\n'

    def close(self):
        self.closed = True
        self.lines.put(None)


def test_the_watcher_reconnects_after_a_lost_link_and_follows_a_route_change():
    opened, route = [], {'address': '10.42.0.1'}

    def transport(address):
        if len(opened) == 1:
            opened.append((address, None))
            raise OSError('No route to host')
        stream = Stream()
        opened.append((address, stream))
        return stream

    watch = UpsWatch(transport, lambda: route['address'], retry_s=0.05)
    watch.start()
    try:
        until(lambda: len(opened) == 1)
        opened[0][1].lines.put({'state': ON_POWER, 'age_s': 1.0, 'missing': False})
        until(lambda: watch.snapshot()['status'] == 'ok')
        opened[0][1].lines.put(None)  # The link drops; the next attempt fails, the one after reads again.
        until(lambda: len(opened) == 3)
        assert opened[1] == ('10.42.0.1', None)
        route['address'] = '192.0.2.180'
        opened[2][1].lines.put({'state': ON_POWER, 'age_s': 1.0, 'missing': False})
        until(lambda: len(opened) == 4)
        assert opened[3][0] == '192.0.2.180' and opened[2][1].closed
    finally:
        watch.close()
    assert opened[3][1].closed


class Teleop:
    def __init__(self, recording='recording'):
        self.active, self.calls = True, []
        self.recording = {'active': recording == 'recording', 'state': recording}

    def snapshot(self):
        return {'task': {'recording': self.recording}}

    def task_action(self, action):
        self.calls.append(action)
        self.recording = {'active': False, 'state': 'complete'}

    def start(self, confirm=False):
        self.calls.append('start')

    def stop(self):
        self.calls.append('stop')
        self.active = False


class Tune:
    def __init__(self):
        self.active, self.calls, self.mode = True, [], 'live'

    def snapshot(self):
        return {'phase': 'running'}

    def guide(self):
        return None

    def stop(self):
        self.calls.append('stop')
        self.active = False


class Link:
    running = False

    def disconnect(self):
        pass


class Ups:
    on_shutdown = None

    def __init__(self, refusal=None):
        self.reason, self.started, self.closed = refusal, False, False

    def snapshot(self):
        return {'status': 'ok', 'state': SHUTTING}

    def refusal(self):
        return self.reason

    def start(self):
        self.started = True

    def close(self):
        self.closed = True


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    yield session
    session.close()


def test_helm_refuses_starts_shows_the_state_and_stops_the_session_on_a_shutdown(helm):
    from aiohttp.test_utils import TestClient, TestServer

    teleop, tune, ups = Teleop(), Tune(), Ups(refusal='The UPS battery is critical')

    async def scenario():
        async with TestClient(
            TestServer(create_app(helm, Link(), teleop=teleop, tune=tune, token='secret', ups=ups))
        ) as client:
            assert ups.started
            headers = {'X-Helm-Token': 'secret'}
            state = await (await client.get('/api/helm/state')).json()
            assert state['ups']['state']['shutdown']['stage'] == 'pending'
            for path, payload in (
                ('/api/helm/teleop/start', {}),
                ('/api/helm/task', {'action': 'start_record'}),
                ('/api/helm/tune/start', {}),
            ):
                teleop.active = path != '/api/helm/teleop/start' and path != '/api/helm/tune/start'
                tune.active = False
                refused = await client.post(path, json=payload, headers=headers)
                assert refused.status == 422 and 'critical' in (await refused.json())['error'], path
            assert 'start' not in teleop.calls
        assert ups.closed

    asyncio.run(scenario())
    teleop.calls.clear()  # Helm's own exit stopped them too.
    tune.calls.clear()
    teleop.active, tune.active = True, True
    teleop.recording = {'active': True, 'state': 'recording'}
    ups.on_shutdown({'state': SHUTTING})
    assert teleop.calls == ['stop_record', 'stop'] and tune.calls == ['stop']
    idle = Teleop(recording='complete')
    create_app(helm, Link(), teleop=idle, token='secret', ups=(quiet := Ups()))
    quiet.on_shutdown({'state': SHUTTING})
    assert idle.calls == ['stop']  # Nothing recording: straight to the stop, which lowers the arm first.
