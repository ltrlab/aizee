"""Minerva's network in Helm (helm.network): the service's state for the header and Settings, the bridge's path
(wired or Wi-Fi, by the interface the route to it leaves on), the addresses the bridge tries next, and the access
point switch, which needs control and waits for nothing to hold the motors."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.helm.network import NetworkView, path_of
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'

STATE = {'version': 1, 'mode': 'lan', 'updated_at': 1000., 'local': '10.42.0.1',
         'ethernet': {'carrier': True, 'connection': 'minerva-lan', 'addresses': ['192.168.8.43/24']},
         'wifi': {'connection': None, 'mode': 'off', 'addresses': []},
         'ap': {'wanted': False, 'on': False, 'ssid': 'minerva', 'address': '10.42.1.1'}}


def routes(device):
    return lambda command, **_: SimpleNamespace(stdout=f'{command[-1]} dev {device} src 192.168.8.43 uid 1000\n')


def test_the_path_is_the_kind_of_interface_the_route_leaves_on(tmp_path):
    (tmp_path / 'wlP1p1s0' / 'wireless').mkdir(parents=True)
    (tmp_path / 'enP8p1s0').mkdir()
    assert path_of('192.168.8.225', run=routes('enP8p1s0'), interfaces=tmp_path) == 'wired'
    assert path_of('10.42.1.23', run=routes('wlP1p1s0'), interfaces=tmp_path) == 'wifi'
    assert path_of('127.0.0.1', run=routes('lo'), interfaces=tmp_path) == 'local'
    assert path_of(None, run=routes('enP8p1s0'), interfaces=tmp_path) is None
    assert path_of('x', run=lambda *a, **k: SimpleNamespace(stdout='RTNETLINK answers: Network is unreachable'),
                   interfaces=tmp_path) is None

    def missing(*args, **kwargs):
        raise FileNotFoundError('ip')
    assert path_of('192.168.8.225', run=missing, interfaces=tmp_path) is None


def test_the_view_reads_the_service_state_and_orders_the_addresses(tmp_path):
    state = tmp_path / 'state.json'
    view = NetworkView(state, clock=lambda: 1003.)
    assert view.snapshot() == {'installed': False} and view.addresses() == []
    state.write_text(json.dumps(STATE), encoding='utf-8')
    snapshot = view.snapshot()
    assert snapshot['installed'] and snapshot['running'] and snapshot['age_s'] == 3. and snapshot['mode'] == 'lan'
    assert view.addresses() == ['192.168.8.43', '10.42.0.1']  # Wired first, then the tether/access point address.
    state.write_text(json.dumps(dict(STATE, wifi={'addresses': ['192.168.8.180/24']}, updated_at=980.)),
                     encoding='utf-8')
    assert view.addresses() == ['192.168.8.43', '10.42.0.1', '192.168.8.180']
    assert view.snapshot()['running'] is False  # Not written for 23 s: the service is not running.


def test_switching_the_access_point_runs_only_the_allowed_command(tmp_path):
    state = tmp_path / 'state.json'
    state.write_text(json.dumps(STATE), encoding='utf-8')
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='{}', stderr='')
    view = NetworkView(state, run=run, clock=lambda: 1000.)
    view.set_ap(True)
    view.set_ap(False)
    assert calls == [['sudo', '-n', '/usr/local/sbin/minerva-net', 'ap', 'on'],
                     ['sudo', '-n', '/usr/local/sbin/minerva-net', 'ap', 'off']]
    refused = NetworkView(state, run=lambda *a, **k: SimpleNamespace(returncode=1, stdout='',
                                                                    stderr='sudo: a password is required'))
    with pytest.raises(ValueError, match='a password is required'):
        refused.set_ap(True)


class Teleop:
    active = False

    def snapshot(self):
        return {'phase': 'stopped', 'receiver': None}

    def stop(self):
        pass

    def guide(self):
        return None


class Link:
    running = False

    def disconnect(self):
        pass


class Network:
    def __init__(self):
        self.switched = []

    def snapshot(self):
        return dict(STATE, installed=True, running=True, age_s=.5)

    def set_ap(self, on):
        self.switched.append(on)


def test_the_access_point_switch_in_helm(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')
    teleop, network = Teleop(), Network()

    async def run():
        app = create_app(session, Link(), teleop=teleop, token='secret', network=network)
        async with TestClient(TestServer(app)) as client:
            state = await (await client.get('/api/helm/state')).json()
            assert state['network']['mode'] == 'lan' and state['network']['ethernet']['addresses'] == ['192.168.8.43/24']
            headers = {'X-Helm-Token': 'secret'}
            for payload in ({'on': 'yes'}, {'on': True, 'x': 1}, {}):
                assert (await client.post('/api/helm/network/ap', json=payload, headers=headers)).status == 422
            assert (await client.post('/api/helm/network/ap', json={'on': True})).status == 403  # No token.
            switched = await client.post('/api/helm/network/ap', json={'on': True}, headers=headers)
            assert switched.status == 200 and network.switched == [True]
            teleop.active = True  # A session holds the motors: the switch waits.
            busy = await client.post('/api/helm/network/ap', json={'on': False}, headers=headers)
            assert busy.status == 422 and 'Stop teleop first' in (await busy.json())['error']
            assert network.switched == [True]
    try:
        asyncio.run(run())
    finally:
        session.close()


def test_helm_on_a_pc_has_no_network_to_switch(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json')

    async def run():
        async with TestClient(TestServer(create_app(session, Link(), teleop=Teleop(), token='secret'))) as client:
            assert (await (await client.get('/api/helm/state')).json())['network'] is None
            refused = await client.post('/api/helm/network/ap', json={'on': True}, headers={'X-Helm-Token': 'secret'})
            assert refused.status == 422 and 'Helm on Minerva' in (await refused.json())['error']
    try:
        asyncio.run(run())
    finally:
        session.close()
