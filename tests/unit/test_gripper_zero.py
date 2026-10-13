"""Gripper zero: one gripper's mechanical zero at its closed jaws, only on the operator's confirmation."""
import asyncio
import json

import pytest

from robot_platform.helm.gripper_zero import GripperZero
from robot_platform.ops import probe

MOTORS = [dict(name=name, bus='canL', id=i + 1, uid=f'{i:016x}', firmware='0.0.3.32')
          for i, name in enumerate(('left_arm_j1', 'left_gripper', 'right_gripper'))]


class Maintenance:
    """Stands in for StoppedMaintenance: records the run it was asked for."""
    runs = []

    def __init__(self, exchange):
        self.exchange = exchange

    def run(self, roster, zero, **options):
        Maintenance.runs.append((roster, options))
        options['checkpoint']()
        zero.update(success=True, motors=[dict(name=roster[0]['name'], before_rad=[1.352, 1.351],
                                                after_zero_rad=[0., .0001], after_save_rad=[.0001, 0.])])
        return zero


def test_the_robot_zeroes_and_saves_only_the_chosen_gripper(tmp_path, monkeypatch):
    monkeypatch.setattr(probe, 'EVIDENCE', tmp_path)
    monkeypatch.setattr(probe, 'identity', lambda root=None: ({'asset_id': 'a'}, {'id': 'ref-1', 'motors': MOTORS}))
    Maintenance.runs.clear()

    def capture(device, reference, *, maintain):
        result = {'success': True}
        maintain('exchange', reference['motors'], result)
        return result
    report = probe.zero_gripper('left_gripper', capture=capture, maintenance=Maintenance)
    (roster, options), = Maintenance.runs
    assert [m['name'] for m in roster] == ['left_gripper']  # Every other drive is left alone.
    assert options['reference_confirmed'] is True and options['save'] is True
    assert report['success'] and report['before_rad'] == 1.351 and report['after_save_rad'] == 0.
    saved = json.loads(next(tmp_path.glob('mechanical-zero-left-gripper-*.json')).read_text(encoding='utf-8'))
    assert saved['zero']['reference']['previous_reference'] == 'ref-1'
    with pytest.raises(ValueError, match='Only a gripper'):
        probe.zero_gripper('left_arm_j1', capture=capture, maintenance=Maintenance)


def fake_robot(monkeypatch, reply, seen=None):
    from robot_platform.ops import ssh

    class Client:
        def close(self):
            pass

    def run(client, command, *, timeout):
        if seen is not None:
            seen.append(command)
        return 0, json.dumps(reply) + '\n', ''
    monkeypatch.setattr(ssh, 'run', run)
    return lambda profile, address: Client()


def test_helm_runs_the_zero_on_the_robot_only_once_confirmed(monkeypatch):
    seen = []
    zeroer = GripperZero(lambda: '10.42.0.1', open_ssh=fake_robot(monkeypatch, {
        'ok': True, 'motor': 'right_gripper', 'success': True, 'before_rad': 1.35, 'after_save_rad': 0.}, seen),
        remote_root='/home/ltr/minerva/current', clock=lambda: 5.)
    with pytest.raises(ValueError, match='confirm'):
        zeroer.zero('right_gripper')
    with pytest.raises(ValueError, match='left or right gripper'):
        zeroer.zero('right_arm_j1', confirm=True)
    assert seen == []
    report = zeroer.zero('right_gripper', confirm=True)
    assert report == {'motor': 'right_gripper', 'success': True, 'before_rad': 1.35, 'after_save_rad': 0.,
                      'finished_at': 5.}
    assert seen == ['cd /home/ltr/minerva/current && PYTHONPATH=src python3 -u -m robot_platform.ops.probe '
                    'zero_gripper right_gripper']
    assert zeroer.snapshot() == {'running': None, 'last': report}


def test_a_failure_on_the_robot_is_reported_not_raised(monkeypatch):
    zeroer = GripperZero(lambda: '10.42.0.1', open_ssh=fake_robot(monkeypatch, {
        'ok': False, 'error': 'RuntimeError: left_gripper: pose changed after preflight'}))
    report = zeroer.zero('left_gripper', confirm=True)
    assert report['success'] is False and 'pose changed after preflight' in report['error']
    assert zeroer.snapshot()['last'] == report and zeroer.snapshot()['running'] is None


class FakeZero:
    def __init__(self):
        self.running, self.calls = None, []

    def zero(self, motor, *, confirm=False):
        self.calls.append((motor, confirm))
        return {'motor': motor, 'success': True}

    def snapshot(self):
        return {'running': self.running, 'last': {'motor': self.calls[-1][0]} if self.calls else None}


class FakeLink:
    def __init__(self):
        self.running, self.calls = True, []

    def connect(self):
        self.calls.append('connect')

    def disconnect(self, timeout=10.):
        self.calls.append('disconnect')
        self.running = False


def test_the_server_takes_the_motors_and_refuses_without_confirmation_or_over_wifi(tmp_path):
    from pathlib import Path
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.server import create_app
    from robot_platform.helm.session import HelmSession
    root = Path(__file__).resolve().parents[2]
    session = HelmSession(root / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=tmp_path / 'limits.json', seed=root / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    link, zero = FakeLink(), FakeZero()
    headers = {'X-Helm-Token': 'secret'}

    async def run():
        async with TestClient(TestServer(create_app(session, link, token='secret', gripper_zero=zero))) as client:
            refused = await client.post('/api/helm/grippers/zero', json={'motor': 'left_gripper'}, headers=headers)
            assert refused.status == 422 and zero.calls == [] and link.calls == []
            state = await (await client.post('/api/helm/grippers/zero', json={'motor': 'left_gripper', 'confirm': True},
                                             headers=headers)).json()
            assert zero.calls == [('left_gripper', True)] and link.calls == ['disconnect']
            assert state['gripper_zero']['last'] == {'motor': 'left_gripper'}
            session.connection = 'wifi'  # Stand-in for Via Wi-Fi: the zero needs the tether.
            wifi = await client.post('/api/helm/grippers/zero', json={'motor': 'left_gripper', 'confirm': True},
                                     headers=headers)
            assert wifi.status == 422 and len(zero.calls) == 1
    try:
        asyncio.run(run())
    finally:
        session.close()
