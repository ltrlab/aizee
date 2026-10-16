"""Simulation-only Helm rejects physical routes even when a client bypasses its UI."""
import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.helm.cli import SimulationLink
from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopSession, TeleopCore

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
        limits=DEVICES / 'minerva-production-01-joint-limits.json',
        seed=ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json')
    yield session
    session.close()


def test_simulation_api_cannot_connect_check_zero_tune_or_select_live(helm, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    teleop = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path, simulation={})
    class ForbiddenLink:
        running = False
        def connect(self):
            pytest.fail('Physical connection attempted')
        def disconnect(self):
            pytest.fail('Physical cleanup attempted in a simulation-only process')
    async def run():
        async with TestClient(TestServer(create_app(helm, ForbiddenLink(), teleop=teleop,
                                                   simulation_only=True, token='test'))) as client:
            state = await (await client.get('/api/helm/state')).json()
            assert state['mode'] == 'simulation'
            assert state['live_view']['enabled'] is False
            assert state['check'] is state['tune'] is state['gripper_zero'] is None
            assert state['teleop']['receiver_choices'] == ['virtual']
            headers = {'X-Helm-Token': 'test'}
            routes = [('/connect', {}), ('/disconnect', {}), ('/connection', {'connection': 'tether'}),
                      ('/check/run', {}), ('/check/live', {'arm': 'both', 'confirm': True}),
                      ('/grippers/zero', {'motor': 'left_gripper', 'confirm': True}),
                      ('/tune/configure', {'arm': 'left'}), ('/tune/start', {'confirm': True}),
                      ('/tune/return', {'arm': 'right', 'confirm': True}), ('/limits/save', {}),
                      ('/teleop/target', {'target': 'jetson'}),
                      ('/teleop/receiver', {'mode': 'live', 'arm': 'both'})]
            for path, payload in routes:
                response = await client.post('/api/helm'+path, json=payload, headers=headers)
                assert response.status == 422, (path, await response.text())
                assert 'Simulation' in (await response.json())['error']
            assert (await client.post('/api/helm/task', json={'action': 'setup'})).status == 403
            state = await (await client.post('/api/helm/task', json={'action': 'setup'}, headers=headers)).json()
            assert state['teleop']['task_configured']
            assert teleop.simulation['task_scene'] is True
            assert not teleop.active  # Setup does not open USB or start a receiver.
            page = await (await client.get('/')).text()
            assert 'task-record' in page and 'task-reset' in page and 'mode-pill' in page
    asyncio.run(run())


def test_simulation_and_live_reader_flags_are_mutually_exclusive(helm):
    with pytest.raises(ValueError, match='physical live reader'):
        create_app(helm, SimulationLink(), simulation_only=True, live=True)


def test_simulation_session_refuses_direct_physical_selection_and_start(helm, tmp_path):
    session = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path, simulation={})
    with pytest.raises(ValueError, match='physical receiver'):
        session.set_receiver('live', 'both')
    session.receiver = {'mode': 'live', 'arm': 'both'}
    with pytest.raises(ValueError, match='virtual receiver'):
        session.start(confirm=True)


def test_task_setup_requires_both_arms_and_explicit_camera_profile(helm, tmp_path):
    session = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path, target='jetson')
    with pytest.raises(ValueError, match='both arms'):
        session.task_action('setup')
    session.set_receiver('live', 'both')
    with pytest.raises(ValueError, match='capture-profile'):
        session.task_action('setup')
    assert not session.active and not session.task_configured


def test_scene_reset_is_refused_while_following_and_recording_requires_ready_lifecycle():
    calls = []
    core = TeleopCore.__new__(TeleopCore)
    core.task_capture = SimpleNamespace(command=lambda action: calls.append(action))
    core.session = SimpleNamespace(lifecycle=SimpleNamespace(state='TELEOP'))
    core.live_arm, core.fault = None, None
    with pytest.raises(ValueError, match='disabled or idle'):
        core.task_action('reset')
    core.task_action('start_record')
    core.session.lifecycle.state = 'DISABLED'
    core.task_action('reset')
    with pytest.raises(ValueError, match='Hold or engage'):
        core.task_action('start_record')
    core.live_arm = 'both'
    with pytest.raises(ValueError, match='simulated'):
        core.task_action('reset')
    assert calls == ['start_record', 'reset']


def test_simulation_cli_constructs_no_physical_workspaces(monkeypatch, tmp_path):
    from aiohttp import web
    from robot_platform.helm import cli, session, leaders, teleop, server, link, tune, check, gripper_zero
    fake = SimpleNamespace(profile_path=tmp_path/'profile.json', asset_id='test', labels={}, evidence=tmp_path,
                           stale_lock_pid=None, close=lambda: None)
    monkeypatch.setattr(session, 'HelmSession', lambda *a, **k: fake)
    monkeypatch.setattr(leaders, 'LeaderWorkspace', lambda *a, **k: object())
    def forbidden(*a, **k):
        pytest.fail('Simulation CLI constructed a physical workspace')
    for module, name in ((link, 'LiveLink'), (tune, 'TuneSession'), (check, 'CheckWorkspace'),
                         (gripper_zero, 'GripperZero')):
        monkeypatch.setattr(module, name, forbidden)
    captured = {}
    monkeypatch.setattr(teleop, 'TeleopSession', lambda *a, **k: captured.update(teleop=k) or object())
    monkeypatch.setattr(server, 'create_app', lambda *a, **k: captured.update(app=k) or object())
    monkeypatch.setattr(web, 'run_app', lambda *a, **k: None)
    assert cli.main(['--profile', str(fake.profile_path), '--teleop', str(tmp_path/'teleop.json'), '--sim']) == 0
    assert captured['teleop']['target'] == 'pc'
    assert captured['app']['simulation_only'] is True and captured['app']['live'] is False
    assert captured['app']['check'] is captured['app']['tune'] is None


@pytest.mark.parametrize('facts, message', [
    ({'cube_position': [0., 0., 0.]}, 'Cube on the table'),
    ({'first_gripper': 'left'}, 'Left gripper picked up'),
    ({'first_gripper': 'right'}, 'Right gripper picked up'),
    ({'first_gripper': 'left', 'handoff': True}, 'Handoff complete'),
    ({'first_gripper': 'left', 'handoff': True, 'bucket_settled': True, 'success': True}, 'Task complete'),
    ({'bucket_settled': True, 'success': True}, 'Cube settled in the bucket; the complete'),
    ({'first_gripper': 'left', 'dropped': True, 'success': True}, 'Cube dropped'),
    ({'success': True, 'bucket_settled': True, 'training_eligible': False}, 'Invalid scene contact'),
    ({'success': True, 'forbidden_contacts': [['finger', 'table']]}, 'Invalid scene contact'),
])
def test_task_progress_uses_recorded_outcomes_and_invalidity_overrides_success(facts, message):
    from robot_platform.helm.server import task_phase
    assert task_phase(facts).startswith(message)
    assert task_phase({}) is None


def test_scene_progress_is_delivered_to_gui_without_changing_recorded_facts(helm, tmp_path, monkeypatch):
    from copy import deepcopy
    from aiohttp.test_utils import TestClient, TestServer
    teleop = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path, simulation={})
    state = teleop.snapshot()
    facts = {'first_gripper': 'left', 'handoff': True, 'bucket_settled': True,
             'success': True, 'training_eligible': True}
    state['task'] = {'available': True, 'task': facts}
    monkeypatch.setattr(teleop, 'snapshot', lambda: deepcopy(state))
    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), teleop=teleop,
                                                   simulation_only=True))) as client:
            result = await (await client.get('/api/helm/state')).json()
            task = result['teleop']['task']
            assert task['display_phase'] == 'Task complete: pickup, handoff and bucket placement.'
            assert task['task'] == facts
    asyncio.run(run())
