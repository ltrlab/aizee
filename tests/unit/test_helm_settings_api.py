"""Helm's Settings workspace over HTTP, and the model's joint telemetry in the state. No hardware."""
import asyncio
from pathlib import Path

import pytest

from robot_platform.helm.server import create_app
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopSession

ROOT = Path(__file__).resolve().parents[2]
DEVICES = ROOT / 'config/devices'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
TOKEN = 'settings-test-token'


class NoLink:
    running = False

    def connect(self):
        raise AssertionError('The settings API never opens the motors')

    def disconnect(self):
        pass


@pytest.fixture
def helm(tmp_path):
    session = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                          limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    yield session
    session.close()


def test_settings_are_read_freely_and_changed_only_with_the_token(helm, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    teleop = TeleopSession(helm, DEVICES / 'minerva-teleop-direct-tuning.json', runs=tmp_path / 'runs')

    async def run():
        async with TestClient(TestServer(create_app(helm, NoLink(), teleop=teleop, token=TOKEN))) as client:
            response = await client.get('/api/helm/settings')
            assert response.status == 200
            catalog = await response.json()
            assert catalog['enabled'] and catalog['can_apply'] and catalog['revision'] == 0
            field = next(f for s in catalog['sections'] for f in s['fields']
                         if f['id'] == 'source.target_max_age_s')
            payload = {'session_id': catalog['session_id'], 'revision': 0,
                       'values': {field['id']: field['value'] / 2}}
            for headers in ({}, {'X-Helm-Token': 'wrong'}):
                assert (await client.post('/api/helm/settings/apply', json=payload, headers=headers)).status == 403
            response = await client.post('/api/helm/settings/apply', json=payload, headers={'X-Helm-Token': TOKEN})
            assert response.status == 200
            applied = await response.json()
            assert applied['revision'] == 1
            assert teleop._bundle()['operator_settings']['source']['target_max_age_s'] == field['value'] / 2
            # Stale revision, unknown action and a loosened value are refused with a reason.
            for path, body in [('apply', payload), ('drop', {'session_id': catalog['session_id'], 'revision': 1}),
                               ('apply', dict(payload, revision=1, values={field['id']: 1.}))]:
                response = await client.post(f'/api/helm/settings/{path}', json=body, headers={'X-Helm-Token': TOKEN})
                assert response.status == 422 and (await response.json())['error']
            response = await client.post('/api/helm/settings/reset', json={'session_id': catalog['session_id'],
                                         'revision': 1}, headers={'X-Helm-Token': TOKEN})
            assert (await response.json())['revision'] == 2
            # The state carries the model's joint telemetry: the stopped reader's, positions only, stale offline.
            state = await (await client.get('/api/helm/state')).json()
            telemetry = state['telemetry']
            assert telemetry['model'] == helm.model.profile.id and telemetry['urdf_sha256'] == helm.model.sha256
            assert telemetry['stale'] and all(row['velocity'] is None and row['effort'] is None
                                              for row in telemetry['rows'])
            assert {row['motor'] for row in telemetry['rows']} >= {'left_arm_j1', 'right_gripper'}

    asyncio.run(run())


def test_a_profile_without_direct_teleop_shows_a_read_only_catalog(helm, tmp_path):
    teleop = TeleopSession(helm, DEVICES / 'minerva-teleop.json', runs=tmp_path / 'runs')
    catalog = teleop.settings_snapshot()
    assert catalog['enabled'] is True and catalog['can_apply'] is False and catalog['reason']
    assert {s['id'] for s in catalog['sections']} == {'profile', 'binding', 'runtime'}
    assert not any(f['editable'] for s in catalog['sections'] for f in s['fields'])
    assert 'operator_settings' not in teleop._bundle()
    with pytest.raises(ValueError, match='direct teleop'):
        teleop.settings_update('reset', {'session_id': catalog['session_id'], 'revision': 0})
