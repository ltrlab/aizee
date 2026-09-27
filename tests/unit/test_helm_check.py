"""Minerva Check in Helm: it runs alone, in the background, and the page gets no evidence payloads."""
import asyncio
import threading
from types import SimpleNamespace

import json

import pytest

from robot_platform.helm.check import CheckWorkspace
from robot_platform.helm.session import HelmSession
from robot_platform.ops import check
from robot_platform.ops.release import ROOT

PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
TELEOP = ROOT / 'config/devices/minerva-teleop.json'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'


def test_helm_runs_the_check_alone(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.server import create_app
    release, published = threading.Event(), threading.Event()

    def runner(**options):
        report = {'kind': 'minerva_check', 'state': 'running', 'stages': [
            {'id': 'pc', 'title': 'Operator PC', 'checks': [{'id': 'x', 'title': 'X', 'status': 'pass',
                                                             'detail': 'ok', 'seconds': .1, 'data': {'big': 1}}]}]}
        options['publish'](report)
        published.set()
        release.wait(5)

    workspace = CheckWorkspace(profile=PROFILE, teleop=TELEOP, runs=tmp_path, runner=runner)
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', seed=SEED)
    link = SimpleNamespace(running=False, connect=lambda: None, disconnect=lambda: None)
    headers = {'X-Helm-Token': 'secret'}

    async def scenario(client):
        state = await (await client.get('/api/helm/state')).json()
        assert state['check'] == {'running': False, 'error': None, 'report': None}
        state = await (await client.post('/api/helm/check/run', json={}, headers=headers)).json()
        assert state['check']['running']
        refused = await client.post('/api/helm/connect', json={}, headers=headers)
        assert refused.status == 422 and 'Minerva Check is running' in (await refused.json())['error']
        again = await client.post('/api/helm/check/run', json={}, headers=headers)
        assert again.status == 422
        assert published.wait(5)
        state = await (await client.get('/api/helm/state')).json()
        assert 'data' not in state['check']['report']['stages'][0]['checks'][0]
        release.set()
        workspace._thread.join(5)
        link.running = True
        refused = await client.post('/api/helm/check/run', json={}, headers=headers)
        assert refused.status == 422 and 'Disconnect the motors' in (await refused.json())['error']

    async def main():
        async with TestClient(TestServer(create_app(session, link, check=workspace, token='secret'))) as client:
            await scenario(client)
    try:
        asyncio.run(main())
    finally:
        release.set()
        session.close()


def saved_report(runs, name, *, legacy=False, failed=()):
    """A finished report on disk: today's checks (or the set before the live sweep), passing."""
    stages = []
    for key, title in check.STAGES.items():
        checks = [{'id': c['id'], 'title': c['title'], 'detail': 'ok', 'seconds': .1,
                   'status': 'fail' if c['id'] in failed else 'pending' if key == 'sweep' else 'pass'}
                  for c in check.CHECKS if c['stage'] == key]
        if legacy and key == 'sweep':
            checks = [{'id': 'sweep', 'title': 'Live joint sweep', 'status': 'pending', 'detail': '', 'seconds': 0}]
        if legacy and key == 'pc':
            checks = [c for c in checks if c['id'] != 'servo']
        stages.append({'id': key, 'title': title, 'checks': checks})
    report = {'schema': 1, 'kind': 'minerva_check', 'state': 'finished', 'verdict': 'go',
              'summary': 'GO: every check passed', 'stages': stages}
    directory = runs / 'minerva-check'
    directory.mkdir(parents=True, exist_ok=True)
    (directory / f'{name}.json').write_text(json.dumps(report), encoding='utf-8')


def test_a_report_saved_before_the_live_sweep_is_outdated_and_cannot_start_one(tmp_path):
    saved_report(tmp_path, '20260926T000000Z', legacy=True)
    workspace = CheckWorkspace(profile=PROFILE, teleop=TELEOP, runs=tmp_path)
    report = workspace.snapshot()['report']
    assert report['verdict'] == 'outdated', report['summary']
    assert 'Servo calibration' in report['summary'] and 'Live joint sweep: left arm' in report['summary']
    assert json.loads(next((tmp_path / 'minerva-check').glob('*.json')).read_text())['verdict'] == 'go'  # As saved.
    with pytest.raises(ValueError, match='run Minerva Check again'):
        workspace.live_ready('left')


def test_the_live_sweep_follows_a_current_check_whose_stopped_checks_passed(tmp_path):
    saved_report(tmp_path, '20260927T000000Z')
    workspace = CheckWorkspace(profile=PROFILE, teleop=TELEOP, runs=tmp_path)
    assert workspace.snapshot()['report']['verdict'] == 'go'  # Not recomputed: current reports stand as saved.
    workspace.live_ready('left')
    saved_report(tmp_path, '20260927T010000Z', failed=('pose',))
    workspace = CheckWorkspace(profile=PROFILE, teleop=TELEOP, runs=tmp_path)
    with pytest.raises(ValueError, match='stopped checks did not pass'):
        workspace.live_ready('right')
