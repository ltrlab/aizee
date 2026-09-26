"""Minerva Check in Helm: it runs alone, in the background, and the page gets no evidence payloads."""
import asyncio
import threading
from types import SimpleNamespace

from robot_platform.helm.check import CheckWorkspace
from robot_platform.helm.session import HelmSession
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
