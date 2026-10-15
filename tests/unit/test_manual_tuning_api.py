"""Manual tuning across HTTP, worker messaging and delayed receiver acknowledgements.

The core below is a deterministic, non-hardware acknowledgement source. These
tests exercise the real session, pipes and web handlers, not physical tracking.
"""
import asyncio
from copy import deepcopy
import json
import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.helm.manual_tuning import values
from robot_platform.helm.server import create_app
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.teleop.guarded_lifecycle import GuardedLifecycleSession


TOKEN = {'X-Helm-Token': 'manual-test-token'}
ENDPOINT = '/api/helm/teleop/tuning/'


class AcknowledgingCore:
    period_s = .005
    clock = staticmethod(time.perf_counter)
    landing = False

    def __init__(self, bundle):
        self.ack = deepcopy(bundle['manual_tuning'])
        # The law it took at start, reported as TeleopCore.status_view does.
        switch = self.ack.get('velocity_feedforward') is True
        self.law = dict(position_only=not switch, velocity_feedforward=switch)
        self.phase, self.state, self.fault = 'starting', 'HOLD', None
        self.keys, self.requests = [], []
        self.pending = self._tuning_result = None
        self.queued, self.release = threading.Event(), threading.Event()
        self.closed, self.reject = False, None

    def build(self):
        self.phase = 'running'

    def queue_tuning(self, payload):
        self.requests.append(deepcopy(payload))
        self.pending = deepcopy(payload)
        self._tuning_result = None
        self.queued.set()

    def tick(self, key, _started):
        if key:
            self.keys.append(key)
        if self.pending is not None and self.release.is_set():
            if self.reject:
                self._tuning_result = (self.reject, None)
            else:
                self.ack = dict(self.pending, revision=self.pending['revision'] + 1)
                self._tuning_result = (None, deepcopy(self.ack))
            self.pending = None
            self.release.clear()

    def view(self):
        return dict(phase=self.phase, state=self.state, fault=self.fault, **self.law,
                    manual_tuning_ack=deepcopy(self.ack), manual_tuning_pending=self.pending is not None)

    def guide(self):
        return None

    def fail(self, reason):
        self.phase, self.fault = 'fault', reason

    def close(self):
        self.closed = True
        self.phase = 'stopped'

    def land(self):
        return False


class FakeLink:
    running = False

    def disconnect(self):
        self.running = False


@pytest.fixture
def setup(tmp_path, monkeypatch):
    from robot_platform.control import commissioning, guarded_config

    names = ('left_arm_j1', 'right_arm_j1')
    derived = {
        'config': {'joints': [dict(name=name, lower=-2., upper=2., hold_kp=10., hold_kd=1.,
            max_kp=20., max_kd=2., max_velocity=1., max_effort=5., max_tracking_error=.2,
            stop_acceleration=2.) for name in names]},
        'lifecycle': [dict(name=name, lower=-2., upper=2., max_velocity=.4, hold_kp=10., hold_kd=1.,
                          max_lead=.12, friction_nm=0., interpolation_s=0.) for name in names],
    }
    monkeypatch.setattr(guarded_config, 'derive_receiver', lambda **_: deepcopy(derived))
    monkeypatch.setattr(commissioning, 'motor_models', lambda *_: {
        'left_arm_j1': 'ROBSTRIDE03', 'right_arm_j1': 'ROBSTRIDE00'})
    helm = SimpleNamespace(asset_id='manual-api-fixture', profile_path=tmp_path/'config/devices/calibration.json',
        reference={'motors': [dict(name=name, uid=f'uid-{i}', firmware='1') for i, name in enumerate(names)]},
        bindings={}, steps=[], mapping={}, transmissions=[], saved_limits={}, device={}, labels={},
        model=SimpleNamespace(profile=SimpleNamespace(id='fixture')), snapshot=lambda: {}, guide=lambda: None)
    profile = dict(id='manual-api-fixture', kind='guarded_teleop_profile', backend='virtual',
        runtime_approved=False, manual_tuning=True, routes={}, receiver=dict(collision_mode='disabled',
            loop_hz=100, interpolation_ms=0, lease_ms=150,
            direct=dict(command_ms=30, velocity_timeout_ms=15, limit_reaction_ms=10, position_only=True)))
    profile_path = tmp_path/'teleop.json'
    profile_path.write_text(json.dumps(profile), encoding='utf-8')
    cores = []

    def core_factory(bundle):
        core = AcknowledgingCore(bundle)
        cores.append(core)
        return core

    teleop = TeleopSession(helm, profile_path, runs=tmp_path/'runs', core_factory=core_factory)
    fixture = SimpleNamespace(helm=helm, teleop=teleop, cores=cores,
                              app=lambda: create_app(helm, FakeLink(), teleop=teleop, token=TOKEN['X-Helm-Token']))
    yield fixture
    teleop.stop(timeout=2.)
    assert all(core.closed for core in cores)


def payload(teleop, *, kp=None, action='apply'):
    result = teleop.manual.bundle()
    if action != 'apply':
        result.pop('joints'), result.pop('velocity_feedforward')  # As the page sends Save and Restore.
    elif kp is not None:
        result['joints'][0]['kp'] = kp
    return result


async def wait_until(predicate, timeout=2.):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out waiting for the test worker'
        await asyncio.sleep(.005)


async def start(setup):
    setup.teleop.start()
    await wait_until(lambda: setup.teleop.snapshot()['phase'] == 'running')
    return setup.cores[-1]


def test_http_authentication_and_request_validation_leave_settings_unchanged(setup):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(setup.app())) as client:
            before = setup.teleop.manual.bundle()
            for headers in ({}, {'X-Helm-Token': 'wrong'}):
                assert (await client.post(ENDPOINT+'apply', json=payload(setup.teleop, kp=20.),
                                          headers=headers)).status == 403
            for action, candidate in [('apply', []), ('apply', {}), ('apply', {**before, 'extra': True}),
                                      ('other', payload(setup.teleop, action='save'))]:
                assert (await client.post(ENDPOINT+action, json=candidate, headers=TOKEN)).status == 422
            for key, invalid in [('kp', float('nan')), ('kd', float('inf')), ('speed', True)]:
                candidate = payload(setup.teleop)
                candidate['joints'][0][key] = invalid
                response = await client.post(ENDPOINT+'apply', json=candidate, headers=TOKEN)
                assert response.status == 422
                assert key in (await response.json())['error']
            for change in ({'revision': -1}, {'revision': True}, {'session_id': 'previous-session'}):
                response = await client.post(ENDPOINT+'apply', json={**before, **change}, headers=TOKEN)
                assert response.status == 422
            assert setup.teleop.manual.bundle() == before
            assert not setup.teleop.active and not setup.cores
    asyncio.run(run())


@pytest.mark.parametrize('state', ['DISABLED', 'TELEOP', 'ENGAGING', 'SHUTTING_DOWN'])
def test_running_tuning_is_rejected_outside_hold(setup, state):
    async def run():
        core = await start(setup)
        core.state = state
        await wait_until(lambda: setup.teleop.snapshot()['state'] == state)
        before = setup.teleop.manual.bundle()
        assert not setup.teleop.snapshot()['tuning']['can_apply']
        with pytest.raises(ValueError, match='Hold to edit'):
            setup.teleop.manual_tuning('apply', payload(setup.teleop, kp=20.))
        assert setup.teleop.manual.bundle() == before and not core.requests
    asyncio.run(run())


def test_pending_apply_waits_for_ack_and_http_remains_responsive_to_h_and_x(setup):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        core = await start(setup)
        before = setup.teleop.manual.bundle()
        candidate = payload(setup.teleop, kp=25.)
        async with TestClient(TestServer(setup.app())) as client:
            apply_task = asyncio.create_task(client.post(ENDPOINT+'apply', json=candidate, headers=TOKEN))
            try:
                await wait_until(core.queued.is_set)
                assert not apply_task.done()
                assert setup.teleop.manual.bundle() == before
                snapshot = await (await client.get('/api/helm/state')).json()
                assert snapshot['teleop']['tuning']['can_apply'] is False
                assert snapshot['teleop']['tuning']['rows'][0]['kp'] == before['joints'][0]['kp']
                for key in ('H', 'X'):
                    response = await asyncio.wait_for(client.post('/api/helm/teleop/key',
                        json={'key': key}, headers=TOKEN), timeout=1.)
                    assert response.status == 200
                await wait_until(lambda: core.keys == ['H', 'X'])
                assert not apply_task.done()  # Keys arrived while the ACK was still withheld.
                refused = await client.post(ENDPOINT+'apply', json=candidate, headers=TOKEN)
                assert refused.status == 422 and len(core.requests) == 1
            finally:
                core.release.set()
                response = await asyncio.wait_for(apply_task, timeout=2.)
            assert response.status == 200
            tuning = (await response.json())['teleop']['tuning']
            assert tuning['revision'] == before['revision'] + 1
            assert tuning['rows'][0]['kp'] == 25.
            assert setup.teleop.manual.active == candidate['joints']
            assert not setup.teleop.manual.path.exists()  # Applied is not implicitly saved.
            stale = await client.post(ENDPOINT+'apply', json=candidate, headers=TOKEN)
            assert stale.status == 422
    asyncio.run(run())


def test_receiver_rejection_preserves_active_settings_and_revision(setup):
    async def run():
        core = await start(setup)
        before = setup.teleop.manual.bundle()
        core.reject = 'Receiver rejected held gain update: feedback_not_settled'
        core.release.set()
        with pytest.raises(ValueError, match='feedback_not_settled'):
            await asyncio.to_thread(setup.teleop.manual_tuning, 'apply', payload(setup.teleop, kp=25.))
        assert setup.teleop.manual.bundle() == before
        assert setup.teleop.snapshot()['tuning']['can_apply']
    asyncio.run(run())


def test_late_view_ack_reconciles_after_caller_timeout_without_resending(setup, monkeypatch):
    original_request = setup.teleop._request
    monkeypatch.setattr(setup.teleop, '_request', lambda kind, *args: original_request(kind, *args, timeout=.05))

    async def run():
        core = await start(setup)
        before = setup.teleop.manual.bundle()
        candidate = payload(setup.teleop, kp=30.)
        with pytest.raises(ValueError, match='did not answer the change in time'):
            await asyncio.to_thread(setup.teleop.manual_tuning, 'apply', candidate)
        assert core.queued.is_set() and setup.teleop.manual.bundle() == before
        await wait_until(lambda: setup.teleop.snapshot()['manual_tuning_pending'])
        assert not setup.teleop.snapshot()['tuning']['can_apply']
        assert not setup.teleop.snapshot()['tuning']['can_save']
        with pytest.raises(ValueError):
            setup.teleop.manual_tuning('save', payload(setup.teleop, action='save'))
        assert not setup.teleop.manual.path.exists()
        core.release.set()
        await wait_until(lambda: setup.teleop.snapshot()['tuning']['revision'] == before['revision'] + 1)
        assert setup.teleop.manual.active == candidate['joints']
        assert len(core.requests) == 1
        assert setup.teleop.snapshot()['tuning']['can_apply']
        with pytest.raises(ValueError, match='changed or the session restarted'):
            setup.teleop.manual_tuning('apply', candidate)
    asyncio.run(run())


def test_http_save_and_restore_keep_preparation_separate_from_motor_enable(setup):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        async with TestClient(TestServer(setup.app())) as client:
            response = await client.post(ENDPOINT+'restore', json=payload(setup.teleop, action='restore'), headers=TOKEN)
            assert response.status == 422 and not setup.teleop.manual.path.exists()
            response = await client.post(ENDPOINT+'apply', json=payload(setup.teleop, kp=20.), headers=TOKEN)
            assert response.status == 200
            applied = setup.teleop.manual.bundle()
            response = await client.post(ENDPOINT+'save', json=payload(setup.teleop, action='save'), headers=TOKEN)
            assert response.status == 200
            document = json.loads(setup.teleop.manual.path.read_text(encoding='utf-8'))
            assert document['joints'] == applied['joints'] and document['measured_calibration'] is False
            await client.post(ENDPOINT+'apply', json=payload(setup.teleop, kp=40.), headers=TOKEN)
            response = await client.post(ENDPOINT+'restore', json=payload(setup.teleop, action='restore'), headers=TOKEN)
            tuning = (await response.json())['teleop']['tuning']
            assert response.status == 200 and values(tuning['rows']) == applied['joints']
            assert tuning['revision'] == applied['revision'] + 2
            assert not tuning['dirty']
            assert not setup.teleop.active and not setup.cores
    asyncio.run(run())


def test_restore_during_hold_also_waits_for_receiver_ack(setup):
    async def run():
        setup.teleop.manual_tuning('save', payload(setup.teleop, action='save'))
        saved = deepcopy(setup.teleop.manual.active)
        setup.teleop.manual_tuning('apply', payload(setup.teleop, kp=40.))
        core = await start(setup)
        before = setup.teleop.manual.bundle()
        task = asyncio.create_task(asyncio.to_thread(setup.teleop.manual_tuning,
                                  'restore', payload(setup.teleop, action='restore')))
        try:
            await wait_until(core.queued.is_set)
            assert core.requests[0]['joints'] == saved
            assert not task.done() and setup.teleop.manual.bundle() == before
        finally:
            core.release.set()
            await asyncio.wait_for(task, timeout=2.)
        assert setup.teleop.manual.active == saved
        assert setup.teleop.manual.revision == before['revision'] + 1
        assert not setup.teleop.snapshot()['tuning']['dirty']
    asyncio.run(run())


def test_faulted_running_session_accepts_http_disable_but_no_motion_or_tuning(setup):
    from aiohttp.test_utils import TestClient, TestServer

    async def run():
        core = await start(setup)
        core.fault, core.state = 'Receiver stopped: tracking_error', 'FAULT'
        await wait_until(lambda: setup.teleop.snapshot()['fault'] == core.fault)
        async with TestClient(TestServer(setup.app())) as client:
            for key in ('I', 'E', 'T', 'X'):
                response = await client.post('/api/helm/teleop/key', json={'key': key}, headers=TOKEN)
                assert response.status == 422
            response = await client.post(ENDPOINT+'apply', json=payload(setup.teleop, kp=20.), headers=TOKEN)
            assert response.status == 422 and not core.requests
            response = await asyncio.wait_for(client.post('/api/helm/teleop/key',
                json={'key': 'H'}, headers=TOKEN), timeout=1.)
            assert response.status == 200
            await wait_until(lambda: core.keys == ['H'])
            assert (await response.json())['teleop']['fault'] == core.fault
    asyncio.run(run())


@pytest.mark.parametrize('state, lowering, owned', [
    ('SHUTDOWN', True, True), ('FAULT', False, False), ('DISABLED', False, False)])
def test_fault_disable_reaches_adapter_while_lowering_exhausted_or_already_disabled(state, lowering, owned):
    calls = []
    adapter = GuardedLifecycleSession.__new__(GuardedLifecycleSession)
    adapter.lifecycle = SimpleNamespace(state=state, action=lambda *args, **kwargs: calls.append('local-disable'))
    adapter.clock = lambda: 0.
    adapter.owned = owned
    adapter.controller = SimpleNamespace(owned=owned, disable=lambda: calls.append('disable'),
        emergency_stop=lambda: calls.append('emergency-stop'))
    core = TeleopCore.__new__(TeleopCore)
    core.session, core.fault, core._lowering, core.live_arm = adapter, 'original tracking fault', lowering, 'left'
    core._lower = lambda _: pytest.fail('H must not schedule another lowering cycle')
    core._tick('H', 0.)
    assert calls == ['local-disable', 'disable' if owned else 'emergency-stop']
    assert not core._lowering and not adapter.owned
    assert core.fault == 'original tracking fault' and 'Disabled at your request' in core.message
    core._tick(None, .01)
    assert len(calls) == 2  # No automatic rearm or further motion after operator Disable.


def test_fault_disable_failure_preserves_original_fault_and_reports_unconfirmed_stop():
    def fail():
        raise OSError('receiver channel closed')

    core = TeleopCore.__new__(TeleopCore)
    core.session = SimpleNamespace(disable_now=fail, stop_error=None)
    core.fault, core._lowering, core.live_arm = 'original tracking fault', True, 'left'
    core._lower = lambda _: pytest.fail('An unconfirmed Disable must not resume lowering')
    core._tick('H', 0.)
    assert core.fault == 'original tracking fault'
    assert core.session.stop_error == 'receiver channel closed'
    assert not core._lowering
    assert 'Disable could not be confirmed' in core.message and 'receiver channel closed' in core.message
    assert 'Disabled at your request' not in core.message
    core._tick(None, .01)


def test_velocity_feedforward_switches_between_sessions_only(setup):
    """Apply sets Motor tuning's switch for the next session; a running session keeps the law it started with."""
    async def run():
        update = payload(setup.teleop)
        update['velocity_feedforward'] = True
        setup.teleop.manual_tuning('apply', update)
        assert setup.teleop.manual.feedforward is True
        assert setup.teleop._bundle()['manual_tuning']['velocity_feedforward'] is True
        core = await start(setup)
        change = payload(setup.teleop)
        change['velocity_feedforward'] = False
        with pytest.raises(ValueError, match='next session start'):
            await asyncio.to_thread(setup.teleop.manual_tuning, 'apply', change)
        assert setup.teleop.manual.feedforward is True and not core.requests
        with pytest.raises(ValueError, match='Invalid manual tuning request'):
            setup.teleop.manual_tuning('apply', dict(payload(setup.teleop), velocity_feedforward='on'))
    asyncio.run(run())


def test_status_names_the_next_sessions_law_then_the_one_the_receiver_runs(setup):
    """The teleop status row's Control cell: between sessions, the law the next one starts with; while one runs,
    the law its core reports, so a receiver that did not take the switch shows."""
    def law():
        snapshot = setup.teleop.snapshot()
        return snapshot['control_law'], snapshot['control_law_next']

    async def run():
        assert law() == ('direct-position-only', True)
        update = payload(setup.teleop)
        update['velocity_feedforward'] = True
        setup.teleop.manual_tuning('apply', update)
        assert law() == ('direct-velocity-feedforward', True)
        core = await start(setup)
        assert law() == ('direct-velocity-feedforward', False)
        core.law = dict(position_only=True, velocity_feedforward=False)  # A release that ignores the switch.
        await wait_until(lambda: law() == ('direct-position-only', False))
        await asyncio.to_thread(setup.teleop.stop, 2.)
        assert law() == ('direct-velocity-feedforward', True)
    asyncio.run(run())
