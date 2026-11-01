"""Joint zero: arm joints' motor zeros at the zero pose, proven by a power cycle, and the arms' new reference with what
is bound to it moved onto it (helm.joint_zero)."""
import asyncio
import json
from pathlib import Path

import pytest

from robot_platform.control.joint_calibration import arms_reference_name, digest, load_plan, operator_reference
from robot_platform.helm.joint_zero import JointZero, next_reference, pending_path
from robot_platform.helm.operator_files import operator_files, prepare
from robot_platform.helm.session import HelmSession
from robot_platform.ops import probe

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-joint-calibration.json'
TELEOP = ROOT / 'config/devices/minerva-teleop.json'
TUNING = ROOT / 'config/devices/minerva-teleop-direct-tuning.json'
SEED = ROOT / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
ASSET = 'minerva-production-01'
ZEROED = -1.654  # What left J5 read at the zero pose before its zero.


class Maintenance:
    """Stands in for StoppedMaintenance: zeroes the roster in order, failing at `fail_at` if given."""
    runs, fail_at = [], None

    def __init__(self, exchange):
        self.exchange = exchange

    def run(self, roster, zero, **options):
        Maintenance.runs.append(([m['name'] for m in roster], options))
        zero.update(success=True, motors=[])
        for motor in roster:
            if motor['name'] == Maintenance.fail_at:
                zero.update(success=False, error=f"{motor['name']}: pose changed after preflight")
                break
            zero['motors'].append(dict(name=motor['name'], uid=motor['uid'], firmware=motor['firmware'],
                                       before_rad=[ZEROED, ZEROED], zero_requested=True, zero_reply={'ok': 1},
                                       after_zero_rad=[0., .0001], save_requested=True, after_save_rad=[.0001, 0.]))
        return zero


def capture(device, reference, *, maintain):
    """As run_capture: a failure in the maintenance is the session's error."""
    result = {'success': True}
    try:
        maintain('exchange', reference['motors'], result)
    except Exception as exc:
        result.update(success=False, error=str(exc))
    return result


def test_the_robot_zeroes_and_saves_only_the_chosen_arm_joints(tmp_path, monkeypatch):
    monkeypatch.setattr(probe, 'EVIDENCE', tmp_path)
    Maintenance.runs.clear()
    Maintenance.fail_at = None
    report = probe.zero_joints(['left_arm_j4', 'left_arm_j5'], capture=capture, maintenance=Maintenance)
    (roster, options), = Maintenance.runs
    assert roster == ['left_arm_j4', 'left_arm_j5']  # Every other drive is left alone.
    assert options['reference_confirmed'] is True and options['save'] is True
    assert report['success'] and [joint['motor'] for joint in report['joints']] == ['left_arm_j4', 'left_arm_j5']
    assert all(joint['zero_sent'] and joint['before_rad'] == ZEROED and joint['after_save_rad'] == 0.
               for joint in report['joints'])
    assert Path(report['evidence']).name.startswith('mechanical-zero-left-j4-j5-')
    for refused in (['left_gripper'], ['left_arm_j7'], ['left_arm_j1', 'left_arm_j1'], []):
        with pytest.raises(ValueError, match='Only arm joints'):
            probe.zero_joints(refused, capture=capture, maintenance=Maintenance)


def test_a_failure_part_way_says_which_drives_were_zeroed(tmp_path, monkeypatch):
    monkeypatch.setattr(probe, 'EVIDENCE', tmp_path)
    Maintenance.fail_at = 'right_arm_j6'
    try:
        report = probe.zero_joints(['right_arm_j5', 'right_arm_j6'], capture=capture, maintenance=Maintenance)
    finally:
        Maintenance.fail_at = None
    assert not report['success'] and 'pose changed' in report['error']
    assert [(joint['motor'], joint['zero_sent']) for joint in report['joints']] == [('right_arm_j5', True),
                                                                                   ('right_arm_j6', False)]
    assert Path(report['evidence']).name.startswith('mechanical-zero-right-j5-j6-')


def test_the_plan_takes_helms_copy_of_the_reference_in_place_of_its_own(tmp_path):
    plan = load_plan(PROFILE)
    assert operator_reference(PROFILE, None) is None and operator_reference(PROFILE, tmp_path) is None
    copy = tmp_path / arms_reference_name(ASSET)
    copy.write_bytes(plan[1]['reference'].read_bytes())
    assert operator_reference(PROFILE, tmp_path) == copy.resolve()
    same = load_plan(PROFILE, reference=copy)
    assert same[-1] == plan[-1] and same[1]['reference'] == copy.resolve()  # A byte copy binds the same.
    revised = dict(json.loads(copy.read_text()), id='minerva-mechanical-reference-revised')
    copy.write_text(json.dumps(revised))
    loaded = load_plan(PROFILE, reference=copy)
    assert loaded[3]['id'] == 'minerva-mechanical-reference-revised'
    assert loaded[-1]['reference'] == digest(copy.read_bytes()) != plan[-1]['reference']
    copy.write_text(json.dumps(dict(revised, power_cycle_verified=False)))
    with pytest.raises(ValueError, match='verified power cycle'):
        load_plan(PROFILE, reference=copy)


def robot(monkeypatch, replies, seen):
    """The robot's probes over a stand-in SSH: `replies` maps a probe's first word to its report."""
    from robot_platform.ops import ssh

    class Client:
        def close(self):
            pass

    def run(client, command, *, timeout):
        seen.append(command)
        word = command.split('robot_platform.ops.probe ')[1].split()[0]
        return 0, json.dumps(dict(ok=True, **replies[word])) + '\n', ''
    monkeypatch.setattr(ssh, 'run', run)
    return lambda profile, address: Client()


def zero_reply(motors, *, before=ZEROED):
    return {'motors': motors, 'success': True, 'error': None, 'power_cycle_verified': False,
            'evidence': '~/minerva/evidence/mechanical-zero-left-j5-20261101T200000Z.json',
            'joints': [dict(motor=motor, uid='u', firmware='f', zero_sent=True, zero_acknowledged=True, save_sent=True,
                            save_response_missing=False, before_rad=before, after_zero_rad=.0001, after_save_rad=0.)
                       for motor in motors]}


def scan_reply(session, readings):
    return {'success': True, 'started_at': '2026-11-01T20:05:00+00:00',
            'evidence': '~/minerva/evidence/check-motors-20261101T200500Z.json',
            'motors': [dict(name=m['name'], bus=m['bus'], id=m['id'], uid=m['uid'], firmware=m['firmware'],
                            position_rad=readings.get(m['name'], .2), reset_confirmed=True, zero_sta=1)
                       for m in session.reference['motors']]}


@pytest.fixture
def minerva(tmp_path):
    """Helm on Minerva's files: the operator folder seeded from the release, the direction session from the seed."""
    folder, runs = tmp_path / 'operator', tmp_path / 'runs'
    prepare(operator_files(PROFILE, ASSET, TELEOP), folder)
    session = HelmSession(PROFILE, runs=runs, seed=SEED, on_robot=True, operator_files=folder)
    teleop = runs / f'{ASSET}-teleop'
    teleop.mkdir(parents=True)
    (teleop / 'minerva-teleop-direct-tuning-manual.json').write_text(json.dumps(
        {'kind': 'manual_teleop_tuning', 'binding': {'calibration': session.bindings, 'servo_sha256': 's'}}))
    (teleop / 'routes.json').write_text(json.dumps({'routes': {}}))
    yield session, folder, runs
    session.close()


def test_a_zero_waits_for_the_power_cycle_and_verify_moves_everything_to_the_new_reference(minerva, monkeypatch):
    session, folder, runs = minerva
    old_reference, old_bindings = session.reference, dict(session.bindings)
    assert session.reference_path == (folder / arms_reference_name(ASSET)).resolve()
    assert 'left_arm_j5' in session.saved_limits['joints'] and session.saved_limits_error is None
    seen, replies = [], {'zero_joints': zero_reply(['left_arm_j5'])}
    zeroer = JointZero(session, runs=runs, open_ssh=robot(monkeypatch, replies, seen), remote_root='/r',
                       clock=lambda: 1793563200.)
    with pytest.raises(ValueError, match='confirm'):
        zeroer.zero(['left_arm_j5'])
    with pytest.raises(ValueError, match='arm joints'):
        zeroer.zero(['left_gripper'], confirm=True)
    assert seen == [] and zeroer.blocking() is None
    report = zeroer.zero(['left_arm_j5'], confirm=True)
    assert report['success'] and report['pending'] == ['left_arm_j5']
    assert seen == ['cd /r && PYTHONPATH=src python3 -u -m robot_platform.ops.probe zero_joints left_arm_j5']
    assert 'waiting for the power cycle' in zeroer.blocking()
    assert zeroer.snapshot()['pending'][0]['before_rad'] == ZEROED
    with pytest.raises(ValueError, match='confirm'):
        zeroer.verify()

    # The arm moved, or the drive lost the zero: nothing changes, the zero still waits.
    replies['motors'] = scan_reply(session, {'left_arm_j5': .2})
    failed = zeroer.verify(confirm=True)
    assert not failed['success'] and 'reads +0.200 rad' in failed['error'] and 'did not keep' not in failed['error']
    replies['motors'] = scan_reply(session, {'left_arm_j5': ZEROED + .003})
    assert 'did not keep the zero' in zeroer.verify(confirm=True)['error']
    assert zeroer.pending() and session.reference_path.read_bytes() == old_reference_bytes(old_reference, folder)

    replies['motors'] = scan_reply(session, {'left_arm_j5': .0004})
    verified = zeroer.verify(confirm=True)
    assert verified['success'] and verified['restart'] and verified['resweep'] == ['left_arm_j5']
    assert not pending_path(folder, ASSET).exists()
    assert 'restarting' in zeroer.blocking()
    with pytest.raises(ValueError, match='restarting'):
        zeroer.zero(['left_arm_j4'], confirm=True)

    reference_file = folder / arms_reference_name(ASSET)
    new = json.loads(reference_file.read_text())
    new_digest = digest(reference_file.read_bytes())
    assert new['id'].startswith('minerva-mechanical-reference-') and new['supersedes'] == old_reference['id']
    assert new['power_cycle_verified'] and new['rezeroed']['motors'] == ['left_arm_j5']
    assert set(new['direction_evidence_from']) >= {old_reference['id'], 'minerva-mechanical-reference-20260924'}
    j5 = next(m for m in new['motors'] if m['name'] == 'left_arm_j5')
    assert j5['raw_reference_rad'] == 0. and j5['before_zero_rad'] == ZEROED
    assert new['power_cycle_verification']['all_14_passed'] is True
    limits = json.loads(session.limits_path.read_text())
    assert limits['reference'] == new['id'] and limits['reference_sha256'] == new_digest
    assert 'left_arm_j5' not in limits['joints'] and len(limits['joints']) == len(session.saved_limits['joints']) - 1
    directions = json.loads(session.state_path.read_text())
    assert directions['bindings'] == dict(old_bindings, reference=new_digest) and directions['reference'] == new['id']
    teleop = runs / f'{ASSET}-teleop'
    preset = json.loads((teleop / 'minerva-teleop-direct-tuning-manual.json').read_text())
    assert preset['binding'] == {'calibration': dict(old_bindings, reference=new_digest), 'servo_sha256': 's'}
    assert json.loads((teleop / 'routes.json').read_text()) == {'routes': {}}
    kept = Path(verified['kept'])
    assert {path.name for path in kept.iterdir()} >= {
        f'replaced-{arms_reference_name(ASSET)}', f'replaced-{ASSET}-joint-limits.json', 'replaced-session.json',
        'replaced-minerva-teleop-direct-tuning-manual.json', f'replaced-{ASSET}-joint-zero.json', 'applied.json'}
    results = session.state['results']
    session.close()

    # Helm started again loads all of it: the new reference, its limits less left J5, the same directions.
    again = HelmSession(PROFILE, runs=runs, seed=SEED, on_robot=True, operator_files=folder)
    try:
        assert again.reference['id'] == new['id'] and again.bindings['reference'] == new_digest
        assert again.saved_limits_error is None and 'left_arm_j5' not in again.saved_limits['joints']
        assert again.directions_dir == runs.resolve() / f'{ASSET}-directions' and again.state['results'] == results
        assert JointZero(again, runs=runs).resweep() == ['left_arm_j5']
        assert JointZero(again, runs=runs).blocking() is None
    finally:
        again.close()


def old_reference_bytes(reference, folder):
    """Helm's copy is still the release's (byte for byte)."""
    return (folder / arms_reference_name(ASSET)).read_bytes()


def test_a_zero_that_stopped_part_way_keeps_the_drives_it_zeroed_waiting(minerva, monkeypatch):
    session, folder, runs = minerva
    reply = zero_reply(['right_arm_j5', 'right_arm_j6'])
    reply.update(success=False, error='right_arm_j6: pose changed after preflight')
    reply['joints'][1].update(zero_sent=False, before_rad=None, after_zero_rad=None, after_save_rad=None)
    zeroer = JointZero(session, runs=runs, open_ssh=robot(monkeypatch, {'zero_joints': reply}, []))
    report = zeroer.zero(['right_arm_j5', 'right_arm_j6'], confirm=True)
    assert not report['success'] and report['pending'] == ['right_arm_j5']
    assert set(zeroer.pending()['zeroed']) == {'right_arm_j5'}


def test_joint_zero_needs_helms_operator_files(tmp_path):
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', seed=SEED)
    try:
        zeroer = JointZero(session, runs=tmp_path / 'runs')
        assert 'operator files' in zeroer.unavailable() and zeroer.blocking() is None
        with pytest.raises(ValueError, match='operator files'):
            zeroer.zero(['left_arm_j5'], confirm=True)
        assert zeroer.snapshot()['available'] is False
    finally:
        session.close()


def test_the_new_revision_keeps_the_roster_and_raw_references(minerva):
    session = minerva[0]
    zeroed = {'left_arm_j4': dict(before_rad=-.62, after_save_rad=0., save_sent=True, save_response_missing=True,
                                  at='2026-11-01T20:00:00+00:00', evidence='e1')}
    scan = scan_reply(session, {'left_arm_j4': .0002})
    document = next_reference(session.reference, zeroed, scan, at=1793563200., labels=session.labels)
    assert [m['name'] for m in document['motors']] == [m['name'] for m in session.reference['motors']]
    assert all(m['raw_reference_rad'] == 0. for m in document['motors'])
    assert all(a['uid'] == b['uid'] for a, b in zip(document['motors'], session.reference['motors']))
    assert 'Left J4' in document['reference_description']
    assert document['power_cycle_verification']['rezeroed_max_absolute_readback_rad'] == .0002


def test_helm_starts_with_a_joint_to_sweep_again_and_refuses_sessions_until_it_is(minerva, monkeypatch):
    from robot_platform.helm.teleop import TeleopSession
    session, folder, runs = minerva
    complete = dict(session.saved_limits, joints=dict(session.saved_limits['joints']))
    session.saved_limits['joints'].pop('left_arm_j5')  # As Joint zero leaves them until the re-sweep.
    teleop = TeleopSession(session, TUNING, runs=runs)
    assert teleop.manual is None and 'left_arm_j5: no saved joint limits' in teleop.manual_error
    with pytest.raises(ValueError, match='no saved joint limits'):
        teleop.start(confirm=True)
    session.saved_limits = complete  # Swept and saved again.
    (runs / f'{ASSET}-teleop' / 'minerva-teleop-direct-tuning-manual.json').unlink()  # The fixture's stand-in.
    teleop.reload_tuning()
    assert teleop.manual is not None and teleop.manual_error is None


class FakeLink:
    def __init__(self):
        self.running, self.calls = True, []

    def connect(self):
        self.calls.append('connect')

    def disconnect(self, timeout=10.):
        self.calls.append('disconnect')
        self.running = False


class FakeZero:
    def __init__(self):
        self.running, self.calls, self.reason, self.verified = None, [], None, {'success': True, 'restart': True}

    def zero(self, motors, *, confirm=False):
        self.calls.append(('zero', motors, confirm))
        return {'success': True}

    def verify(self, *, confirm=False):
        self.calls.append(('verify', confirm))
        return self.verified

    def blocking(self):
        return self.reason

    def snapshot(self):
        return {'calls': len(self.calls)}


class FakeTune:
    active, mode = False, 'tune'
    receiver = {'mode': 'live', 'arm': 'left'}

    def __init__(self):
        self.started = []

    def start(self, *, confirm=False):
        self.started.append(confirm)

    def stop(self, timeout=None):
        pass

    def snapshot(self):
        return None

    def guide(self):
        return None


def test_the_server_zeroes_on_confirmation_and_holds_the_arms_until_verify(tmp_path, monkeypatch):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm import server
    monkeypatch.setattr(server, 'RESTART_DELAY_S', 0.)
    session = HelmSession(PROFILE, runs=tmp_path / 'runs', limits=tmp_path / 'limits.json', seed=SEED)
    link, zero, tune, restarts = FakeLink(), FakeZero(), FakeTune(), []
    headers = {'X-Helm-Token': 'secret'}

    async def run():
        app = server.create_app(session, link, token='secret', joint_zero=zero, tune=tune,
                                restart=lambda: restarts.append(True))
        async with TestClient(TestServer(app)) as client:
            post = lambda path, payload: client.post(path, json=payload, headers=headers)  # noqa: E731
            refused = await post('/api/helm/joint-zero/zero', {'motors': ['left_arm_j5']})
            assert refused.status == 422 and zero.calls == [] and link.calls == []
            state = await (await post('/api/helm/joint-zero/zero', {'motors': ['left_arm_j5'], 'confirm': True})).json()
            assert zero.calls == [('zero', ['left_arm_j5'], True)] and link.calls == ['disconnect']
            assert state['joint_zero'] == {'calls': 1, 'restarts': True}
            zero.reason = 'Left wrist J5: zeroed, waiting for the power cycle'
            held = await post('/api/helm/tune/start', {'confirm': True})
            assert held.status == 422 and 'waiting for the power cycle' in (await held.json())['error']
            assert tune.started == []
            assert (await post('/api/helm/joint-zero/verify', {})).status == 422
            await post('/api/helm/joint-zero/verify', {'confirm': True})
            await asyncio.sleep(.05)
            assert zero.calls[-1] == ('verify', True) and restarts == [True]
            session.connection = 'wifi'  # Stand-in for Via Wi-Fi: the zero needs the tether.
            wifi = await post('/api/helm/joint-zero/zero', {'motors': ['left_arm_j5'], 'confirm': True})
            assert wifi.status == 422 and len(zero.calls) == 2
    try:
        asyncio.run(run())
    finally:
        session.close()


def test_the_loop_on_minerva_reads_helms_copy_of_the_reference():
    from types import SimpleNamespace
    from robot_platform.helm.teleop import calibration_reference, edge_reference
    from robot_platform.ops.release import ROOT as RELEASE
    assert calibration_reference(SimpleNamespace(reference_path=None)) == {}
    copy = RELEASE.parent / 'operator' / arms_reference_name(ASSET)  # Outside the release (the gate's tmp is in it).
    assert calibration_reference(SimpleNamespace(reference_path=copy)) == {'calibration_reference': str(copy)}
    assert edge_reference(copy, SimpleNamespace(on_robot=True)) == str(copy.resolve())
    with pytest.raises(ValueError, match="this PC's operator files"):
        edge_reference(copy, SimpleNamespace(on_robot=False))
    inside = RELEASE / 'config/devices/minerva-reference-20261009.json'
    assert edge_reference(inside, SimpleNamespace(on_robot=False)) == 'config/devices/minerva-reference-20261009.json'


def test_minerva_check_reads_helms_copy_of_the_reference(minerva):
    from robot_platform.ops.check import Context, _calibration
    session, folder, runs = minerva
    reference_file = folder / arms_reference_name(ASSET)
    reference_file.write_text(json.dumps(dict(json.loads(reference_file.read_text()), id='revised-on-minerva')))
    context = Context(root=ROOT, profile=PROFILE, teleop=TELEOP, runs=runs, limits=None, address=None, motors=False,
                      open_ssh=None, operator_files=folder)
    status, detail = _calibration(context)[:2]
    assert 'revised-on-minerva' in detail and context.facts['bindings']['reference'] == digest(reference_file.read_bytes())
