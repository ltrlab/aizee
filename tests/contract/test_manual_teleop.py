"""Manual settings through the real Python lifecycle and native receiver, without motors."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from robot_platform.helm.manual_tuning import ManualTuning
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, Receiver, run, DEVICES, SEED


@pytest.fixture(params=['minerva-teleop-direct-tuning.json', 'minerva-teleop-direct-tuning-vff.json'])
def manual_bundle(bundle, tmp_path, request):
    """Each manual-tuning profile: position-only, and the opt-in velocity feed-forward one."""
    data, path = bundle
    profile = json.loads((DEVICES / request.param).read_text(encoding='utf-8'))
    profile['leader_profile'] = str(DEVICES / 'gello-dual-viewer.json')
    path.write_text(json.dumps(profile))
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'other',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    try:
        manual = ManualTuning(helm, profile, tmp_path / 'manual', None)
        yield dict(data, manual_tuning=manual.bundle()), manual
    finally:
        helm.close()


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_manual_gain_apply_holds_pose_and_requires_ack_before_publication(manual_bundle, arm):
    data, manual = manual_bundle
    clock = Clock()
    leaders = Leaders(clock)
    requests = []

    def receiver(config, device, reference):
        result = Receiver(config, reference, clock)
        original = result.exchange
        def exchange(request):
            requests.append(deepcopy(request))
            return original(request)
        result.exchange = exchange
        return result

    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': arm}), clock=clock,
        open_leaders=lambda _: leaders, open_receiver=receiver,
        open_backend=lambda _: pytest.fail('No collision backend'))
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock)
        before = core.session.lifecycle.target
        update = manual.bundle()
        row = next(r for r in update['joints'] if r['motor'] == arm + '_arm_j1')
        row.update(kp=45., kd=3., follow=True, speed=.08, travel_rad=.05)
        core.queue_tuning(update)
        run(core, clock)
        for _ in range(3):
            if core._tuning_result:
                break
            run(core, clock)
        assert core._tuning_result[0] is None
        assert core._manual_state['revision'] == 1
        assert core.session.lifecycle.target == before
        writes = [r for r in requests if r.get('update_hold_gains')]
        assert len(writes) == 1
        assert writes[0]['joints'][0]['kp'] == 45.
        assert core.session.lifecycle.joints[0]['hold_kp'] == 45.
        run(core, clock, 'T', until='TELEOP', limit=1000)
        for _ in range(25):
            previous = core.session.lifecycle.target
            leaders.offsets[arm][0] += .002
            run(core, clock)
            assert abs(core.session.lifecycle.target[0] - previous[0]) <= .08 * core.period_s + 1e-9
            assert core.session.lifecycle.target[1:] == before[1:]
        with pytest.raises(ValueError, match='Hold'):
            core.queue_tuning(core._manual_state)
        run(core, clock, 'T')
        with pytest.raises(ValueError, match='changed|session'):
            core.queue_tuning(update)
        run(core, clock, 'H')
    finally:
        core.close()


def test_manual_tuning_drives_both_live_arms_in_one_receiver(manual_bundle):
    """Both arms live: one gain write covers both, and each J1 follows its own leader within its own cap."""
    data, manual = manual_bundle
    clock = Clock()
    leaders = Leaders(clock)
    requests = []

    def receiver(config, device, reference):
        result = Receiver(config, reference, clock)
        original = result.exchange
        def exchange(request):
            requests.append(deepcopy(request))
            return original(request)
        result.exchange = exchange
        return result

    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'both'}), clock=clock,
        open_leaders=lambda _: leaders, open_receiver=receiver,
        open_backend=lambda _: pytest.fail('No collision backend'))
    try:
        core.build()
        names = [joint['name'] for joint in core.derived['config']['joints']]
        assert len(names) == 14
        left, right = names.index('left_arm_j1'), names.index('right_arm_j1')
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock)
        before = list(core.session.lifecycle.target)
        update = manual.bundle()
        for motor, kp in (('left_arm_j1', 45.), ('right_arm_j1', 55.)):
            next(r for r in update['joints'] if r['motor'] == motor).update(
                kp=kp, kd=3., follow=True, speed=.08, travel_rad=.05)
        core.queue_tuning(update)
        run(core, clock)
        for _ in range(3):
            if core._tuning_result:
                break
            run(core, clock)
        assert core._tuning_result[0] is None
        assert core._manual_state['revision'] == 1
        assert list(core.session.lifecycle.target) == before  # Apply in HOLD moves nothing.
        writes = [r for r in requests if r.get('update_hold_gains')]
        assert len(writes) == 1  # One write for both arms.
        assert writes[0]['joints'][left]['kp'] == 45. and writes[0]['joints'][right]['kp'] == 55.
        run(core, clock, 'T', until='TELEOP', limit=1000)
        for _ in range(25):
            previous = list(core.session.lifecycle.target)
            leaders.offsets['left'][0] += .002
            leaders.offsets['right'][0] += .002
            run(core, clock)
            target = core.session.lifecycle.target
            for index in (left, right):
                assert abs(target[index] - previous[index]) <= .08 * core.period_s + 1e-9
            assert all(target[i] == before[i] for i in range(len(names)) if i not in (left, right))
        assert target[left] != before[left] and target[right] != before[right]  # Both arms followed.
        # What the receiver took while following: the law's velocity, held joints and grippers none.
        moves = [r for r in requests[-20:] if r.get('type') == 'direct_update' and not r.get('update_hold_gains')]
        held = [i for i, name in enumerate(names) if i not in (left, right)]
        assert moves and all(r['joints'][i]['velocity'] == 0. for r in moves for i in held)
        followed = [r['joints'][i]['velocity'] for r in moves for i in (left, right)]
        if core.derived['config']['direct']['position_only']:
            assert all(v == 0. for v in followed)
        else:
            assert all(0. < abs(v) <= .08 + 1e-9 for v in followed[-4:]), followed  # Its smoothed speed, in the cap.
        run(core, clock, 'H')
    finally:
        core.close()


def test_stopped_manual_edits_save_restore_and_stale_revision(manual_bundle, tmp_path):
    data, _ = manual_bundle
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'web',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    try:
        session = TeleopSession(helm, data['teleop_profile'], runs=tmp_path / 'web')
        original = session.manual.bundle()
        edited = deepcopy(original)
        edited['joints'][0].update(kp=42., follow=True, speed=.2, travel_rad=None)
        session.manual_tuning('apply', edited)
        assert session.snapshot()['tuning']['rows'][0]['kp'] == 42.
        assert session.snapshot()['tuning']['can_apply']
        with pytest.raises(ValueError, match='changed'):
            session.manual_tuning('apply', original)
        version = {k: session.manual.bundle()[k] for k in ('session_id', 'revision')}
        session.manual_tuning('save', version)
        session.manual_tuning('apply', dict(session.manual.bundle(), joints=original['joints']))
        version['revision'] += 1
        session.manual_tuning('restore', version)
        assert session.snapshot()['tuning']['rows'][0]['kp'] == 42.
        assert session.snapshot()['tuning']['dirty'] is False
        assert not session.active
        assert session.snapshot()['receiver_choices'] == ['virtual', 'left', 'right', 'both']
    finally:
        helm.close()
