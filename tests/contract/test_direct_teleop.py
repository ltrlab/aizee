"""Direct one/two-arm sessions against the actual receiver; no collision backend."""
import json
import math
import gc
from copy import deepcopy

import pytest

from robot_platform.helm.teleop import TeleopCore, load_profile
from robot_platform.control.live_receiver import service_config
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, Receiver, run  # noqa: F401


@pytest.mark.parametrize('previously_enabled', [False, True])
@pytest.mark.parametrize('failed_stop', [False, True])
def test_direct_loop_restores_gc_only_after_all_resources_close(bundle, monkeypatch,
                                                              previously_enabled, failed_stop):
    data, path = bundle
    direct_profile(path)
    clock = Clock()
    leaders = Leaders(clock)
    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'left', 'motor_models': {}}),
                     open_leaders=lambda _: leaders,
                     open_receiver=lambda c, d, r: Receiver(c, r, clock),
                     open_backend=lambda _: pytest.fail('No geometry in direct mode'), clock=clock)
    original = gc.isenabled()
    (gc.enable if previously_enabled else gc.disable)()
    try:
        core.build()
        assert not gc.isenabled()
        closed = []
        core._closers.append(lambda: closed.append(gc.isenabled()))
        if failed_stop:
            def broken(*args, **kwargs):
                raise RuntimeError('injected stop failure')
            monkeypatch.setattr(core.session, 'disable', broken)
            monkeypatch.setattr(core.session, 'fault', broken)
        core.close()
        assert closed == [False]
        assert gc.isenabled() is previously_enabled
        core.close()
        assert gc.isenabled() is previously_enabled
    finally:
        core.close()
        (gc.enable if original else gc.disable)()


def direct_profile(path, hz=100):
    profile = json.loads(path.read_text())
    receiver = profile['receiver']
    receiver.update(collision_mode='disabled', loop_hz=hz, interpolation_ms=20 if hz == 50 else 0,
                    direct=dict(command_ms=max(30, 3000//hz), velocity_timeout_ms=math.ceil(1500/hz), limit_reaction_ms=10))
    receiver.pop('collision', None)
    receiver.pop('compute_budget_s', None)
    path.write_text(json.dumps(profile))
    return profile


@pytest.mark.parametrize('arm', ['left', 'both'])
@pytest.mark.parametrize('hz', [50, 100, 200])
def test_direct_session_never_opens_geometry_and_follows_with_one_update(bundle, arm, hz):
    data, path = bundle
    direct_profile(path, hz)
    clock = Clock()
    leaders = Leaders(clock)
    requests, opened = [], []

    def open_receiver(config, device, reference):
        receiver = Receiver(config, reference, clock)
        original = receiver.exchange
        def exchange(request):
            requests.append(request)
            return original(request)
        receiver.exchange = exchange
        opened.append((config, device, reference))
        return receiver

    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': arm, 'motor_models': {}}),
                     open_leaders=lambda _: leaders, open_receiver=open_receiver,
                     open_backend=lambda _: pytest.fail('collision backend must not be opened'), clock=clock)
    try:
        core.build()
        assert core.compiler is None and core.collision_mode == 'disabled'
        assert len(core.derived['config']['joints']) == (7 if arm == 'left' else 14)
        assert 'collision' not in core.derived['config']
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=hz*10)
        requests.clear()
        before = core.session.lifecycle.target
        for n in range(25):
            leaders.offsets['left'][0] += .0005
            run(core, clock)
        assert len(requests) == 25
        assert {r['type'] for r in requests} == {'direct_update'}
        assert core.session.lifecycle.target != before
        assert core.harness.last['status']['collision_expires_at_ms'] is None
        run(core, clock, 'T')
        assert core.session.lifecycle.state == 'HOLD'
        seconds = max(abs(q - j['zero_position']) / j['shutdown_speed']
                      for q, j in zip(core.session.lifecycle.target, core.session.lifecycle.joints)) + 2
        try:
            run(core, clock, 'X', until='DISABLED', limit=math.ceil(seconds / core.period_s))
        except AssertionError:
            evidence = dict(view=core.view(), cycle=core.session.last_cycle,
                            feedback=core.session.last_feedback, config=core.derived)
            (path.parent / 'direct-failure.json').write_text(json.dumps(evidence, indent=2))
            print(json.dumps(core.view()['joints'], indent=2))
            raise
    finally:
        core.close()


def test_disabled_profile_rejects_implicit_or_conflicting_configuration(bundle):
    _, path = bundle
    profile = direct_profile(path)
    profile['receiver']['collision'] = {'scope': 'all'}
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match='Direct teleop'):
        load_profile(path)
    profile['receiver'].pop('collision')
    profile['receiver'].pop('collision_mode')
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError, match='require disabled'):
        load_profile(path)


@pytest.mark.parametrize('arm', ['left', 'both'])
def test_direct_repeated_moving_goal_keeps_bounded_motion_and_loss_requires_engage(bundle, arm):
    data, path = bundle
    direct_profile(path)
    clock, cached, frozen = Clock(), {}, [False]
    leaders = Leaders(clock)
    poll = leaders.poll
    def source():
        if not frozen[0]:
            cached['samples'] = poll()
        return cached['samples']
    leaders.poll = source
    requests = []
    def open_receiver(config, _, reference):
        receiver = Receiver(config, reference, clock)
        exchange = receiver.exchange
        def recorded(request):
            requests.append(deepcopy(request))
            return exchange(request)
        receiver.exchange = recorded
        return receiver
    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': arm, 'motor_models': {}}),
        open_leaders=lambda _: leaders, open_receiver=open_receiver,
        open_backend=lambda _: pytest.fail('no geometry'), clock=clock)
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=1000)
        assert core.session.lifecycle.max_source_misses == 8
        previous = core.session.lifecycle.target[0]
        leaders.offsets['left'][0] += .03
        run(core, clock)
        target = deepcopy(core.session.last_cycle['target'])
        first = core.session.last_cycle['proposal']['joints'][0]
        goal = target[0]['position']
        direction = math.copysign(1., goal-previous)
        assert direction * first['velocity'] > 0
        assert direction * (goal-first['position']) > .005
        joint = core.session.lifecycle.joints[0]
        requests.clear()
        frozen[0] = True
        previous = first
        for _ in range(2):
            run(core, clock)
            cycle = core.session.last_cycle
            command = cycle['proposal']['joints'][0]
            assert cycle['accepted'] and cycle['rpc_count'] == 1
            assert cycle['target'] == target  # No new sequence, timestamp or goal.
            assert core._input_timing['repeated_source'] is True
            assert core.session.lifecycle.state == 'TELEOP'
            assert direction * (command['position']-previous['position']) > 0
            assert direction * (goal-command['position']) >= -1e-12
            assert direction * command['velocity'] > 0
            # Unramped direct control feeds the commands' smoothed speed forward (test_velocity_feedforward), not
            # one cycle's step: a repeated sample keeps it, in the direction of travel, within every bound.
            assert joint['velocity_feedforward'] is True
            assert abs(command['velocity']) <= joint['max_velocity'] + 1e-9
            assert abs(command['velocity']) <= abs(previous['velocity']) + (
                joint['approach']['stop_acceleration'] * core.period_s) + 1e-9
            previous = command
        assert len(requests) == 2 and all(r['type'] == 'direct_update' for r in requests)
        assert core.mapper.fault is None
        assert all(0 < r['valid_for_ms'] <= 30 for r in requests)
        run(core, clock, until='HOLD', limit=20)
        frozen[0] = False
        run(core, clock, limit=10)
        assert core.session.lifecycle.state == 'HOLD'
        assert all(j['velocity'] == 0 for j in core.session.last_cycle['proposal']['joints'])
        run(core, clock, 'T', until='TELEOP', limit=1000)
        run(core, clock, 'H')
        assert core.session.lifecycle.state == 'DISABLED'
    finally:
        core.close()


def test_repeated_goal_keeps_original_source_deadline_across_new_receiver_offers(bundle):
    """New receiver tokens cannot extend the original source sample's validity."""
    data, path = bundle
    direct_profile(path)
    clock, requests, receivers = Clock(), [], []
    leaders = Leaders(clock)

    def open_receiver(config, _, reference):
        receiver = Receiver(config, reference, clock)
        exchange = receiver.exchange
        def recorded(request):
            offered = deepcopy((receiver.last.get('reply') or {}).get('offer'))
            reply = exchange(request)
            if request['type'] == 'direct_update':
                requests.append(dict(request=deepcopy(request), offered=offered, reply=deepcopy(reply)))
            return reply
        receiver.exchange = recorded
        receivers.append(receiver)
        return receiver

    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'left', 'motor_models': {}}),
        open_leaders=lambda _: leaders, open_receiver=open_receiver,
        open_backend=lambda _: pytest.fail('no geometry'), clock=clock)
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=1000)
        requests.clear()
        run(core, clock)
        target = deepcopy(core.session.last_cycle['target'])
        original = deepcopy(target)
        expires = min(row['sampled_at'] for row in target) + core.session.lifecycle.target_max_age_s
        receiver_bound = math.floor((expires-receivers[0].epoch)*1000 + 1e-7)

        # Exercise the session's own freshness guard up to the boundary. Helm
        # additionally withholds goals within its unchanged dispatch margin.
        for _ in range(4):
            cycle = core.session.cycle(target=deepcopy(target), started_at=clock())
            assert cycle['accepted'] and cycle['rpc_count'] == 1
            assert cycle['target'] == original
            assert core.session.lifecycle.state == 'TELEOP'
            clock.now += core.period_s

        assert target == original and len(requests) == 5
        issued = [r['offered']['issued_at_ms'] for r in requests]
        lifetimes = [r['request']['valid_for_ms'] for r in requests]
        deadlines = [r['reply']['status']['command_expires_at_ms'] for r in requests]
        assert all(a < b for a, b in zip(issued, issued[1:]))
        assert all(0 < lifetime <= 30 for lifetime in lifetimes)
        assert all(a >= b for a, b in zip(lifetimes, lifetimes[1:]))
        assert lifetimes[-1] < lifetimes[0]
        assert all(deadline <= receiver_bound for deadline in deadlines)
        # Receiver offer time and remaining source lifetime are independently
        # floored to milliseconds, so the conservative endpoint may lose 2 ms.
        assert deadlines[-1] >= receiver_bound-2
        assert deadlines[-1] <= deadlines[-2] + 1  # Fresh token, original source expiry.
        for row, deadline in zip(requests, deadlines):
            assert row['reply']['accepted']
            assert deadline == min(row['offered']['expires_at_ms'],
                                   row['offered']['issued_at_ms'] + row['request']['valid_for_ms'])

        assert clock()-min(row['sampled_at'] for row in target) >= .05
        with pytest.raises(ValueError, match='stale mapped source'):
            core.session.cycle(target=deepcopy(target), started_at=clock())
        assert len(requests) == 5  # Expired source does not dispatch another direct command.
        assert core.session.lifecycle.state == 'FAULT'
        assert core.session.last_cycle['target'] == original
        assert len(core.session.last_cycle['feedback']) == len(original)
        assert len(core.session.last_cycle['feedback_age_ms']) == len(original)
        assert core.session.last_cycle['lifecycle_before'] == 'TELEOP'
        assert core.session.last_cycle['lifecycle_after'] == 'FAULT'
        assert receivers[0].last['status']['state'] in ('stopping', 'stopped')
    finally:
        core.close()


@pytest.mark.parametrize('arm', ['left', 'both'])
def test_direct_command_expiry_recovers_only_to_shutdown(bundle, arm):
    """A stalled producer revokes ownership; recovery lowers instead of resuming mirroring."""
    data, path = bundle
    direct_profile(path)
    clock = Clock()
    leaders = Leaders(clock)
    requests = []

    def open_receiver(config, device, reference):
        receiver = Receiver(config, reference, clock)
        exchange = receiver.exchange

        def recorded(request):
            requests.append(request['type'])
            return exchange(request)

        receiver.exchange = recorded
        return receiver

    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': arm, 'motor_models': {}}),
                     open_leaders=lambda _: leaders, open_receiver=open_receiver,
                     open_backend=lambda _: pytest.fail('recovery must not open geometry'), clock=clock)
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=1000)
        for _ in range(25):
            leaders.offsets['left'][0] += .001
            run(core, clock)
        run(core, clock, limit=20)
        joints = core.session.lifecycle.joints
        assert any(abs(q - j['zero_position']) > j['tolerance']
                   for q, j in zip(core.session.lifecycle.target, joints))

        # Beyond the direct command deadline, but still inside the 150 ms lease.
        # The harness advances when the next RPC observes the stalled producer.
        clock.now += .06
        core.tick(None, clock())
        stopped = core.harness.last['status']
        assert stopped['state'] in ('stopping', 'stopped')
        assert stopped['reason'] == 'command_expired'
        assert stopped['lease'] is None and not core.session.controller.owned
        assert core.session.lifecycle.state == 'FAULT'
        assert core.fault and 'command_expired' in core.fault
        assert core._lowering
        fault = core.fault

        # Even an Engage key on every recovery cycle must not follow the leaders.
        # Keep the leaders displaced so merely re-entering TELEOP could not pass.
        states = []
        shutdown_seconds = max(abs(row['position'] - joint['zero_position']) / joint['shutdown_speed']
                               for row, joint in zip(core.harness.last['feedback'], joints)) + 3
        for _ in range(math.ceil(shutdown_seconds / core.period_s)):
            clock.now += core.period_s
            core.tick('T', clock())
            state = core.session.lifecycle.state
            states.append(state)
            assert state not in ('ENGAGING', 'TELEOP')
            if state == 'DISABLED':
                break

        assert 'SHUTDOWN' in states
        assert states[-1] == 'DISABLED', core.message
        assert core.fault == fault and not core._lowering
        assert requests.count('acquire') == 2 and requests.count('arm') == 2
        assert not core.session.owned and not core.session.controller.owned
        assert core.harness.last['status']['state'] == 'disarmed'
        assert not core.harness.last['hardware_enabled']
        assert all(abs(row['position'] - joint['zero_position']) <= joint['tolerance'] + 1e-9
                   for row, joint in zip(core.harness.last['feedback'], joints))
        assert 'lowered to the reference and disabled' in core.message
    finally:
        core.close()
