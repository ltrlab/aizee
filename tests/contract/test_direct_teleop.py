"""Direct one/two-arm sessions against the actual receiver; no collision backend."""
import json
import math

import pytest

from robot_platform.helm.teleop import TeleopCore, load_profile
from robot_platform.control.live_receiver import service_config
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, Receiver, run  # noqa: F401


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


def test_direct_repeated_source_holds_and_loss_requires_explicit_engage(bundle):
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
    core = TeleopCore(dict(data, receiver={'mode': 'live', 'arm': 'both', 'motor_models': {}}),
        open_leaders=lambda _: leaders, open_receiver=lambda config, _, ref: Receiver(config, ref, clock),
        open_backend=lambda _: pytest.fail('no geometry'), clock=clock)
    try:
        core.build()
        run(core, clock, 'I')
        run(core, clock, 'E')
        run(core, clock, 'T', until='TELEOP', limit=1000)
        frozen[0] = True
        run(core, clock)
        assert core.mapper.fault is None
        assert all(j['velocity'] == 0 for j in core.session.last_cycle['proposal']['joints'])
        run(core, clock, until='HOLD', limit=20)
        frozen[0] = False
        run(core, clock, limit=10)
        assert core.session.lifecycle.state == 'HOLD'
        run(core, clock, 'H')
        assert core.session.lifecycle.state == 'DISABLED'
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
