"""Direct one/two-arm sessions against the actual receiver; no collision backend."""
import json
import math

import pytest

from robot_platform.helm.teleop import TeleopCore, load_profile
from robot_platform.control.live_receiver import service_config
from tests.contract.test_helm_teleop_live import bundle, Clock, Leaders, Receiver, run  # noqa: F401


def direct_profile(path):
    profile = json.loads(path.read_text())
    receiver = profile['receiver']
    receiver.update(collision_mode='disabled', loop_hz=50, interpolation_ms=20,
                    direct=dict(command_ms=60, velocity_timeout_ms=20, limit_reaction_ms=10))
    receiver.pop('collision', None)
    receiver.pop('compute_budget_s', None)
    path.write_text(json.dumps(profile))
    return profile


@pytest.mark.parametrize('arm', ['left', 'both'])
def test_direct_session_never_opens_geometry_and_follows_with_one_update(bundle, arm):
    data, path = bundle
    direct_profile(path)
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
        run(core, clock, 'T', until='TELEOP', limit=400)
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
