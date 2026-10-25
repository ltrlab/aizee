"""The head in a teleop session with the arms: one TeleopCore, the arms' receiver and the head's (both the native
harness), the same keys for both. Without a headset the head holds at its zero; a headset pose steers it while the
arms follow the leaders; a fault in the head holds the arms."""
import pytest

from robot_platform.helm.teleop import TeleopCore
from robot_platform.neck import head
from tests.contract.test_direct_teleop import direct_profile
from tests.contract.test_helm_teleop_live import ROOT, Clock, Leaders, bundle, run  # noqa: F401


def head_bundle():
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    return {'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
            'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}


@pytest.fixture
def core(bundle):
    data, path = bundle
    direct_profile(path, 100)
    clock = Clock()
    core = TeleopCore(dict(data, receiver={'mode': 'virtual'}, head=head_bundle()),
                      open_leaders=lambda _: Leaders(clock), clock=clock)
    core.build()
    yield core, clock
    core.close()


def test_the_head_takes_the_arms_keys_and_holds_still_without_a_headset(core):
    core, clock = core
    assert core.head.state == 'DISABLED'
    run(core, clock, 'I')
    run(core, clock, 'E')
    assert (core.session.lifecycle.state, core.head.state) == ('HOLD', 'HOLD')
    run(core, clock, 'T', until='TELEOP', limit=1000)
    run(core, clock, limit=100)
    assert core.head.state == 'TELEOP' and core.head.source == 'zero'
    view = core.view()['head']
    assert view['state'] == 'TELEOP' and view['headset'] is None
    assert view['positions'] == pytest.approx(core.head.zero, abs=1e-6)


def test_a_headset_pose_steers_the_head_while_the_arms_follow_the_leaders(core):
    core, clock = core
    for key in ('I', 'E'):
        run(core, clock, key)
    run(core, clock, 'T', until='TELEOP', limit=1000)
    run(core, clock, limit=100)
    for _ in range(300):
        core.head_pose(.25, .1)
        run(core, clock)
    assert core.head.source == 'headset'
    pan = core.view()['head']['positions']['head_pan'] - core.head.zero['head_pan']
    assert pan == pytest.approx(.25, abs=.02)
    assert core.view()['head']['joints'] == pytest.approx({'neck_z': .25, 'neck_y': .1}, abs=.02)  # Helm's 3D view.
    assert core.session.lifecycle.state == 'TELEOP'  # The arms follow on.
    run(core, clock, 'X', until='DISABLED', limit=3000)
    run(core, clock, limit=300)  # The head returns to its zero and is disabled with the arms.
    assert core.head.state == 'DISABLED'


def test_a_head_fault_holds_the_arms(core):
    core, clock = core
    for key in ('I', 'E'):
        run(core, clock, key)
    run(core, clock, 'T', until='TELEOP', limit=1000)
    core.head.receiver.close()  # The head's receiver goes away mid-session.
    run(core, clock, limit=20)
    assert core.head.fault and core.session.lifecycle.state == 'HOLD'
    assert 'The arms hold' in core.message
    run(core, clock, 'T', until='TELEOP', limit=1000)  # The arms follow again, without the head.
