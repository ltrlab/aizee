"""The head in a teleop session (helm.head_teleop): its own receiver (the native harness here) through the arms'
lifecycle, holding still at its zero without a headset, following a fresh headset pose, holding briefly when the
headset goes quiet, then returning to zero."""
import math
from pathlib import Path

import pytest

from robot_platform.helm.head_teleop import PERIOD_S, RETURN_AFTER_S, HeadTeleop
from robot_platform.neck import head
from robot_platform.simulation.control_harness import default_binary

ROOT = Path(__file__).resolve().parents[2]
pytestmark = [pytest.mark.integration, pytest.mark.skipif(not default_binary().is_file(),
                                                          reason='Build control_harness first')]


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


@pytest.fixture
def teleop():
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    clock = Clock()
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
                       'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None}, live=False, clock=clock)
    core.build()
    yield core, clock
    core.close()


def run(core, clock, seconds, action=None, pose=None):
    """Cycle the head for `seconds` at the arms' 10 ms rate (the head cycles every PERIOD_S), the first cycle
    taking `action`, a headset `pose` (yaw, pitch) arriving each cycle when given."""
    for _ in range(int(seconds / .01)):
        if pose is not None:
            core.pose(*pose, now=clock.now)
        core.cycle(action, clock.now)
        action = None
        clock.now += .01


def positions(core):
    return {name: value for name, value in core.status()['positions'].items()}


def test_without_a_headset_the_head_holds_still_at_its_zero(teleop):
    core, clock = teleop
    zero = dict(core.zero)
    run(core, clock, .1, 'idle')
    run(core, clock, .1, 'enable')
    assert core.state == 'HOLD'
    run(core, clock, .5, 'engage')
    run(core, clock, 1.)
    assert core.state == 'TELEOP' and core.source == 'zero'
    assert positions(core) == pytest.approx(zero, abs=1e-6)
    assert core.status()['headset'] is None


def test_the_head_follows_a_fresh_headset_pose_then_holds_and_returns_to_zero(teleop):
    core, clock = teleop
    for action in ('idle', 'enable', 'engage'):
        run(core, clock, .2, action)
    run(core, clock, 2., pose=(.3, -.2))  # Look left and down.
    assert core.state == 'TELEOP' and core.source == 'headset'
    pan, tilt = (positions(core)[name] - core.zero[name] for name in ('head_pan', 'head_tilt'))
    assert pan == pytest.approx(.3, abs=.02) and tilt == pytest.approx(-.2, abs=.02)
    # The neck's CAD angles (the 3D view's) are the headset's yaw and pitch it follows.
    assert core.status()['joints'] == pytest.approx({'neck_z': .3, 'neck_y': -.2}, abs=.02)
    run(core, clock, .5)  # The headset goes quiet: the head holds where it was.
    assert core.source == 'holding' and positions(core)['head_pan'] - core.zero['head_pan'] == pytest.approx(.3, abs=.02)
    run(core, clock, RETURN_AFTER_S + 2.)  # Then it goes back to its zero, at the lifecycle's speed.
    assert core.source == 'zero' and positions(core) == pytest.approx(core.zero, abs=.01)
    run(core, clock, 2., pose=(math.radians(170), 0.))  # Past its range: clamped to the usable end.
    low, high = core.usable['head_pan']
    assert positions(core)['head_pan'] == pytest.approx(high, abs=.02)


def test_disable_leaves_the_head_limp_and_a_fault_stops_its_cycles(teleop):
    core, clock = teleop
    for action in ('idle', 'enable'):
        run(core, clock, .2, action)
    run(core, clock, .1, 'disable')
    assert core.state == 'DISABLED' and core.status()['motors_enabled'] is False
    core.fault = 'test'
    assert core.cycle('idle', clock.now) is None


def test_the_head_keeps_its_grid_on_the_arms_ticks_so_its_commands_never_expire(teleop):
    """Each head command takes the offer of the reply before it: two gaps must stay under the 100 ms command
    lifetime. On the guarded arms' 20 ms ticks a relative due check lost a tick to float rounding (60 ms gaps) and
    the head's command expired (2026-10-25); a late tick must not push the next slot either."""
    core, clock = teleop
    starts = []
    original = core.session.cycle

    def cycle(**kwargs):
        starts.append(kwargs['started_at'])
        return original(**kwargs)
    core.session.cycle = cycle
    core.cycle('idle', clock.now)
    clock.now += .02
    core.cycle('enable', clock.now)
    clock.now += .02
    core.cycle('engage', clock.now)
    for tick in range(150):  # Three seconds of the guarded arms' 20 ms ticks, one of them 15 ms late.
        late = .015 if tick == 70 else 0.
        clock.now += .02 + late
        core.cycle(None, clock.now)
        clock.now -= late  # The arms' loop keeps its schedule: the next tick is on time.
    assert core.fault is None and core.state == 'TELEOP'
    gaps = [b - a for a, b in zip(starts[2:], starts[3:])]
    assert len(gaps) > 70 and min(gaps) > .02
    assert max(a + b for a, b in zip(gaps, gaps[1:])) < .1 - 1e-9  # Two gaps inside the command lifetime.
    assert sum(abs(gap - PERIOD_S) > 1e-6 for gap in gaps) == 2  # The late tick moved one cycle; the grid held.
    assert starts[-1] - starts[2] == pytest.approx(PERIOD_S * len(gaps))


def test_with_its_shoulder_clearance_the_head_never_reaches_the_shoulder_following_or_returning():
    # Turned to a side and tilted all the way down, the head reaches its shoulder (2026-10-29): with the clearance
    # saved, every pose it takes stays above the floor its corners make, following the headset or returning.
    from robot_platform.neck.clearance import section
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    limits = dict(head.virtual_limits(reference, span_rad=.8), reference_id=reference['id'],
                  clearance=section([{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}]))
    clock = Clock()
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference, 'limits': limits,
                       'servo': None}, live=False, clock=clock)
    core.build()
    try:
        def pose_now():
            measured = positions(core)
            return measured['head_pan'] - core.centre['head_pan'], measured['head_tilt'] - core.centre['head_tilt']

        def follow(seconds, pose=None, action=None):
            for _ in range(int(seconds / .01)):
                if pose is not None:
                    core.pose(*pose, now=clock.now)
                core.cycle(action, clock.now)
                action = None
                clock.now += .01
                assert core.clearance.clear(*pose_now()), pose_now()

        for action in ('idle', 'enable', 'engage'):
            follow(.2, action=action)
        follow(4., (0., -.75))  # Straight down: its whole range there.
        assert pose_now() == (pytest.approx(0., abs=.01), pytest.approx(-.75, abs=.01))
        follow(4., (.75, -.75))  # Then turned far to the side, still looking all the way down.
        assert pose_now() == (pytest.approx(.75, abs=.01), pytest.approx(-.3, abs=.01))  # As far down as clears it.
        assert core.status()['clearance'] == {'limited': True}
        follow(.2, action='shutdown')
        follow(15.)  # Return: back to its zero, then down to where it rests (drooped), and disabled there.
        rest = core.rest_target['head_tilt'] - core.centre['head_tilt']
        assert rest < -.7
        assert core.state == 'DISABLED' and pose_now() == (pytest.approx(0., abs=.01), pytest.approx(rest, abs=.03))
    finally:
        core.close()


def test_a_virtual_head_rests_drooped_like_the_real_one_so_return_never_ends_straight_ahead():
    # Virtual teleop has no head to read: with its shoulder clearance saved, it rests where the real head droops,
    # at its tilt end on the shoulder's down side, and Return comes back there through its reference (the user,
    # 2026-10-31: "the shutdown brings the head back to zero, but it should bring it back to its rest position").
    from robot_platform.neck.clearance import section
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    limits = dict(head.virtual_limits(reference, span_rad=.8), reference_id=reference['id'])
    bundle = {'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference, 'servo': None}
    clock = Clock()
    core = HeadTeleop(dict(bundle, limits=dict(limits, clearance=section([{'pan': -.6, 'tilt': -.4},
                                                                          {'pan': .6, 'tilt': -.4}]))),
                      live=False, clock=clock)
    core.build()
    try:
        low = core.usable['head_tilt'][0]  # Tilted down reads lower here (its corners' sign).
        assert core.rest == {'head_pan': pytest.approx(core.centre['head_pan']), 'head_tilt': pytest.approx(low)}
        run(core, clock, .02)  # A cycle reads it, limp.
        assert positions(core)['head_tilt'] == pytest.approx(low)
        for action in ('idle', 'enable', 'engage'):
            run(core, clock, .2, action)
        run(core, clock, 6., pose=(0., 0.))  # Raised to look straight ahead.
        assert positions(core)['head_tilt'] == pytest.approx(core.centre['head_tilt'], abs=.01)
        stages = []
        core.cycle('shutdown', clock.now)
        for _ in range(1500):
            clock.now += .01
            core.cycle(None, clock.now)
            stages.append(core.returning)
        assert stages.index('rest') > stages.index('centre') and core.state == 'DISABLED'
        assert positions(core)['head_tilt'] == pytest.approx(low, abs=.03)
    finally:
        core.close()
    # With no clearance saved yet, which way is down is not known: it rests at its zero, as before.
    core = HeadTeleop(dict(bundle, limits=limits), live=False, clock=Clock())
    core.build()
    try:
        assert core.rest == pytest.approx(core.centre)
    finally:
        core.close()


def drooped(span=.8, rest_tilt=-.6, corners=True):
    """A head that rests limp tilted down (rest_tilt from its reference), with or without its shoulder corners."""
    from robot_platform.neck.clearance import section
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    centre = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
    limits = dict(head.virtual_limits(reference, span_rad=span), reference_id=reference['id'])
    if corners:
        limits['clearance'] = section([{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}])
    clock = Clock()
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference, 'limits': limits,
                       'servo': None, 'rest_pose': {'head_tilt': centre['head_tilt'] + rest_tilt}},
                      live=False, clock=clock)
    core.build()
    return core, clock, centre


def test_the_head_starts_where_it_rests_and_return_takes_it_back_there_through_its_reference():
    # Limp, the head droops (the user, 2026-10-30): Enable holds it there, Engage raises it slowly, and Return goes
    # back to straight ahead first, then down to where it rested, before its torque goes: never dropped, never
    # across the shoulder corner, however each joint's speed shares the move.
    core, clock, centre = drooped()
    try:
        def rel():
            measured = positions(core)
            return measured['head_pan'] - centre['head_pan'], measured['head_tilt'] - centre['head_tilt']

        def follow(seconds, pose=None, action=None):
            seen = []
            for _ in range(int(seconds / .01)):
                if pose is not None:
                    core.pose(*pose, now=clock.now)
                core.cycle(action, clock.now)
                action = None
                clock.now += .01
                assert core.clearance.clear(*rel()), rel()
                seen.append((core.returning, core.state))
            return seen

        follow(.2, action='idle')
        assert rel() == (pytest.approx(0.), pytest.approx(-.6))  # Where it rests, read before anything moved.
        follow(.2, action='enable')
        assert core.state == 'HOLD' and rel() == (pytest.approx(0., abs=.01), pytest.approx(-.6, abs=.01))
        follow(.2, action='engage')
        follow(6., (.75, -.1))  # Up from where it rested, then turned far to the side.
        assert core.state == 'TELEOP' and rel() == (pytest.approx(.75, abs=.01), pytest.approx(-.1, abs=.01))
        seen = follow(.2, action='shutdown')
        seen += follow(15.)
        stages = [stage for stage, _ in seen]
        assert stages.index('rest') > stages.index('centre') and core.returning is None
        assert core.state == 'DISABLED' and rel() == (pytest.approx(0., abs=.03), pytest.approx(-.6, abs=.03))
        assert core.status()['motors_enabled'] is False
    finally:
        core.close()


def test_return_from_hold_engages_where_the_head_is_and_from_idle_just_lets_it_rest():
    core, clock, centre = drooped(corners=False)
    try:
        run(core, clock, .2, 'idle')
        run(core, clock, .2, 'shutdown')  # Limp already: nothing moves, it rests where it is.
        assert core.state == 'DISABLED' and core.returning is None
    finally:
        core.close()
    core, clock, centre = drooped(corners=False)
    try:
        for action in ('idle', 'enable'):
            run(core, clock, .2, action)
        run(core, clock, .2, 'shutdown')
        assert core.returning is not None and core.state in ('ENGAGING', 'TELEOP')
        run(core, clock, .2, 'disengage')  # Returning, only Disable acts.
        assert core.returning is not None
        run(core, clock, 15.)
        assert core.state == 'DISABLED'
        assert positions(core)['head_tilt'] - centre['head_tilt'] == pytest.approx(-.6, abs=.03)
    finally:
        core.close()
    core, clock, centre = drooped(corners=False)
    try:
        for action in ('idle', 'enable', 'engage'):
            run(core, clock, .2, action)
        run(core, clock, 2.)
        run(core, clock, .2, 'shutdown')
        run(core, clock, .1, 'disable')  # H mid-return: limp at once, where it is.
        assert core.state == 'DISABLED' and core.returning is None
    finally:
        core.close()


def test_the_head_runs_on_a_receiver_that_answers_each_request_at_its_ticks():
    # Live, each request waits for the head service's next 10 ms tick, and the Jetson's own time: Idle, which takes
    # the receiver and a fresh offer before its command, ran past the 35 ms budget of a following cycle, and the
    # session faulted before the head was powered (2026-10-31, "cycle exceeded its deadline"). A cycle following
    # nothing has its command's lifetime, as the arms' holds have their verdict's; following cycles keep 35 ms.
    from robot_platform.neck.clearance import section
    from robot_platform.simulation.control_harness import NativeControlHarness
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    limits = dict(head.virtual_limits(reference, span_rad=.8), reference_id=reference['id'],
                  clearance=section([{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}]))
    clock = Clock()

    class Ticking:
        """The harness, each request answered 12 ms after it was sent."""

        def __init__(self, config, initial):
            self.inner = NativeControlHarness(config, initial)
            self.close = self.inner.close

        now_ms = property(lambda self: self.inner.now_ms)
        last = property(lambda self: self.inner.last)

        def step(self, now_ms, request=None):
            if request is not None:
                clock.now += .012
            return self.inner.step(max(now_ms, self.inner.now_ms), request)
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference, 'limits': limits,
                       'servo': None}, live=False, clock=clock, open_harness=Ticking)
    core.build()
    try:
        for action in ('idle', 'enable', 'engage'):
            run(core, clock, .3, action)
            assert core.fault is None, core.fault
        run(core, clock, 6., pose=(.3, 0.))
        assert core.fault is None and core.state == 'TELEOP'
        assert positions(core)['head_pan'] - core.centre['head_pan'] == pytest.approx(.3, abs=.02)
        run(core, clock, .2, 'shutdown')
        run(core, clock, 15.)
        assert core.fault is None and core.state == 'DISABLED'
        assert positions(core)['head_tilt'] == pytest.approx(core.rest_target['head_tilt'], abs=.03)
    finally:
        core.close()


def test_a_refused_head_command_leaves_its_cycle_beside_the_receivers_evidence(tmp_path):
    # Live, the receiver refused a head command ("Head: Guarded controller rejected request: invalid_command",
    # 2026-10-31), and nothing said which joint or limit: the failing cycle is kept beside the receiver's evidence.
    import json
    from robot_platform.simulation.control_harness import NativeControlHarness
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    clock = Clock()

    class Refusing:
        """The harness as a live receiver (with its evidence directory), its tenth direct update over the gains'
        ceiling, which the receiver core refuses."""

        def __init__(self, config, initial):
            self.inner, self.updates, self.directory = NativeControlHarness(config, initial), 0, tmp_path
            self.close = self.inner.close

        now_ms = property(lambda self: self.inner.now_ms)
        last = property(lambda self: self.inner.last)

        def step(self, now_ms, request=None):
            if request is not None and request.get('type') == 'direct_update':
                self.updates += 1
                if self.updates == 10:
                    request = dict(request, joints=[dict(joint, kp=1e3) for joint in request['joints']])
            return self.inner.step(max(now_ms, self.inner.now_ms), request)
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference,
                       'limits': head.virtual_limits(reference, span_rad=.8), 'servo': None},
                      live=False, clock=clock, open_harness=Refusing)
    core.build()
    try:
        with pytest.raises(Exception, match='invalid_command'):
            for action in ('idle', 'enable', 'engage', None):
                run(core, clock, .5, action)
        record = json.loads(next(tmp_path.glob('fault-*.json')).read_text(encoding='utf-8'))
        assert record['fault'] == 'Head: Guarded controller rejected request: invalid_command'
        cycle = record['cycle']
        assert 'invalid_command' in cycle['failure'] and cycle['lifecycle_before'] in ('HOLD', 'ENGAGING', 'TELEOP')
        assert [len(cycle['proposal']['joints']), len(cycle['feedback'])] == [2, 2]
        assert set(cycle['proposal']['joints'][0]) >= {'position', 'velocity', 'effort', 'kp', 'kd'}
    finally:
        core.close()


def test_engage_holds_where_the_head_is_held_not_where_its_weight_sags_it():
    # Live, Engage faulted ("Head: Guarded controller rejected request: invalid_command", 2026-10-31): the head
    # rests below the lowest command its receiver takes, Enable held it at that floor, and its weight sagged it
    # 0.022 rad below the hold. Aligning, the head took that reading as where it is and its command stepped down
    # after it, under the floor (fault-20261031T030047Z.json: 2.2110 to 2.2090 at -0.05 rad/s). Engage now holds
    # where the head is held, and every command stays inside the receiver's range.
    from robot_platform.neck.clearance import section
    from robot_platform.simulation.control_harness import NativeControlHarness
    profile = head.load_profile(ROOT / head.PROFILE)
    reference = head.virtual_reference(profile)
    centre = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
    limits = dict(head.virtual_limits(reference, span_rad=.8), reference_id=reference['id'],
                  clearance=section([{'pan': -.6, 'tilt': -.4}, {'pan': .6, 'tilt': -.4}]))
    clock = Clock()
    sent = []

    class Sagging:
        """The harness on a plant: held, tilt reads 0.022 rad below its command (its weight on the servo's
        stiffness); limp, the head droops to where it rests, below the receiver's floor."""

        def __init__(self, config, initial):
            self.inner, self.rest = NativeControlHarness(config, initial), list(initial)
            self.close = self.inner.close

        now_ms = property(lambda self: self.inner.now_ms)
        last = property(lambda self: self.inner.last)

        def step(self, now_ms, request=None):
            # Every frame reads the plant: a request is answered from the same readings as the ticks.
            if request is not None and request.get('type') == 'direct_update':
                sent.append(request['joints'][1]['position'])
            if self.inner.last is None:
                return self.inner.step(max(now_ms, self.inner.now_ms), request)
            feedback = [{'position': max(rest, joint['position'] - (.022 if i == 1 else 0.)) if joint['kp'] > 0
                         else rest, 'velocity': 0., 'effort': 0.}
                        for i, (joint, rest) in enumerate(zip(self.inner.last['status']['joints'], self.rest))]
            return self.inner.step(max(now_ms, self.inner.now_ms), request, feedback=feedback)
    core = HeadTeleop({'teleop_profile': str(ROOT / head.PROFILE), 'head_reference': reference, 'limits': limits,
                       'servo': None, 'rest_pose': {'head_tilt': centre['head_tilt'] - .8}},
                      live=False, clock=clock, open_harness=Sagging)
    core.build()
    try:
        floor = core.usable['head_tilt'][0]
        assert core.rest['head_tilt'] < floor - .03  # It rests below the lowest command its receiver takes.
        for action in ('idle', 'enable'):
            run(core, clock, .5, action)
        assert positions(core)['head_tilt'] == pytest.approx(floor - .022, abs=1e-6)  # Held at the floor, sagged.
        run(core, clock, .5, 'engage')
        run(core, clock, 1.)
        assert core.fault is None and core.state == 'TELEOP'
        assert sent and min(sent) >= floor - 1e-9
    finally:
        core.close()
