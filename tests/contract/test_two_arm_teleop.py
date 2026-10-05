"""Both arms in one receiver (the virtual one, with Minerva's calibration, keep-out model and the actual Rust
receiver): the motion collision bound holds the arms where their boxes could meet, never faulting, and they follow
again once their targets part. Targets are scripted in place of the leaders (test_helm_teleop_virtual covers the
leader mapping)."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.control.guarded_config import to_cad
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import library_path
from robot_platform.teleop.leader_ranges import KIND, SCHEMA, UNITS, pins

from test_helm_teleop_virtual import CENTERS, DEVICES, HANG, SEED, SPANS, Clock, Leaders

pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file() or not library_path().is_file(),
    reason='Build control_harness and the robot_collision library first')]

# The hands overlap here (model joints): found by sampling both arms' J1-J4 over their swept ranges.
MEETING = {'right_arm_j1': .76, 'right_arm_j2': .52, 'right_arm_j3': -.82, 'right_arm_j4': -1.42,
           'left_arm_j1': .74, 'left_arm_j2': -.4, 'left_arm_j3': .31, 'left_arm_j4': .11}


@pytest.fixture(scope='module')
def rig(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp('two-arm-teleop')
    helm = HelmSession(DEVICES / 'minerva-joint-calibration.json', runs=tmp_path / 'runs',
                       limits=DEVICES / 'minerva-production-01-joint-limits.json', seed=SEED)
    leader_profile = json.loads((DEVICES / 'gello-dual-viewer.json').read_text(encoding='utf-8'))
    ranges = {'schema': SCHEMA, 'kind': KIND, 'profile': leader_profile['id'], 'pins': pins(leader_profile),
              'units': UNITS, 'runtime_approved': False, 'leaders': {
                  name: {joint['name']: {'center_rad': center, 'span_rad': span}
                         for joint, center, span in zip(item['joints'], CENTERS[name], SPANS)}
                  for name, item in leader_profile['leaders'].items()},
              'reference': {'captured_at': 'fixture', 'leaders': {
                  name: {joint['name']: (center + HANG) % (2 * math.pi)
                         for joint, center in zip(item['joints'], CENTERS[name])}
                  for name, item in leader_profile['leaders'].items()}}}
    profile = json.loads((DEVICES / 'minerva-teleop.json').read_text(encoding='utf-8'))
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'))
    profile_path = tmp_path / 'teleop.json'
    profile_path.write_text(json.dumps(profile))
    saved = SimpleNamespace(saved=ranges, missing=lambda routes: [])
    bundle = TeleopSession(helm, profile_path, runs=tmp_path / 'runs', leaders=saved)._bundle()
    bundle['receiver'] = {'mode': 'virtual', 'collision': 'motion'}
    clock = Clock()
    leaders = Leaders(clock)

    def open_harness(config, initial):
        # Timing is not what this checks: a busy test machine may answer slower than a cycle's budget.
        harness = NativeControlHarness(config, initial)
        harness.step(0)
        harness.timeout_s = 1.
        return harness

    core = TeleopCore(bundle, open_leaders=lambda _profile: leaders, open_harness=open_harness, clock=clock)
    core.build()
    yield core, clock
    core.close()
    helm.close()


def run(core, clock, key=None, *, seconds, until=None):
    for _ in range(max(1, round(seconds / core.period_s))):
        core.tick(key, clock())
        key = None
        clock.now += core.period_s
        assert core.fault is None, core.fault
        if until is not None and core.session.lifecycle.state == until:
            return
    assert until is None, f'still {core.session.lifecycle.state}, expected {until}'


@pytest.mark.parametrize('approach_s', [5., 1.5])
def test_two_arms_hold_short_of_each_other_and_follow_again_once_apart(rig, approach_s):
    core, clock = rig
    assert core.collision_mode == 'motion' and core.derived['config']['collision']['bound'] == 'motion'
    names = core.session.lifecycle.names
    joint_map, report = core.derived['joint_map'], {row['motor']: row for row in core.derived['report']}
    home = {motor: row['home'] for motor, row in report.items()}
    # The meeting pose in motor targets, within each joint's usable range.
    meeting = dict(home)
    for joint, q in MEETING.items():
        row = joint_map[joint]
        low, high = report[row['actuator']]['usable']
        meeting[row['actuator']] = min(high, max(low, (q - row['offset']) / row['scale']))
    backend = core.compiler.backend

    scene = backend.runtime_model.scene
    parents = {joint.child: joint for joint in scene._order}

    def below_wrist(link):
        while link in parents:
            if parents[link].name.endswith('_arm_j5'):
                return True
            link = parents[link].parent
        return False

    hands = {shape['id'] for shape in backend.runtime_model.shapes if below_wrist(shape['link'])}

    def gap(motors):
        """How far apart the hands are (m), nominally, with the joints at `motors`."""
        cad = dict(core.pose)
        cad.update(to_cad(joint_map, motors, core.derived['cad_limits']))
        pairs = backend.proximity(cad, threshold=1., scope='inter_branch')['near_pairs']
        return min((pair['distance_m'] for pair in pairs if set(pair['shapes']) <= hands), default=1.)

    assert gap(meeting) < .005  # Followed all the way, the hands would touch.
    script = {'fraction': 0., 'sequence': 0}

    def scripted(engaged):
        script['sequence'] += 1
        f = script['fraction']
        return [dict(name=name, position=home[name] + f * (meeting[name] - home[name]), sampled_at=clock() - .005,
                     sequence=script['sequence']) for name in names], None

    core._leader_target = scripted
    run(core, clock, 'I', seconds=.1)
    run(core, clock, 'E', seconds=.1)
    run(core, clock, 'T', until='TELEOP', seconds=16)
    closest, held, told = 1., 0, False
    # The targets go to the meeting pose, stay 2 s, come back as quickly and stay 4 s.
    total = 2 * approach_s + 6
    for k in range(round(total / core.period_s)):
        t = k * core.period_s
        script['fraction'] = (min(1., t / approach_s) if t < approach_s + 2
                              else max(0., 1 - (t - approach_s - 2) / approach_s))
        run(core, clock, seconds=core.period_s)
        measured = {row['name']: row['position'] for row in core.session.last_feedback}
        closest = min(closest, gap(measured))
        held += core.view()['collision']['holding'] is not None
        told = told or 'HOLDING' in core.message
    assert held and told
    # Held apart, never faulted (run() checks); following again at the end, back home.
    assert closest > .05, closest
    assert core.view()['collision']['holding'] is None and 'following again' in core.message
    assert core.session.lifecycle.state == 'TELEOP'
    measured = {row['name']: row['position'] for row in core.session.last_feedback}
    assert max(abs(measured[m] - home[m]) for m in names if 'gripper' not in m) < .05
    print(f'approach {approach_s} s: hands at least {closest * 1000:.0f} mm apart, held {held} cycles of '
          f'{round(total / core.period_s)}')
    run(core, clock, 'X', until='DISABLED', seconds=30)
