"""The servo sweep engine end to end on a simulated arm (inertia, gravity, friction, the drives' PD loop)."""
import math

import numpy as np
import pytest

from robot_platform.control.approach import limit_speed
from robot_platform.control.guarded_config import usable_margin
from robot_platform.control.servo_sweep import (DEFAULTS, Recording, SweepEngine, damping_for, loosened, result_rows,
                                                sweep_profile, sweep_settings)

TICK = .0025


class Plant:
    """Independent joints driven by the drive's MIT loop: effort = kp (reference - q) + kd (velocity - v) + effort.
    With `interpolation` (s), the receiver ramps each command's reference over that period from where its
    velocity says the stream came from, with the velocity fed forward, then holds it; otherwise it is applied as
    sent."""

    def __init__(self, joints, bounds, *, noise=0., seed=1, interpolation=0.):
        self.joints = joints  # {name: dict(inertia, gravity, friction, q)}
        self.bounds = bounds  # {name: tracking bound}; the receiver faults beyond it.
        self.state = {name: [spec['q'], 0.] for name, spec in joints.items()}
        self.t = 0.
        self.noise, self.random = noise, np.random.default_rng(seed)
        self.worst_lag = {name: 0. for name in joints}
        self.fastest = {name: 0. for name in joints}  # The receiver faults on a joint over its speed cap.
        self.curve = {}  # {name: receiver joint}: track speed against its limit curve (reaction 0.04 s).
        self.over_curve = {name: 0. for name in joints}  # Worst |speed| / speed allowed where the joint was.
        self.shove = None  # (name, start_s, end_s, torque): an external push, as from a bump.
        self.interpolation = interpolation
        self.commands = []  # Every command, as sent.
        self.coupling = None  # callable(name, state) -> Nm on that joint from the others (a cable, say).

    def run(self, names, command, seconds=.04):
        rows = []
        self.commands.append(command)
        count = len(names)
        velocities = command.get('velocities') or [0.] * count
        efforts = command.get('efforts') or [0.] * count
        ramps = command.get('ramp_efforts') or [0.] * count
        for tick in range(round(seconds / TICK)):
            self.t += TICK
            elapsed = tick * TICK
            row = [self.t * 1000]
            for name, target, velocity, effort, ramp, (kp, kd) in zip(names, command['targets'], velocities,
                                                                       efforts, ramps, command['gains']):
                spec, (q, v) = self.joints[name], self.state[name]
                if not self.interpolation:
                    reference, speed = target, velocity
                elif elapsed < self.interpolation:  # As the receiver: the ramp effort only while it ramps.
                    reference, speed, effort = target - velocity * (self.interpolation - elapsed), velocity, effort + ramp
                else:
                    reference, speed = target, 0.
                torque = kp * (reference - q) + kd * (speed - v) + effort
                net = torque - spec['gravity'](q)
                if self.shove and self.shove[0] == name and self.shove[1] <= self.t < self.shove[2]:
                    net += self.shove[3]
                if self.coupling:
                    net += self.coupling(name, self.state)
                if abs(v) < 1e-4 and abs(net) <= spec.get('static', spec['friction']):  # Sticks, then slides.
                    v = 0.
                else:
                    v += (net - spec['friction'] * math.copysign(1, v if abs(v) >= 1e-4 else net)) / spec['inertia'] * TICK
                    q += v * TICK
                self.state[name] = [q, v]
                self.fastest[name] = max(self.fastest[name], abs(v))
                if name in self.curve and abs(v) > 1e-3:
                    limit = self.curve[name]
                    distance = limit['upper'] - q if v > 0 else q - limit['lower']
                    self.over_curve[name] = max(self.over_curve[name], abs(v) / limit_speed(limit, .04, distance))
                lag = abs(reference - q)
                self.worst_lag[name] = max(self.worst_lag[name], lag)
                if lag > self.bounds[name]:
                    raise AssertionError(f'the receiver would fault: {name} lagged {lag:.3f} rad')
                reading = v + (self.random.normal(0, self.noise) if self.noise else 0.)
                row += [q, reading, torque, reference]
            rows.append(row)
        return rows


def limits(name, *, lower, upper, bound, speed, stop, effort, max_kd, max_kp):
    return dict(name=name, lower=lower, upper=upper, max_tracking_error=bound, max_velocity=speed,
                stop_acceleration=stop, max_effort=effort, max_kd=max_kd, max_kp=max_kp)


PROFILE = {'sweep': {'defaults': {'amplitudes': [.1, .3, .6, 1.], 'kp_ladder': [1., .5, .25, .12],
                                  'damping': [1., .6, 1.6], 'speeds': [.4, .8, 1.2], 'decelerations': [4., 16.],
                                  'hold_s': .3, 'rest_s': .2},
                     'joints': [{'match': '*_j2', 'kp_ceiling': 300., 'speed_ceiling': 1.2},
                                {'match': '*_j5', 'kp_ceiling': 20., 'speed_ceiling': 2.5}]}}


def run_sweep(plant, joints, gains, *, seconds=4000):
    names = [joint['name'] for joint in joints]
    settings = {name: sweep_settings(PROFILE, name) for name in names}
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings, sweep=names,
                         interpolation_s=plant.interpolation)
    now = 0.
    engine.observe(plant.run(names, {'targets': [0.] * len(names), 'gains': [gains[n] for n in names]}))
    while now < seconds:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    return engine, now


def arm():
    joints = [limits('right_arm_j2', lower=-1.3, upper=1.3, bound=.08, speed=1.2, stop=4., effort=120., max_kd=60.,
                     max_kp=300.),
              limits('right_arm_j5', lower=-2., upper=2., bound=.1, speed=2.5, stop=15., effort=17., max_kd=4.,
                     max_kp=20.)]
    plant = Plant({'right_arm_j2': dict(inertia=.08, gravity=lambda q: 4. * math.sin(q), friction=.5, q=.01),
                   'right_arm_j5': dict(inertia=.004, gravity=lambda q: 0., friction=.15, q=-.02)},
                  {'right_arm_j2': .08, 'right_arm_j5': .1}, noise=.03)
    gains = {'right_arm_j2': (220., 15.), 'right_arm_j5': (8., 1.)}
    return joints, plant, gains


def test_a_full_sweep_characterises_steps_the_settings_and_recommends_without_a_receiver_fault():
    joints, plant, gains = arm()
    engine, elapsed = run_sweep(plant, joints, gains)
    assert engine.done and engine.task == 'finished', engine.task
    summary = engine.summary()
    shoulder, wrist = summary['right_arm_j2'], summary['right_arm_j5']
    assert shoulder['status'] == wrist['status'] == 'done'
    # The torque map finds the shoulder's gravity (4 sin q) and friction, within its verified range.
    assert shoulder['torque']['friction'] == pytest.approx(.5, abs=.12)
    assert shoulder['verified']['up'] >= .6 and shoulder['verified']['down'] >= .6
    reach = min(shoulder['verified']['up'], 1.) - .05
    assert shoulder['torque']['gravity_max'] == pytest.approx(4 * math.sin(reach), rel=.25)
    # Settings come from what was measured, with the reasons.
    recommendation = shoulder['recommendation']
    assert recommendation['gains_met'] and recommendation['hold_kp'] >= recommendation['kp_floor']
    assert recommendation['max_velocity'] and recommendation['stop_acceleration']
    assert recommendation['notes']
    assert shoulder['verification']['passed'] and wrist['verification']['passed']
    # Speeds are recorded per direction at their commanded speed, and the bound covers the pass across the range.
    assert {s['direction'] for s in shoulder['speeds']} == {1, -1} and all(s['target_speed'] for s in shoulder['speeds'])
    for joint in (shoulder, wrist):
        assert joint['recommendation']['max_tracking_error'] >= 2 * joint['verification']['max_lag'] - 1e-9
    # Every rung says why it was not chosen; a step that never came to rest reads None, not 0.
    assert all('why_not' in entry and (entry['settle_s'] is None or entry['settle_s'] >= 0) for entry in shoulder['grid'])
    # A friction-only wrist needs little stiffness; it is still enough to settle within half the tolerance.
    assert wrist['recommendation']['hold_kp'] >= .15 / .015
    assert elapsed < 4000


def test_a_heavy_joint_is_swept_within_its_bound_without_needing_a_guard():
    joints, plant, gains = arm()
    # Three times the gravity: the lead cap and the torque map keep every test within the bound. Near its
    # upper end its pull is held to what the braking curve allows, so it characterises short of the end
    # (until gravity feed-forward), and still maps most of its load.
    plant.joints['right_arm_j2']['gravity'] = lambda q: 12. * math.sin(q)
    engine, _ = run_sweep(plant, joints, gains)
    assert engine.done and engine.task == 'finished'
    shoulder = engine.summary()['right_arm_j2']
    assert shoulder['torque']['gravity_max'] > 6.
    assert shoulder['recommendation']['hold_kp'] >= shoulder['recommendation']['kp_floor']
    assert plant.worst_lag['right_arm_j2'] < .08  # Never reached the receiver's fault.


def test_a_guard_abandons_the_task_on_a_shove_and_the_sweep_carries_on():
    joints, plant, gains = arm()
    plant.shove = ('right_arm_j5', 150., 150.3, 1.2)  # Well into the wrist's sweep.
    plant.bounds['right_arm_j5'] = 1.  # Let the shove through; the guard must still see it.
    engine, _ = run_sweep(plant, joints, gains)
    assert engine.done and engine.task == 'finished'
    # The guard watches every joint of the arm, including those held while another is tested.
    assert any(event['text'].startswith('right_arm_j5 lagged') for event in engine.events), engine.events
    summary = engine.summary()
    assert all(summary[motor]['status'] == 'done' and summary[motor]['verification']['passed'] for motor in summary)


def test_stopping_returns_the_joint_to_the_reference_and_ends():
    joints, plant, gains = arm()
    names = [joint['name'] for joint in joints]
    settings = {name: sweep_settings(PROFILE, name) for name in names}
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings, sweep=names)
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    now = 0.
    for _ in range(300):  # Well into the shoulder's characterisation.
        engine.observe(plant.run(names, engine.command(now)))
        now += .04
    engine.stop()
    for _ in range(2000):
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done and engine.task == 'stopped'
    assert abs(plant.state['right_arm_j2'][0]) < .05
    assert engine.summary()['right_arm_j2']['status'] == 'stopped'


def test_the_sweep_profile_raises_only_the_ceilings_and_damping_keeps_its_ratio():
    profile = {'motion': {'defaults': {}, 'joints': [{'match': '*_arm_j2', 'max_kp': 220., 'max_kd': 15.}]},
               **PROFILE}
    result = sweep_profile(profile, ['right_arm_j2'], {'right_arm_j2': (220., 15.)})
    override = result['motion']['joints'][-1]
    assert override['match'] == 'right_arm_j2' and override['max_kp'] == 300.
    assert override['max_kd'] == pytest.approx(15. * math.sqrt(300. / 220.) * 1.6)
    assert override['max_velocity'] == 1.2
    assert profile['motion']['joints'][-1]['max_kp'] == 220.  # The original is untouched.
    assert damping_for(55., reference=(220., 15.), multiplier=1.) == pytest.approx(7.5)
    assert sweep_settings({}, 'x')['guard_fraction'] == DEFAULTS['guard_fraction']


def test_rows_come_from_the_recorder_or_one_per_virtual_step():
    names = ['a', 'b']
    recorded = {'recent': [[1., 0., 0., 0., 0., 1., 1., 1., 1.]], 'recent_dropped': 2}
    assert result_rows(recorded, names) == ([[1., 0., 0., 0., 0., 1., 1., 1., 1.]], 2)
    virtual = {'status': {'now_ms': 7, 'joints': [{'position': .1}, {'position': .2}]},
               'feedback': [{'position': .09, 'velocity': 0., 'effort': 1.},
                            {'position': .2, 'velocity': .1, 'effort': 0.}]}
    assert result_rows(virtual, names) == ([[7., .09, 0., 1., .1, .2, .1, 0., .2]], 0)
    recording = Recording(names)
    recording.add([[1., 0, 0, 0, 0, 0, 0, 0, 0], [2., 0, 0, 0, 0, 0, 0, 0, 0]])
    assert len(recording.add([[2., 0, 0, 0, 0, 0, 0, 0, 0], [3., 0, 0, 0, 0, 0, 0, 0, 0]])) == 1  # No tick twice.
    assert len(recording.between(1., 3.)) == 3


class SlowChecker:
    """A collision model that takes 4 ms a pose and reports a contact past +0.25 rad on the wrist."""
    step = .02

    def samples(self, start, end):
        count = max(1, math.ceil(abs(end - start) / self.step))
        return [start + (end - start) * k / count for k in range(1, count + 1)]

    def check(self, motor, position):
        import time
        until = time.perf_counter() + .004  # Busy: Windows sleeps round up to a 15.6 ms tick.
        while time.perf_counter() < until:
            pass
        return 'wrist_link within 12 mm of body at +0.26 rad' if motor.endswith('j5') and position > .25 else None


def test_path_checks_spread_over_cycles_and_still_stop_the_widening():
    import time
    joints, plant, gains = arm()
    names = [joint['name'] for joint in joints]
    settings = {name: sweep_settings(PROFILE, name) for name in names}
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings,
                         sweep=['right_arm_j5'], checker=SlowChecker())
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    now, slowest = 0., 0.
    while now < 400:
        started = time.perf_counter()
        command = engine.command(now)
        slowest = max(slowest, time.perf_counter() - started)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done
    assert slowest < .03, f'one cycle took {slowest * 1000:.0f} ms'  # The loop keeps the receiver's verdict fresh.
    wrist = engine.summary()['right_arm_j5']
    assert wrist['verified']['up'] <= .25 + 1e-9  # Stopped widening at the contact.
    assert any('stopped widening' in event['text'] for event in engine.events)


def test_verify_mode_checks_each_joint_at_its_settings_and_flags_new_friction():
    joints, plant, gains = arm()
    names = [joint['name'] for joint in joints]
    settings = {name: sweep_settings(PROFILE, name) for name in names}
    from robot_platform.control.guarded_config import CONTROL_LAW
    calibration = {'right_arm_j2': {'max_velocity': .4, 'control': CONTROL_LAW, 'measured': {
        'verified': {'up': .6, 'down': .6}, 'verification': {'max_lag': .03}, 'torque': {'friction': .5}}},
        'right_arm_j5': {'max_velocity': .8, 'control': CONTROL_LAW, 'measured': {
            'verified': {'up': .6, 'down': .6}, 'verification': {'max_lag': .03}, 'torque': {'friction': .02}}}}
    gains = dict(gains, right_arm_j2=(300., 10.5))  # Verify runs at the calibrated gains.
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings, sweep=names,
                         mode='verify', calibration=calibration)
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    now = 0.
    while now < 600:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done and engine.task == 'finished'
    summary = engine.summary()
    shoulder, wrist = summary['right_arm_j2']['check'], summary['right_arm_j5']['check']
    assert shoulder['passed'], shoulder['reasons']
    assert shoulder['friction'] == pytest.approx(.5, abs=.12) and shoulder['calibrated']
    # The wrist's friction (0.15 Nm) is well up on what it was swept with (0.02): a maintenance flag.
    assert not wrist['passed'] and any('friction' in reason for reason in wrist['reasons']), wrist['reasons']
    assert now < 200  # A quick check: seconds per joint, not minutes.
    assert shoulder['speed'] == .4  # Its saved speed.
    # A sweep saved under the old position-only commands: its speed and lag measured another control law, so the
    # check runs at half the receiver's cap (the speed teleop uses is the profile's) and compares no lag.
    old = {motor: {k: v for k, v in entry.items() if k != 'control'} for motor, entry in calibration.items()}
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings,
                         sweep=['right_arm_j5'], mode='verify', calibration=old)
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    while (command := engine.command(now)) is not None:
        engine.observe(plant.run(names, command))
        now += .04
    check = engine.summary()['right_arm_j5']['check']
    assert check['speed'] == pytest.approx(.5 * 2.5)
    assert not any('when swept' in reason and 'lagged' in reason for reason in check['reasons'])


RECEIVER = {'feedback_timeout_ms': 30, 'collision': {'max_verdict_ms': 80, 'reaction_ms': 10}}


def test_a_session_takes_the_bound_and_speed_ceilings_as_neither_costs_travel():
    motion = {'max_tracking_error': .08, 'max_velocity': .8, 'stop_acceleration': 4.}
    # Near an end a joint slows (the braking curve) and the receiver checks its limits on feedback, so each end
    # keeps only the approach floor's stopping margin: 0.5 rad/s for 40 ms, then braking at 4 rad/s2.
    assert usable_margin(motion, RECEIVER) == pytest.approx(.5 * .04 + .25 / 8, abs=1e-5)
    assert usable_margin(dict(motion, max_velocity=2.5), RECEIVER) == usable_margin(motion, RECEIVER)
    assert usable_margin(dict(motion, max_tracking_error=.15), RECEIVER) == usable_margin(motion, RECEIVER)
    assert loosened(motion, dict(DEFAULTS, bound_ceiling=.15, speed_ceiling=4.)) == (.15, 4.)
    assert loosened(motion, dict(DEFAULTS, bound_ceiling=.05)) == (.08, .8)  # Never lowered.
    assert loosened(dict(motion, gripper=True), dict(DEFAULTS, bound_ceiling=.15)) == (.15, .8)


def wrist():
    """A light wrist that its profile gains (8, 1) can barely push at the lead cap, like right J6."""
    joints = [limits('right_arm_j6', lower=-.9, upper=.9, bound=.15, speed=.8, stop=4., effort=14., max_kd=4.,
                     max_kp=24.)]
    plant = Plant({'right_arm_j6': dict(inertia=.01, gravity=lambda q: .15 * math.sin(2 * q), friction=.3, q=0.)},
                  {'right_arm_j6': .15})
    profile = {'sweep': {'defaults': dict(PROFILE['sweep']['defaults']),
                         'joints': [{'match': '*_j6', 'kp_ceiling': 24.}]}}
    return joints, plant, {'right_arm_j6': (8., 1.)}, profile


def test_the_plan_waits_for_a_joint_it_cannot_push_and_stiffens_it_only_at_rest():
    joints, plant, gains, profile = wrist()
    names = ['right_arm_j6']
    settings = {name: sweep_settings(profile, name) for name in names}
    engine = SweepEngine(joints, home={'right_arm_j6': 0.}, gains=gains, settings=settings, sweep=names,
                         travel={'right_arm_j6': {'operational': (-.9, .9), 'swept': (-.95, .95), 'usable': (-.6, .6)}},
                         receiver=RECEIVER)
    plant.curve['right_arm_j6'] = joints[0]
    engine.observe(plant.run(names, {'targets': [0.], 'gains': [gains['right_arm_j6']]}))
    now = 0.
    while now < 4000:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done and engine.task == 'finished', engine.task
    # The first live J6 sweep lunged at 0.97 rad/s against its 0.8 cap when its stiffness tripled behind its plan,
    # and a later one ran into its braking curve near an end, pulled by its command's lead.
    assert plant.fastest['right_arm_j6'] < .8, plant.fastest
    assert plant.over_curve['right_arm_j6'] < 1., plant.over_curve
    result = engine.summary()['right_arm_j6']
    assert result['usable']['swept'] == pytest.approx(1.9) and result['usable']['today'] == pytest.approx(1.2)
    # The commanded cap (the receiver enforces 1.25x), as no row gives today's settings here.
    assert result['current']['max_velocity'] == pytest.approx(.8 / 1.25)
    assert 0 < result['usable']['recommended'] <= 1.8


def test_a_joint_that_cannot_follow_its_plan_ends_the_task_and_the_sweep_goes_on():
    joints, plant, gains, profile = wrist()
    plant.joints['right_arm_j6']['friction'] = 1.  # More than 8 Nm/rad can push at the lead cap (0.72 Nm).
    names = ['right_arm_j6']
    settings = {name: sweep_settings(profile, name) for name in names}
    engine = SweepEngine(joints, home={'right_arm_j6': 0.}, gains=gains, settings=settings, sweep=names)
    engine.observe(plant.run(names, {'targets': [0.], 'gains': [gains['right_arm_j6']]}))
    now = 0.
    while now < 4000:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done
    assert any('could not follow its plan' in event['text'] for event in engine.events), engine.events


def test_a_session_that_ends_under_the_sweep_stops_the_joint_under_test():
    joints, plant, gains = arm()
    names = [joint['name'] for joint in joints]
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains,
                         settings={name: sweep_settings(PROFILE, name) for name in names}, sweep=names)
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    now = 0.
    while engine.current != 'right_arm_j2' or engine.task.endswith('reference'):
        engine.observe(plant.run(names, engine.command(now)))
        now += .04
    engine.halt('Receiver faulted: limit_violation')
    summary = engine.summary()
    assert summary['right_arm_j2']['status'] == 'stopped' and engine.done and engine.command(now) is None
    assert engine.progress()['motor'] is None and engine.task == 'stopped'
    assert 'limit_violation' in summary['right_arm_j2']['events'][-1]['text']


def test_the_profile_lets_the_sweep_speed_up_every_arm_joint_and_loosen_only_the_distal_bounds():
    import json
    from robot_platform.ops.release import ROOT
    profile = json.loads((ROOT / 'config/devices/minerva-teleop.json').read_text(encoding='utf-8'))
    for joint in ('j1', 'j2', 'j3'):
        settings = sweep_settings(profile, f'right_arm_{joint}')
        assert settings['speed_ceiling'] == 1.5 and not settings.get('bound_ceiling')
        assert 1.5 in settings['speeds']  # The ladder reaches the ceiling itself.
    assert sweep_settings(profile, 'left_arm_j4')['bound_ceiling'] == .12
    assert sweep_settings(profile, 'left_arm_j4')['speed_ceiling'] == 2.5
    for joint in ('j5', 'j6'):
        settings = sweep_settings(profile, f'left_arm_{joint}')
        assert settings['speed_ceiling'] == 4. and 4. in settings['speeds']


def test_a_joint_breaking_free_of_static_friction_near_an_end_stays_well_under_its_curve():
    # Right J6's sweep session (2.5 rad/s commanded, 3.125 enforced, 16 rad/s^2, bound 0.15, kp to 24). The
    # third live J6 sweep stuck on static friction short of an end while its lead built up, then lunged to
    # 1.1 rad/s where 0.98 was allowed. Pulling toward an end by position alone keeps it under half the
    # curve here; a pull that grew back while the joint read slow reached 0.79 of it.
    joints = [limits('right_arm_j6', lower=-.885, upper=.9225, bound=.15, speed=3.125, stop=16., effort=14.,
                     max_kd=4., max_kp=24.)]
    plant = Plant({'right_arm_j6': dict(inertia=.01, gravity=lambda q: .15 * math.sin(2 * q), friction=.1,
                                        static=.35, q=0.)}, {'right_arm_j6': .15})
    plant.curve['right_arm_j6'] = joints[0]
    profile = {'sweep': {'defaults': dict(PROFILE['sweep']['defaults'], speeds=[.4, .8, 1., 1.2],
                                         decelerations=[2., 4., 8.]),
                         'joints': [{'match': '*_j6', 'kp_ceiling': 24.}]}}
    names = ['right_arm_j6']
    engine = SweepEngine(joints, home={'right_arm_j6': 0.}, gains={'right_arm_j6': (8., 1.)},
                         settings={name: sweep_settings(profile, name) for name in names}, sweep=names)
    engine.observe(plant.run(names, {'targets': [0.], 'gains': [(8., 1.)]}))
    now = 0.
    while now < 4000:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done and engine.task == 'finished', engine.task
    assert plant.over_curve['right_arm_j6'] < .7, plant.over_curve
    # Held softly (kp 8) while characterising, it still pushes past friction to both edges of its travel.
    low, high = engine._allowed('right_arm_j6')
    verified = engine.summary()['right_arm_j6']['verified']
    assert verified['up'] == pytest.approx(high) and verified['down'] == pytest.approx(-low), verified
    assert not any('could not follow' in event['text'] for event in engine.events), engine.events


def test_speeds_are_measured_in_open_travel_where_the_braking_curve_allows_them():
    # Right J6's session (commanded 2.5 rad/s, enforced 3.125, 16 rad/s^2, limit reaction 0.04 s): at 1 rad/s
    # commands may move freely from 0.139 rad inside each end (1.25 x 0.04 + 1.25^2 / 32, then one 40 ms step).
    joints = [limits('right_arm_j6', lower=-.885, upper=.9225, bound=.15, speed=3.125, stop=16., effort=14.,
                     max_kd=4., max_kp=24.)]
    engine = SweepEngine(joints, home={'right_arm_j6': 0.}, gains={'right_arm_j6': (8., 1.)},
                         settings={'right_arm_j6': sweep_settings(PROFILE, 'right_arm_j6')}, sweep=['right_arm_j6'])
    low, high = engine._open_travel('right_arm_j6', 1.)
    room = 1.25 * .04 + 1.25 ** 2 / 32 + 1. * .04
    assert (low, high) == (pytest.approx(-.885 + room), pytest.approx(.9225 - room))
    # There the plan is not slowed: commands may move at the full speed toward either end.
    assert engine.limits['right_arm_j6']['max_velocity'] >= 1.
    engine.targets['right_arm_j6'] = high - 1e-9
    assert engine._toward_end('right_arm_j6', 1, 1.) >= 1. - 1e-6
    engine.targets['right_arm_j6'] = low + 1e-9
    assert engine._toward_end('right_arm_j6', -1, 1.) >= 1. - 1e-6


def test_a_calibration_names_its_evidence_without_the_robots_home_directory():
    from robot_platform.control.servo_calibration import evidence_name
    assert evidence_name('/home/someone/minerva/evidence/sweep-20260928T000115Z') ==         '~/minerva/evidence/sweep-20260928T000115Z'
    assert evidence_name('2026-09-28T00:14:27+00:00') == '2026-09-28T00:14:27+00:00'  # A virtual sweep's time.


def test_under_the_interpolated_feed_forward_law_a_sweep_follows_faster_with_less_lag():
    # The receiver ramps each command over 40 ms with its velocity fed forward, and friction once measured: the
    # drive's damping no longer brakes the motion, so the same arm follows its speeds with a fraction of the lag.
    joints, plant, gains = arm()
    before, _ = run_sweep(plant, joints, gains)
    joints, plant, gains = arm()
    plant.interpolation = .04
    after, _ = run_sweep(plant, joints, gains)
    assert after.done and after.task == 'finished', after.task
    old, new = before.summary()['right_arm_j2'], after.summary()['right_arm_j2']
    assert new['status'] == 'done' and new['verification']['passed']
    top = max(r['target_speed'] for r in new['speeds'])

    def lag(summary, speed):
        return max(r['max_lag'] for r in summary['speeds'] if r['target_speed'] == speed)
    assert lag(new, top) < .6 * lag(old, top)
    assert (new['recommendation']['max_velocity'] or 0.) >= (old['recommendation']['max_velocity'] or 0.)
    # Moving commands carried their velocity and, once the joint's friction was measured, friction feed-forward;
    # a step test is sent as a step.
    assert any(abs(v) > .1 for command in plant.commands for v in command.get('velocities') or [])
    assert any(abs(e) > 0 for command in plant.commands for e in command.get('efforts') or [])
    assert after.friction['right_arm_j2'] == pytest.approx(.5, abs=.12)


def test_a_joint_short_of_room_for_faster_speeds_is_verified_at_its_cap():
    # A wrist with 1.2 rad of travel has no open-travel room to test its faster speeds; everything it had room
    # for was followed, so the pass across its range tries the cap (the braking curve slows it near each end).
    joints = [limits('right_arm_j6', lower=-.6, upper=.6, bound=.15, speed=3.125, stop=32., effort=14., max_kd=4.,
                     max_kp=24.)]
    plant = Plant({'right_arm_j6': dict(inertia=.01, gravity=lambda q: .15 * math.sin(2 * q), friction=.1, q=0.)},
                  {'right_arm_j6': .15}, interpolation=.04)
    profile = {'sweep': {'defaults': dict(PROFILE['sweep']['defaults'], speeds=[.4, .8, 1.2, 1.6, 2., 2.5]),
                         'joints': [{'match': '*_j6', 'kp_ceiling': 24.}]}}
    names = ['right_arm_j6']
    engine = SweepEngine(joints, home={'right_arm_j6': 0.}, gains={'right_arm_j6': (24., .6)},
                         settings={name: sweep_settings(profile, name) for name in names}, sweep=names,
                         interpolation_s=.04)
    engine.observe(plant.run(names, {'targets': [0.], 'gains': [(24., .6)]}))
    now = 0.
    while now < 4000:
        command = engine.command(now)
        if command is None:
            break
        engine.observe(plant.run(names, command))
        now += .04
    assert engine.done and engine.task == 'finished', engine.task
    result = engine.summary()['right_arm_j6']
    assert any('verified room' in event['text'] for event in engine.events), engine.events
    assert result['verification']['passed'] and result['recommendation']['max_velocity'] == pytest.approx(2.5)
    assert plant.fastest['right_arm_j6'] < 3.125



def test_gravity_feed_forward_lets_a_heavy_joint_sweep_and_verify_its_range():
    # A shoulder with 14 sin(q) of gravity: its lead cap (220 x 0.048 = 10.6 Nm) stops it lifting past 0.6 rad.
    # Fed its gravity from a slice 20% short (11 sin q), the position loop only answers the friction and the
    # model's error: it characterises, follows and verifies twice the range, and its torque map refits the slice.
    def sweep(gravity):
        joints, plant, gains = arm()
        plant.joints['right_arm_j2']['gravity'] = lambda q: 14. * math.sin(q)
        plant.bounds['right_arm_j2'] = 10.  # Let the plan's own guards show what it can do.
        plant.interpolation = .04
        names = [joint['name'] for joint in joints]
        engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains,
                             settings={name: sweep_settings(PROFILE, name) for name in names},
                             sweep=['right_arm_j2'], interpolation_s=.04, gravity=gravity)
        engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
        now = 0.
        while now < 4000:
            command = engine.command(now)
            if command is None:
                break
            engine.observe(plant.run(names, command))
            now += .04
        assert engine.done and engine.task == 'finished', engine.task
        return engine.summary()['right_arm_j2']
    held, fed = sweep(None), sweep({'right_arm_j2': [[11., {'right_arm_j2': 1}]]})
    assert held['verified']['up'] < .7 and fed['verified']['up'] >= 1. - 1e-9 and fed['verified']['down'] >= 1. - 1e-9
    # Past the widest widening step (1 rad), out to its ends (±1.3 rad less the stopping margin).
    assert fed['verification']['passed']
    assert fed['verification']['span'] == pytest.approx((-fed['verified']['down'], fed['verified']['up']))
    assert min(fed['verified'].values()) > 1.3 - .06
    # Both follow the profile's speeds; fed, with a fraction of the lag, so with a tighter tracking bound.
    assert fed['recommendation']['max_velocity'] >= held['recommendation']['max_velocity']
    def worst(result):
        return max(s['max_lag'] for s in result['speeds'] if s['max_lag'] is not None)
    assert worst(fed) < .5 * worst(held)
    assert fed['recommendation']['max_tracking_error'] < held['recommendation']['max_tracking_error']
    assert fed['torque']['gravity_fit']['A'] == pytest.approx(14., rel=.02)


def test_the_guard_sees_every_row_since_the_last_command():
    # The loop hands the engine each receiver result's rows, then the temperatures with no rows: that emptied the
    # rows the guard reads, so on the live arm the guard never ran.
    joints, plant, gains = arm()
    names = [joint['name'] for joint in joints]
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains,
                         settings={name: sweep_settings(PROFILE, name) for name in names}, sweep=names)
    engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
    now = 0.
    while engine.current is None or engine.task.endswith('reference'):
        engine.observe(plant.run(names, engine.command(now)))
        now += .04
    rows = np.array(plant.run(names, engine.command(now)))
    rows[-1, 4 + 4 * 1] = rows[-1, 1 + 4 * 1] + .09  # The wrist lags its command by 0.09 rad (guard 0.075).
    engine.observe(rows)
    engine.observe([], temperatures={'right_arm_j5': 30.})
    engine.command(now + .04)
    assert any(event['text'].startswith('right_arm_j5 lagged 0.090') for event in engine.events), engine.events


def test_the_recording_keeps_few_chunks_however_the_rows_arrive():
    # Each receiver result brings a row or two. Kept one chunk each, a 90 s window held 11,000 chunks, every
    # lookup walked them all, and right J1's sweep missed the receiver's 80 ms verdict (2026-09-29).
    names = [f'j{i}' for i in range(7)]
    recording = Recording(names)
    t = 0.
    for _ in range(int(90 / .04) * 5):
        rows = np.zeros((3, 1 + 4 * len(names)))
        rows[:, 0] = t + np.arange(3) * 2.5
        t += 7.5
        recording.add(rows)
    assert len(recording.chunks) < 200
    assert len(recording.between(t - 40, t)) == 16
    assert recording.rows_total == int(90 / .04) * 15


def test_a_braked_move_slows_steadily_to_its_target_and_ramps_its_last_step():
    # Braking only once the stopping distance reached what was left, a move braked a step, sped up again and
    # ended with one abrupt step, sent with no velocity: at 1.5 rad/s right J3's command jumped 0.053 rad and
    # it lagged 0.047 (2026-09-30).
    joints = [limits('right_arm_j3', lower=-3., upper=3., bound=.05, speed=1.875, stop=25., effort=60., max_kd=20.,
                     max_kp=215.)]
    engine = SweepEngine(joints, home={'right_arm_j3': 0.}, gains={'right_arm_j3': (215., 14.)},
                         settings={'right_arm_j3': sweep_settings(PROFILE, 'right_arm_j3')}, sweep=['right_arm_j3'],
                         interpolation_s=.04)
    steps, velocities = [], []
    before = engine.targets['right_arm_j3']
    for _ in engine._go('right_arm_j3', 1.2, 1.5, acceleration=8., deceleration=8.):
        steps.append(engine.targets['right_arm_j3'] - before)
        velocities.append(engine.velocity['right_arm_j3'])
        before = engine.targets['right_arm_j3']
    assert engine.targets['right_arm_j3'] == pytest.approx(1.2)
    peak = steps.index(max(steps))
    assert all(b <= a + 1e-12 for a, b in zip(steps[peak:], steps[peak + 1:]))  # Never speeds up again.
    assert steps[-1] < .01  # No abrupt last step.
    assert all(v > 0 for v in velocities)  # Each step, the last too, is a ramp at its speed.
    assert engine.velocity['right_arm_j3'] == 0.  # At rest once there.


def test_a_move_handing_over_at_speed_keeps_its_speed_into_the_next():
    # The braking test cruises to where it brakes (a move ending at speed), then brakes. That first move's short last
    # step halved the command's speed for a cycle, and left J2, chasing it back, reached 2.0 rad/s against 1.5: the
    # receiver tripped (2026-09-30). Now every step into the braking is a whole cruise step.
    joints = [limits('left_arm_j2', lower=-1.6, upper=1.6, bound=.08, speed=1.875, stop=16., effort=120., max_kd=30.,
                     max_kp=220.)]
    engine = SweepEngine(joints, home={'left_arm_j2': 0.}, gains={'left_arm_j2': (220., 15.)},
                         settings={'left_arm_j2': sweep_settings(PROFILE, 'left_arm_j2')}, sweep=['left_arm_j2'],
                         interpolation_s=.04)
    engine.targets['left_arm_j2'] = -1.
    steps = []
    before = -1.
    for move in (engine._go('left_arm_j2', .4999, 1.5, acceleration=8., stop=False),
                 engine._go('left_arm_j2', .8, 1.5, deceleration=4.)):
        for _ in move:
            steps.append(engine.targets['left_arm_j2'] - before)
            before = engine.targets['left_arm_j2']
    cruise = 1.5 * .04
    peak = steps.index(max(steps))
    assert max(steps) == pytest.approx(cruise)
    # From the cruise on, each step is no longer than the last, and none before the braking is short.
    assert all(b <= a + 1e-12 for a, b in zip(steps[peak:], steps[peak + 1:]))
    braking = next(i for i in range(peak, len(steps)) if steps[i] < cruise - 1e-9)
    assert steps[braking] > cruise - 4. * .04 * .04 - 1e-9  # Its first braking step slows by one cycle's 4 rad/s2.
    assert engine.targets['left_arm_j2'] == pytest.approx(.8) and engine.velocity['left_arm_j2'] == 0.


def return_arm(start):
    """A shoulder swivel and a wrist resting at `start` ({motor: rad}), for a return to the reference."""
    joints = [limits('right_arm_j1', lower=-.83, upper=1.61, bound=.04, speed=1.875, stop=16., effort=60., max_kd=5.,
                     max_kp=150.),
              limits('right_arm_j6', lower=-.94, upper=.97, bound=.064, speed=5., stop=16., effort=14., max_kd=1.,
                     max_kp=24.)]
    plant = Plant({'right_arm_j1': dict(inertia=.3, gravity=lambda q: 0., friction=.5, q=start['right_arm_j1']),
                   'right_arm_j6': dict(inertia=.004, gravity=lambda q: 0., friction=.1, q=start['right_arm_j6'])},
                  {'right_arm_j1': .04, 'right_arm_j6': .064}, noise=.03, interpolation=.04)
    gains = {'right_arm_j1': (150., 5.), 'right_arm_j6': (24., .61)}
    return joints, plant, gains


def run_return(joints, plant, gains, start, *, checker=None, seconds=90, on_cycle=None):
    names = [joint['name'] for joint in joints]
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, mode='return', sweep=[],
                         checker=checker, settings={name: sweep_settings(PROFILE, name) for name in names},
                         interpolation_s=.04)
    engine.observe(plant.run(names, {'targets': [start[n] for n in names], 'gains': [gains[n] for n in names]}))
    engine.begin(start)
    now, moving = 0., []
    while now < seconds:
        command = engine.command(now)
        if command is None:
            break
        before = {name: plant.state[name][0] for name in names}
        engine.observe(plant.run(names, command))
        moving.append({name for name in names if abs(plant.state[name][0] - before[name]) > 1e-3})
        if on_cycle:
            on_cycle()
        now += .04
    return engine, moving


def test_a_return_brings_the_whole_arm_home_together_along_one_line_slowly():
    # The right arm rested with its shoulder swivel 1.22 rad and its wrist 0.58 rad off (2026-09-30): both return
    # together along one line, arriving together, the swivel (farthest out) at its characterisation speed.
    start = {'right_arm_j1': 1.22, 'right_arm_j6': .58}
    joints, plant, gains = return_arm(start)
    names = [joint['name'] for joint in joints]
    halfway = []
    engine, moving = run_return(joints, plant, gains, start,
                                on_cycle=lambda: halfway.append((plant.state['right_arm_j1'][0],
                                                                 plant.state['right_arm_j6'][0])))
    assert engine.task == 'finished'
    assert all(abs(plant.state[name][0]) < .01 for name in names), plant.state
    q1, q6 = next((q1, q6) for q1, q6 in halfway if q1 < .61)
    assert q6 == pytest.approx(q1 * .58 / 1.22, abs=.03)  # On the line from the start to the reference.
    slow = sweep_settings(PROFILE, 'right_arm_j1')['characterize_speed']
    assert plant.fastest['right_arm_j1'] < 1.5 * slow and plant.fastest['right_arm_j6'] < slow
    assert not engine.summary()


def test_a_wrist_dragged_by_the_shoulders_harness_returns_with_it():
    # The live return of 2026-09-30 moved the wrist first: it stalled at +0.40 rad pushing 0.95 Nm, rising as it
    # went, against a harness the shoulder swivel's 1.22 rad had twisted. Here the harness pulls the wrist toward
    # 0.58 of the swivel's share (4.5 Nm/rad): together they return with the harness slack.
    start = {'right_arm_j1': 1.22, 'right_arm_j6': .58}
    joints, plant, gains = return_arm(start)
    names = [joint['name'] for joint in joints]
    plant.coupling = lambda name, state: (-4.5 * (state['right_arm_j6'][0] - .58 * state['right_arm_j1'][0] / 1.22)
                                          if name == 'right_arm_j6' else 0.)
    engine, _moving = run_return(joints, plant, gains, start)
    assert engine.task == 'finished', engine.events
    assert all(abs(plant.state[name][0]) < .01 for name in names), plant.state
    assert plant.worst_lag['right_arm_j6'] < .02


def test_a_return_does_not_start_when_its_line_is_not_clear():
    class Blocked:
        step = .1

        def near(self, motors):
            return set()

        def check_pose(self, pose, *, baseline=frozenset()):
            return 'right_arm_j6 link within 5 mm of left_arm_j6 link' if pose['right_arm_j6'] < .2 else None
    start = {'right_arm_j1': .5, 'right_arm_j6': .4}
    joints, plant, gains = return_arm(start)
    engine, _moving = run_return(joints, plant, gains, start, checker=Blocked())
    assert engine.task == 'stopped' and engine.done
    assert [e['text'] for e in engine.events] == ['the arm was not moved: right_arm_j6 link within 5 mm of '
                                                  'left_arm_j6 link']
    assert abs(plant.state['right_arm_j1'][0] - .5) < .01 and abs(plant.state['right_arm_j6'][0] - .4) < .01


def test_a_heavy_joints_inertia_is_fitted_and_fed_forward_as_it_speeds_up():
    # Right J2 (0.9 kg m2 fitted, 2026-09-30) lagged while it accelerated, not while it cruised. Here a shoulder
    # of 0.6 kg m2, its gravity fed forward: the sweep fits its inertia from its speed tests and feeds its
    # acceleration forward from then on, so its later tests lag less than without.
    def sweep(fit):
        joints, plant, gains = arm()
        plant.joints['right_arm_j2'].update(inertia=.6, gravity=lambda q: 14. * math.sin(q))
        plant.bounds['right_arm_j2'] = 10.
        plant.interpolation = .04
        names = [joint['name'] for joint in joints]
        engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains,
                             settings={name: sweep_settings(PROFILE, name) for name in names},
                             sweep=['right_arm_j2'], interpolation_s=.04,
                             gravity={'right_arm_j2': [[14., {'right_arm_j2': 1}]]})
        if not fit:
            engine._fit_inertia = lambda motor, report: None
        engine.observe(plant.run(names, {'targets': [0., 0.], 'gains': [gains[n] for n in names]}))
        now = 0.
        while now < 4000:
            command = engine.command(now)
            if command is None:
                break
            engine.observe(plant.run(names, command))
            now += .04
        assert engine.done and engine.task == 'finished', engine.task
        return engine.summary()['right_arm_j2']
    fed, bare = sweep(True), sweep(False)
    assert fed['torque']['inertia'] == pytest.approx(.6, rel=.25)
    assert 'inertia' not in bare['torque']
    def worst(result, speed):
        return max(s['max_lag'] for s in result['speeds'] if s['target_speed'] == speed and s['max_lag'] is not None)
    assert worst(fed, .8) < .7 * worst(bare, .8)
    assert fed['verification']['max_lag'] < bare['verification']['max_lag']
