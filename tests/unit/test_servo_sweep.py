"""The servo sweep engine end to end on a simulated arm (inertia, gravity, friction, position-only PD)."""
import math

import numpy as np
import pytest

from robot_platform.control.servo_sweep import (DEFAULTS, Recording, SweepEngine, damping_for, result_rows,
                                                sweep_profile, sweep_settings)

TICK = .0025


class Plant:
    """Independent joints driven by the drive's MIT loop: effort = kp (target - q) - kd v."""

    def __init__(self, joints, bounds, *, noise=0., seed=1):
        self.joints = joints  # {name: dict(inertia, gravity, friction, q)}
        self.bounds = bounds  # {name: tracking bound}; the receiver faults beyond it.
        self.state = {name: [spec['q'], 0.] for name, spec in joints.items()}
        self.t = 0.
        self.noise, self.random = noise, np.random.default_rng(seed)
        self.worst_lag = {name: 0. for name in joints}
        self.shove = None  # (name, start_s, end_s, torque): an external push, as from a bump.

    def run(self, names, command, seconds=.04):
        rows = []
        for _ in range(round(seconds / TICK)):
            self.t += TICK
            row = [self.t * 1000]
            for name, target, (kp, kd) in zip(names, command['targets'], command['gains']):
                spec, (q, v) = self.joints[name], self.state[name]
                torque = kp * (target - q) - kd * v
                net = torque - spec['gravity'](q)
                if self.shove and self.shove[0] == name and self.shove[1] <= self.t < self.shove[2]:
                    net += self.shove[3]
                if abs(v) < 1e-4 and abs(net) <= spec['friction']:
                    v = 0.
                else:
                    v += (net - spec['friction'] * math.copysign(1, v if abs(v) >= 1e-4 else net)) / spec['inertia'] * TICK
                    q += v * TICK
                self.state[name] = [q, v]
                lag = abs(target - q)
                self.worst_lag[name] = max(self.worst_lag[name], lag)
                if lag > self.bounds[name]:
                    raise AssertionError(f'the receiver would fault: {name} lagged {lag:.3f} rad')
                reading = v + (self.random.normal(0, self.noise) if self.noise else 0.)
                row += [q, reading, torque, target]
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
    engine = SweepEngine(joints, home={name: 0. for name in names}, gains=gains, settings=settings, sweep=names)
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
    # Three times the gravity: the lead cap and the torque map keep every test within the bound.
    plant.joints['right_arm_j2']['gravity'] = lambda q: 12. * math.sin(q)
    engine, _ = run_sweep(plant, joints, gains)
    assert engine.done and engine.task == 'finished'
    shoulder = engine.summary()['right_arm_j2']
    assert shoulder['torque']['gravity_max'] > 8.
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
    calibration = {'right_arm_j2': {'max_velocity': .4, 'measured': {
        'verified': {'up': .6, 'down': .6}, 'verification': {'max_lag': .03}, 'torque': {'friction': .5}}},
        'right_arm_j5': {'max_velocity': .8, 'measured': {
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
