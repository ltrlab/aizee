"""The manual-tuning qualification's scripted tuning step, for one arm or both (no receiver, CAN or timing)."""
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import qualify_manual_tuning_loop as qualify  # noqa: E402

MOTORS = ('left_arm_j1', 'right_arm_j1', 'left_arm_j2', 'right_arm_j2')


def core():
    # The receiver's default preset: only the left J1 follows, within +/-0.03 rad.
    joints = [dict(motor=m, kp=100., kd=5., speed=.05, follow=m == 'left_arm_j1',
                   travel_rad=.03 if m == 'left_arm_j1' else None) for m in MOTORS]
    bounds = {'kp': [0., 500.], 'kd': [0., 50.], 'speed': [.001, 1.5], 'travel_rad': [.001, 3.]}
    return SimpleNamespace(_manual_state=dict(session_id='s', revision=0, joints=joints),
                           _manual_rows=[dict(motor=m, bounds=bounds) for m in MOTORS],
                           session=SimpleNamespace(lifecycle=SimpleNamespace(target=[0.] * len(MOTORS))),
                           clock=lambda: 0.)


def test_both_arm_qualification_tunes_each_j1_and_makes_the_right_one_follow():
    _, _, payload = qualify.Driver(core(), 125, 'both').tune('apply')
    rows = {r['motor']: r for r in payload['joints']}
    for name in ('left_arm_j1', 'right_arm_j1'):
        assert rows[name]['kp'] == 90. and rows[name]['kd'] == 4.5
        assert rows[name]['follow'] is True and rows[name]['travel_rad'] == .03 and rows[name]['speed'] == .05
    assert all(rows[name] == dict(motor=name, kp=100., kd=5., speed=.05, follow=False, travel_rad=None)
               for name in ('left_arm_j2', 'right_arm_j2'))


def test_one_arm_qualification_keeps_its_original_tuning_step():
    _, _, payload = qualify.Driver(core(), 125).tune('apply')
    rows = {r['motor']: r for r in payload['joints']}
    assert rows['left_arm_j1']['kp'] == 90. and rows['left_arm_j1']['follow'] is True
    assert rows['right_arm_j1'] == dict(motor='right_arm_j1', kp=100., kd=5., speed=.05, follow=False, travel_rad=None)
    _, _, invalid = qualify.Driver(core(), 125).tune('invalid')
    assert next(r for r in invalid['joints'] if r['motor'] == 'left_arm_j1')['kd'] == -1.
