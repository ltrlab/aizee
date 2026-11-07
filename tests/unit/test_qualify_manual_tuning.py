"""The manual-tuning qualification's scripted tuning step, for one arm or both (no receiver, CAN or timing)."""

from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import qualify_manual_tuning_loop as qualify  # noqa: E402

MOTORS = ('left_arm_j1', 'right_arm_j1', 'left_arm_j2', 'right_arm_j2')


def core():
    # The receiver's default preset: only the left J1 follows, within +/-0.03 rad.
    joints = [
        dict(
            motor=m,
            kp=100.0,
            kd=5.0,
            speed=0.05,
            follow=m == 'left_arm_j1',
            travel_rad=0.03 if m == 'left_arm_j1' else None,
        )
        for m in MOTORS
    ]
    bounds = {'kp': [0.0, 500.0], 'kd': [0.0, 50.0], 'speed': [0.001, 1.5], 'travel_rad': [0.001, 3.0]}
    return SimpleNamespace(
        _manual_state=dict(session_id='s', revision=0, joints=joints),
        _manual_rows=[dict(motor=m, bounds=bounds) for m in MOTORS],
        session=SimpleNamespace(lifecycle=SimpleNamespace(target=[0.0] * len(MOTORS))),
        clock=lambda: 0.0,
    )


def test_both_arm_qualification_tunes_each_j1_and_makes_the_right_one_follow():
    _, _, payload = qualify.Driver(core(), 125, 'both').tune('apply')
    rows = {r['motor']: r for r in payload['joints']}
    for name in ('left_arm_j1', 'right_arm_j1'):
        assert rows[name]['kp'] == 90.0 and rows[name]['kd'] == 4.5
        assert rows[name]['follow'] is True and rows[name]['travel_rad'] == 0.03 and rows[name]['speed'] == 0.05
    assert all(
        rows[name] == dict(motor=name, kp=100.0, kd=5.0, speed=0.05, follow=False, travel_rad=None)
        for name in ('left_arm_j2', 'right_arm_j2')
    )


def test_one_arm_qualification_keeps_its_original_tuning_step():
    _, _, payload = qualify.Driver(core(), 125).tune('apply')
    rows = {r['motor']: r for r in payload['joints']}
    assert rows['left_arm_j1']['kp'] == 90.0 and rows['left_arm_j1']['follow'] is True
    assert rows['right_arm_j1'] == dict(
        motor='right_arm_j1', kp=100.0, kd=5.0, speed=0.05, follow=False, travel_rad=None
    )
    _, _, invalid = qualify.Driver(core(), 125).tune('invalid')
    assert next(r for r in invalid['joints'] if r['motor'] == 'left_arm_j1')['kd'] == -1.0


def test_the_velocity_check_follows_the_profiles_law():
    record = lambda q, v: dict(accepted=True, commanded=list(q), commanded_velocity=list(v))
    arm = dict(name='left_arm_j1', max_velocity=1.0, velocity_feedforward=True)
    gripper = dict(name='left_gripper', max_velocity=10.0, velocity_feedforward=False)
    smooth = [record((0.01 * k, 0.0), (0.5, 0.0)) for k in range(6)]
    assert qualify.velocity_output(smooth, [arm, gripper])['ok']
    # A cycle whose velocity drops to zero while its command keeps moving one way: the 2026-10-10 pulse.
    dropped = [record((0.01 * k, 0.0), (0.0 if k == 3 else 0.5, 0.0)) for k in range(6)]
    assert qualify.velocity_output(dropped, [arm, gripper])['zero_drops_while_moving'] == 1
    assert not qualify.velocity_output(dropped, [arm, gripper])['ok']
    assert not qualify.velocity_output([record((0.0, 0.0), (1.5, 0.0))] * 2, [arm, gripper])['ok']  # Over its cap.
    assert not qualify.velocity_output([record((0.0, 0.0), (0.0, 0.1))] * 2, [arm, gripper])['ok']  # A gripper's.
    # Position-only: nothing but zero.
    position = [dict(arm, velocity_feedforward=False), gripper]
    assert qualify.velocity_output([record((0.0, 0.0), (0.0, 0.0))], position) == dict(
        law='direct-position-only', ok=True
    )
    assert not qualify.velocity_output(smooth, position)['ok']
