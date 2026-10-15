"""Helm's virtual receiver driving the simulated Minerva (helm --sim), with scripted leader arms.

The receiver is the one Helm derives for the real arms; the simulated joints carry their Tune
sweeps' inertia, friction and gravity. The MuJoCo window is off here; ModelPose is what it draws.
"""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.helm.teleop import TeleopCore, TeleopSession
from robot_platform.simulation.minerva_sim import NOMINAL, SCENE, ModelPose, joint_specs
from robot_platform.teleop.leader_ranges import KIND, SCHEMA, UNITS, pins

# The rehearsals' scripted leaders, clock and Helm session (tests run with --import-mode=importlib).
_spec = importlib.util.spec_from_file_location('rehearsal_fixtures', Path(__file__).with_name('test_rehearsal.py'))
rehearsal = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rehearsal)
CENTERS, DEVICES, HANG, SPANS = rehearsal.CENTERS, rehearsal.DEVICES, rehearsal.HANG, rehearsal.SPANS
Clock, Leaders, helm, pytestmark = rehearsal.Clock, rehearsal.Leaders, rehearsal.helm, rehearsal.pytestmark

mujoco = pytest.importorskip('mujoco')


def leader_ranges():
    leader_profile = json.loads((DEVICES / 'gello-dual-viewer.json').read_text(encoding='utf-8'))
    return {'schema': SCHEMA, 'kind': KIND, 'profile': leader_profile['id'], 'pins': pins(leader_profile),
            'units': UNITS, 'runtime_approved': False, 'leaders': {
                name: {joint['name']: {'center_rad': center, 'span_rad': span}
                       for joint, center, span in zip(item['joints'], CENTERS[name], SPANS)}
                for name, item in leader_profile['leaders'].items()},
            'reference': {'captured_at': 'fixture', 'leaders': {
                name: {joint['name']: (center + HANG) % (2 * math.pi)
                       for joint, center in zip(item['joints'], CENTERS[name])}
                for name, item in leader_profile['leaders'].items()}}}


def sim_session(helm, tmp_path, profile_name='minerva-teleop.json'):
    profile = json.loads((DEVICES / profile_name).read_text(encoding='utf-8'))
    profile.update(leader_profile=str(DEVICES / 'gello-dual-viewer.json'))
    path = tmp_path / 'teleop.json'
    path.write_text(json.dumps(profile))
    return TeleopSession(helm, path, runs=tmp_path / 'runs', simulation={'viewer': False},
                         leaders=SimpleNamespace(saved=leader_ranges(), missing=lambda routes: []))


def test_the_simulated_minerva_runs_on_this_pc_only(helm, tmp_path):
    session = sim_session(helm, tmp_path)
    with pytest.raises(ValueError, match='runs on this PC'):
        session.set_target('jetson')
    with pytest.raises(ValueError, match='runs on this PC'):
        TeleopSession(helm, session.profile_path, runs=tmp_path / 'runs', simulation={}, target='jetson')
    assert session._bundle()['simulation'] == {'viewer': False, 'directory': str(tmp_path / 'runs' /
                                                                                 f'{helm.asset_id}-sim')}
    session.receiver = {'mode': 'live', 'arm': 'left'}
    assert 'simulation' not in session._bundle()  # Only the virtual receiver is simulated.


@pytest.mark.parametrize('profile_name', ['minerva-teleop.json', 'minerva-teleop-direct.json'])
def test_the_leaders_drive_both_simulated_arms_through_the_real_receiver(helm, tmp_path, profile_name):
    bundle = sim_session(helm, tmp_path, profile_name)._bundle()
    clock = Clock()
    leaders = Leaders(clock)
    core = TeleopCore(bundle, open_leaders=lambda _profile: leaders, clock=clock)

    def run(key=None, *, until=None, seconds):
        for k in range(max(1, round(seconds / core.period_s))):
            core.tick(key if k == 0 else None, clock())
            clock.now += core.period_s
            assert core.fault is None, core.fault
            if until is not None and core.session.lifecycle.state == until:
                return
        assert until is None, f'still {core.session.lifecycle.state}, expected {until}: {core.message}'
    try:
        core.build()
        assert core._receiver_name == 'SIMULATED MINERVA'
        plant = core.harness.plant
        names = list(plant.names)
        assert len(names) == 14  # Both arms and both grippers in one receiver.
        run('I', seconds=.1)
        run('E', seconds=.4)
        run('T', until='TELEOP', seconds=20)
        start, moved = clock(), {name: 0. for name in names}
        before = dict(zip(names, plant.q))
        while clock() - start < 4.:
            t = clock() - start
            for name in CENTERS:
                for k in range(4):
                    leaders.offsets[name][k] = HANG + .25 * math.sin(1.2 * t + .5 * k) * min(1., t)
            run(seconds=core.period_s)
            for name, q in zip(names, plant.q):
                moved[name] = max(moved[name], abs(q - before[name]))
        # Each arm's shoulder and elbow joints moved under the receiver's commands, not the wrists left alone.
        for side in ('left', 'right'):
            assert all(moved[f'{side}_arm_j{k}'] > .05 for k in (1, 2, 3, 4)), moved
            assert all(moved[f'{side}_arm_j{k}'] < .02 for k in (5, 6)), moved
        for name in CENTERS:
            leaders.offsets[name] = [HANG] * 7
        run(seconds=2.)
        run('X', until='DISABLED', seconds=40)
        evidence = core.harness.directory
    finally:
        core.close()
    # A live session's evidence, beside the simulation's own record of which joints are nominal.
    assert evidence.parent == tmp_path / 'runs' / f'{helm.asset_id}-sim'
    columns = json.loads((evidence / 'columns.json').read_text(encoding='utf-8'))['columns']
    samples = np.fromfile(evidence / 'samples.f32', dtype=np.float32).reshape(-1, len(columns))
    assert len(samples) > 1000 and np.allclose(np.diff(samples[:, 0]), 2.5)
    events = [json.loads(line) for line in (evidence / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
    simulation = next(e for e in events if e['kind'] == 'simulation')
    assert simulation['nominal']['left_arm_j1'] == ['inertia', 'friction', 'static']
    assert 'left_arm_j2' not in simulation['nominal']  # Its sweep measured both.
    assert [e['key'] for e in events if e['kind'] == 'key'] == ['I', 'E', 'T', 'X']


def test_joints_without_a_sweep_measurement_take_the_nominal_values():
    servo = {'joints': {
        'left_arm_j1': {'hold_kp': 150., 'hold_kd': 9.8, 'measured': {'torque': {'friction': 5e-20}}},
        'left_arm_j2': {'hold_kp': 220., 'hold_kd': 21., 'measured': {'torque': {'inertia': 1.3, 'friction': .9}}},
        'left_arm_j3': {'hold_kp': 215., 'hold_kd': 3.5, 'measured': {'torque': {'inertia': .32, 'friction': .46}}}}}
    specs, nominal = joint_specs(['left_arm_j1', 'left_arm_j2', 'left_arm_j3', 'left_gripper'], servo)
    j1, j2, j3, gripper = specs
    assert (j1['inertia'], j1['friction'], j1['static']) == NOMINAL['j1']
    assert (j2['inertia'], j2['friction']) == (1.3, .9) and j2['static'] == pytest.approx(1.2)
    # Damping ratio 0.6 at the sweep's gains: J2's kd 21 exceeds 2 * 0.6 * sqrt(220 * 1.3) = 20.3, so the gearbox
    # adds none; J3's kd 3.5 falls short of 2 * 0.6 * sqrt(215 * 0.32) = 9.95 and the gearbox makes up the rest.
    assert j1['viscous'] == j2['viscous'] == 0.
    assert j3['viscous'] == pytest.approx(2 * .6 * math.sqrt(215 * .32) - 3.5)
    assert (gripper['inertia'], gripper['friction'], gripper['viscous']) == (*NOMINAL['gripper'][:2], 0.)
    assert nominal == {'left_arm_j1': ['inertia', 'friction', 'static'],
                       'left_gripper': ['inertia', 'friction', 'static']}


def test_the_window_poses_the_model_through_helms_directions_and_the_jaw_coupling():
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    joint_map = {'right_arm_j1': {'actuator': 'right_arm_j1', 'scale': -1., 'offset': 0.},
                 'left_arm_j2': {'actuator': 'left_arm_j2', 'scale': 1., 'offset': 0.},
                 'left_gripper_finger_a_slide': {'actuator': 'left_gripper', 'scale': .0092, 'offset': 0.},
                 'right_arm_j6': {'actuator': 'right_arm_j6', 'scale': 1., 'offset': 0.}}
    pose = ModelPose(model, joint_map, ['right_arm_j1', 'left_arm_j2', 'left_gripper'])
    assert len(pose.rows) == 3  # right_arm_j6's motor is not in this receiver.
    pose.apply(data, [.4, -.3, 2.])

    def q(joint):
        return data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)]]
    assert q('right_arm_j1') == pytest.approx(-.4) and q('left_arm_j2') == pytest.approx(-.3)
    assert q('left_gripper_finger_a_slide') == pytest.approx(.0184)
    assert q('left_gripper_finger_b_slide') == pytest.approx(.0184)  # The B jaw mimics A (multiplier 1).
    assert q('right_arm_j6') == 0.
