"""Independent physics expectations, coordinate conventions and fault semantics."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('mujoco')

from robot_platform.simulation.mujoco_plant import JointBinding, MujocoPlant

FIXTURES = Path(__file__).resolve().parents[1] / 'fixtures/physics'


def command(*, effort=0., position=0., velocity=0., kp=0., kd=0.):
    return dict(effort=effort, position=position, velocity=velocity, kp=kp, kd=kd)


def pendulum(filename='pendulum.xml', **kwargs):
    return MujocoPlant(FIXTURES / filename, [JointBinding('motor', 'hinge', 20.)],
                       position_step=0., **kwargs)


@pytest.mark.parametrize('filename', ['pendulum.xml', 'pendulum.urdf'])
def test_gravity_acceleration_matches_pendulum_equation(filename):
    plant = pendulum(filename)
    plant.step(None, .0005)
    # m*g*l / (I_com + m*l^2), at the horizontal pose.
    assert plant.v[0] / .0005 == pytest.approx(2 * 9.81 * .5 / (1 / 6 + 2 * .5**2), rel=1e-5)
    assert plant.q[0] > 0


def test_feedforward_holds_pendulum_and_disabling_clears_torque():
    plant = pendulum()
    for _ in range(200):
        plant.step([command(effort=-9.81)], .0025)
    assert plant.q[0] == pytest.approx(0., abs=1e-10)
    plant.step(None, .0025)
    assert plant.q[0] > 0 and plant.measured()[0]['effort'] == 0


def test_distal_torque_produces_correct_coupled_accelerations_in_receiver_order():
    plant = MujocoPlant(FIXTURES / 'two_link.xml', [JointBinding('second', 'elbow', 5.),
                        JointBinding('first', 'shoulder', 5.)], position_step=0.)
    plant.step([command(effort=1.), command()], .0005)
    # Analytic planar 1 m, 1 kg uniform rods at q=0. XML order is shoulder, elbow.
    inertia = np.array([[8 / 3, 5 / 6], [5 / 6, 1 / 3]])
    expected = np.linalg.solve(inertia, [0., 1.])
    assert plant.v[::-1] / .0005 == pytest.approx(expected, rel=1e-6)
    assert plant.v[1] < 0


@pytest.mark.parametrize('scale', [.02, -.02])
def test_slider_transmission_preserves_power_and_motor_feedback(scale):
    plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('motor', 'slide', 5., scale, .01)],
                       positions=[.5], position_step=0.)
    assert plant.q[0] == pytest.approx(.5)
    plant.step([command(effort=.1)], .0005)
    assert plant.data.qvel[0] / .0005 == pytest.approx(.1 / scale / 2)
    measured = plant.measured()[0]
    assert measured['effort'] * measured['velocity'] == pytest.approx(
        plant.data.qfrc_applied[0] * plant.data.qvel[0])


def test_contact_prevents_falling_through_plane():
    plant = MujocoPlant(FIXTURES / 'slider.xml', [JointBinding('motor', 'slide', 5.)], position_step=0.)
    plant.model.opt.gravity[:] = [0., 0., -9.81]
    for _ in range(400):
        plant.step(None, .0025)
    assert plant.data.ncon > 0
    assert plant.q[0] == pytest.approx(-.05, abs=.002)


def test_noisy_feedback_is_repeatable_and_dropped_motor_goes_limp():
    a, b = [pendulum(speed_noise=.08, position_noise=.0006, seed=21) for _ in range(2)]
    for plant in (a, b):
        plant.quiet.add('motor')
        plant.step([command(effort=100.)], .0025)
    assert a.measured() == b.measured()
    assert a.torque[0] == 20.
    a.dropped.add('motor')
    a.step([command(effort=3.)], .0025)
    assert a.measured()[0]['quiet'] and a.torque[0] == 0.


@pytest.mark.parametrize('dt', [0., -.1, float('nan'), .0007])
def test_invalid_tick_cannot_advance_simulation(dt):
    plant = pendulum()
    with pytest.raises(ValueError):
        plant.step(None, dt)
    assert plant.t == 0.


@pytest.mark.parametrize('commands', [[], [command(kp=-1.)], [command(effort=float('inf'))]])
def test_bad_commands_are_rejected_before_integrating(commands):
    plant = pendulum()
    with pytest.raises(ValueError):
        plant.step(commands, .0025)
    assert plant.t == 0.


def test_bindings_must_identify_real_unique_joints():
    with pytest.raises(ValueError, match='missing'):
        MujocoPlant(FIXTURES / 'pendulum.xml', [JointBinding('motor', 'missing', 5.)])
    with pytest.raises(ValueError, match='one motor'):
        MujocoPlant(FIXTURES / 'pendulum.xml', [JointBinding('a', 'hinge', 5.), JointBinding('b', 'hinge', 5.)])


@pytest.mark.parametrize('fixture,joint', [('pendulum.xml', 'hinge'), ('slider.xml', 'slide')])
def test_joint_validation_accepts_numeric_enums_without_numpy_equality(monkeypatch, fixture, joint):
    import mujoco

    class NumericEnum:
        def __init__(self, value):
            self.value = int(value)

        def __int__(self):
            return self.value

        def __eq__(self, other):
            return False

    # Some platform wheels expose enum equality that does not accept NumPy scalars.
    monkeypatch.setattr(mujoco, 'mjtJoint', SimpleNamespace(
        mjJNT_HINGE=NumericEnum(mujoco.mjtJoint.mjJNT_HINGE),
        mjJNT_SLIDE=NumericEnum(mujoco.mjtJoint.mjJNT_SLIDE)))
    plant = MujocoPlant(FIXTURES / fixture, [JointBinding('motor', joint, 5.)])
    assert plant.q.shape == (1,)


def test_ball_joint_cannot_be_bound_to_a_single_motor(tmp_path):
    path = tmp_path / 'ball.xml'
    path.write_text('<mujoco><worldbody><body><joint name="ball" type="ball"/>'
                    '<geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>')
    with pytest.raises(ValueError, match='hinge and slide'):
        MujocoPlant(path, [JointBinding('motor', 'ball', 5.)])


def test_embedded_actuators_cannot_double_apply_control(tmp_path):
    source = (FIXTURES / 'pendulum.xml').read_text()
    path = tmp_path / 'actuated.xml'
    path.write_text(source.replace('</mujoco>', '<actuator><motor joint="hinge"/></actuator></mujoco>'))
    with pytest.raises(ValueError, match='actuator-free'):
        MujocoPlant(path, [JointBinding('motor', 'hinge', 20.)])


def test_minerva_placeholder_model_is_rejected_before_mujoco_compilation():
    root = Path(__file__).resolve().parents[2]
    with pytest.raises(ValueError, match='URDF physics audit failed'):
        MujocoPlant(root / 'urdf/minerva/minerva.urdf', [JointBinding('motor', 'right_arm_j1', 5.)])
