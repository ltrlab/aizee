"""The headset's controllers as targets for Minerva's arms (teleop.vr_arms): a grab and drag per arm, driven by
scripted controller tracks as AIZEE's fake_quest.py drove its Quest leader. Hands report at 60 Hz and the loop
updates at 100 Hz, as on the robot; the arms' limits are the hand-swept travel (signs +1, see test_arm_ik)."""

import json
import math

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.ops.release import ROOT
from robot_platform.teleop.arm_ik import ArmKinematics
from robot_platform.teleop.vr_arms import DragSettings, HandSample, VRArms
from robot_platform.vision.geometry import rotvec

LOOP_HZ, HAND_HZ = 100.0, 60.0
# Working poses: each tool in front of the robot at chest-to-table height, well away from a singular pose (the
# smallest singular value of the Jacobian 0.16; half of all poses in the swept ranges are below 0.06).
WORKING = {
    'left': (-0.1, 0.6, -1.6, -0.6, -2.8, 0.2),
    'right': (0.6, -0.4, -1.7, -1.0, 2.7, -0.4),
}
HOME = (
    0.0,
    1.2,
    -0.4,
)  # Where the scripted hand starts in the VR world: chest height, 40 cm ahead.


@pytest.fixture(scope='module')
def kinematics():
    model = UrdfModel(ModelProfile.load(ROOT / 'urdf/minerva/model.json'))
    swept = json.loads((ROOT / 'config/devices/minerva-production-01-joint-limits.json').read_text())['joints']
    return {
        arm: ArmKinematics(
            model,
            arm,
            {
                f'{arm}_arm_j{n}': (
                    swept[f'{arm}_arm_j{n}']['min_rad'],
                    swept[f'{arm}_arm_j{n}']['max_rad'],
                )
                for n in range(1, 7)
            },
        )
        for arm in ('left', 'right')
    }


def make(kinematics, seed=WORKING, **settings):
    vr = VRArms(kinematics, DragSettings(**settings))
    for arm, positions in seed.items():
        vr.seed(arm, positions)
    return vr


def quaternion(rotation_vector):
    v = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(v))
    if angle == 0:
        return (0.0, 0.0, 0.0, 1.0)
    x, y, z = v / angle * math.sin(angle / 2)
    return (x, y, z, math.cos(angle / 2))


def hand(at, position=HOME, turn=(0.0, 0.0, 0.0), grip=1.0, trigger=0.0, hold=False, ui=False):
    return HandSample(tuple(float(v) for v in position), quaternion(turn), grip, trigger, hold, at, ui)


class Script:
    """Drives `vr` in loop ticks; `hands(t)` gives {arm: hand(...) keyword arguments, or None for no pose} at the 60 Hz
    sample times, held between them."""

    def __init__(self, vr, t=0.0):
        self.vr, self.t = vr, t

    def run(self, seconds, hands):
        commands = []
        for _ in range(int(round(seconds * LOOP_HZ))):
            self.t += 1 / LOOP_HZ
            sampled = math.floor(self.t * HAND_HZ + 1e-9) / HAND_HZ
            samples = {}
            for arm, kwargs in hands(sampled).items():
                samples[arm] = None if kwargs is None else hand(sampled, **kwargs)
            commands.append(self.vr.update(self.t, samples))
        return commands


def tool(kinematics, command, arm='left'):
    return kinematics[arm].pose(command[arm].positions)


def still(**kwargs):
    return lambda t: {'left': dict(kwargs)}


def test_an_unseeded_arm_commands_nothing(kinematics):
    vr = VRArms(kinematics)
    command = vr.update(1.0, {'left': hand(1.0)})['left']
    assert command.positions is None and not command.gripped


def test_moving_without_the_grip_moves_nothing(kinematics):
    vr = make(kinematics)
    commands = Script(vr).run(
        1.0,
        lambda t: {'left': dict(position=(HOME[0] + 0.2 * t, HOME[1], HOME[2]), grip=0.0)},
    )
    assert all(c['left'].positions == WORKING['left'] for c in commands)
    assert not any(c['left'].gripped for c in commands)


def test_a_grip_held_when_the_arm_is_seeded_must_be_let_go_first(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    commands = script.run(0.5, lambda t: {'left': dict(position=(HOME[0], HOME[1], HOME[2] - 0.2 * t))})
    assert not commands[-1]['left'].gripped and commands[-1]['left'].positions == WORKING['left']
    script.run(0.1, still(position=(HOME[0], HOME[1], HOME[2] - 0.1), grip=0.0))
    commands = script.run(0.1, still(position=(HOME[0], HOME[1], HOME[2] - 0.1)))
    assert commands[-1]['left'].gripped


@pytest.mark.parametrize(
    'vr_move, robot_move',
    [
        ((0.0, 0.0, -0.05), (0.0, -0.05, 0.0)),  # Ahead: the robot's front (-y).
        ((0.05, 0.0, 0.0), (-0.05, 0.0, 0.0)),  # Right: the robot's right (-x).
        ((0.0, 0.05, 0.0), (0.0, 0.0, 0.05)),  # Up: up (+z).
    ],
)
def test_dragging_moves_the_tool_as_the_hand_moved_in_the_robots_frame(kinematics, vr_move, robot_move):
    vr = make(kinematics)
    script = Script(vr)
    start = tool(kinematics, script.run(0.1, still(grip=0.0))[-1])
    script.run(0.1, still())
    moved = tuple(h + m for h, m in zip(HOME, vr_move))
    end = tool(kinematics, script.run(0.5, still(position=moved))[-1])
    np.testing.assert_allclose(end[:3, 3] - start[:3, 3], robot_move, atol=1e-3)
    assert np.linalg.norm(rotvec(end[:3, :3] @ start[:3, :3].T)) < 5e-3


def test_the_forward_yaw_turns_the_hands_frame(kinematics):
    # The operator faces a quarter turn left of where the session started (their ahead is the VR world's -x).
    vr = make(kinematics)
    vr.set_forward(math.pi / 2)
    script = Script(vr)
    start = tool(kinematics, script.run(0.1, still(grip=0.0))[-1])
    script.run(0.1, still())
    end = tool(
        kinematics,
        script.run(0.5, still(position=(HOME[0] - 0.05, HOME[1], HOME[2])))[-1],
    )
    np.testing.assert_allclose(end[:3, 3] - start[:3, 3], (0.0, -0.05, 0.0), atol=1e-3)


@pytest.mark.parametrize(
    'vr_turn, robot_turn',
    [
        ((0.0, 0.2, 0.0), (0.0, 0.0, 0.2)),  # About up: about up.
        (
            (0.2, 0.0, 0.0),
            (-0.2, 0.0, 0.0),
        ),  # About the operator's right: about the robot's right (-x).
    ],
)
def test_turning_the_hand_turns_the_tool_about_the_same_world_axis(kinematics, vr_turn, robot_turn):
    vr = make(kinematics)
    script = Script(vr)
    start = tool(kinematics, script.run(0.1, still(grip=0.0))[-1])
    script.run(0.1, still())
    end = tool(kinematics, script.run(0.6, still(turn=vr_turn))[-1])
    np.testing.assert_allclose(rotvec(end[:3, :3] @ start[:3, :3].T), robot_turn, atol=5e-3)
    np.testing.assert_allclose(end[:3, 3], start[:3, 3], atol=1e-3)  # It turns about the tool point.


def test_letting_go_holds_and_the_next_grip_goes_on_from_there(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    start = tool(kinematics, script.run(0.1, still(grip=0.0))[-1])
    script.run(0.1, still())
    script.run(0.5, still(position=(HOME[0], HOME[1] + 0.05, HOME[2])))
    held = script.run(0.5, still(position=HOME, grip=0.0))  # Let go and bring the hand back down.
    assert all(c['left'].positions == held[0]['left'].positions for c in held)
    script.run(0.1, still())
    end = tool(
        kinematics,
        script.run(0.5, still(position=(HOME[0], HOME[1] + 0.05, HOME[2])))[-1],
    )
    np.testing.assert_allclose(end[:3, 3] - start[:3, 3], (0.0, 0.0, 0.1), atol=1.5e-3)


def test_a_lost_hand_lets_go_and_attaches_again_without_a_jump(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    before = script.run(0.4, still(position=(HOME[0], HOME[1] + 0.05, HOME[2])))[-1]
    lost = script.run(0.2, lambda t: {'left': None})
    assert all(not c['left'].tracking and not c['left'].gripped and c['left'].closure is None for c in lost)
    assert lost[-1]['left'].positions == before['left'].positions
    # Back 10 cm away with the grip still held: attached there, nothing moves; then a 3 cm drag moves 3 cm.
    back = (HOME[0] + 0.1, HOME[1] + 0.05, HOME[2])
    resumed = script.run(0.2, still(position=back))[-1]
    np.testing.assert_allclose(tool(kinematics, resumed)[:3, 3], tool(kinematics, before)[:3, 3], atol=1e-6)
    end = script.run(0.4, still(position=(back[0], back[1] + 0.03, back[2])))[-1]
    np.testing.assert_allclose(
        tool(kinematics, end)[:3, 3] - tool(kinematics, before)[:3, 3],
        (0.0, 0.0, 0.03),
        atol=1e-3,
    )


def test_a_recenter_attaches_again_where_the_hand_is(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    before = script.run(0.4, still(position=(HOME[0], HOME[1] + 0.05, HOME[2])))[-1]
    vr.recenter()
    moved = (
        HOME[0] + 1.0,
        HOME[1] + 0.05,
        HOME[2] + 1.0,
    )  # The VR world starts again a metre away.
    after = script.run(0.3, still(position=moved))[-1]
    np.testing.assert_allclose(tool(kinematics, after)[:3, 3], tool(kinematics, before)[:3, 3], atol=1e-6)
    assert after['left'].gripped


def test_a_one_pose_glitch_moves_nothing(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    before = script.run(0.3, still())[-1]
    glitch = 1 / HAND_HZ * math.ceil(script.t * HAND_HZ + 1)
    commands = script.run(
        0.3,
        lambda t: {'left': dict(position=(HOME[0] + 0.5, HOME[1], HOME[2]) if abs(t - glitch) < 1e-6 else HOME)},
    )
    for command in commands:
        np.testing.assert_allclose(tool(kinematics, command)[:3, 3], tool(kinematics, before)[:3, 3], atol=1e-6)


def test_a_glitch_that_persists_attaches_again_and_moves_nothing(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    before = script.run(0.3, still())[-1]
    away = (HOME[0] + 0.5, HOME[1], HOME[2])
    after = script.run(0.3, still(position=away))[-1]
    np.testing.assert_allclose(tool(kinematics, after)[:3, 3], tool(kinematics, before)[:3, 3], atol=1e-6)
    end = script.run(0.5, still(position=(away[0], away[1] + 0.04, away[2])))[-1]
    np.testing.assert_allclose(
        tool(kinematics, end)[:3, 3] - tool(kinematics, before)[:3, 3],
        (0.0, 0.0, 0.04),
        atol=1e-3,
    )


def test_the_tools_target_moves_at_most_the_tool_speed(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    start = previous = tool(kinematics, script.run(0.05, still())[-1])
    commands = script.run(0.6, still(position=(HOME[0], HOME[1] - 0.2, HOME[2])))  # 20 cm down in one pose.
    assert any(c['left'].clamped for c in commands)
    for command in commands:
        position = tool(kinematics, command)[:3, 3]
        assert np.linalg.norm(position - previous[:3, 3]) <= 0.6 / LOOP_HZ + 1e-3
        previous = tool(kinematics, command)
    np.testing.assert_allclose(previous[:3, 3] - start[:3, 3], (0.0, 0.0, -0.2), atol=1e-3)


def test_near_the_singular_reference_pose_each_joint_moves_at_most_its_speed(
    kinematics,
):
    vr = make(kinematics, seed={'left': (0.0,) * 6})
    script = Script(vr)
    first = script.run(0.1, still(grip=0.0))[-1]
    assert first['left'].near_singular  # The arm hangs straight: J1 and J5 both vertical.
    script.run(0.1, still())
    commands = script.run(1.0, still(position=(HOME[0] + 0.05, HOME[1], HOME[2])))
    previous = np.zeros(6)
    for command in commands:
        assert np.max(np.abs(np.asarray(command['left'].positions) - previous)) <= 3.0 / LOOP_HZ + 1e-9
        previous = np.asarray(command['left'].positions)
    assert any(c['left'].clamped for c in commands)


def test_the_workspace_box_holds_the_target_inside(kinematics):
    start = kinematics['left'].pose(WORKING['left'])[:3, 3]
    floor = start[2] - 0.02
    vr = make(kinematics, workspace={'left': ((-2.0, -2.0, floor), (2.0, 2.0, 3.0))})
    script = Script(vr)
    script.run(0.1, still(grip=0.0))
    script.run(0.1, still())
    commands = script.run(0.6, still(position=(HOME[0], HOME[1] - 0.1, HOME[2])))
    assert tool(kinematics, commands[-1])[2, 3] == pytest.approx(floor, abs=1e-3)
    assert commands[-1]['left'].clamped


def test_the_trigger_closes_the_gripper_and_a_lost_hand_leaves_it_alone(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    assert script.run(0.1, still(grip=0.0, trigger=0.7))[-1]['left'].closure == pytest.approx(0.7)
    assert script.run(0.1, still(grip=0.0, trigger=1.3))[-1]['left'].closure == 1.0
    assert script.run(0.2, lambda t: {'left': None})[-1]['left'].closure is None


def test_b_or_y_asks_for_hold_once_a_press(kinematics):
    vr = make(kinematics)
    commands = Script(vr).run(0.5, lambda t: {'left': dict(grip=0.0, hold=0.1 <= t < 0.3)})
    assert sum(c['left'].hold for c in commands) == 1


def test_a_scripted_circle_is_followed(kinematics):
    # fake_quest.py's circle: 10 cm in the operator's vertical plane at a quarter turn a second, the grip held.
    vr = make(kinematics)
    script = Script(vr)
    start = tool(
        kinematics,
        script.run(0.1, lambda t: {'left': dict(grip=0.0), 'right': dict(grip=0.0)})[-1],
    )
    script.run(0.1, lambda t: {'left': {}, 'right': {}})
    origin = script.t

    def circle(t):
        phase = 2 * math.pi * 0.25 * (t - origin)
        offset = (0.1 * (math.cos(phase) - 1), 0.1 * math.sin(phase), 0.0)
        return {
            'left': dict(position=tuple(h + o for h, o in zip(HOME, offset))),
            'right': dict(grip=0.0),
        }

    commands = script.run(4.0, circle)
    for command in commands[50:]:
        phase = (
            2
            * math.pi
            * 0.25
            * (
                math.floor((script.t - (len(commands) - 1 - commands.index(command)) / LOOP_HZ) * HAND_HZ + 1e-9)
                / HAND_HZ
                - origin
            )
        )
        expected = start[:3, 3] + np.array([-0.1 * (math.cos(phase) - 1), 0.0, 0.1 * math.sin(phase)])
        assert np.linalg.norm(tool(kinematics, command)[:3, 3] - expected) < 0.008
        assert command['left'].converged and not command['left'].clamped
        assert command['right'].positions == WORKING['right']


def test_a_new_scale_attaches_again_without_a_jump_and_scales_the_drag(kinematics):
    vr = make(kinematics)
    script = Script(vr)
    start = tool(kinematics, script.run(0.1, still(grip=0.0))[-1])
    script.run(0.1, still())
    vr.set_scale(0.5)
    after = script.run(0.05, still())[-1]
    np.testing.assert_allclose(tool(kinematics, after)[:3, 3], start[:3, 3], atol=1e-6)
    end = script.run(0.5, still(position=(HOME[0], HOME[1] + 0.1, HOME[2])))[-1]
    np.testing.assert_allclose(tool(kinematics, end)[:3, 3] - start[:3, 3], (0.0, 0.0, 0.05), atol=1e-3)


def test_settings_and_samples_are_checked(kinematics):
    with pytest.raises(ValueError, match='positive'):
        DragSettings(scale=0.0)
    with pytest.raises(ValueError, match='lets go below'):
        DragSettings(grip_on=0.3, grip_off=0.4)
    with pytest.raises(ValueError, match='workspace box'):
        DragSettings(workspace={'left': ((0.0, 0.0, 1.0), (1.0, 1.0, 0.0))})
    with pytest.raises(ValueError, match='unit quaternion'):
        HandSample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0), 1.0, 0.0, False, 0.0)
    with pytest.raises(ValueError, match='finite'):
        HandSample((0.0, math.nan, 0.0), (0.0, 0.0, 0.0, 1.0), 1.0, 0.0, False, 0.0)
    with pytest.raises(ValueError, match='Kinematics'):
        VRArms({'left': kinematics['right']})


def test_a_hand_working_the_headsets_screens_leaves_its_arm_and_gripper_alone(kinematics):
    # Its grip moving a screen, or its trigger on the bar: the arm does not attach, the gripper stays where it was,
    # and when the screen is let go of the grip must be let go too before the arm attaches.
    vr = make(kinematics)
    script = Script(vr)
    script.run(0.1, still(grip=0.0, trigger=0.3))
    busy = script.run(
        0.4, lambda t: {'left': dict(position=(HOME[0], HOME[1] + 0.1 * t, HOME[2]), trigger=0.9, ui=True)}
    )
    assert all(c['left'].positions == WORKING['left'] and not c['left'].gripped for c in busy)
    assert all(c['left'].closure == pytest.approx(0.3) for c in busy)
    after = script.run(0.2, still(position=(HOME[0], HOME[1] + 0.04, HOME[2])))
    assert all(not c['left'].gripped for c in after)  # Still held from the screen: not attached.
    script.run(0.1, still(position=(HOME[0], HOME[1] + 0.04, HOME[2]), grip=0.0))
    assert script.run(0.1, still(position=(HOME[0], HOME[1] + 0.04, HOME[2])))[-1]['left'].gripped


@pytest.mark.parametrize('arm', ('left', 'right'))
def test_raised_from_hanging_straight_the_forearm_folds_forward(kinematics, arm):
    # The hanging arm is singular (straight elbow). Raising the hand 5 cm bends the elbow; unbent the IK folded it
    # the wrong way (forearm back, elbow ahead of the hand). Minerva's forearm folds forward: CAD J3 negative.
    vr = make(kinematics, seed={arm: (0.0,) * 6})
    script = Script(vr)
    hands = lambda grip, position=HOME: lambda t: {arm: dict(position=position, grip=grip)}  # noqa: E731
    start = tool(kinematics, script.run(0.1, hands(0.0))[-1], arm)[:3, 3]
    script.run(0.1, hands(1.0))
    command = script.run(1.5, hands(1.0, (HOME[0], HOME[1] + 0.05, HOME[2])))[-1][arm]
    np.testing.assert_allclose(kinematics[arm].pose(command.positions)[:3, 3] - start, (0.0, 0.0, 0.05), atol=1e-3)
    assert command.converged and command.positions[2] < -0.5
    with pytest.raises(ValueError, match='elbow_bend'):
        DragSettings(elbow_bend=0.5)
