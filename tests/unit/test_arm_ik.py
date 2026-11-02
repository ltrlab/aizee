"""Inverse kinematics for Minerva's arms (teleop.arm_ik), for driving them from the headset's controllers: the chain
agrees with the URDF, the solver comes back to poses it was given, stays inside the limits and says when a target is
out of reach.

The limits are the hand-swept travel (config/devices/minerva-production-01-joint-limits.json) as CAD angles. Each
joint's direction sign lives in Helm's run state, not in the repository, so every check runs with all signs +1 and
all -1: each joint's travel is then tried both ways round.
"""

import json
import math

import numpy as np
import pytest

from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.ops.release import ROOT
from robot_platform.teleop.arm_ik import ArmKinematics, cad_limits
from robot_platform.vision.chain import Chain
from robot_platform.vision.geometry import rotation, rotvec, transform

ARMS = ("left", "right")
SIGNS = (1, -1)


@pytest.fixture(scope="module")
def model():
    return UrdfModel(ModelProfile.load(ROOT / "urdf/minerva/model.json"))


@pytest.fixture(scope="module")
def swept():
    return json.loads(
        (ROOT / "config/devices/minerva-production-01-joint-limits.json").read_text()
    )["joints"]


def limits_for(swept, arm, sign):
    return {
        f"{arm}_arm_j{n}": tuple(
            sorted(
                (
                    sign * swept[f"{arm}_arm_j{n}"]["min_rad"],
                    sign * swept[f"{arm}_arm_j{n}"]["max_rad"],
                )
            )
        )
        for n in range(1, 7)
    }


def arms(model, swept):
    return [
        (arm, sign, ArmKinematics(model, arm, limits_for(swept, arm, sign)))
        for arm in ARMS
        for sign in SIGNS
    ]


def inside(kinematics, positions):
    q = np.asarray(positions)
    return bool(np.all(q >= kinematics.lower) and np.all(q <= kinematics.upper))


def test_the_chain_places_the_tool_where_the_urdf_does(model, swept):
    rng = np.random.default_rng(0)
    for arm in ARMS:
        kinematics = ArmKinematics(model, arm, limits_for(swept, arm, 1))
        chain = Chain(model.profile.urdf, model.by_name[f"{arm}_arm_j6"]["child"])
        for _ in range(20):
            q = rng.uniform(kinematics.lower, kinematics.upper)
            expected = chain.pose(dict(zip(kinematics.joints, q))) @ transform(
                np.eye(3), kinematics.tool
            )
            np.testing.assert_allclose(kinematics.pose(q), expected, atol=1e-12)


def test_the_tool_point_is_midway_between_the_finger_carriages_and_the_arms_mirror(
    model, swept
):
    left = ArmKinematics(model, "left", limits_for(swept, "left", 1))
    right = ArmKinematics(model, "right", limits_for(swept, "right", 1))
    # The finger slides' origins in each gripper link (urdf/minerva/minerva.urdf).
    np.testing.assert_allclose(left.tool, [0.0, 0.0288, -0.045], atol=1e-12)
    np.testing.assert_allclose(right.tool, [0.0, 0.0288, -0.045], atol=1e-12)
    # At the reference pose the arms hang either side of the robot (left +x, right -x), level with each other.
    at_left, at_right = left.pose(np.zeros(6))[:3, 3], right.pose(np.zeros(6))[:3, 3]
    assert at_left[0] > 0.2 and at_right[0] < -0.2
    np.testing.assert_allclose(at_left * [-1, 1, 1], at_right, atol=1e-3)


def test_the_jacobian_matches_finite_differences(model, swept):
    rng, h = np.random.default_rng(1), 1e-6
    for _arm, _sign, kinematics in arms(model, swept):
        q = rng.uniform(kinematics.lower, kinematics.upper)
        jacobian = kinematics.jacobian(q)
        for i in range(6):
            step = np.zeros(6)
            step[i] = h
            before, after = kinematics.pose(q - step), kinematics.pose(q + step)
            np.testing.assert_allclose(
                jacobian[:3, i], (after[:3, 3] - before[:3, 3]) / (2 * h), atol=1e-6
            )
            np.testing.assert_allclose(
                jacobian[3:, i],
                rotvec(after[:3, :3] @ before[:3, :3].T) / (2 * h),
                atol=1e-6,
            )


def test_poses_come_back_from_a_seed_near_their_answer(model, swept):
    rng = np.random.default_rng(2)
    for arm, sign, kinematics in arms(model, swept):
        for _ in range(200):
            q = rng.uniform(kinematics.lower, kinematics.upper)
            seed = np.clip(
                q + rng.uniform(-0.1, 0.1, 6), kinematics.lower, kinematics.upper
            )
            solution = kinematics.solve(kinematics.pose(q), seed, max_iterations=12)
            assert solution.converged, (arm, sign, q, solution)
            assert (
                solution.position_error_m <= 1e-3
                and solution.orientation_error_rad <= 0.01
            )
            assert inside(kinematics, solution.positions)


def test_almost_every_pose_comes_back_from_a_seed_farther_away(model, swept):
    # 0.3 rad from the answer a local solver can settle on another configuration with a joint at its limit
    # (1 to 3 in 300 when this was written); it then says it did not converge.
    rng = np.random.default_rng(3)
    for arm, sign, kinematics in arms(model, swept):
        converged = 0
        for _ in range(200):
            q = rng.uniform(kinematics.lower, kinematics.upper)
            seed = np.clip(
                q + rng.normal(0.0, 0.3, 6), kinematics.lower, kinematics.upper
            )
            solution = kinematics.solve(kinematics.pose(q), seed, max_iterations=50)
            converged += solution.converged
            assert inside(kinematics, solution.positions)
            assert solution.converged == (
                solution.position_error_m <= 2e-4
                and solution.orientation_error_rad <= 2e-3
            )
        assert converged >= 194, (arm, sign, converged)


def test_a_moving_target_is_followed_from_each_answer_in_a_few_iterations(model, swept):
    # A smooth reachable path, as a hand drags the gripper: 60 frames, each joint moving at most 0.01 rad a frame
    # (0.6 rad/s at 60 Hz), from a bent elbow well inside the limits.
    rng = np.random.default_rng(4)
    for arm, sign, kinematics in arms(model, swept):
        middle, half = (
            (kinematics.lower + kinematics.upper) / 2,
            (kinematics.upper - kinematics.lower) / 2,
        )
        start = middle + 0.4 * half * rng.choice([-1.0, 1.0], 6)
        rate = rng.uniform(-0.01, 0.01, 6)
        q = start.copy()
        for frame in range(1, 61):
            solution = kinematics.solve(kinematics.pose(start + rate * frame), q)
            assert solution.converged and solution.iterations <= 4, (
                arm,
                sign,
                frame,
                solution,
            )
            q = np.asarray(solution.positions)


def test_a_target_out_of_reach_returns_the_nearest_answer_unconverged(model, swept):
    for _arm, _sign, kinematics in arms(model, swept):
        seed = (kinematics.lower + kinematics.upper) / 2
        start = kinematics.pose(seed)
        target = start.copy()
        target[:3, 3] += [
            0.0,
            -3.0,
            0.0,
        ]  # Three metres in front: beyond the arm's reach.
        solution = kinematics.solve(target, seed, max_iterations=50)
        assert not solution.converged
        assert inside(kinematics, solution.positions)
        reached = kinematics.pose(solution.positions)[:3, 3]
        assert (
            np.linalg.norm(target[:3, 3] - reached)
            < np.linalg.norm(target[:3, 3] - start[:3, 3]) - 0.3
        )
        assert solution.position_error_m == pytest.approx(
            np.linalg.norm(target[:3, 3] - reached)
        )


def test_a_joint_pinned_at_its_limit_leaves_the_others_to_move(model, swept):
    # Shoulder swivel J1 held to a narrow window: a target that needs it beyond the window is approached with the
    # other joints, and J1 is reported at its limit.
    arm = "right"
    limits = limits_for(swept, arm, 1)
    q = np.array([0.3, 0.4, -0.8, 0.5, 0.3, 0.2])
    limits[f"{arm}_arm_j1"] = (0.25, 0.35)
    kinematics = ArmKinematics(model, arm, limits)
    beyond = q.copy()
    beyond[0] = 0.6
    target = kinematics.pose(beyond)
    solution = kinematics.solve(target, q, max_iterations=50)
    assert f"{arm}_arm_j1" in solution.at_limit
    assert solution.positions[0] == pytest.approx(0.35)
    # The others take up what they can (61 to 31 mm when this was written, the orientation kept to 0.003 rad).
    assert np.max(np.abs(np.asarray(solution.positions[1:]) - q[1:])) > 0.05
    assert solution.position_error_m < 0.6 * np.linalg.norm(
        target[:3, 3] - kinematics.pose(q)[:3, 3]
    )


def test_cad_limits_take_the_usable_motor_range_through_each_joints_mapping():
    report = [
        {
            "motor": "right_arm_j1",
            "cad_joint": "right_arm_j1",
            "scale": -1.0,
            "offset": 0.5,
            "usable": [-1.0, 2.0],
        },
        {
            "motor": "right_arm_j2",
            "cad_joint": "right_arm_j2",
            "scale": 1.0,
            "offset": 0.0,
            "usable": [-0.5, 0.5],
        },
        {
            "motor": "left_arm_j1",
            "cad_joint": "left_arm_j1",
            "scale": 1.0,
            "offset": 0.0,
            "usable": [0.0, 1.0],
        },
        {
            "motor": "right_gripper",
            "cad_joint": "right_gripper_finger_a_slide",
            "scale": 0.01,
            "offset": 0.0,
            "usable": [0.0, 4.0],
        },
    ]
    assert cad_limits(report, "right") == {
        "right_arm_j1": (-1.5, 1.5),
        "right_arm_j2": (-0.5, 0.5),
    }


def test_bad_inputs_are_refused(model, swept):
    limits = limits_for(swept, "left", 1)
    with pytest.raises(ValueError, match="left or right"):
        ArmKinematics(model, "middle", limits)
    with pytest.raises(ValueError, match="exactly the left arm joints"):
        ArmKinematics(
            model, "left", {k: v for k, v in limits.items() if not k.endswith("j6")}
        )
    with pytest.raises(ValueError, match="low below high"):
        ArmKinematics(model, "left", dict(limits, left_arm_j2=(1.0, -1.0)))
    kinematics = ArmKinematics(model, "left", limits)
    with pytest.raises(ValueError, match="6 finite angles"):
        kinematics.pose([0.0] * 5)
    with pytest.raises(ValueError, match="6 finite angles"):
        kinematics.solve(np.eye(4), [math.nan] * 6)
    with pytest.raises(ValueError, match="finite 4x4"):
        kinematics.solve(np.eye(3), np.zeros(6))


def test_a_small_turn_of_the_target_turns_the_tool_about_the_same_world_axis(
    model, swept
):
    # What grab and drag needs: a rotation applied to the target in the robot frame is the tool's rotation.
    kinematics = ArmKinematics(model, "left", limits_for(swept, "left", 1))
    q = np.array([-0.4, 0.3, 0.6, -0.3, 0.5, 0.2])
    start = kinematics.pose(q)
    turn = rotation(np.array([0.0, 0.0, 0.05]))  # 0.05 rad about the robot's up axis.
    target = start.copy()
    target[:3, :3] = turn @ start[:3, :3]
    solution = kinematics.solve(target, q)
    assert solution.converged
    np.testing.assert_allclose(
        rotvec(kinematics.pose(solution.positions)[:3, :3] @ start[:3, :3].T),
        [0.0, 0.0, 0.05],
        atol=2e-3,
    )
