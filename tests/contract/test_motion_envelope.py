"""Motion-bound collision boxes (two arms): the compiler's boxes against the Rust receiver's own check."""
from copy import deepcopy
import math

import pytest

pytest.importorskip("fcl")
from robot_platform.control.approach import motion_reserve, reaction_s, reserve_toward
from robot_platform.control.collision_envelope import CollisionBlocked, MotionEnvelopeCompiler, calibration_revision
from robot_platform.control.guarded import GuardedControllerClient, GuardRejected
from robot_platform.models import ModelProfile
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.runtime_model import RuntimeCollisionModel, build_runtime_model

pytestmark = pytest.mark.skipif(not library_path().is_file() or not default_binary().is_file(),
                               reason="Build native collision library and controller harness")


@pytest.fixture
def setup(tmp_path):
    backend, cfg, mapping = sliders(tmp_path)
    yield backend, cfg, mapping
    backend.close()


def sliders(tmp_path):
    # Two 0.2 m cubes sliding toward each other along x, each on its own branch.
    urdf = tmp_path / "two_sliders.urdf"
    urdf.write_text('''<robot name="fixture"><link name="root"/>
    <link name="a"><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link>
    <link name="b"><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link>
    <joint name="left" type="prismatic"><parent link="root"/><child link="a"/>
    <axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint>
    <joint name="right" type="prismatic"><parent link="root"/><child link="b"/>
    <axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint></robot>''')
    profile = ModelProfile.from_urdf(urdf)
    artifact = tmp_path / "runtime.json"
    build_runtime_model(profile, artifact)
    runtime = RuntimeCollisionModel(profile, artifact)
    backend = NativeCollisionBackend(runtime, geometry_mode="keepout")
    mapping = {"left": {"actuator": "a", "scale": 1., "offset": 0.},
               "right": {"actuator": "b", "scale": 1., "offset": 0.}}
    cfg = {"group": "whole_robot", "identity": {"model": runtime.revision,
           "calibration": calibration_revision(mapping), "collision_policy": backend.policy_revision},
           "lease_ms": 150, "feedback_timeout_ms": 20, "settled_velocity": .001, "interpolation_ms": 20,
           "collision": {"world_revision": runtime.world_revision, "scope": "inter_branch",
                         "max_verdict_ms": 80, "reaction_ms": 10, "bound": "motion"},
           "joints": [{"name": name, "unit": "metres", "lower": -3., "upper": 3.,
                       "max_velocity": 2., "max_effort": 5., "max_kp": 20., "max_kd": 5.,
                       "stop_acceleration": 2., "hold_kp": 10., "hold_kd": 1.,
                       "max_tracking_error": .01, "max_temperature": 70.} for name in ("a", "b")]}
    return backend, cfg, mapping


def client(harness, cfg):
    return GuardedControllerClient(harness.exchange, group=cfg["group"], identity=cfg["identity"],
                                   joint_names=("a", "b"))


def commands(a, b, va=0., vb=0.):
    return [dict(position=q, velocity=v, effort=0., kp=10., kd=1.) for q, v in ((a, va), (b, vb))]


def armed(harness, cfg, compiler):
    control = client(harness, cfg)
    control.acquire()
    offered = control.collision_offer()
    control.arm(collision=compiler.compile(offered, harness.last["feedback"]))
    return control


def test_a_motion_box_is_the_ramp_widened_by_tracking_and_a_stop_from_the_speed_bound(setup):
    backend, cfg, mapping = setup
    compiler = MotionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.05)
    with NativeControlHarness(cfg, [-1., 1.]) as harness:
        control = armed(harness, cfg, compiler)
        # At rest each joint may move at its approach floor (0.5): the tracking bound, then 30 ms of reaction and
        # a stop at 2 m/s2 from there, 0.0875 m in all.
        rest = motion_reserve(cfg["joints"][0], .03, .5)
        assert rest == pytest.approx(.01 + .015 + .0625)
        offered = control.collision_offer()
        held = compiler.compile(offered, harness.last["feedback"], commands(-1., 1.))
        assert held["lower"] == pytest.approx([-1 - rest, 1 - rest])
        assert held["upper"] == pytest.approx([-1 + rest, 1 + rest])
        # A reserve bound would keep a stop from the 2 m/s cap over the verdict's lifetime: over 1.2 m.
        assert reserve_toward(cfg["joints"][0], reaction_s(cfg), 2.) > 10 * rest
        # The receiver takes the motion box, and refuses one a millimetre short of its own reserve: the two agree.
        control.move(commands(-1., 1.), collision=held)
        offered = control.collision_offer()
        held = compiler.compile(offered, harness.last["feedback"], commands(-1., 1.))
        short = dict(held, lower=[held["lower"][0] + .001, held["lower"][1]])
        with pytest.raises(GuardRejected, match="collision_violation"):
            control.move(commands(-1., 1.), collision=short)


def test_a_stream_of_moves_is_certified_and_taken_and_the_box_follows_the_speed(setup):
    backend, cfg, mapping = setup
    compiler = MotionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.05)
    with NativeControlHarness(cfg, [-1., 1.]) as harness:
        control = armed(harness, cfg, compiler)
        widths = []
        for cycle in range(1, 60):
            for now in range(20 * cycle - 18, 20 * cycle + 1, 2):  # The receiver ticks between commands.
                harness.step(now)
            # Each cube sways at up to 1.2 m/s, ramped as a teleop stream is: from the last target.
            t = cycle * .02
            a, b = -1 + .1 * math.sin(12 * t), 1 - .1 * math.sin(12 * t)
            va, vb = 1.2 * math.cos(12 * t), -1.2 * math.cos(12 * t)
            target = commands(a, b, va, vb)
            offered = control.collision_offer()
            envelope = compiler.compile(offered, harness.last["feedback"], target)
            control.move(target, collision=envelope)
            widths.append(envelope["upper"][0] - envelope["lower"][0])
        # Faster commands keep a longer stop: 1.25x their speed (a servo overshoots what it follows), falling after
        # a slower one at the stop acceleration.
        assert max(widths) > min(widths) + .3
        assert harness.last["status"]["state"] == "executing"
        assert .5 < max(harness.last["status"]["speed_bounds"]) <= 1.25 * 1.2 + 1e-9


def test_a_box_that_could_bring_the_bodies_together_is_blocked_not_sent(setup):
    backend, cfg, mapping = setup
    compiler = MotionEnvelopeCompiler(backend, cfg, mapping, compute_budget_s=.05)
    with NativeControlHarness(cfg, [-.2, .2]) as harness:
        control = armed(harness, cfg, compiler)
        # 0.2 m apart: holding is clear (two 0.0875 m reserves and the clearance fit).
        offered = control.collision_offer()
        control.move(commands(-.2, .2), collision=compiler.compile(offered, harness.last["feedback"],
                                                                   commands(-.2, .2)))
        assert compiler.last_verdict["clear"]
        # Moving them together at 1 m/s each: a stop from 1.25 m/s takes 0.39 m at 2 m/s2; the boxes meet.
        offered = control.collision_offer()
        with pytest.raises(CollisionBlocked, match="could come within") as blocked:
            compiler.compile(offered, harness.last["feedback"], commands(-.18, .18, 1., -1.))
        assert blocked.value.verdict["reason"] == "insufficient_swept_clearance"
        assert set(blocked.value.verdict["links"]) == {"a", "b"}
        assert compiler.blocked == 1 and not compiler.last_verdict["clear"]
        # Bad inputs are not a block: they fault as before.
        stale = deepcopy(harness.last["feedback"])
        for sample in stale:
            sample["sampled_at_ms"] -= 50
        with pytest.raises(ValueError, match="Stale") as unusable:
            compiler.compile(control.collision_offer(), stale, commands(-.2, .2))
        assert not isinstance(unusable.value, CollisionBlocked)


def test_the_compiler_refuses_a_reserve_bound_receiver(setup):
    backend, cfg, mapping = setup
    with pytest.raises(ValueError, match="motion-bound"):
        MotionEnvelopeCompiler(backend, dict(cfg, collision=dict(cfg["collision"], bound="reserve")), mapping)
    with pytest.raises(ValueError, match="ramped"):
        MotionEnvelopeCompiler(backend, dict(cfg, interpolation_ms=0), mapping)
