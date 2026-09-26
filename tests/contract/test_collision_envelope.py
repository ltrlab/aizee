"""Real collision queries and Rust receiver integration, with synthetic geometry."""
from copy import deepcopy
import pytest

pytest.importorskip("fcl")
from robot_platform.control.collision_envelope import CollisionEnvelopeCompiler, calibration_revision
from robot_platform.control.guarded import GuardedControllerClient, GuardRejected
from robot_platform.models import ModelProfile
from robot_platform.simulation.runtime_model import RuntimeCollisionModel, build_runtime_model
from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary

pytestmark = pytest.mark.skipif(not library_path().is_file() or not default_binary().is_file(),
                               reason="Build native collision library and controller harness")


@pytest.fixture
def setup(tmp_path):
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
           "lease_ms": 100, "feedback_timeout_ms": 20, "settled_velocity": .001,
           "collision": {"world_revision": runtime.world_revision, "scope": "all",
                         "max_verdict_ms": 60, "reaction_ms": 10},
           "joints": [{"name": name, "unit": "metres", "lower": -3., "upper": 3.,
                       "max_velocity": .1, "max_effort": 5., "max_kp": 20., "max_kd": 5.,
                       "stop_acceleration": 1., "hold_kp": 10., "hold_kd": 1.,
                       "max_tracking_error": .01, "max_temperature": 70.} for name in ("a", "b")]}
    yield backend, cfg, mapping
    backend.close()


def client(harness, cfg):
    return GuardedControllerClient(harness.exchange, group=cfg["group"], identity=cfg["identity"],
                                   joint_names=("a", "b"))


def commands(a=-.9, b=.9):
    return [dict(position=q, velocity=0., effort=0., kp=10., kd=1.) for q in (a, b)]


def test_native_collision_envelope_is_enforced_and_expires_without_new_messages(setup):
    backend, cfg, mapping = setup
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping)
    with NativeControlHarness(cfg, [-1., 1.]) as harness:
        control = client(harness, cfg)
        control.acquire()
        with pytest.raises(GuardRejected, match="collision_required"):
            control.arm()
        offered = control.collision_offer()
        control.arm(collision=compiler.compile(offered, harness.last["feedback"]))
        offered = control.collision_offer()
        envelope = compiler.compile(offered, harness.last["feedback"], commands())
        assert compiler.last_verdict["queries"] == 1
        control.move(commands(), collision=envelope)
        harness.step(20)
        control.renew()
        assert harness.last["status"]["lease"]["expires_at_ms"] == 120
        result = harness.step(60)
        assert result["status"]["reason"] == "collision_expired"
        assert result["status"]["state"] == "stopped"
        assert result["status"]["lease"] is None


def test_a_gripper_reaches_its_limit_and_keeps_its_target_when_the_receiver_stops(setup):
    backend, cfg, mapping = setup
    cfg = deepcopy(cfg)
    cfg["joints"][1]["gripper"] = True
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping)
    with NativeControlHarness(cfg, [-1., 1.]) as harness:
        control = client(harness, cfg)
        control.acquire()
        offered = control.collision_offer()
        control.arm(collision=compiler.compile(offered, harness.last["feedback"]))
        offered = control.collision_offer()
        with pytest.raises(ValueError, match="exceeds operational interval"):
            compiler.compile(offered, harness.last["feedback"], commands(a=3., b=1.))  # An arm keeps its reserve.
        offered = control.collision_offer()
        control.move(commands(a=-.9, b=3.), collision=compiler.compile(offered, harness.last["feedback"],
                                                                        commands(a=-.9, b=3.)))
        result = harness.step(80)  # The verdict expired: the receiver stops by itself.
        assert (result["status"]["state"], result["status"]["reason"]) == ("stopped", "collision_expired")
        assert result["status"]["joints"][1]["position"] == 3.  # The gripper holds its target at the limit.


def test_box_check_rejects_asynchronous_crossing_even_if_endpoints_are_clear(setup):
    backend, cfg, mapping = setup
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping)
    with NativeControlHarness(cfg, [-1., -.6]) as harness:
        control = client(harness, cfg)
        control.acquire()
        offered = control.collision_offer()
        assert not backend.proximity({"left": -1., "right": -.6})["near_pairs"]
        assert not backend.proximity({"left": 1., "right": 1.4})["near_pairs"]
        with pytest.raises(ValueError, match="insufficient_swept_clearance"):
            compiler.compile(offered, harness.last["feedback"], commands(1., 1.4))
        assert not harness.last["hardware_enabled"]


@pytest.mark.parametrize("failure", ["stale", "skew", "fault", "nan", "roster", "world", "deadline", "limits", "feedforward"])
def test_bad_state_or_late_collision_work_never_produces_an_envelope(setup, failure):
    backend, cfg, mapping = setup
    clock = [1.]
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping, clock=lambda: clock[0])
    with NativeControlHarness(cfg, [-1., 1.]) as harness:
        control = client(harness, cfg)
        control.acquire()
        harness.step(30)
        offered = control.collision_offer()
        feedback = deepcopy(harness.last["feedback"])
        target = commands()
        if failure == "stale":
            feedback[0]["sampled_at_ms"] = 10
        elif failure == "skew":
            feedback[0]["sampled_at_ms"] = 19
        elif failure == "fault":
            feedback[0]["fault"] = True
        elif failure == "nan":
            feedback[0]["position"] = float("nan")
        elif failure == "roster":
            offered["status"]["joint_names"].reverse()
        elif failure == "world":
            backend.runtime_model.world_revision = "changed"
        elif failure == "limits":
            target[0]["position"] = -2.999
        elif failure == "feedforward":
            target[0]["effort"] = .1
        with pytest.raises(ValueError):
            compiler.compile(offered, feedback, target, started_at=.9 if failure == "deadline" else 1.)
        assert compiler.last_verdict["clear"] is False


def test_negative_transmission_scale_maps_both_extrema_and_pins_revision(setup):
    backend, cfg, mapping = setup
    mapping["left"]["scale"] = -1.
    with pytest.raises(ValueError, match="revision"):
        CollisionEnvelopeCompiler(backend, cfg, mapping)
    cfg["identity"]["calibration"] = calibration_revision(mapping)
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping)
    with NativeControlHarness(cfg, [1., 1.]) as harness:
        control = client(harness, cfg)
        control.acquire()
        offered = control.collision_offer()
        envelope = compiler.compile(offered, harness.last["feedback"], commands(.9, .9))
        assert envelope["lower"][0] < .9 and envelope["upper"][0] > 1.


def teleop(harness, cfg, backend, mapping, clock):
    from robot_platform.teleop.leader_mapping import LeaderTargetMapper
    from robot_platform.teleop.supervisor import GuardedTeleopSupervisor
    mapper = LeaderTargetMapper(cfg["joints"], [
        dict(actuator="a", leader="left", channel=0, scale=1.),
        dict(actuator="b", leader="right", channel=0, scale=1.)])
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping, clock=lambda: clock[0])
    return GuardedTeleopSupervisor(client(harness, cfg), compiler, mapper,
                                   lambda: harness.last, clock=lambda: clock[0])


def leader_pair(now=0., sequence=1, left=0., right=0.):
    from robot_platform.teleop.openrb import LeaderSample
    return {name: LeaderSample((q,), (0,), sequence, now, now, (0, 0, 1, 1), 0, 0)
            for name, q in (("left", left), ("right", right))}


def test_whole_robot_teleop_registers_without_jump_and_stops_before_collision(setup):
    backend, cfg, mapping = setup
    clock = [0.]
    with NativeControlHarness(cfg, [-.14, .14]) as h:
        supervisor = teleop(h, cfg, backend, mapping, clock)
        supervisor.begin(leader_pair(), engaged=True)
        assert [j["position"] for j in h.last["status"]["joints"]] == [-.14, .14]
        accepted = 0
        for i in range(1, 20):
            clock[0] = i * .04
            h.step(i * 40)
            try:
                result = supervisor.step(leader_pair(clock[0], i + 1, i * .05, -i * .05), engaged=True)
                assert result["accepted"] is True
                accepted += 1
            except ValueError as exc:
                assert "insufficient_swept_clearance" in str(exc)
                break
        assert 0 < accepted < 19
        assert supervisor.state == "stopped"
        assert h.last["status"]["state"] == "stopping"
        h.step(h.now_ms + 1)
        assert h.last["status"]["state"] == "stopped"
        assert not backend.proximity(dict(left=h.last["feedback"][0]["position"],
                                         right=h.last["feedback"][1]["position"]))["near_pairs"]
        with pytest.raises(RuntimeError, match="recovery"):
            supervisor.begin(leader_pair(clock[0]), engaged=True)


@pytest.mark.parametrize("failure", ["missing", "release", "slow_collision", "snapshot", "lost_ack", "producer_exit"])
def test_teleop_faults_latch_and_do_not_replay_commands(setup, failure):
    backend, cfg, mapping = setup
    clock = [0.]
    with NativeControlHarness(cfg, [-1., 1.]) as h:
        supervisor = teleop(h, cfg, backend, mapping, clock)
        supervisor.begin(leader_pair(), engaged=True)
        clock[0] = .04
        h.step(40)
        inputs = leader_pair(.04, 2, .01, .01)
        if failure == "producer_exit":
            result = h.step(60)
            assert result["status"]["reason"] == "collision_expired"
            assert result["status"]["state"] == "stopped"
            return
        if failure == "missing":
            inputs.pop("left")
        elif failure == "slow_collision":
            original = backend.proximity
            def slow(*args, **kwargs):
                clock[0] += .04
                return original(*args, **kwargs)
            backend.proximity = slow
        elif failure == "snapshot":
            old = deepcopy(h.last)
            supervisor.snapshot = lambda: old
        elif failure == "lost_ack":
            original = supervisor.controller.exchange
            def drop(request):
                result = original(request)
                if request["type"] == "move":
                    raise TimeoutError("lost acknowledgement")
                return result
            supervisor.controller.exchange = drop
        with pytest.raises((ValueError, TimeoutError)):
            supervisor.step(inputs, engaged=failure != "release")
        assert supervisor.state == "stopped"
        assert h.last["status"]["state"] == "stopping"
        h.step(h.now_ms + 1)
        assert h.last["status"]["state"] == "stopped"
        assert supervisor.last_cycle["accepted"] is (None if failure == "lost_ack" else False)
        with pytest.raises(RuntimeError, match="registration"):
            supervisor.step(leader_pair(.08, 3), engaged=True)


def test_collision_profile_receiver_rejects_unmodelled_feedforward_even_with_envelope(setup):
    backend, cfg, mapping = setup
    compiler = CollisionEnvelopeCompiler(backend, cfg, mapping)
    with NativeControlHarness(cfg, [-1., 1.]) as h:
        control = client(h, cfg)
        control.acquire()
        offered = control.collision_offer()
        control.arm(collision=compiler.compile(offered, h.last["feedback"]))
        offered = control.collision_offer()
        envelope = compiler.compile(offered, h.last["feedback"], commands())
        for field in ("velocity", "effort"):
            target = commands()
            target[0][field] = .01
            with pytest.raises(GuardRejected, match="invalid_command"):
                control.move(target, collision=envelope)


def test_distant_joint_motion_does_not_inflate_stationary_pair_reserve(tmp_path):
    from robot_platform.control.collision_guard import check_linear_motion
    urdf = tmp_path / "three.urdf"
    parts = ['<robot name="fixture"><link name="root"/>']
    for name in ("a", "b", "far"):
        parts.append(f'''<link name="{name}"><collision><geometry><box size=".2 .2 .2"/>
        </geometry></collision></link><joint name="{name}" type="prismatic">
        <parent link="root"/><child link="{name}"/><axis xyz="1 0 0"/>
        <limit lower="-200" upper="200"/></joint>''')
    urdf.write_text("".join(parts) + "</robot>")
    profile = ModelProfile.from_urdf(urdf)
    artifact = tmp_path / "runtime.json"
    build_runtime_model(profile, artifact)
    backend = NativeCollisionBackend(RuntimeCollisionModel(profile, artifact), geometry_mode="keepout")
    try:
        a = dict(a=-1., b=1., far=100.)
        b = dict(a=-1., b=1., far=140.)
        assert check_linear_motion(backend, a, b, subdivide=False)["clear"]
        # The closest pair (a,b) is stationary and clear, but far can cross b.
        # A nearest-pair-only query would miss this swept collision.
        a["far"], b["far"] = -50., 55.
        assert not check_linear_motion(backend, a, b, subdivide=False)["clear"]
    finally:
        backend.close()
