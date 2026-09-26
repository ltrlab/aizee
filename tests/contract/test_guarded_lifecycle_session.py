"""Existing GUI lifecycle through the actual Rust receiver (native harness).

The collision compiler is replaced by a whole-interval verdict so the receiver's
Idle, Enable, Engage, Shutdown and Disable handling is exercised on its own.
"""
import pytest

from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.teleop.guarded_lifecycle import GuardedLifecycleSession, minerva_action
from robot_platform.teleop.lifecycle import TeleopLifecycle


pytestmark = [pytest.mark.integration, pytest.mark.skipif(
    not default_binary().is_file(), reason="Build control_harness with scripts/test_control_core.ps1 or cargo build -p robot_control")]

NAMES = ("left_joint", "right_joint")


def config():
    return {"group": "fixture", "lease_ms": 150, "feedback_timeout_ms": 30, "settled_velocity": .001,
            "identity": {"model": "m", "calibration": "c", "collision_policy": "p"},
            "collision": {"world_revision": "fixture-world", "scope": "inter_branch",
                          "max_verdict_ms": 80, "reaction_ms": 10},
            "joints": [{"name": name, "unit": "radians", "lower": -3., "upper": 3.,
                        "max_velocity": .5, "max_effort": 5., "max_kp": 20., "max_kd": 5.,
                        "stop_acceleration": 2., "hold_kp": 10., "hold_kd": 1.,
                        "max_tracking_error": .05, "max_temperature": 70.} for name in NAMES]}


class WholeIntervalCompiler:
    """Certifies the whole joint interval; the receiver still checks reserves and offers."""
    compute_budget_s = .02

    def __init__(self, cfg):
        self.config, self.names = cfg, NAMES
        self.last_verdict = None

    def compile(self, offered, feedback, joints=None, *, started_at=None):
        self.last_verdict = {"clear": True}
        return {"offer": offered["offer"]["id"], "world_revision": "fixture-world", "scope": "inter_branch",
                "lower": [j["lower"] for j in self.config["joints"]],
                "upper": [j["upper"] for j in self.config["joints"]]}


class Rig:
    def __init__(self, harness, cfg, *, misses=2):
        self.now = 0.
        self.harness = harness
        self.sequence = 0
        clock = lambda: self.now
        controller = GuardedControllerClient(lambda request: harness.step(round(self.now * 1000), request)["reply"],
                                             group=cfg["group"], identity=cfg["identity"], joint_names=NAMES)
        lifecycle = TeleopLifecycle([{**{k: j[k] for k in ("name", "lower", "upper", "hold_kp", "hold_kd", "max_velocity")},
                                      "align_speed": .5, "shutdown_speed": .25, "max_lead": .05, "tolerance": .01,
                                      "settled_velocity": .05, "zero_position": 0.} for j in cfg["joints"]],
                                    max_source_misses=misses, clock=clock)
        self.session = GuardedLifecycleSession(controller, WholeIntervalCompiler(cfg), lifecycle,
                                               lambda: harness.last, clock=clock)

    def target(self, positions):
        self.sequence += 1
        return [dict(name=n, position=q, sampled_at=self.now - .005, sequence=self.sequence)
                for n, q in zip(NAMES, positions)]

    def cycle(self, action=None, positions=None):
        record = self.session.cycle(action=action, target=None if positions is None else self.target(positions))
        self.now += .04
        return record

    @property
    def receiver(self):
        return self.harness.last["status"]["state"]


def test_existing_gui_lifecycle_runs_through_the_actual_receiver():
    cfg = config()
    with NativeControlHarness(cfg, [.2, -.1]) as harness:
        rig = Rig(harness, cfg)
        rig.cycle("idle")
        assert rig.session.lifecycle.state == "IDLE" and rig.receiver == "idle"
        assert harness.last["hardware_enabled"]
        assert all(j["kp"] == j["kd"] == 0 for j in harness.last["status"]["joints"])
        rig.cycle()
        assert rig.receiver == "idle"
        rig.cycle("enable")
        assert rig.session.lifecycle.state == "HOLD" and rig.receiver == "holding"
        rig.cycle(positions=(1., 1.))  # Leaders cannot move a Hold.
        assert [f["position"] for f in harness.last["feedback"]] == pytest.approx([.2, -.1])
        rig.cycle("engage", positions=(.3, 0.))
        for _ in range(40):
            rig.cycle(positions=(.3, 0.))
            if rig.session.lifecycle.state == "TELEOP":
                break
        assert rig.session.lifecycle.state == "TELEOP" and rig.receiver == "executing"
        assert [f["position"] for f in harness.last["feedback"]] == pytest.approx([.3, 0.])
        # Harness feedback is read before each dispatch, so it shows one cycle later.
        rig.cycle(positions=(.32, .01))
        rig.cycle(positions=(.32, .01))
        assert [f["position"] for f in harness.last["feedback"]] == pytest.approx([.32, .01])
        rig.cycle("disengage")
        assert rig.session.lifecycle.state == "HOLD"
        rig.cycle("shutdown")
        for _ in range(100):
            rig.cycle()
            if rig.session.lifecycle.state == "DISABLED":
                break
        assert rig.session.lifecycle.state == "DISABLED" and rig.receiver == "disarmed"
        assert not harness.last["hardware_enabled"] and harness.last["status"]["lease"] is None
        assert [f["position"] for f in harness.last["feedback"]] == pytest.approx([0., 0.], abs=.011)
        assert not rig.session.owned
        rig.cycle("idle")  # A new explicit session can start again.
        assert rig.receiver == "idle"


def test_leader_loss_while_engaged_holds_then_needs_an_explicit_engage():
    cfg = config()
    with NativeControlHarness(cfg, [0., 0.]) as harness:
        rig = Rig(harness, cfg)
        rig.cycle("idle"); rig.cycle("enable"); rig.cycle("engage", positions=(0., 0.))
        for _ in range(10):
            rig.cycle(positions=(0., 0.))
        assert rig.session.lifecycle.state == "TELEOP"
        for _ in range(3):
            rig.cycle()  # No fresh mapped target.
        assert rig.session.lifecycle.state == "HOLD"
        assert "Leader input lost" in rig.session.lifecycle.reason
        assert rig.receiver == "executing" and harness.last["hardware_enabled"]
        rig.cycle(positions=(.2, .2))  # Returning input does not re-engage.
        assert rig.session.lifecycle.state == "HOLD"
        assert [f["position"] for f in harness.last["feedback"]] == pytest.approx([0., 0.])
        rig.cycle(minerva_action("T", rig.session.lifecycle.state), positions=(.2, .2))
        assert rig.session.lifecycle.state == "ENGAGING"


@pytest.mark.parametrize("state", ["IDLE", "HOLD", "TELEOP"])
def test_disable_removes_torque_immediately_from_any_phase(state):
    cfg = config()
    with NativeControlHarness(cfg, [0., 0.]) as harness:
        rig = Rig(harness, cfg)
        rig.cycle("idle")
        if state != "IDLE":
            rig.cycle("enable")
        if state == "TELEOP":
            rig.cycle("engage", positions=(0., 0.))
            for _ in range(10):
                rig.cycle(positions=(0., 0.))
        assert rig.session.lifecycle.state == state
        record = rig.cycle("disable")
        assert record["accepted"] and rig.session.lifecycle.state == "DISABLED"
        assert rig.receiver == "disarmed" and not harness.last["hardware_enabled"]
        assert not rig.session.owned
