"""Actual MuJoCo contacts, not mocked grasp outcomes. No leaders or receiver involved."""
import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_platform.simulation.task_scene import (BUCKET_CENTER, CUBE_HALF, CUBE_START,
                                                TABLE_TOP, MinervaTaskScene)

FIXTURE = json.loads((Path(__file__).parents[1] / "fixtures/minerva_task_contact_path.json").read_text())
NAMES = FIXTURE["joint_names"]


def fixture_map():
    mapping = {n: dict(actuator=n, scale=1., offset=0.) for n in NAMES if "arm" in n}
    for side in ("left", "right"):
        mapping[f"{side}_gripper_finger_a_slide"] = dict(actuator=f"{side}_gripper", scale=.01, offset=0.)
    return mapping


@pytest.fixture(scope="module")
def scene():
    scene = MinervaTaskScene(fixture_map(), NAMES, width=160, height=120)
    yield scene
    scene.close()


def test_scene_has_one_free_object_and_truthful_contact_manifest(scene):
    scene._q = np.zeros(14)
    state = scene.reset()
    assert scene.model.nq == 7 and scene.model.nv == 6
    assert scene.model.neq == 0  # No object attachment or grasp equality.
    assert np.allclose(state["cube_position"], CUBE_START)
    assert not state["success"] and not state["sim2real_validated"]
    assert not scene.capabilities["attachment_assistance"]
    assert not scene.capabilities["coupled_robot_dynamics"]
    assert "terminal" in scene.capabilities["finger_collision"]
    assert len([n for n in scene.capabilities["collision_geometries"] if n.endswith("terminal_pad")]) == 4
    assert scene.model.ncam == 3
    before = state["run_id"]
    for _ in range(10): state = scene.update(np.zeros(14), .05)
    assert state["cube_position"][2] == pytest.approx(TABLE_TOP+CUBE_HALF, abs=.0001)
    assert "task/table" in state["cube_contacts"]
    assert state["first_gripper"] is None
    assert scene.reset()["run_id"] != before


def test_real_contact_pickup_handoff_and_bucket_outcome(scene):
    waypoints = FIXTURE["waypoints"]
    q = np.array(waypoints[0]["q"])
    scene._q = q.copy()
    scene.reset()  # Fixture starts prepositioned; does not claim a collision-free approach from hang.
    for point in waypoints[1:]:
        target = np.array(point["q"])
        for k in range(point["steps"]):
            state = scene.update(q+(target-q)*(k+1)/point["steps"], FIXTURE["step_s"])
        q = target
    assert state["first_gripper"] == "left" and state["handoff"]
    assert state["bucket_settled"] and state["success"] and not state["dropped"]
    assert state["training_eligible"] and not state["forbidden_contacts"]
    assert not state["sim2real_validated"]
    assert [e["kind"] for e in state["events"]] == ["reset", "pickup", "handoff", "bucket_settled"]
    assert "task/bucket/bottom" in state["cube_contacts"]


def test_bucket_contact_alone_does_not_count_as_handoff_success(scene):
    scene._q = np.zeros(14)
    scene.reset()
    # Initial-condition fixture only: no robot grasp occurred.
    scene.data.qpos[scene.cube_qadr:scene.cube_qadr+3] = [*BUCKET_CENTER, TABLE_TOP+.15]
    mujoco.mj_forward(scene.model, scene.data)
    for _ in range(20): state = scene.update(np.zeros(14), .05)
    assert state["bucket_settled"]
    assert state["first_gripper"] is None and not state["handoff"] and not state["success"]


def test_robot_table_penetration_is_reported_and_ineligible(scene):
    q = np.array(FIXTURE["waypoints"][0]["q"])
    q[1] -= .12
    scene._q = q.copy()
    scene.reset()
    state = scene.update(q, .05)
    assert state["forbidden_contacts"] and not state["training_eligible"]
    assert any(e["kind"] == "forbidden_robot_environment_penetration" for e in state["events"])


def test_scene_refuses_unsupported_mujoco_before_contact_fallback(monkeypatch):
    monkeypatch.setattr(mujoco, "__version__", "3.3.0")
    with pytest.raises(RuntimeError, match="requires? MuJoCo >=3.15"):
        MinervaTaskScene(fixture_map(), NAMES)


def test_transient_penetration_is_audited_and_revokes_latched_success(scene, monkeypatch):
    q = np.array(FIXTURE["waypoints"][0]["q"])
    scene._q = q.copy()
    scene.reset()
    scene.success = True  # Previously completed task; a later invalid contact must revoke it.
    original = mujoco.mj_geomDistance
    def transient(model, data, *args):
        if .009 <= data.time <= .011:
            return -.003
        return original(model, data, *args)
    monkeypatch.setattr(mujoco, "mj_geomDistance", transient)
    target = q.copy(); target[0] += .001
    state = scene.update(target, .025)
    invalid = [e for e in state["events"] if e["kind"] == "forbidden_robot_environment_penetration"]
    assert invalid and invalid[0]["scene_time"] < .025
    assert not state["training_eligible"] and not state["success"]


def test_source_intervals_preserve_elapsed_time_and_reject_stalls():
    from robot_platform.simulation.task_scene import SourceTimeline
    clock = SourceTimeline()
    assert clock.advance(100.) == (0., 0.)
    durations = [clock.advance(t)[0] for t in (100.0333, 100.1267, 100.2)]
    assert sum(durations) == pytest.approx(.2, abs=.00025)
    assert durations[1] == pytest.approx(.093)
    with pytest.raises(ValueError, match="outside"):
        clock.advance(100.5)


def test_view_callback_does_not_wait_on_renderer_and_stale_readiness_expires(tmp_path, monkeypatch):
    import threading
    import time
    from types import SimpleNamespace
    from robot_platform.simulation import task_scene
    from robot_platform.simulation.minerva_sim import SCENE
    entered, release = threading.Event(), threading.Event()
    class BlockedScene:
        def __init__(self, *args, **kwargs):
            self.names = NAMES; self.capabilities = {}; self.run_id = "blocked-fixture"
            self.data = SimpleNamespace(time=0.)
            self.model = object(); self._q = np.zeros(14)
            self.mj = SimpleNamespace(mj_forward=lambda *args: None)
        def _pose(self, q): pass
        def snapshot(self): return {"scene_time": self.data.time, "training_eligible": True}
        def update(self, q, dt): self._q = np.array(q); self.data.time += dt
        def render(self, camera_roles):
            entered.set(); assert release.wait(3)
            return {n: np.zeros((8, 8, 3), np.uint8) for n in camera_roles}, {n: self.data.time for n in camera_roles}
        def close(self): pass
    monkeypatch.setattr(task_scene, "MinervaTaskScene", BlockedScene)
    view = task_scene.TaskSceneView(SCENE, {}, NAMES, directory=tmp_path, viewer=False, rate_hz=100., record_hz=20.)
    def until(predicate):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            if predicate(): return
            time.sleep(.005)
        pytest.fail("view did not reach expected state")
    try:
        view.show(np.zeros(14), np.zeros(14), 100.)
        until(lambda: view.health()["capture_ready"])
        view.command("start_record")
        assert entered.wait(2)
        returned = threading.Event()
        def callback():
            view.show(np.ones(14), np.ones(14), 100.05)
            assert view.health()["available"]
            returned.set()
        thread = threading.Thread(target=callback); thread.start()
        assert returned.wait(.2), "renderer held the control mailbox lock"
        thread.join()
        until(lambda: not view.health()["capture_ready"])
    finally:
        release.set()
        view.close()
    assert not list(tmp_path.glob("task_episode_*.hdf5")) or all("partial" in p.name for p in tmp_path.glob("task_episode_*.hdf5"))
