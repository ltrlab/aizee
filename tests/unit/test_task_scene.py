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
    # This fixture injects a substep-only narrow-phase failure. Make broadphase
    # bounds conservatively huge so it does not cull the deliberately fake contact.
    previous_bounds = scene.model.geom_aabb[scene._environment_ids].copy()
    scene.model.geom_aabb[scene._environment_ids, 3:] = 10.
    def transient(model, data, *args):
        if .009 <= data.time <= .011:
            return -.003
        return original(model, data, *args)
    monkeypatch.setattr(mujoco, "mj_geomDistance", transient)
    target = q.copy(); target[0] += .001
    try:
        state = scene.update(target, .025)
    finally:
        scene.model.geom_aabb[scene._environment_ids] = previous_bounds
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


@pytest.mark.parametrize("late_invalid", [False, True])
def test_view_callback_does_not_wait_on_renderer_and_stale_readiness_expires(tmp_path, monkeypatch, late_invalid):
    import threading
    import time
    from types import SimpleNamespace
    from robot_platform.simulation import task_scene
    from robot_platform.simulation.minerva_sim import SCENE
    entered, release, invalid = threading.Event(), threading.Event(), threading.Event()
    class BlockedScene:
        def __init__(self, *args, **kwargs):
            self.names = NAMES; self.capabilities = {}; self.run_id = "blocked-fixture"
            self.data = SimpleNamespace(time=0.)
            self.model = object(); self._q = np.zeros(14); self.rendered = False
            self.mj = SimpleNamespace(mj_forward=lambda *args: None, MjData=lambda m: SimpleNamespace(time=0.), mj_copyData=lambda dest, m, src: setattr(dest, "time", src.time))
        def _pose(self, q): pass
        def snapshot(self): return {"scene_time": self.data.time, "training_eligible": not invalid.is_set(),
                                   "forbidden_contacts": [["robot", "table"]] if invalid.is_set() else []}
        def update(self, q, dt): self._q = np.array(q); self.data.time += dt
        def render(self, camera_roles, **kwargs):
            if self.rendered:
                entered.set(); assert release.wait(3)
            self.rendered = True
            return {n: np.zeros((8, 8, 3), np.uint8) for n in camera_roles}, {n: self.data.time for n in camera_roles}
        def close(self): pass
    class FixtureRenderer:
        def __init__(self, scene, roles, **kwargs):
            self.scene, self.roles = scene, roles
            self.ready = threading.Event(); self.ready.set(); self.error = None
            self.packet = None; self.thread = None
            scene.render(roles)
        def submit(self, packet):
            def render():
                images, stamps = self.scene.render(self.roles)
                packet.update(images=images, camera_times=stamps)
                self.packet = packet
            self.thread = threading.Thread(target=render); self.thread.start()
        def result(self):
            packet, self.packet = self.packet, None
            return packet
        def close(self):
            if self.thread: self.thread.join(3)
    monkeypatch.setattr(task_scene, "_SceneRenderer", FixtureRenderer)
    monkeypatch.setattr(task_scene, "MinervaTaskScene", BlockedScene)
    view = task_scene._ThreadTaskSceneView(SCENE, {}, NAMES, directory=tmp_path, viewer=False, rate_hz=100., record_hz=20.)
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
        if late_invalid:
            # The pending camera snapshot predates this invalid physical outcome.
            # An explicit stop must not publish that old frame as a valid episode.
            invalid.set()
            view.command("stop_record")
            release.set()
            until(lambda: "task became invalid" in (view.health()["error"] or ""))
        else:
            until(lambda: not view.health()["capture_ready"])
    finally:
        release.set()
        view.close()
    assert not list(tmp_path.glob("task_episode_*.hdf5")) or all("partial" in p.name for p in tmp_path.glob("task_episode_*.hdf5"))


def test_proxy_visuals_keep_identical_contacts_and_remove_unused_gpu_meshes(scene):
    cad = MinervaTaskScene(fixture_map(), NAMES, visual_mode="cad", width=160, height=120)
    try:
        assert cad.capabilities["contact_geometry_sha256"] == scene.capabilities["contact_geometry_sha256"]
        assert cad.capabilities["collision_geometries"] == scene.capabilities["collision_geometries"]
        assert scene.capabilities["mesh_faces"] < cad.capabilities["mesh_faces"]
        assert scene.capabilities["visual_mode"] == "collision_proxy"
        assert cad.capabilities["visual_mode"] == "cad"
    finally:
        cad.close()


def test_transient_invalid_receiver_command_latches_before_latest_sample_can_replace_it():
    import threading
    from robot_platform.simulation.task_scene_process import TaskSceneView
    view = object.__new__(TaskSceneView)
    view._lock = threading.Lock(); view._wake = threading.Event()
    view._failure = None; view._closing = False; view._record_intent = True
    view._state = {"last_command": {"action": "start_record", "state": "pending"},
                   "recording": {"active": False}}
    view.show(np.zeros(14), np.zeros(14), 100., command_valid=False)
    reason = view._failure
    assert "valid armed commands" in reason
    view.show(np.zeros(14), np.zeros(14), 100.01, command_valid=True)
    assert view._failure == reason


def test_rotated_world_aabb_contains_every_transformed_local_corner():
    from types import SimpleNamespace
    from robot_platform.simulation.task_scene import _world_aabbs
    angle=.73
    rotation=np.array([[np.cos(angle),-np.sin(angle),0.],[np.sin(angle),np.cos(angle),0.],[0.,0.,1.]])
    local=np.array([[.2,-.3,.4,.11,.22,.33]])
    model=SimpleNamespace(geom_aabb=local)
    data=SimpleNamespace(geom_xmat=rotation.reshape(1,9),geom_xpos=np.array([[1.,2.,3.]]))
    center,half=_world_aabbs(model,data,[0])
    import itertools
    for signs in itertools.product((-1,1),repeat=3):
        point=data.geom_xpos[0]+rotation@(local[0,:3]+np.array(signs)*local[0,3:])
        assert np.all(np.abs(point-center[0]) <= half[0]+1e-12)


def test_aabb_broadphase_never_culls_actual_penetration_at_fixture_poses(scene):
    from robot_platform.simulation.task_scene import _world_aabbs
    poses=[np.array(point["q"]) for point in FIXTURE["waypoints"]]
    invalid=poses[0].copy();invalid[1]-=.12;poses.append(invalid)
    found=0
    for q in poses:
        scene._pose(q);mujoco.mj_forward(scene.model,scene.data)
        centers,halves=_world_aabbs(scene.model,scene.data,scene._collision_ids)
        env_centers,env_halves=_world_aabbs(scene.model,scene.data,scene._environment_ids)
        overlap=np.all(np.abs(centers[:,None]-env_centers[None,:]) <= halves[:,None]+env_halves[None,:]+1e-12,axis=2)
        for r,robot in enumerate(scene._collision_ids):
            for e,environment in enumerate(scene._environment_ids):
                if mujoco.mj_geomDistance(scene.model,scene.data,robot,environment,.002,None)<-.002:
                    assert overlap[r,e]
                    found+=1
    assert found>0


def test_renderer_deadline_is_independent_of_status_heartbeat():
    import queue,time
    from robot_platform.simulation.task_render_process import SceneRenderer
    renderer=object.__new__(SceneRenderer)
    renderer.error=None;renderer._results=queue.Queue(maxsize=1)
    renderer._submitted_at=time.monotonic()-.251
    with pytest.raises(RuntimeError,match="250 ms"):
        renderer.result()


@pytest.mark.parametrize("completed_late", [False, True])
def test_renderer_validates_completion_deadline_even_if_result_is_already_ready(completed_late):
    import queue,time
    from robot_platform.simulation.task_render_process import SceneRenderer
    renderer=object.__new__(SceneRenderer)
    renderer.error=None;renderer._results=queue.Queue(maxsize=1)
    renderer._submitted_at=time.monotonic()-1.
    packet={"preview":True}
    renderer._results.put((packet,renderer._submitted_at+(.3 if completed_late else .2)))
    if completed_late:
        with pytest.raises(RuntimeError,match="250 ms"):renderer.result()
    else:
        assert renderer.result()==packet  # Timely completion, late polling is allowed.
