"""Contact manipulation scene for the existing ArmPlant-driven Minerva.

The robot is a set of FK-posed mocap bodies. Only the cube is dynamically
integrated: robot masses, grip force and contact parameters are NOT calibrated.
No attachments, constraints or scripted success motions are used.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np

from .minerva_sim import ModelPose, SCENE

CAMERAS = ("left_wrist", "right_wrist", "head")
JAW_BODIES = {"right": ("carriage_lwl7b", "carriage_lwl7b_2"),
              "left": ("carriage_lwl7b_3", "carriage_lwl7b_4")}
WRISTS = {"right": "1_1_02_eb571_960_stp", "left": "1_1_02_eb571_960_stp_2"}
TABLE_TOP = .28
CUBE_HALF = .018
CUBE_START = (.18, -.34, TABLE_TOP + CUBE_HALF + .002)
BUCKET_CENTER = (-.20, -.40)
BUCKET_HALF = .085
BUCKET_HEIGHT = .08
PHYSICS_DT = .0005
MAX_SOURCE_GAP_S = .25


def _world_aabbs(model, data, ids):
    """Conservative world bounds of compiled local geom bounds, including rotation."""
    local = model.geom_aabb[ids]
    rotation = data.geom_xmat[ids].reshape(-1, 3, 3)
    centers = data.geom_xpos[ids] + np.einsum("nij,nj->ni", rotation, local[:, :3])
    half_sizes = np.einsum("nij,nj->ni", np.abs(rotation), local[:, 3:])
    return centers, half_sizes


def _numbers(values):
    return " ".join(str(float(v)) for v in values)


def _scene_xml(source, fk_model, fk_data, visual_mode="cad"):
    """Flatten CAD bodies into mocap bodies, preserving local visual/collision geometry.

    The source's placeholder inertias and joints are deliberately not copied. All
    CAD collision geoms collide with the cube. Robot/environment penetration is
    measured separately because two kinematic bodies cannot react to each other.
    """
    import mujoco
    mujoco.mj_kinematics(fk_model, fk_data)
    source = Path(source).resolve()
    original = ET.parse(source).getroot()
    # scene.xml is a presentation wrapper; its single include holds the robot.
    if original.find("include") is not None:
        source = source.parent / original.find("include").get("file")
        original = ET.parse(source).getroot()
    root = ET.Element("mujoco", model="minerva_cube_handoff_v1")
    ET.SubElement(root, "compiler", angle="radian", meshdir=str((source.parent / "..").resolve()))
    ET.SubElement(root, "option", timestep=str(PHYSICS_DT), gravity="0 0 -9.81", integrator="implicitfast")
    ET.SubElement(root, "size", njmax="4000", nconmax="1000")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", ambient=".35 .35 .35", diffuse=".7 .7 .7")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960", azimuth="90", elevation="-20")
    ET.SubElement(root, "statistic", center="0 -.15 .65", extent="1.3")
    root.append(copy.deepcopy(original.find("asset")))
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 2.5", dir="0 .3 -1", directional="true")
    ET.SubElement(world, "geom", name="task/floor", type="plane", size="2 2 .05", rgba=".17 .21 .24 1",
                  contype="1", conaffinity="3")
    environment = []
    def box(name, pos, size, rgba):
        environment.append(name)
        ET.SubElement(world, "geom", name=name, type="box", pos=_numbers(pos), size=_numbers(size),
                      rgba=rgba, contype="1", conaffinity="3", friction="1 .005 .0001")
    box("task/table", (0, -.38, TABLE_TOP - .025), (.48, .26, .025), ".46 .34 .23 1")
    for x in (-.42, .42):
        for y in (-.60, -.16):
            box(f"task/leg/{x}/{y}", (x, y, (TABLE_TOP-.05)/2), (.022, .022, (TABLE_TOP-.05)/2), ".3 .3 .32 1")
    bx, by = BUCKET_CENTER
    box("task/bucket/bottom", (bx, by, TABLE_TOP+.007), (BUCKET_HALF, BUCKET_HALF, .007), ".10 .35 .64 1")
    for axis in (0, 1):
        for sign in (-1, 1):
            pos = [bx, by, TABLE_TOP + BUCKET_HEIGHT/2]
            pos[axis] += sign * (BUCKET_HALF + .006)
            size = [BUCKET_HALF + .012, BUCKET_HALF + .012, BUCKET_HEIGHT/2]
            size[axis] = .006
            box(f"task/bucket/wall/{axis}/{sign}", pos, size, ".12 .42 .74 1")
    cube = ET.SubElement(world, "body", name="task/cube", pos=_numbers(CUBE_START))
    ET.SubElement(cube, "freejoint", name="task/cube/free")
    ET.SubElement(cube, "geom", name="task/cube/geom", type="box", size=_numbers([CUBE_HALF]*3),
                  mass=".035", rgba=".95 .30 .06 1", contype="1", conaffinity="3",
                  friction="1 .005 .0001", condim="4", solref=".004 1", solimp=".95 .99 .001")
    bodies, collision_names = [], []
    for body in original.find("worldbody").iter("body"):
        name = body.get("name")
        if not body.findall("geom"):
            continue
        flat = ET.SubElement(world, "body", name=name, mocap="true")
        bodies.append(name)
        for geom in body.findall("geom"):
            item = copy.deepcopy(geom)
            collides = "/collision/" in item.get("name", "")
            if visual_mode == "collision_proxy" and not collides:
                continue
            item.set("contype", "2" if collides else "0")
            item.set("conaffinity", "1" if collides else "0")
            if collides and name in {n for pair in JAW_BODIES.values() for n in pair} and \
                    Path(item.get("mesh", "")).name.startswith(("part_40", "part_41")):
                # A convex hull of the concave one-piece finger fills the open jaw.
                # Fit only the straight terminal 30 mm pad from the actual CAD vertices.
                gid = mujoco.mj_name2id(fk_model, mujoco.mjtObj.mjOBJ_GEOM, item.get("name"))
                bid = int(fk_model.geom_bodyid[gid])
                mid = int(fk_model.geom_dataid[gid])
                start, count = int(fk_model.mesh_vertadr[mid]), int(fk_model.mesh_vertnum[mid])
                vertices = (fk_model.mesh_vert[start:start+count] @ fk_data.geom_xmat[gid].reshape(3, 3).T
                            + fk_data.geom_xpos[gid])
                tip = vertices[vertices[:, 2] < vertices[:, 2].min() + .030001]
                lo, hi = tip.min(axis=0), tip.max(axis=0)
                rotation = fk_data.xmat[bid].reshape(3, 3)
                center = rotation.T @ ((lo+hi)/2 - fk_data.xpos[bid])
                quat = np.empty(4)
                mujoco.mju_mat2Quat(quat, rotation.T.reshape(9))
                item.attrib.pop("mesh")
                item.set("name", item.get("name") + "/terminal_pad")
                item.set("type", "box")
                item.set("pos", _numbers(center))
                item.set("quat", _numbers(quat))
                item.set("size", _numbers((hi-lo)/2))
            if collides:
                collision_names.append(item.get("name"))
                item.set("friction", "1 .005 .0001")
                item.set("condim", "4")
                item.set("solref", ".004 1")
            flat.append(item)
        for side, wrist in WRISTS.items():
            if name == wrist:
                # The wrist camera's local optical direction follows the tool.
                ET.SubElement(flat, "camera", name=f"{side}_wrist", pos="0 .035 .035",
                              xyaxes="1 0 0 0 .606 .795", fovy="85")
    # Head overview is fixed to the unactuated torso, aimed at the workspace.
    ET.SubElement(world, "camera", name="head", pos="0 -.18 1.10",
                  xyaxes="1 0 0 0 .975 -.222", fovy="75")
    # Hidden CAD meshes still allocate/upload GPU assets in MuJoCo. Remove them
    # from the explicit proxy model; the full CAD mode retains every visual.
    used_meshes = {g.get("mesh") for g in world.iter("geom") if g.get("mesh")}
    asset = root.find("asset")
    for mesh in list(asset.findall("mesh")):
        if mesh.get("name") not in used_meshes:
            asset.remove(mesh)
    return ET.tostring(root, encoding="unicode"), bodies, collision_names, environment


class MinervaTaskScene:
    """Single-thread-owned MuJoCo scene. q uses receiver motor coordinates/names."""

    def __init__(self, joint_map, names, *, scene=SCENE, seed=1, width=320, height=240, visual_mode="collision_proxy"):
        import mujoco
        if tuple(int(x) for x in mujoco.__version__.split(".")[:2]) < (3, 15):
            raise RuntimeError("Minerva task contacts require MuJoCo >=3.15 with native geom_surfacevel support")
        if visual_mode not in {"collision_proxy", "cad"}:
            raise ValueError("visual_mode must be collision_proxy or cad")
        self.source = Path(scene).resolve()
        self.visual_mode = visual_mode
        self.mj = mujoco
        self.names = tuple(names)
        if len(set(self.names)) != len(self.names) or not self.names:
            raise ValueError("joint names must be nonempty and unique")
        self.width, self.height = int(width), int(height)
        if min(self.width, self.height) <= 0 or self.width > 1280 or self.height > 960:
            raise ValueError("camera dimensions exceed scene framebuffer")
        self.fk_model = mujoco.MjModel.from_xml_path(str(scene))
        self.fk_data = mujoco.MjData(self.fk_model)
        self.pose = ModelPose(self.fk_model, joint_map, self.names)
        xml, bodies, collision_names, environment = _scene_xml(scene, self.fk_model, self.fk_data, visual_mode)
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.model.flg_surfacevel = True
        self.model.vis.quality.offsamples = 0
        self._body_pairs = [(self._id("body", n, self.fk_model),
                             int(self.model.body_mocapid[self._id("body", n)])) for n in bodies]
        self._fk_ids = np.array([p[0] for p in self._body_pairs])
        self._mocap_ids = np.array([p[1] for p in self._body_pairs])
        self.cube_id = self._id("body", "task/cube")
        self.cube_geom = self._id("geom", "task/cube/geom")
        self.cube_qadr = int(self.model.jnt_qposadr[self._id("joint", "task/cube/free")])
        self._collision_ids = [self._id("geom", n) for n in collision_names]
        self._environment_ids = [self._id("geom", n) for n in environment]
        contact_hash = hashlib.sha256()
        for gid, name in zip(self._collision_ids, collision_names):
            contact_hash.update(name.encode())
            for value in (self.model.geom_type[gid:gid+1], self.model.geom_pos[gid],
                          self.model.geom_quat[gid], self.model.geom_size[gid]):
                contact_hash.update(value.tobytes())
            if int(self.model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_MESH):
                mid = int(self.model.geom_dataid[gid])
                first, count = int(self.model.mesh_vertadr[mid]), int(self.model.mesh_vertnum[mid])
                contact_hash.update(self.model.mesh_vert[first:first+count].tobytes())
                first, count = int(self.model.mesh_faceadr[mid]), int(self.model.mesh_facenum[mid])
                contact_hash.update(self.model.mesh_face[first:first+count].tobytes())

        self._audit_radii = (self.model.geom_rbound[self._collision_ids, None] +
                             self.model.geom_rbound[None, self._environment_ids])
        self._jaw_ids = {self._id("body", n): (side, jaw) for side, ns in JAW_BODIES.items()
                         for jaw, n in enumerate(ns)}
        self._renderer = None
        self._q = None
        self.seed = int(seed)
        self.capabilities = {
            "schema": 1, "data_source": "simulation", "evidence": "synthetic",
            "joint_names": list(self.names), "motor_to_model_joint_map": copy.deepcopy(joint_map),
            "robot_motion": "ArmPlant motor response; FK-posed MuJoCo mocap bodies",
            "object_physics": "MuJoCo free cube with robot, table and bucket contacts",
            "finger_collision": "straight terminal 30mm pad boxes fitted to CAD vertex bounds; concave finger stems omitted",
            "other_robot_collision": "source CAD convex meshes",
            "attachment_assistance": False, "coupled_robot_dynamics": False,
            "robot_self_collision": False, "robot_environment_response": False,
            "robot_environment_penetration_audit": True,
            "collision_geometries": collision_names,
            "contact_geometry_sha256": contact_hash.hexdigest(),
            "mesh_vertices": int(self.model.nmeshvert), "mesh_faces": int(self.model.nmeshface),
            "render_geometry_count": int(sum(self.model.geom_group == (3 if visual_mode == "collision_proxy" else 2))),
            "scene_sha256": hashlib.sha256(xml.encode()).hexdigest(),
            "physics_timestep_s": PHYSICS_DT, "mujoco_version": mujoco.__version__,
            "moving_contact_velocity": "native MuJoCo geom_surfacevel from finite-difference FK poses",
            "task_parameters": {"cube_edge_m": CUBE_HALF*2, "cube_mass_kg": .035,
                                "friction": [1., .005, .0001], "table_top_m": TABLE_TOP,
                                "cube_start_m": CUBE_START, "bucket_center_m": BUCKET_CENTER},
            "contact_parameters_measured": False, "sim2real_validated": False,
            "cameras": list(CAMERAS), "camera_calibration": "synthetic task viewpoints",
            "visual_mode": visual_mode,
            "visual_geometry": "source collision meshes" if visual_mode == "collision_proxy" else "full CAD visuals",
            "render_shadows": False, "render_reflections": False, "offscreen_msaa": 0,
            "limitations": ["Robot motion ignores contact reaction and motor force limits at the fingers",
                            "Finger stems are omitted from contact; terminal pads use CAD-bounds boxes; other meshes are convex",
                            "Real contact properties are unmeasured",
                            "Kinematic robot/environment penetration invalidates episode eligibility"],
        }
        self.reset()

    def _id(self, kind, name, model=None):
        value = self.mj.mj_name2id(model or self.model, getattr(self.mj.mjtObj, f"mjOBJ_{kind.upper()}"), name)
        if value < 0:
            raise ValueError(f"scene lacks {kind} {name}")
        return value

    def _pose(self, q):
        self.pose.apply(self.fk_data, q)
        self.data.mocap_pos[self._mocap_ids] = self.fk_data.xpos[self._fk_ids]
        self.data.mocap_quat[self._mocap_ids] = self.fk_data.xquat[self._fk_ids]

    def reset(self, seed=None):
        if seed is not None:
            self.seed = int(seed)
        self.mj.mj_resetData(self.model, self.data)
        self.model.geom_surfacevel[:] = 0.
        self.run_id = uuid.uuid4().hex
        self.events = []
        self.first_gripper = None
        self.handoff = False
        self._overlap = False
        self._grasp_time = {"left": 0., "right": 0.}
        self._bucket_time = 0.
        self.bucket_settled = False
        self.success = False
        self.dropped = False
        self._drop_time = 0.
        self._invalid_contacts = {}
        self._audited_poses = None
        self._contacts = {"left": [], "right": []}
        self._contact_names = []
        self._pose(self._q if self._q is not None else np.zeros(len(self.names)))
        self.mj.mj_forward(self.model, self.data)
        self._emit("reset")
        return self.snapshot()

    def _emit(self, kind, **values):
        self.events.append(dict(kind=kind, scene_time=float(self.data.time), **values))

    def update(self, q, dt):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.names),) or not np.all(np.isfinite(q)):
            raise ValueError("q must contain one finite motor coordinate per named joint")
        if not math.isfinite(dt) or dt <= 0 or dt > .1:
            raise ValueError("scene dt must be finite in (0, .1] seconds")
        count = round(dt / PHYSICS_DT)
        if count < 1 or abs(count*PHYSICS_DT - dt) > 1e-8:
            raise ValueError("scene dt must be an integer number of physics steps")
        if self._q is None:
            self._pose(q)
            self.mj.mj_kinematics(self.model, self.data)
        start = self._q if self._q is not None else q
        warning = int(self.data.warning.number.sum())
        for step in range(count):
            previous_pos = self.data.geom_xpos[self._collision_ids].copy()
            previous_rot = self.data.geom_xmat[self._collision_ids].reshape(-1, 3, 3).copy()
            self._pose(start + (q-start) * ((step+1)/count))
            self.mj.mj_kinematics(self.model, self.data)
            rotation = self.data.geom_xmat[self._collision_ids].reshape(-1, 3, 3)
            linear = (self.data.geom_xpos[self._collision_ids] - previous_pos) / PHYSICS_DT
            delta = rotation @ previous_rot.transpose(0, 2, 1)
            angular = np.stack((delta[:, 2, 1]-delta[:, 1, 2], delta[:, 0, 2]-delta[:, 2, 0],
                                delta[:, 1, 0]-delta[:, 0, 1]), axis=1) / (2*PHYSICS_DT)
            # Mocap bodies have no generalized velocity. The native surface velocity
            # contributes moving-boundary friction without attaching the cube or
            # inventing a robot mass. Normal contact still comes from penetration.
            self.model.geom_surfacevel[self._collision_ids, :3] = np.einsum("nji,nj->ni", rotation, linear)
            self.model.geom_surfacevel[self._collision_ids, 3:] = np.einsum("nji,nj->ni", rotation, angular)
            self.mj.mj_step(self.model, self.data)
            if step == 0 or np.any(q != start):
                self._audit_penetration()
            self._track_contacts()
        self._q = q.copy()
        self.mj.mj_forward(self.model, self.data)
        if not np.all(np.isfinite(self.data.qpos)) or int(self.data.warning.number.sum()) > warning:
            raise RuntimeError("MuJoCo numerical warning; episode must be quarantined")
        self._audit_penetration()
        return self.snapshot()

    def _track_contacts(self):
        jaws = {"left": set(), "right": set()}
        touched = []
        force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if self.cube_geom not in (c.geom1, c.geom2):
                continue
            other = int(c.geom2 if c.geom1 == self.cube_geom else c.geom1)
            self.mj.mj_contactForce(self.model, self.data, i, force)
            if force[0] <= .001:
                continue
            touched.append(self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, other))
            jaw = self._jaw_ids.get(int(self.model.geom_bodyid[other]))
            if jaw:
                jaws[jaw[0]].add(jaw[1])
        self._contacts = {side: sorted(value) for side, value in jaws.items()}
        self._contact_names = sorted(set(touched))
        xyz = self.data.xpos[self.cube_id]
        raised = xyz[2] > TABLE_TOP + CUBE_HALF + .025
        grasped = set()
        for side, value in jaws.items():
            self._grasp_time[side] = self._grasp_time[side] + PHYSICS_DT if len(value) == 2 and raised else 0.
            if self._grasp_time[side] >= .05:
                grasped.add(side)
        if self.first_gripper is None and len(grasped) == 1:
            self.first_gripper = next(iter(grasped))
            self._emit("pickup", gripper=self.first_gripper)
        if self.first_gripper and len(grasped) == 2:
            self._overlap = True
        if self.first_gripper and self._overlap and not self.handoff:
            receiver = "right" if self.first_gripper == "left" else "left"
            if receiver in grasped and not jaws[self.first_gripper]:
                self.handoff = True
                self._emit("handoff", giver=self.first_gripper, receiver=receiver)
        returned_to_surface = self.first_gripper and ("task/table" in touched or "task/floor" in touched)
        self._drop_time = self._drop_time + PHYSICS_DT if returned_to_surface else 0.
        if self._drop_time >= .03 and not self.dropped:
            self.dropped = True
            self._overlap = False
            self._emit("dropped_outside_bucket")
        inside = (abs(xyz[0]-BUCKET_CENTER[0]) < BUCKET_HALF-CUBE_HALF and
                  abs(xyz[1]-BUCKET_CENTER[1]) < BUCKET_HALF-CUBE_HALF and
                  TABLE_TOP < xyz[2] < TABLE_TOP + .014 + CUBE_HALF + .008)
        settled = (inside and "task/bucket/bottom" in touched and not any(jaws.values()) and
                   np.linalg.norm(self.data.qvel[:3]) < .04)
        self._bucket_time = self._bucket_time + PHYSICS_DT if settled else 0.
        if self._bucket_time >= .25 and not self.bucket_settled:
            self.bucket_settled = True
            self.success = bool(self.first_gripper and self.handoff and not self.dropped and not self._invalid_contacts)
            self._emit("bucket_settled", task_success=self.success)

    def _audit_penetration(self):
        # MuJoCo does not resolve mocap-vs-world contacts. Rotated world AABBs
        # conservatively cull separated pairs; every overlapping changed substep
        # still uses the same exact signed-distance query and verdict.
        centers, halves = _world_aabbs(self.model, self.data, self._collision_ids)
        env_centers, env_halves = _world_aabbs(self.model, self.data, self._environment_ids)
        overlap = np.all(np.abs(centers[:, None]-env_centers[None, :]) <=
                         halves[:, None]+env_halves[None, :]+1e-12, axis=2)
        # Environment is fixed. Reuse only exactly identical robot geometry poses;
        # every changed interpolated substep still gets the penetration audit.
        poses = np.concatenate((self.data.geom_xpos[self._collision_ids],
                                self.data.geom_xmat[self._collision_ids]), axis=1)
        changed = np.ones(len(poses), dtype=bool) if self._audited_poses is None else np.any(poses != self._audited_poses, axis=1)
        self._audited_poses = poses.copy()
        candidates = np.argwhere(overlap & changed[:, None])
        for r, e in candidates:
            robot, env = self._collision_ids[r], self._environment_ids[e]
            key = (robot, env)
            if key in self._invalid_contacts:
                continue
            distance = self.mj.mj_geomDistance(self.model, self.data, robot, env, .002, None)
            if distance < -.002:
                names = [self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_GEOM, i) for i in key]
                self._invalid_contacts[key] = names
                self.success = False
                self._emit("forbidden_robot_environment_penetration", geoms=names, depth_m=-distance)

    def snapshot(self):
        return {"run_id": self.run_id, "scene_time": float(self.data.time), "seed": self.seed,
                "cube_position": self.data.xpos[self.cube_id].tolist(),
                "cube_quaternion": self.data.xquat[self.cube_id].tolist(),
                "jaw_contacts": self._contacts, "cube_contacts": self._contact_names,
                "first_gripper": self.first_gripper, "handoff": self.handoff,
                "bucket_settled": self.bucket_settled, "success": self.success, "dropped": self.dropped,
                "training_eligible": not bool(self._invalid_contacts),
                "sim2real_validated": False,
                "forbidden_contacts": list(self._invalid_contacts.values()), "events": copy.deepcopy(self.events)}

    @property
    def visual_groups(self):
        return [1, 0, 0, 1, 0, 0] if self.visual_mode == "collision_proxy" else [1, 0, 1, 0, 0, 0]

    def render(self, camera_roles=CAMERAS, *, data=None):
        if isinstance(camera_roles, str) or not camera_roles or len(set(camera_roles)) != len(camera_roles) or \
                not set(camera_roles).issubset(CAMERAS):
            raise ValueError("render camera roles must be a distinct supported nonempty subset")
        if self._renderer is None:
            self._renderer = self.mj.Renderer(self.model, height=self.height, width=self.width)
        data = self.data if data is None else data
        images = {}
        options = self.mj.MjvOption()
        options.geomgroup[:] = self.visual_groups
        for name in camera_roles:
            self._renderer.update_scene(data, camera=name, scene_option=options)
            for flag in (self.mj.mjtRndFlag.mjRND_SHADOW, self.mj.mjtRndFlag.mjRND_REFLECTION, self.mj.mjtRndFlag.mjRND_FOG):
                self._renderer.scene.flags[flag] = False
            images[name] = self._renderer.render().copy()
        return images, {name: float(data.time) for name in camera_roles}

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


from .task_render_process import SceneRenderer as _SceneRenderer


class SourceTimeline:
    """Quantize source intervals without accumulating clock drift; never compress a stall."""

    def __init__(self):
        self.previous = None
        self.remainder = 0.

    def advance(self, stamp):
        if stamp is None or not math.isfinite(stamp):
            raise ValueError("paired source timestamp must be finite")
        if self.previous is None:
            self.previous = float(stamp)
            return 0., 0.
        interval = float(stamp) - self.previous
        if interval <= 0 or interval > MAX_SOURCE_GAP_S + 1e-8:
            raise ValueError(f"source interval {interval:.6f}s outside (0, {MAX_SOURCE_GAP_S}]s; reset required")
        duration = round((interval+self.remainder)/PHYSICS_DT)*PHYSICS_DT
        self.remainder += interval-duration
        self.previous = float(stamp)
        return duration, interval


class _ThreadTaskSceneView:
    """Latest-sample mailbox; all scene, renderer and episode work runs off control thread."""

    def __init__(self, scene, joint_map, names, *, directory, metadata=None, seed=1,
                 viewer=True, rate_hz=30., record_hz=20., width=320, height=240,
                 camera_roles=("right_wrist", "head"), visual_mode="collision_proxy"):
        if rate_hz <= 0 or record_hz <= 0 or record_hz > rate_hz:
            raise ValueError("rates must satisfy 0 < record_hz <= rate_hz")
        if isinstance(camera_roles, str) or not camera_roles or len(set(camera_roles)) != len(camera_roles) or \
                not set(camera_roles).issubset(CAMERAS):
            raise ValueError("capture camera roles must be a distinct supported nonempty subset")
        self.camera_roles = tuple(camera_roles)
        self._args = (joint_map, tuple(names))
        self._kwargs = dict(scene=scene, seed=seed, width=width, height=height, visual_mode=visual_mode)
        self.directory = Path(directory)
        self.metadata = copy.deepcopy(metadata or {})
        self.metadata["camera_roles"] = list(self.camera_roles)
        self.viewer, self.rate_hz, self.record_hz = viewer, rate_hz, record_hz
        self._writer = None
        self._latest = None
        self._source_changed_wall = None
        self._lock = threading.Lock()
        self._commands = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._state = {"available": False, "capabilities": {}, "task": {},
                       "recording": {"active": False}, "error": None, "last_command": None,
                       "cameras": {"ready": False, "timestamps": {}}, "capture_ready": False}
        self._thread = threading.Thread(target=self._run, daemon=True, name="minerva-task-scene")
        self._thread.start()

    def show(self, q, qcmd=None, source_time=None, authority="human", command_valid=True):
        sample = (tuple(q), None if qcmd is None else tuple(qcmd), source_time, str(authority), command_valid is True)
        with self._lock:
            if self._latest is None or source_time != self._latest[2]:
                self._source_changed_wall = time.monotonic()
            self._latest = sample

    def command(self, action, payload=None):
        if action not in {"reset", "start_record", "stop_record"}:
            raise ValueError("unknown task scene command")
        if self._stop.is_set():
            raise RuntimeError("task scene is closed")
        identity = uuid.uuid4().hex
        self._publish(last_command={"id": identity, "action": action, "state": "pending"})
        try:
            self._commands.put_nowait((identity, action, dict(payload or {})))
        except queue.Full as exc:
            self._publish(last_command={"id": identity, "action": action, "state": "failed", "error": "command queue full"})
            raise RuntimeError("task scene command queue is full") from exc
        return {"accepted": True, "id": identity}

    def snapshot(self):
        with self._lock:
            state = self._state.copy()
            changed = self._source_changed_wall
        # UI-sized copies never hold the mailbox lock used by the control callback.
        result = copy.deepcopy(state)
        age = None if changed is None else time.monotonic()-changed
        result["source_age_s"] = age
        result["capture_ready"] = bool(result["capture_ready"] and age is not None and age <= MAX_SOURCE_GAP_S)
        return result

    def health(self):
        """Small control-callback status; no geometry, event, image or disk work."""
        with self._lock:
            age = None if self._source_changed_wall is None else time.monotonic()-self._source_changed_wall
            recording = self._state["recording"]
            return {"available": self._state["available"], "error": self._state["error"],
                    "source_age_s": age,
                    "capture_ready": bool(self._state["capture_ready"] and age is not None and age <= MAX_SOURCE_GAP_S),
                    "recording": {key: recording.get(key) for key in ("active", "error", "state", "complete")}}

    def _publish(self, **values):
        with self._lock:
            self._state.update(values)

    def _run(self):
        scene, writer, renderer = None, None, None
        render_pending, pending_stop = False, None
        next_record = 0.
        last_source = None
        timeline = SourceTimeline()
        try:
            from robot_platform.data.task_capture import TaskEpisodeWriter
            scene = MinervaTaskScene(*self._args, **self._kwargs)
            # Compile/upload render assets before advertising readiness or admitting a recording.
            renderer = _SceneRenderer(scene, self.camera_roles, directory=self.directory, viewer=self.viewer)
            if not renderer.ready.wait(30):
                raise RuntimeError("camera renderer initialization timed out")
            if renderer.error:
                raise RuntimeError(renderer.error)
            self._publish(available=True, capabilities=scene.capabilities, task=scene.snapshot(),
                          renderer={"pid": getattr(getattr(renderer, "_process", None), "pid", None)})
            dt = max(PHYSICS_DT, round(1/self.rate_hz/PHYSICS_DT)*PHYSICS_DT)
            while not self._stop.is_set():
                started = time.monotonic()
                rendered = renderer.result()
                if rendered is not None:
                    render_pending = False
                if rendered is not None and not rendered.get("preview"):
                    if writer is None:
                        raise RuntimeError("rendered frame lost its recording owner")
                    writer.append(**rendered)
                    render_pending = False
                    self._publish(recording=writer.status(), cameras={"ready": True,
                        "timestamps": rendered["camera_times"], "source_time": rendered["source_time"],
                        "clock": "simulation seconds"})
                with self._lock:
                    sample = self._latest
                while True:
                    try:
                        identity, action, payload = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    result = {"id": identity, "action": action, "ok": False}
                    try:
                        if action == "reset":
                            if writer is not None:
                                raise ValueError("finish recording before resetting the scene")
                            scene.reset(payload.get("seed"))
                            timeline = SourceTimeline()
                        elif action == "start_record":
                            if writer is not None:
                                raise ValueError("a recording is already active")
                            if sample is None or sample[1] is None or sample[2] is None or not self.snapshot()["capture_ready"]:
                                raise ValueError("recording requires paired q, qcmd and source timestamp")
                            metadata = dict(self.metadata, data_source="simulation", evidence="synthetic",
                                            run_id=scene.run_id, scene=scene.capabilities,
                                            sample_policy="latest paired motor snapshot; FK interpolation over actual source-time interval",
                                            max_source_gap_s=MAX_SOURCE_GAP_S,
                                            language_instruction=payload.get("language_instruction", "Pick up the cube, pass it to the other gripper, then drop it into the bucket"))
                            writer = TaskEpisodeWriter(self.directory, self._args[1], metadata, hz=self.record_hz)
                            self._writer = writer
                            next_record, last_source = float(scene.data.time), None
                        elif action == "stop_record":
                            if writer is None:
                                raise ValueError("no recording is active")
                            if pending_stop is not None:
                                raise ValueError("recording stop is already pending")
                            pending_stop = result
                            continue
                        result["ok"] = True
                    except Exception as exc:
                        result["error"] = str(exc)
                        if writer is not None:
                            self._publish(recording=writer.status())
                    result["state"] = "complete" if result["ok"] else "failed"
                    self._publish(last_command=result)
                if sample is not None:
                    age = time.monotonic() - self._source_changed_wall
                    ready = sample[1] is not None and sample[2] is not None and sample[4] and age <= MAX_SOURCE_GAP_S
                    self._publish(capture_ready=ready)
                    if writer is not None and not ready:
                        raise RuntimeError("paired motor source stalled, became incomplete, or lost valid armed commands during recording")
                    timing = {"max_source_gap_s": MAX_SOURCE_GAP_S, "source_interval_s": 0., "physics_interval_s": 0.}
                    if sample[2] is not None and sample[2] != timeline.previous:
                        duration, interval = timeline.advance(sample[2])
                        timing.update(source_interval_s=interval, physics_interval_s=duration)
                        target = np.asarray(sample[0], dtype=float)
                        if target.shape != (len(scene.names),) or not np.all(np.isfinite(target)):
                            raise ValueError("source pose must have one finite coordinate per joint")
                        if interval == 0.:
                            scene._q = target.copy()
                            scene._pose(target)
                            scene.mj.mj_forward(scene.model, scene.data)
                        else:
                            initial = scene._q.copy()
                            steps = round(duration/PHYSICS_DT)
                            completed = 0
                            while completed < steps:
                                count = min(200, steps-completed)
                                completed += count
                                here = initial + (target-initial)*(completed/steps)
                                scene.update(here, count*PHYSICS_DT)
                    self._publish(timing=timing)
                    if writer is not None and sample[2] != last_source and scene.data.time >= next_record-1e-8:
                        if sample[2] is None or sample[2] == last_source:
                            raise RuntimeError("motor source stalled while recording")
                        if not render_pending and pending_stop is None:
                            renderer.submit(dict(q=sample[0], qcmd=sample[1], source_time=sample[2],
                                scene_time=float(scene.data.time), task_state=dict(scene.snapshot(), timing=timing, command_valid=sample[4]),
                                authority=sample[3]))
                            render_pending = True
                            while next_record <= scene.data.time + 1e-8:
                                next_record += 1/self.record_hz
                            last_source = sample[2]
                if writer is not None and (scene.snapshot().get("forbidden_contacts") or scene.snapshot().get("dropped")):
                    raise RuntimeError("task became invalid during recording: forbidden contact or dropped cube")
                if pending_stop is not None and not render_pending:
                    writer.finish()
                    self._publish(recording=writer.status(), last_command=dict(pending_stop, ok=True, state="complete"))
                    writer = self._writer = pending_stop = None
                self._publish(task=scene.snapshot())
                if self.viewer and writer is None and not render_pending:
                    renderer.submit(None)  # Bounded exact-state preview; no episode row or pixels copied.
                    render_pending = True
                self._stop.wait(max(0., dt-(time.monotonic()-started)))
        except Exception as exc:
            self._publish(error=f"{type(exc).__name__}: {exc}", available=False, capture_ready=False)
            if writer is not None:
                writer.abort(str(exc))
                self._publish(recording=writer.status())
                writer = None
        finally:
            if writer is not None:
                writer.abort("scene or session closed before explicit stop_record")
                self._publish(recording=writer.status())
            if renderer is not None:
                renderer.close()
            if scene is not None:
                scene.close()
            self._stop.set()
            self._publish(available=False, capture_ready=False)

    def abort(self, reason):
        writer = self._writer
        if writer is not None:
            writer.abort(reason)
            self._publish(recording=writer.status())

    def close(self):
        self.abort("scene or session closed before explicit stop_record")
        self._stop.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            self._publish(error="task scene shutdown still draining; partial episode is not published")


# A subprocess isolates CAD rendering and Python physics/audit work from the control GIL.
from .task_scene_process import TaskSceneView
