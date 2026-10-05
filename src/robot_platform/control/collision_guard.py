"""Whole-robot motion checks and an opt-in simulation command gate.

Physical enforcement intentionally has no configuration switch yet: joint/motor
calibration, stopping response, proxy/contact review and controller-side expiry
still need evidence. Observation mode can evaluate recorded mechanical states.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import time

import numpy as np


def _compile_motion_bounds(scene, corners=None):
    """Cache immutable radii and ancestry; joint displacements remain per-query."""
    parents = {joint.child: joint for joint in scene._order}
    plan = []
    for index, shape in enumerate(scene._shapes):
        vertices = shape.corners if corners is None else corners[index]
        radius = float(np.linalg.norm(vertices, axis=1).max() + np.linalg.norm(shape.local[:3,3]))
        chain, link = [], shape.link
        while link in parents:
            joint = parents[link]
            chain.append((joint.name, joint.kind, float(np.linalg.norm(joint.origin[:3,3]))))
            link = joint.parent
        plan.append((radius, tuple(chain)))
    return tuple(plan)


def motion_bounds(scene, start, end, *, corners=None, plan=None):
    """Upper bound on each shape point's travel during linear joint interpolation.

    Walk each ancestry backwards. Downstream translation lengths bound rotational
    lever arms; summing joint contributions bounds simultaneous articulated motion.
    Mimic coordinates are expanded by the scene, including signs and offsets.
    """
    a, b = scene._positions(start), scene._positions(end)
    if plan is None:
        plan = _compile_motion_bounds(scene, corners)
    bounds = []
    for radius, chain in plan:
        reach, distance = radius, 0.
        for name, kind, origin_length in chain:
            if kind in {"revolute", "continuous"}:
                distance += reach * abs(b[name]-a[name])
            elif kind == "prismatic":
                distance += abs(b[name]-a[name])
                reach += max(abs(a[name]),abs(b[name]))
            reach += origin_length
        bounds.append(distance)
    return np.asarray(bounds)


def check_linear_motion(backend, start, end, *, clearance_m=0.005,
                        max_segment_travel_m=0.01, max_queries=128,
                        deadline=None, clock=time.monotonic, subdivide=True, scope="all"):
    """Conservative clearance test for one specified linear joint-space path.

    Midpoint distances must exceed the clearance PLUS a bound on both shapes'
    motion to either endpoint. This addresses rotational motion and gaps between
    samples; it is not a prediction of uncalibrated physical servo dynamics.
    """
    if (not math.isfinite(clearance_m) or clearance_m < 0 or not math.isfinite(max_segment_travel_m)
            or max_segment_travel_m <= 0 or type(max_queries) is not int or max_queries < 1):
        raise ValueError("Invalid motion-check resolution/budget")
    runtime = backend.runtime_model
    selected = runtime.pairs_for_scope(scope,geometry_mode=backend.geometry_mode)
    def report(**fields):
        return dict(query_scope=scope, whole_robot_checked=scope=='all', geometry_basis=backend.geometry_basis,
                    collision_policy_revision=backend.policy_revision, ignored_links=dict(backend.ignored_links),
                    full_geometry_coverage=scope=='all' and not backend.ignored_links, **fields)
    if not hasattr(backend, '_motion_bound_plan'):
        backend._motion_bound_plan = _compile_motion_bounds(runtime.scene, getattr(backend,'motion_corners',None))
    travel = motion_bounds(runtime.scene, start, end, plan=backend._motion_bound_plan)
    if not hasattr(backend, '_motion_shape_indices'):
        backend._motion_shape_indices = {shape['id']: index for index, shape in enumerate(runtime.shapes)}
    largest = max((travel[i]+travel[j] for i,j in selected),default=0.)
    # Without subdivision the single midpoint bound covers the entire joint
    # coordinate box, including joints stopping at different times.
    count = max(1,math.ceil(largest/max_segment_travel_m)) if subdivide else 1
    if count > max_queries:
        return report(clear=False, reason="motion_query_budget", queries=0)
    threshold = clearance_m + largest/(2*count) + 2e-6
    lower = math.inf
    for index in range(count):
        if deadline is not None and clock() >= deadline:
            return report(clear=False,reason="collision_deadline",queries=index)
        t = (index+.5)/count
        q = {name:(1-t)*start[name]+t*end[name] for name in start}
        # Request every candidate under the global broad-phase threshold. The
        # nearest pair is not necessarily the pair with the least swept clearance.
        result = backend.proximity(q,threshold=threshold,scope=scope)
        if result.get("whole_robot_checked") is not (scope=='all') or result.get("query_scope") != scope:
            return report(clear=False, reason="incomplete_collision_scope", queries=index+1)
        if (result.get('collision_policy_revision') != backend.policy_revision
                or result.get('geometry_basis') != backend.geometry_basis):
            return report(clear=False, reason="collision_policy_mismatch", queries=index+1)
        if result.get('pairs_truncated', False):
            return report(clear=False, reason="incomplete_collision_pairs", queries=index+1)
        for pair in result["near_pairs"]:
            key = tuple(sorted(pair["shapes"]))
            i, j = (backend._motion_shape_indices[name] for name in key)
            # Use this pair's actual ancestry bound. The global query threshold
            # still protects every unreported pair, including asynchronous motion.
            separation = pair["distance_m"]-(travel[i]+travel[j])/(2*count)-2e-6
            lower = min(lower,separation)
            if separation <= clearance_m:
                return report(clear=False,reason="insufficient_swept_clearance",queries=index+1,
                            shapes=list(key),clearance_lower_bound_m=separation)
    if deadline is not None and clock() >= deadline:
        return report(clear=False,reason="collision_deadline",queries=count)
    return report(clear=True,reason="clear_linear_motion",queries=count,
                clearance_lower_bound_m=lower if math.isfinite(lower) else clearance_m)


class JointBoxBound:
    """Each collision shape's travel over a box of joint positions, from the joints' axes at its centre.

    Bring the joints from the centre to any pose in the box one at a time, from the shape's own link toward
    the root. Joint k then turns about its axis where the centre puts it (the joints above it have not moved
    yet), carrying points no farther from that axis than at the centre plus what the joints below it moved
    them; a turn of h moves such a point at most that distance times h, a slide of h moves it h. Summed
    along the chain, that bounds every point's distance from its centre position anywhere in the box,
    whatever order the joints move in, or none.

    motion_bounds instead takes each joint's lever as the shape's radius about its link frame plus every link
    length below it. On Minerva's keep-out model that radius is about 2.2 m for a gripper part about 0.13 m
    from its link frame (its mesh frame sits at the assembly origin), and a hand turning about its forearm's
    own axis was charged the whole forearm: this bound is about 7x tighter at the hanging pose.
    """

    def __init__(self, scene, vertices):
        self.scene = scene
        # Kinematics over the moving joints only (Minerva's CAD has about 60 fixed ones): every link hangs off
        # its nearest moving joint's child frame (its anchor; None for the root) by a fixed transform.
        anchor = {scene.root_link: (None, np.eye(4))}
        self.joints = []  # Moving joints, parents first: (joint, its parent link's anchor index, fixed transform).
        index = {None: 0}
        for joint in scene._order:
            above, offset = anchor[joint.parent]
            if joint.kind == "fixed":
                anchor[joint.child] = (above, offset @ joint.origin)
                continue
            axis = np.asarray(joint.axis, dtype=float)
            self.joints.append((joint, index[above], offset @ joint.origin, axis / np.linalg.norm(axis)))
            index[joint.name] = len(self.joints)
            anchor[joint.child] = (joint.name, np.eye(4))
        # Every shape's world transform is its anchor's frame times a fixed transform (its local one).
        self.count = len(scene._shapes)
        self.anchors = np.asarray([0 if shape.obstacle else index[anchor[shape.link][0]]
                                   for shape in scene._shapes], dtype=np.intp)
        self.fixed = np.asarray([shape.local if shape.obstacle else anchor[shape.link][1] @ shape.local
                                 for shape in scene._shapes]).reshape(-1, 4, 4)
        # Each shape's moving joints, from its own link toward the root.
        parents = {joint.child: joint for joint in scene._order}
        chains = []
        for shape in scene._shapes:
            chain, link = [], None if shape.obstacle else shape.link
            while link in parents:
                joint = parents[link]
                if joint.kind != "fixed":
                    chain.append(joint)
                link = joint.parent
            chains.append(chain)
        # Shapes in a preorder walk of the moving joints (by anchor): every joint's subtree, the shapes it
        # carries, is then one contiguous run of shapes and of their vertex rows.
        below = {}
        for k, (joint, above, origin, axis) in enumerate(self.joints, 1):
            below.setdefault(above, []).append(k)
        preorder, pending = [], [0]
        while pending:
            a = pending.pop()
            preorder.append(a)
            pending.extend(reversed(below.get(a, [])))
        rank = np.empty(len(self.joints) + 1, dtype=np.intp)
        rank[preorder] = np.arange(len(preorder))
        extent = np.ones(len(self.joints) + 1, dtype=np.intp)  # Anchors in each subtree, itself included.
        for a in reversed(preorder):
            extent[a] += sum(extent[child] for child in below.get(a, []))
        self.moving = np.asarray(sorted((s for s, chain in enumerate(chains) if chain),
                                        key=lambda s: (rank[self.anchors[s]], s)), dtype=np.intp)
        # Hull vertices in their anchor's frame.
        blocks = []
        for s in self.moving:
            block = np.asarray(vertices[s], dtype=float).reshape(-1, 3)
            if not len(block) or not np.isfinite(block).all():
                raise ValueError("Every moving shape needs finite vertices")
            blocks.append(block @ self.fixed[s][:3, :3].T + self.fixed[s][:3, 3])
        sizes = np.asarray([len(block) for block in blocks], dtype=np.intp)
        starts = np.concatenate(([0], np.cumsum(sizes))).astype(np.intp)
        self.vertices = np.concatenate(blocks) if blocks else np.zeros((0, 3))
        # One rotation places each anchor's vertices.
        shape_ranks = rank[self.anchors[self.moving]]
        self.placements = []
        for a in preorder:
            first, last = np.searchsorted(shape_ranks, [rank[a], rank[a] + 1])
            if last > first:
                self.placements.append((a, slice(int(starts[first]), int(starts[last]))))
        # Deepest joints first: a shape's nearest joint is the deepest in its chain. Each joint carries the
        # shapes of its subtree: (joint, its frame, its shapes, their rows, each shape's first row in them).
        depth = {}
        for chain in chains:
            for i, joint in enumerate(chain):
                depth[joint.name] = max(depth.get(joint.name, 0), len(chain) - i)
        self.plan = []
        for name in sorted(depth, key=lambda name: -depth[name]):
            k = index[name]
            first, last = (int(i) for i in np.searchsorted(shape_ranks, [rank[k], rank[k] + extent[k]]))
            self.plan.append((scene._joints[name], k, slice(first, last), slice(int(starts[first]), int(starts[last])),
                              starts[first:last] - starts[first]))

    def _half(self, joint, half):
        if joint.mimic is not None:
            source, multiplier, _ = joint.mimic
            return abs(multiplier) * self._half(self.scene._joints[source], half)
        return abs(float(half.get(joint.name, 0.)))

    def frames(self, center):
        """The world frame of each moving joint's child link (index 0: the root) at `center`."""
        positions = self.scene._positions(center)
        frames = np.empty((len(self.joints) + 1, 4, 4))
        frames[0] = np.eye(4)
        for k, (joint, above, origin, axis) in enumerate(self.joints, 1):
            motion = np.eye(4)
            if joint.kind == "prismatic":
                motion[:3, 3] = axis * positions[joint.name]
            else:
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                angle = positions[joint.name]
                motion[:3, :3] = np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)
            frames[k] = frames[above] @ origin @ motion
        if not np.isfinite(frames).all():
            raise ValueError("Nonfinite forward kinematics")
        return frames

    def shape_transforms(self, frames):
        """Every shape's world transform (the scene's order), as RuntimeCollisionModel.transforms gives it."""
        return frames[self.anchors] @ self.fixed

    def travel(self, center, half, frames=None):
        """Per shape (the scene's order): how far any point of it may be from where `center` puts it, for
        every pose within `half` of `center` in each joint (independent joints; a mimic follows its source)."""
        frames = self.frames(center) if frames is None else frames
        result = np.zeros(self.count)
        if not len(self.moving):
            return result
        points = np.empty_like(self.vertices)
        for a, rows in self.placements:
            points[rows] = self.vertices[rows] @ frames[a][:3, :3].T + frames[a][:3, 3]
        total = np.zeros(len(self.moving))
        for joint, k, shapes, rows, offsets in self.plan:
            h = self._half(joint, half)
            if not math.isfinite(h):
                raise ValueError(f"Nonfinite joint box: {joint.name}")
            if h == 0:
                continue
            if joint.kind == "prismatic":
                total[shapes] += h
                continue
            frame = frames[k]
            direction = frame[:3, :3] @ self.joints[k - 1][3]
            relative = points[rows] - frame[:3, 3]
            along = relative @ direction
            distance = np.sqrt(np.maximum(0., np.einsum('ij,ij->i', relative, relative) - along * along))
            total[shapes] += (np.maximum.reduceat(distance, offsets) + total[shapes]) * h
        result[self.moving] = total
        return result


def check_joint_box(backend, bound, center, half, *, clearance_m=0.005, margin_m=0., scope="all", deadline=None,
                    clock=time.monotonic):
    """Clearance for every pose in a box of joint positions (`center` +/- `half` per joint), from one
    proximity query at its centre: a pair is clear when its distance there exceeds the clearance plus both
    shapes' travel (JointBoxBound). Joints may move or stop at different times anywhere in the box.

    clearance_lower_bound_m is the least such margin over every pair: exact below clearance_m + margin_m (the
    query reaches that far past each pair's travel), at least that otherwise. The pair is in `shapes`."""
    if not math.isfinite(clearance_m) or clearance_m < 0 or not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError("Invalid clearance or margin")
    runtime = backend.runtime_model
    selected = runtime.pairs_for_scope(scope, geometry_mode=backend.geometry_mode)

    def report(**fields):
        return dict(query_scope=scope, whole_robot_checked=scope == 'all', geometry_basis=backend.geometry_basis,
                    collision_policy_revision=backend.policy_revision, ignored_links=dict(backend.ignored_links),
                    full_geometry_coverage=scope == 'all' and not backend.ignored_links, **fields)

    if deadline is not None and clock() >= deadline:
        return report(clear=False, reason="collision_deadline", queries=0)
    frames = bound.frames(center)
    travel = bound.travel(center, half, frames)
    pairs = np.asarray(selected, dtype=np.intp).reshape(-1, 2)
    largest = float((travel[pairs[:, 0]] + travel[pairs[:, 1]]).max()) if len(pairs) else 0.
    # The query places the shapes from the same kinematics (a backend that cannot take them recomputes them).
    shared = dict(worlds=bound.shape_transforms(frames)) if getattr(backend, 'accepts_worlds', False) else {}
    reach = clearance_m + margin_m  # Pairs farther than this past their travel are not reported.
    result = backend.proximity(center, threshold=reach + largest + 2e-6, scope=scope, **shared)
    if result.get("whole_robot_checked") is not (scope == 'all') or result.get("query_scope") != scope:
        return report(clear=False, reason="incomplete_collision_scope", queries=1)
    if (result.get('collision_policy_revision') != backend.policy_revision
            or result.get('geometry_basis') != backend.geometry_basis):
        return report(clear=False, reason="collision_policy_mismatch", queries=1)
    if result.get('pairs_truncated', False):
        return report(clear=False, reason="incomplete_collision_pairs", queries=1)
    if not hasattr(backend, '_motion_shape_indices'):
        backend._motion_shape_indices = {shape['id']: index for index, shape in enumerate(runtime.shapes)}
    lower, nearest = math.inf, None
    for pair in result["near_pairs"]:
        key = tuple(sorted(pair["shapes"]))
        i, j = (backend._motion_shape_indices[name] for name in key)
        separation = pair["distance_m"] - travel[i] - travel[j] - 2e-6
        if separation < lower:
            lower, nearest = separation, (list(key), pair["distance_m"])
    if lower <= clearance_m:
        return report(clear=False, reason="insufficient_swept_clearance", queries=1, shapes=nearest[0],
                      distance_m=nearest[1], clearance_lower_bound_m=lower, travel_m=float(travel.max()))
    if deadline is not None and clock() >= deadline:
        return report(clear=False, reason="collision_deadline", queries=1)
    within = lower < reach
    return report(clear=True, reason="clear_joint_box", queries=1, travel_m=float(travel.max()),
                  shapes=nearest[0] if within else None, distance_m=nearest[1] if within else None,
                  clearance_lower_bound_m=min(lower, reach))


@dataclass(frozen=True)
class JointCalibration:
    actuator: str
    scale: float
    offset: float


class CollisionCommandGate:
    """Attach to RobotSession(command_gate=...). Enforcement is simulation-only.

    Every command is rechecked using fresh full state; no verdict survives a new
    state/command. Rejection sends explicit simulation emergency stops to every
    endpoint. It does not claim a hardware braking/controlled-stop implementation.
    """
    def __init__(self, backend, joint_map, *, braking_acceleration, max_speed,
                 mode="observe", clearance_m=.005, reaction_s=.02,
                 max_age_s=.05, max_skew_s=.01, deadline_s=.01,
                 clock=time.monotonic):
        self.backend, self.clock, self.mode = backend, clock, mode
        names = set(backend.runtime_model.scene.independent_joints)
        if mode not in {"observe","enforce_simulation"}:
            raise ValueError("Only observe or enforce_simulation collision modes are supported")
        self.mapping = {k:v if isinstance(v,JointCalibration) else JointCalibration(**v) for k,v in joint_map.items()}
        if set(self.mapping) != names or set(braking_acceleration) != names or set(max_speed) != names:
            raise ValueError("Calibrate every independent mechanical joint and its braking/speed bounds")
        if any(not c.actuator or not math.isfinite(c.scale) or c.scale == 0 or not math.isfinite(c.offset)
               for c in self.mapping.values()):
            raise ValueError("Invalid motor-to-mechanical calibration")
        self.acceleration, self.max_speed = dict(braking_acceleration),dict(max_speed)
        values = [*self.acceleration.values(),*self.max_speed.values(),max_age_s,max_skew_s,deadline_s]
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError("Timing, braking and speed bounds must be positive and finite")
        if not math.isfinite(clearance_m) or clearance_m<0 or not math.isfinite(reaction_s) or reaction_s<0:
            raise ValueError("Invalid clearance/reaction margin")
        self.clearance_m, self.reaction_s = clearance_m,reaction_s
        self.max_age_s,self.max_skew_s,self.deadline_s = max_age_s,max_skew_s,deadline_s
        self.last_verdict = None

    @staticmethod
    def _motion(commands):
        return any(c["type"] in {"arm_joints","bundle","drive"} for c in commands.values())

    @staticmethod
    def stops(session):
        return {arm:{"type":"emergency_stop"} for arm in session.arms}

    def _require_simulation(self, session):
        if self.mode == "enforce_simulation" and not session.simulation_only:
            raise ValueError("Collision enforcement currently requires a synthetic loopback session")

    def filter_commands(self, session, commands):
        self._require_simulation(session)
        if not self._motion(commands):
            return commands
        started = self.clock()
        verdict = dict(clear=False,reason="unavailable",model_revision=self.backend.runtime_model.revision,
                       world_revision=self.backend.runtime_model.world_revision,
                       collision_policy_revision=self.backend.policy_revision, geometry_basis=self.backend.geometry_basis,
                       ignored_links=dict(self.backend.ignored_links), query_scope="all",
                       full_geometry_coverage=not self.backend.ignored_links,
                       state_sequence=session.state_sequence, started_at=started,expires_at=started+self.deadline_s,
                       command_sha256=self._fingerprint(commands),mode=self.mode,queries=0)
        try:
            observation = session.observation(now=started)
            freshness = list(observation.arm_freshness.values())
            if not observation.position_ready or any(not f.source_known or f.age_s>self.max_age_s for f in freshness):
                raise ValueError("stale_or_unknown_whole_robot_state")
            origins = [origin for values,_ in session._source_origins.values() for origin in values]
            if not origins or max(origins)-min(origins)>self.max_skew_s:
                raise ValueError("unsynchronized_whole_robot_state")
            verdict["expires_at"] = min(verdict["expires_at"],min(origins)+self.max_age_s)
            active = {n for n,a in zip(session.joint_names,session.active_mask) if a}
            if {c.actuator for c in self.mapping.values()} != active:
                raise ValueError("calibration_does_not_cover_all_active_actuators")
            positions = dict(zip(session.joint_names,observation.positions))
            velocities = dict(zip(session.joint_names,session.joint_field("velocity",now=started)))
            target, target_velocity = {},{}
            for arm,endpoint in session.arms.items():
                command = commands.get(arm,{})
                if command.get("type") != "arm_joints":
                    raise ValueError("complete_whole_robot_arm_target_required")
                if any(v != 0 for v in command.get("torques",[])):
                    raise ValueError("feedforward_torque_is_not_modeled")
                target.update(zip(endpoint.joints,command["positions"]))
                target_velocity.update(zip(endpoint.joints,command["velocities"]))
            q = {n:positions[c.actuator]*c.scale+c.offset for n,c in self.mapping.items()}
            goal = {n:target[c.actuator]*c.scale+c.offset for n,c in self.mapping.items()}
            v = {n:velocities[c.actuator]*c.scale for n,c in self.mapping.items()}
            gv = {n:target_velocity[c.actuator]*c.scale for n,c in self.mapping.items()}
            if any(not math.isfinite(x) or abs(x)>self.max_speed[n] for speeds in (v,gv) for n,x in speeds.items()):
                raise ValueError("velocity_unavailable_or_outside_bound")
            # Cover reaction age and constant-deceleration displacement in each
            # mechanical coordinate. These are configured simulation assumptions.
            reaction = self.reaction_s + max(f.age_s for f in freshness)
            def stopping(start,speed):
                return {n:start[n]+speed[n]*reaction+math.copysign(speed[n]**2/(2*self.acceleration[n]),speed[n]) for n in q}
            for a,b,subdivide in ((q,stopping(q,v),False),(q,goal,True),(goal,stopping(goal,gv),False)):
                result = check_linear_motion(self.backend,a,b,clearance_m=self.clearance_m,
                         deadline=verdict["expires_at"],clock=self.clock,subdivide=subdivide)
                verdict["queries"] += result["queries"]
                if not result["clear"]:
                    verdict.update({k:v for k,v in result.items() if k!="queries"})
                    break
            else:
                verdict.update(clear=True,reason="clear_command_and_stopping_paths")
            if session.state_sequence != verdict["state_sequence"] or self.clock() >= verdict["expires_at"]:
                verdict.update(clear=False,reason="expired_or_changed_state")
        except (ValueError,RuntimeError,OSError,KeyError) as exc:
            verdict.update(clear=False,reason=str(exc))
        verdict["completed_at"] = self.clock()
        self.last_verdict = verdict
        return commands if verdict["clear"] or self.mode=="observe" else self.stops(session)

    @staticmethod
    def _fingerprint(commands):
        return hashlib.sha256(json.dumps(commands,sort_keys=True,allow_nan=False).encode()).hexdigest()

    def dispatch_valid(self, session, commands):
        if self.mode=="observe" or not self._motion(commands): return True
        v = self.last_verdict
        return bool(v and v["clear"] and self.clock()<v["expires_at"] and session.state_sequence==v["state_sequence"]
                    and self.backend.runtime_model.revision==v["model_revision"]
                    and self.backend.policy_revision==v["collision_policy_revision"]
                    and self.backend.geometry_basis==v["geometry_basis"]
                    and self._fingerprint(commands)==v["command_sha256"])
