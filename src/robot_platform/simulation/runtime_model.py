"""Revision-bound broad-phase enclosures and exact CAD narrow-phase queries.

Enclosures may fill holes: they are only used to reject distant pairs. A contact
or distance verdict always comes from the source CAD (cached FCL geometry).
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time

import numpy as np

from robot_platform.models import UrdfModel
from robot_platform.models.urdf import asset_digest, verified_asset, verified_release_active
from .collision import CollisionError, CollisionScene, _read_asset, _validate_gltf


def digest(path):
    return asset_digest(path)


def build_runtime_model(profile, output):
    """Write a new deterministic artifact; never replace source CAD or exemptions."""
    import trimesh
    from scipy.spatial import ConvexHull, HalfspaceIntersection
    from itertools import product

    output = Path(output)
    if output.exists():
        raise ValueError("Refusing to overwrite a collision model; generate a new revision")
    model = UrdfModel(profile, apply_review=False)
    physical = {d["link"] for d in model.geometry if d["role"] == "visual"}
    collisions = [d for d in model.geometry if d["role"] == "collision"]
    if not collisions or physical - {d["link"] for d in collisions}:
        raise ValueError("Every visible physical link needs source collision geometry")
    shapes, assets, source_triangles, proxy_triangles = [], {}, 0, 0
    for desc in collisions:
        item = {k: desc[k] for k in ("link", "origin", "shape")}
        item["id"] = desc["collision_id"]
        if desc["shape"] == "mesh":
            path = model.assets[desc["asset"]]
            raw = _read_asset(path)
            if path.suffix.lower() in {".gltf", ".glb"}:
                # External GLTF resources need their own revision tracking.
                # Inline-only assets are accepted; fail rather than miss changes.
                _validate_gltf(path, raw)
                data = json.loads(raw if path.suffix.lower()==".gltf" else raw[20:20+int.from_bytes(raw[12:16],"little")])
                if any(not x.get("uri", "").startswith("data:") and "uri" in x
                       for x in data.get("buffers", []) + data.get("images", [])):
                    raise ValueError("Runtime export requires embedded GLTF resources")
            mesh = trimesh.load_scene(path, process=False, skip_materials=True,
                                      allow_remote=False).to_geometry()
            vertices = np.unique(np.asarray(mesh.vertices, dtype=float) * desc["scale"], axis=0)
            if not np.isfinite(vertices).all() or len(mesh.faces) == 0:
                raise ValueError(f"Invalid source mesh: {path}")
            # 26 supporting directions give a small OUTER polytope. Removing
            # triangles from a hull can shrink it; intersecting supporting planes
            # keeps every source point inside and bounds the runtime complexity.
            normals = np.array([v for v in product((-1, 0, 1), repeat=3) if any(v)], dtype=float)
            normals /= np.linalg.norm(normals, axis=1)[:, None]
            offsets = (vertices @ normals.T).max(axis=0) + 1e-5
            planes = np.column_stack((normals, -offsets))
            enclosure = HalfspaceIntersection(planes, vertices.mean(axis=0)).intersections
            convex = ConvexHull(enclosure)
            faces = convex.simplices.copy()
            triangles = enclosure[faces]
            reverse = np.einsum("ij,ij->i", np.cross(triangles[:, 1]-triangles[:, 0],
                triangles[:, 2]-triangles[:, 0]), convex.equations[:, :3]) < 0
            faces[reverse] = faces[reverse, ::-1]
            # Avoid automatic face cleanup: dropping tiny but valid triangles
            # opens seams in otherwise closed half-space intersections.
            hull = trimesh.Trimesh(vertices=enclosure, faces=faces, process=False)
            hull.remove_unreferenced_vertices()
            if not hull.is_watertight or not hull.is_convex or hull.volume <= 0:
                raise ValueError(f"Invalid convex enclosure topology: {path}")
            # Check the actual exported hull planes, not just the construction.
            equations = np.column_stack((hull.face_normals,
                -np.einsum("ij,ij->i", hull.face_normals, hull.triangles[:, 0])))
            excess = max(float(np.max(points @ equations[:, :3].T + equations[:, 3]))
                         for points in np.array_split(vertices, max(1, len(vertices) // 256 + 1)))
            if excess > 1e-8:
                raise ValueError(f"Convex enclosure coverage failed: {path}")
            item.update(shape="convex", vertices=hull.vertices.tolist(), faces=hull.faces.tolist(),
                        coverage_max_excess_m=excess, source_asset=desc["asset"],
                        source_triangles=len(mesh.faces))
            source_triangles += len(mesh.faces)
            proxy_triangles += len(hull.faces)
            assets[str(desc["asset"])] = digest(path)
        else:
            item.update({key: desc[key] for key in ("size", "radius", "length") if key in desc})
        shapes.append(item)
    artifact = dict(schema_version=1, kind="runtime_collision_model", model_id=profile.id,
                    urdf_sha256=model.sha256, assets=assets, shapes=shapes,
                    generator="per_instance_26_plane_outer_enclosure_v1", geometry_role="broad_phase_only", geometry_reviewed=False,
                    allowed_shape_pairs=[], source_triangles=source_triangles,
                    proxy_triangles=proxy_triangles,
                    limitation="Broad-phase bounds only. Confirm all candidate contacts/distances against source CAD.")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as file:
        json.dump(artifact, file, separators=(",", ":"), allow_nan=False)
        file.write("\n")
    return artifact


class RuntimeCollisionModel:
    """Owns immutable model identity, cached FK/FCL objects and query metadata."""

    def __init__(self, profile, path=None, *, environment=()):
        self.profile = profile
        self.path = Path(path or profile.collision_model)
        raw = self.path.read_bytes()
        self.revision = hashlib.sha256(raw).hexdigest()
        from dataclasses import asdict
        environment = tuple(environment)
        world = json.dumps([asdict(box) for box in environment],sort_keys=True,allow_nan=False).encode()
        self.world_revision = hashlib.sha256(world).hexdigest()
        if environment:
            self.revision = hashlib.sha256(raw+world).hexdigest()
        self.data = data = json.loads(raw)
        self.model = UrdfModel(profile, apply_review=False)
        if (data.get("schema_version") != 1 or data.get("kind") != "runtime_collision_model"
                or data.get("model_id") != profile.id or data.get("urdf_sha256") != self.model.sha256):
            raise CollisionError("Runtime collision model does not match the robot/URDF revision")
        if data.get("geometry_role") != "broad_phase_only":
            raise CollisionError("Runtime bounds must be labeled broad_phase_only")
        descriptors = {d["collision_id"]: d for d in self.model.geometry if d["role"] == "collision"}
        shapes = data["shapes"]
        if len(shapes) != len(descriptors) or {s["id"] for s in shapes} != set(descriptors):
            raise CollisionError("Runtime model must cover every collision instance exactly once")
        expected_assets = {str(d["asset"]) for d in descriptors.values() if d["shape"] == "mesh"}
        if set(data["assets"]) != expected_assets:
            raise CollisionError("Incomplete runtime source asset manifest")
        for index, sha in data["assets"].items():
            if digest(self.model.assets[int(index)]) != sha:
                raise CollisionError("Runtime collision source mesh revision changed; regenerate proxies")
        # A verified release checked every enclosure on the build PC and ships no
        # meshes: check transforms and primitives here, then load kinematics only.
        kinematics_only = verified_release_active() and all(
            verified_asset(self.model.assets[int(index)]) for index in data["assets"])
        if not kinematics_only:
            import trimesh
            from scipy.spatial import ConvexHull
        source_points = {}
        for shape in shapes:
            desc = descriptors[shape["id"]]
            if shape["link"] != desc["link"] or shape["origin"] != desc["origin"]:
                raise CollisionError("Runtime shape transform/link differs from source")
            if desc["shape"] == "mesh":
                if shape["shape"] != "convex":
                    raise CollisionError("Mesh proxy must be a convex solid")
                if kinematics_only:
                    continue
                key=(data['assets'][str(desc['asset'])],tuple(desc['scale']))
                if key not in source_points:
                    mesh=trimesh.load_scene(self.model.assets[desc['asset']],process=False,
                                           skip_materials=True,allow_remote=False).to_geometry()
                    source_points[key]=np.unique(np.asarray(mesh.vertices)*desc['scale'],axis=0)
                # Recheck coverage at load time, not only when generating the
                # file. A hand-edited/incorrect bound must never prune real CAD.
                vertices=np.asarray(shape['vertices'],dtype=float)
                if vertices.ndim!=2 or vertices.shape[1]!=3 or not np.isfinite(vertices).all():
                    raise CollisionError("Invalid broad-phase vertices")
                try: planes=ConvexHull(vertices).equations
                except Exception as exc: raise CollisionError("Invalid broad-phase hull") from exc
                for points in np.array_split(source_points[key],max(1,len(source_points[key])//256+1)):
                    if np.max(points @ planes[:,:3].T+planes[:,3])>1e-8:
                        raise CollisionError("Broad-phase bound does not enclose source CAD")
            elif any(shape.get(k) != desc.get(k) for k in ("shape", "size", "radius", "length")):
                raise CollisionError("Primitive proxy differs from source")
        if data.get("allowed_shape_pairs"):
            raise CollisionError("Contact-policy approval is not implemented; runtime exclusions must be empty")
        self.kinematics_only = kinematics_only
        # Proxy hulls enclose their meshes, so hull bounds are conservative stand-ins.
        hull_bounds = {s["id"]: np.array([np.min(s["vertices"], axis=0), np.max(s["vertices"], axis=0)])
                       for s in shapes if s["shape"] == "convex"}
        self.scene = CollisionScene.from_urdf(profile.urdf, profile.package_root,
                         mesh_resolver=profile.resolve_mesh, environment=environment,
                         mesh_bounds=hull_bounds.__getitem__ if kinematics_only else None)
        order = {s["id"]: s for s in shapes}
        for box in environment:
            key=f"environment:{box.name}"
            order[f"{key}/0"] = dict(id=f"{key}/0",link=key,shape="box",size=list(box.size),
                                    origin=dict(xyz=list(box.xyz),rpy=list(box.rpy)),obstacle=True)
        self.shapes = [order[s["id"]] for s in self.scene.shape_metadata]
        self.pairs = tuple(self.scene._pair_indices)
        from .collision_groups import CollisionGroups, SCOPES
        self.groups = CollisionGroups(self.model)
        self.shapes = [dict(s, group=self.groups.group(s)) for s in self.shapes]
        self.keepout_ignored_links = dict(profile.keepout_ignored_links)
        self.keepout_pairs = tuple((i,j) for i,j in self.pairs if
            self.shapes[i]['link'] not in self.keepout_ignored_links and self.shapes[j]['link'] not in self.keepout_ignored_links)
        self.scoped_pairs = {scope: self.groups.pairs(self.shapes, self.pairs, scope) for scope in SCOPES}
        self.keepout_scoped_pairs = {scope: self.groups.pairs(self.shapes, self.keepout_pairs, scope) for scope in SCOPES}
        self.pair_kinds = {pair: self.groups.kind(self.shapes[pair[0]], self.shapes[pair[1]]) for pair in self.pairs}
        policy = json.dumps(self.groups.links, sort_keys=True).encode()
        self.policy_revision = hashlib.sha256(policy).hexdigest()
        self.keepout_policy_revision = hashlib.sha256(json.dumps(dict(
            groups=self.groups.links, ignored_links=self.keepout_ignored_links,
            geometry_basis='conservative_envelope'), sort_keys=True).encode()).hexdigest()
        self._corners = np.asarray([s.corners for s in self.scene._shapes])
        # Bounds can extend beyond source mesh corners. Use their own support
        # radius when bounding swept motion, including the outer hull padding.
        self.keepout_motion_corners = [np.asarray(s['vertices']) if s['shape']=='convex' else source.corners
                                       for s,source in zip(self.shapes,self.scene._shapes)]
        # These are the physical shapes. The artifact's convex shapes are only
        # broad-phase bounds and must never provide the final contact verdict.
        self._pair_set = set(self.pairs)

    def transforms(self, positions):
        links = self.scene.link_transforms(positions)
        return np.asarray([s.local if s.obstacle else links[s.link] @ s.local for s in self.scene._shapes])

    def pairs_for_scope(self, scope, *, geometry_mode="cad"):
        from .collision_groups import scope_mask
        scope_mask(scope)
        if geometry_mode not in {"cad", "keepout"}:
            raise CollisionError("Unknown collision geometry mode")
        return (self.keepout_scoped_pairs if geometry_mode == "keepout" else self.scoped_pairs)[scope]

    def proximity(self, positions, *, threshold=0.05, scope="all"):
        """Independent FCL query on source CAD, without native pair filtering."""
        start = time.perf_counter()
        near = self.refine(positions, self.pairs_for_scope(scope), threshold=threshold)
        return self.result(near, threshold, (time.perf_counter()-start)*1000, "fcl-cad", scope=scope)

    def refine(self, positions, pairs, *, threshold, worlds=None, stats=None):
        """Confirm candidates using the original surfaces and analytic primitives.

        Mesh distances describe surfaces, as in the source collision checker;
        they do not certify solid containment or physical stopping safety.
        """
        if not np.isfinite(threshold) or threshold < 0:
            raise CollisionError("Proximity threshold must be finite and nonnegative")
        pairs = [tuple(map(int,pair)) for pair in pairs]
        if any(pair not in self._pair_set for pair in pairs):
            raise CollisionError("Unknown narrow-phase pair")
        scene = self.scene
        if scene._objects is None:
            raise CollisionError("Loaded for kinematics only (verified release); CAD refinement needs source meshes")
        with scene._query_lock:
            # Reuse the same pose snapshot as the native hierarchy. Batch the
            # source bounds instead of recomputing FK and 91 Python reductions.
            worlds = self.transforms(positions) if worlds is None else worlds
            if stats is not None:
                stats.update(fcl_collision_tests=0, fcl_distance_tests=0)
            if not pairs:
                return []
            corners = self._corners @ worlds[:, :3, :3].transpose(0, 2, 1) + worlds[:, None, :3, 3]
            if not np.isfinite(corners).all():
                raise CollisionError("Nonfinite source collision bounds")
            lows, highs = corners.min(axis=1), corners.max(axis=1)
            indices = np.asarray(pairs)
            gap = np.maximum(0, np.maximum(lows[indices[:, 0]] - highs[indices[:, 1]],
                                          lows[indices[:, 1]] - highs[indices[:, 0]]))
            eligible = np.einsum("ij,ij->i", gap, gap) <= threshold * threshold
            updated=set()
            def obj(index):
                if index not in updated:
                    world=worlds[index]
                    scene._objects[index].setTransform(scene._fcl.Transform(world[:3,:3],world[:3,3]))
                    updated.add(index)
                return scene._objects[index]
            near=[]
            for (i,j), check in zip(pairs, eligible):
                if not check: continue
                a,b=obj(i),obj(j)
                if stats is not None: stats["fcl_collision_tests"] += 1
                contact=scene._fcl.CollisionResult()
                count=scene._fcl.collide(a,b,scene._fcl.CollisionRequest(),contact)
                if count<0 or bool(count)!=bool(contact.is_collision):
                    raise CollisionError("FCL narrow-phase query failed")
                if not count and threshold == 0: continue
                if not count and stats is not None: stats["fcl_distance_tests"] += 1
                distance=0. if count else scene._fcl.distance(a,b,scene._fcl.DistanceRequest(),scene._fcl.DistanceResult())
                if not np.isfinite(distance): raise CollisionError("Nonfinite FCL distance")
                distance=max(0.,float(distance))
                if distance<=threshold: near.append(dict(indices=[i,j],distance_m=distance))
            return near

    def result(self, near, threshold, elapsed_ms, backend, *, scope="all"):
        return dict(model_revision=self.revision, world_revision=self.world_revision, backend=backend, threshold_m=threshold,
                    collision_policy_revision=self.policy_revision, query_scope=scope,
                    whole_robot_checked=scope == "all", eligible_pairs=len(self.pairs_for_scope(scope)),
                    minimum_distance_m=min((p["distance_m"] for p in near), default=None),
                    near_pairs=[dict(shapes=[self.shapes[i]["id"] for i in p["indices"]],
                                     distance_m=p["distance_m"], kind=self.pair_kinds[tuple(p["indices"])]) for p in near],
                    compute_ms=elapsed_ms, geometry_basis="source_cad", geometry_reviewed=False,
                    runtime_interlock_active=False)
