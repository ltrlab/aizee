"""Native enclosing shapes, with CAD confirmation or explicit keep-out policy."""
from __future__ import annotations

import ctypes as C
import json
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .collision import CollisionError
from .collision_groups import scope_mask

COUNTERS = ("eligible_pairs", "group_tests", "group_rejections", "link_tests", "link_rejections",
            "part_aabb_tests", "convex_tests", "candidate_pairs")


def library_path():
    if os.environ.get("ROBOT_COLLISION_LIBRARY"):
        return Path(os.environ["ROBOT_COLLISION_LIBRARY"]).resolve()
    name = "robot_collision.dll" if sys.platform == "win32" else (
        "librobot_collision.dylib" if sys.platform == "darwin" else "librobot_collision.so")
    return Path(__file__).resolve().parents[3] / "rust" / "target" / "release" / name


class NativeCollisionBackend:
    def __init__(self, runtime_model, library=None, *, geometry_mode="cad"):
        if geometry_mode not in {"cad", "keepout"}:
            raise CollisionError("Unknown collision geometry mode")
        self.runtime_model = runtime_model
        self.geometry_mode = geometry_mode
        self.geometry_basis = "conservative_envelope" if geometry_mode == "keepout" else "source_cad"
        self.pairs = runtime_model.pairs_for_scope("all", geometry_mode=geometry_mode)
        self.ignored_links = dict(runtime_model.keepout_ignored_links) if geometry_mode == "keepout" else {}
        self.policy_revision = runtime_model.keepout_policy_revision if geometry_mode == "keepout" else runtime_model.policy_revision
        self.motion_corners = runtime_model.keepout_motion_corners if geometry_mode == "keepout" else None
        self.lock = threading.RLock()
        self.handle = None
        self.lib = lib = C.CDLL(str(library or library_path()))
        lib.rpc_abi_version.restype = C.c_uint32
        if lib.rpc_abi_version() != 3:
            raise CollisionError("Native collision ABI version mismatch")
        lib.rpc_create.argtypes = [C.c_char_p, C.c_size_t]
        lib.rpc_create.restype = C.c_void_p
        lib.rpc_destroy.argtypes = [C.c_void_p]
        lib.rpc_destroy.restype = None
        lib.rpc_query.argtypes = [C.c_void_p, C.POINTER(C.c_double), C.c_size_t,
                                 C.c_double, C.c_uint32, C.POINTER(C.c_double), C.c_size_t, C.POINTER(C.c_uint64)]
        lib.rpc_query.restype = C.c_int32
        collision_links = {s['link'] for s in runtime_model.shapes}
        raw = json.dumps({"shapes": runtime_model.shapes,
                          "ignored_links": sorted(set(self.ignored_links) & collision_links)}, allow_nan=False).encode()
        self.handle = lib.rpc_create(raw, len(raw))
        if not self.handle:
            raise CollisionError("Native backend rejected the collision model")
        self._distances = np.empty(len(self.pairs), dtype=np.float64)
        self._counters = (C.c_uint64 * len(COUNTERS))()

    def close(self):
        with self.lock:
            if self.handle:
                self.lib.rpc_destroy(self.handle)
                self.handle = None

    def __del__(self):
        if getattr(self, "handle", None):
            self.close()

    def proximity(self, positions, *, threshold=0.05, max_pairs=None, scope="all"):
        start = time.perf_counter()
        mask = scope_mask(scope)
        if not np.isfinite(threshold) or threshold < 0:
            raise CollisionError("Proximity threshold must be finite and nonnegative")
        if max_pairs is not None and (type(max_pairs) is not int or max_pairs<1):
            raise CollisionError("max_pairs must be positive")
        with self.lock:
            if not self.handle:
                raise CollisionError("Native collision backend is closed")
            worlds = self.runtime_model.transforms(positions)
            poses = np.ascontiguousarray(np.column_stack((worlds[:, :3, 3],
                            Rotation.from_matrix(worlds[:, :3, :3]).as_quat())), dtype=np.float64)
            transformed = time.perf_counter()
            # Enclosures give a lower bound on CAD separation. Keep-out mode
            # treats their boundary as a restriction, never as a CAD contact.
            status = self.lib.rpc_query(self.handle, poses.ctypes.data_as(C.POINTER(C.c_double)),
                                       len(poses), threshold + 2e-6, mask,
                                       self._distances.ctypes.data_as(C.POINTER(C.c_double)), len(self._distances), self._counters)
            if status != 0:
                raise CollisionError(f"Native collision query unavailable ({status})")
            indices=np.flatnonzero(np.isfinite(self._distances))
            candidates=len(indices)
            coarse_done = time.perf_counter()
            stats = dict(zip(COUNTERS, self._counters))
            if self.geometry_mode == "keepout":
                near = [dict(indices=self.pairs[k], distance_m=max(0.,float(self._distances[k])-2e-6)) for k in indices]
                stats.update(fcl_collision_tests=0, fcl_distance_tests=0)
            else:
                # Confirm all candidates before compacting to the nearest CAD
                # pair. A nearest enclosure need not be the nearest real part.
                near = self.runtime_model.refine(positions, [self.pairs[k] for k in indices],
                                                 threshold=threshold, worlds=worlds, stats=stats)
            fine_done = time.perf_counter()
            kinds = ("inter_branch", "within_branch", "body", "environment")
            by_scope = {kind: dict(candidates=0, contacts=0) for kind in kinds}
            for k in indices:
                by_scope[self.runtime_model.pair_kinds[self.pairs[k]]]["candidates"] += 1
            for pair in near:
                by_scope[self.runtime_model.pair_kinds[tuple(pair["indices"])]]["contacts"] += 1
            total=len(near)
            if max_pairs is not None and total>max_pairs:
                near=sorted(near,key=lambda pair:pair['distance_m'])[:max_pairs]
        result=self.runtime_model.result(near, threshold, (time.perf_counter()-start)*1000, "parry-bounds+fcl-cad", scope=scope)
        result.update(total_near_pairs=total,pairs_truncated=total>len(near))
        result['candidate_pairs']=candidates
        eligible = len(self.runtime_model.pairs_for_scope(scope,geometry_mode=self.geometry_mode))
        result.update(geometry_basis=self.geometry_basis, geometry_mode=self.geometry_mode,
                      backend="parry-keepout" if self.geometry_mode == "keepout" else "parry-bounds+fcl-cad",
                      collision_policy_revision=self.policy_revision, eligible_pairs=eligible,
                      ignored_links=dict(self.ignored_links), excluded_pair_count=len(self.runtime_model.pairs_for_scope(scope))-eligible,
                      full_geometry_coverage=scope == "all" and not self.ignored_links,
                      verdict_kind="keepout_boundary" if self.geometry_mode == "keepout" else "cad_proximity")
        result.update(broad_phase=stats, by_scope=by_scope, stage_ms=dict(
            transforms=(transformed-start)*1000, coarse=(coarse_done-transformed)*1000,
            fine=0. if self.geometry_mode == "keepout" else (fine_done-coarse_done)*1000))
        return result
