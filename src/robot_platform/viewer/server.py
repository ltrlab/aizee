"""Loopback-only viewer serving selected geometry, not repository directories."""
import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
from pathlib import Path
import struct
import threading
from xml.etree.ElementTree import ParseError

from robot_platform.models import UrdfModel


class ModelStore:
    def __init__(self, profiles):
        self.profiles = {p.id: p for p in profiles}
        if len(self.profiles) != len(profiles) or not profiles:
            raise ValueError("Select at least one model, with distinct ids")
        self.models, self.meshes, self.scenes = {}, {}, {}
        self.runtime_models, self.runtime_backends = {}, {}
        self.lock = threading.RLock()
        self.collision_lock = threading.Lock()

    def model(self, name):
        if name not in self.profiles:
            raise KeyError(name)
        with self.lock:
            if name not in self.models:
                self.models[name] = UrdfModel(self.profiles[name])
            return self.models[name]

    def mesh(self, name, index):
        import numpy as np
        import trimesh
        from robot_platform.simulation.collision import _validate_gltf
        model = self.model(name)
        if not 0 <= index < len(model.assets):
            raise KeyError(index)
        key = (name, index)
        with self.lock:
            if key in self.meshes:
                return self.meshes[key]
        path = model.assets[index]
        if path.suffix.lower() in {".gltf", ".glb"}:
            _validate_gltf(path, path.read_bytes())
        mesh = trimesh.load_scene(path, process=False, skip_materials=True, allow_remote=False).to_geometry()
        if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
            raise ValueError(f"Mesh has no finite triangles: {path.name}")
        # Preserve every source triangle. Automatic decimation can remove thin
        # CAD features and turn planar housings into visibly distorted surfaces.
        vertices = np.asarray(mesh.vertices, dtype="<f4")
        faces = np.asarray(mesh.faces, dtype="<u4")
        if not np.isfinite(vertices).all() or faces.max() >= len(vertices):
            raise ValueError("Invalid converted mesh")
        payload = struct.pack("<II", len(vertices), len(faces)) + vertices.tobytes() + faces.tobytes()
        with self.lock:
            self.meshes[key] = payload
        return payload

    def runtime_model(self, name):
        from robot_platform.simulation.runtime_model import RuntimeCollisionModel
        with self.lock:
            if name not in self.runtime_models:
                profile = self.profiles[name]
                if profile.collision_model is None:
                    raise ValueError("This model has no runtime collision geometry")
                self.runtime_models[name] = RuntimeCollisionModel(profile)
            return self.runtime_models[name]

    def runtime_geometry(self, name):
        runtime = self.runtime_model(name)
        return {"revision": runtime.revision, "geometry_role": "enclosing_bounds", "bounds": runtime.shapes,
                "keepout_ignored_links": runtime.keepout_ignored_links,
                "source_triangles": runtime.data["source_triangles"],
                "proxy_triangles": runtime.data["proxy_triangles"], "geometry_reviewed": False}

    def collisions(self, name, positions, mode="source", scope="all", clearance_m=0.0):
        import math
        from robot_platform.simulation.collision import CollisionScene
        from robot_platform.simulation.collision_groups import CollisionGroups, scope_mask
        scope_mask(scope)
        model = self.model(name)
        model.positions(positions)
        groups = CollisionGroups(model)
        extra = {"mode": mode, "query_scope": scope, "whole_robot_checked": scope == "all"}
        if mode not in {"source", "runtime", "keepout"}:
            raise ValueError("Unknown collision mode")
        if type(clearance_m) not in (int,float) or not math.isfinite(clearance_m) or not 0 <= clearance_m <= 1:
            raise ValueError("Inspection gap must be finite and between 0 and 1 metre")
        with self.collision_lock:
            if mode in {"runtime", "keepout"}:
                from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path
                runtime = self.runtime_model(name)
                key = (name,mode)
                if key not in self.runtime_backends:
                    if mode == "keepout" and not library_path().is_file():
                        raise ValueError("Keep-out mode requires the native collision library; rebuild it")
                    self.runtime_backends[key] = NativeCollisionBackend(runtime,geometry_mode="keepout" if mode=="keepout" else "cad") if library_path().is_file() else runtime
                extra.update(self.runtime_backends[key].proximity(positions, threshold=clearance_m if mode=="keepout" else 0.0, scope=scope))
                scene = runtime.scene
                shape_pairs = sorted(tuple(sorted(p["shapes"])) for p in extra["near_pairs"])
            else:
                if name not in self.scenes:
                    p = model.profile
                    self.scenes[name] = CollisionScene.from_urdf(p.urdf, p.package_root, mesh_resolver=p.resolve_mesh)
                scene = self.scenes[name]
                selected = groups.pairs(scene.shape_metadata, scene._pair_indices, scope)
                shape_pairs = scene._check_pairs(positions, selected, detailed=True)
                extra["eligible_pairs"] = len(selected)
            metadata = {shape["id"]: shape for shape in scene.shape_metadata}
        # The revision-bound sweep notes classify individual interfaces for
        # display only. Keep every detected contact, including those interfaces.
        interfaces = {} if mode == "keepout" else {tuple(sorted(item["shapes"])): item["reason"]
                      for item in (model.limit_review or {}).get("assembly_interfaces", [])}
        contacts = [{"shapes": [metadata[a], metadata[b]],
                     "kind": groups.kind(metadata[a], metadata[b]),
                     "provisional_interface": (a, b) in interfaces,
                     "reason": interfaces.get((a, b))} for a, b in shape_pairs]
        pairs = tuple(sorted({tuple(sorted(shape["link"] for shape in c["shapes"]))
                              for c in contacts}))
        return {**extra, "positions": positions, "pairs": pairs, "contacts": contacts,
                "runtime_interlock_active": False,
                "scope": ("Sampled keep-out boundaries with explicit link exclusions; not CAD contacts or a hardware interlock."
                          if mode == "keepout" else "Sampled pose only; no collision exclusions. "
                         "Provisional interfaces are labeled, not removed or approved.")}


def create_app(profiles, selected=None, local_only=True):
    from aiohttp import web
    store = ModelStore(profiles)
    selected = selected or profiles[0].id
    if selected not in store.profiles:
        raise ValueError(f"Unknown selected model: {selected}")
    static = Path(__file__).parent / "static"

    @web.middleware
    async def boundary(request, handler):
        # No cross-origin mutation, directory browsing or arbitrary file paths.
        host = request.host.split(":")[0]
        if local_only and host not in {"127.0.0.1", "localhost"}:
            raise web.HTTPForbidden(text="Viewer is local-only")
        if request.method == "POST":
            if request.headers.get("Origin") not in (None, f"{request.scheme}://{request.host}"):
                raise web.HTTPForbidden(text="Cross-origin request rejected")
            if request.content_type != "application/json":
                raise web.HTTPUnsupportedMediaType()
        try:
            response = await handler(request)
        except KeyError:
            response = web.json_response({"error": "Unknown model or asset"}, status=404)
        except (ValueError, OSError, RuntimeError, ImportError, TypeError, ParseError) as exc:
            response = web.json_response({"error": str(exc)}, status=422)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Cache-Control"] = "no-cache"
        return response

    app = web.Application(middlewares=[boundary], client_max_size=1024*1024)
    executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="model-viewer")
    async def offline(fn, *args):
        return await asyncio.get_running_loop().run_in_executor(executor, partial(fn, *args))
    async def close(_app):
        executor.shutdown(wait=False, cancel_futures=True)
    app.on_cleanup.append(close)

    async def index(_request):
        return web.FileResponse(static / "index.html")
    async def catalog(_request):
        return web.json_response({"models": [{"id": p.id, "title": p.title} for p in profiles], "selected": selected})
    async def metadata(request):
        model = await offline(store.model, request.match_info["model"])
        return web.json_response(model.metadata())
    async def mesh(request):
        payload = await offline(store.mesh, request.match_info["model"], int(request.match_info["index"]))
        response = web.Response(body=payload, content_type="application/octet-stream")
        response.enable_compression()
        return response
    async def collision(request):
        payload = await request.json()
        if not isinstance(payload, dict) or "positions" not in payload or set(payload) - {"positions", "mode", "scope", "clearance_m"}:
            raise ValueError("Expected positions object")
        result = await offline(store.collisions, request.match_info["model"], payload["positions"], payload.get("mode", "source"), payload.get("scope", "all"), payload.get("clearance_m",0.0))
        return web.json_response(result)
    async def runtime_geometry(request):
        return web.json_response(await offline(store.runtime_geometry, request.match_info["model"]))
    async def limit_review(request):
        model = await offline(store.model, request.match_info["model"])
        if model.profile.limit_review is None:
            raise KeyError("No limit review")
        return web.FileResponse(model.profile.limit_review, headers={"Content-Type": "application/json"})
    app.router.add_get("/", index)
    app.router.add_get("/api/models", catalog)
    app.router.add_get("/api/models/{model}", metadata)
    app.router.add_get("/api/models/{model}/limit-review", limit_review)
    app.router.add_get("/api/models/{model}/meshes/{index}", mesh)
    app.router.add_post("/api/models/{model}/collisions", collision)
    app.router.add_get("/api/models/{model}/runtime-model", runtime_geometry)
    app.router.add_static("/static", static, show_index=False)
    return app
