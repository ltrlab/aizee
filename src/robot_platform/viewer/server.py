"""Loopback-only viewer serving selected geometry, not repository directories."""

import asyncio
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import gzip
import json
from pathlib import Path
import struct
import threading
import math
import time
from xml.etree.ElementTree import ParseError

from robot_platform.models import UrdfModel
from robot_platform.models.urdf import asset_digest

MESH_FORMAT = (
    'mesh1'  # mesh_payload's layout; a stored mesh's name carries it, so a new layout never reads an old file.
)
# The headset's model (helm/static/xr_model.js): each mesh's vertices merged within 5 mm cubes. Minerva's display
# model, 2.66 M triangles and 128 MB as drawn in the 3D view, is too much for a Quest to load and draw in stereo; so
# coarse it is 89 k triangles and 0.67 MB gzip-compressed (2026-10-31).
COARSE_CELL_M = 0.005


def mesh_payload(path):
    """A mesh as the viewer sends it: vertex and face counts (<II), float32 vertices, uint32 faces, little-endian."""
    import numpy as np
    import trimesh
    from robot_platform.simulation.collision import _validate_gltf

    path = Path(path)
    if path.suffix.lower() in {'.gltf', '.glb'}:
        _validate_gltf(path, path.read_bytes())
    mesh = trimesh.load_scene(path, process=False, skip_materials=True, allow_remote=False).to_geometry()
    if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
        raise ValueError(f'Mesh has no finite triangles: {path.name}')
    # Preserve every source triangle. Automatic decimation can remove thin
    # CAD features and turn planar housings into visibly distorted surfaces.
    vertices = np.asarray(mesh.vertices, dtype='<f4')
    faces = np.asarray(mesh.faces, dtype='<u4')
    if not np.isfinite(vertices).all() or faces.max() >= len(vertices):
        raise ValueError('Invalid converted mesh')
    return struct.pack('<II', len(vertices), len(faces)) + vertices.tobytes() + faces.tobytes()


def coarse_payload(payload, cell_m=COARSE_CELL_M):
    """A mesh payload (mesh_payload) with its vertices merged within `cell_m` cubes, each at its vertices' mean, and the
    triangles that collapse dropped (vertex clustering): a light copy for the headset, never the 3D view's own."""
    import numpy as np

    vertex_count, face_count = struct.unpack('<II', payload[:8])
    vertices = np.frombuffer(payload, '<f4', vertex_count * 3, 8).reshape(-1, 3)
    faces = np.frombuffer(payload, '<u4', face_count * 3, 8 + vertex_count * 12).reshape(-1, 3)
    cells = np.floor(vertices / cell_m).astype(np.int64)
    cells -= cells.min(axis=0)
    span = cells.max(axis=0) + 1
    _keys, cell_of = np.unique((cells[:, 0] * span[1] + cells[:, 1]) * span[2] + cells[:, 2], return_inverse=True)
    count = np.bincount(cell_of).astype(np.float64)
    merged = np.stack([np.bincount(cell_of, vertices[:, k]) for k in range(3)], axis=1) / count[:, None]
    mapped = cell_of.reshape(-1)[faces]
    mapped = mapped[(mapped[:, 0] != mapped[:, 1]) & (mapped[:, 1] != mapped[:, 2]) & (mapped[:, 0] != mapped[:, 2])]
    _same, first = np.unique(np.sort(mapped, axis=1), axis=0, return_index=True)  # One of each, in its own winding.
    mapped = mapped[np.sort(first)]
    used, mapped = np.unique(mapped, return_inverse=True)  # Only the vertices the triangles left use.
    merged, mapped = merged[used].astype('<f4'), mapped.reshape(-1, 3).astype('<u4')
    return struct.pack('<II', len(merged), len(mapped)) + merged.tobytes() + mapped.tobytes()


def stored_mesh_name(sha256):
    """A converted mesh's file in a mesh store (Helm on Minerva): its gzip-compressed payload, named by its source's
    digest. ops.helm_robot converts and uploads them from the build PC, where the CAD meshes are."""
    return f'{sha256}.{MESH_FORMAT}.gz'


def read_guide(path):
    path = Path(path)
    if path.suffix != '.jsonl':
        return json.loads(path.read_text())
    with path.open('rb') as source:
        source.seek(0, 2)
        source.seek(max(0, source.tell() - 65536))
        tail = source.read()
    if b'\n' not in tail:
        raise ValueError('Waiting for a complete calibration frame')
    complete = tail.rsplit(b'\n', 1)[0]
    return json.loads(complete.rsplit(b'\n', 1)[-1])


class ModelStore:
    """`mesh_store`: a directory of converted meshes (stored_mesh_name) to serve instead of converting the CAD."""

    def __init__(self, profiles, mesh_store=None):
        self.mesh_store = None if mesh_store is None else Path(mesh_store)
        self.profiles = {p.id: p for p in profiles}
        if len(self.profiles) != len(profiles) or not profiles:
            raise ValueError('Select at least one model, with distinct ids')
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
        model = self.model(name)
        if not 0 <= index < len(model.assets):
            raise KeyError(index)
        key = (name, index)
        with self.lock:
            if key in self.meshes:
                return self.meshes[key]
        payload = mesh_payload(model.assets[index])
        with self.lock:
            self.meshes[key] = payload
        return payload

    def stored_mesh(self, name, index):
        """(gzip-compressed payload, ETag) of a mesh from the mesh store, by its verified digest: Helm on Minerva
        serves the 3D view without the CAD files or trimesh, and a browser revalidates what it already holds."""
        model = self.model(name)
        if not 0 <= index < len(model.assets):
            raise KeyError(index)
        sha = asset_digest(model.assets[index])
        try:
            body = (self.mesh_store / stored_mesh_name(sha)).read_bytes()
        except FileNotFoundError:
            raise ValueError(
                f'{model.assets[index].name} is not in the mesh store yet: upload the meshes from the '
                'build PC (python -m robot_platform.ops.helm_robot meshes)'
            ) from None
        return body, f'"{sha}-{MESH_FORMAT}"'

    def coarse_mesh(self, name, index):
        """(gzip-compressed coarse payload, ETag) of a mesh for the headset (coarse_payload), made once from the mesh
        store's copy where Helm has one (on Minerva), else from the CAD."""
        model = self.model(name)
        if not 0 <= index < len(model.assets):
            raise KeyError(index)
        key = (name, index, 'coarse')
        with self.lock:
            if key in self.meshes:
                return self.meshes[key]
        if self.mesh_store is not None:
            body, etag = self.stored_mesh(name, index)
            payload = gzip.decompress(body)
        else:
            payload, etag = self.mesh(name, index), f'"{asset_digest(model.assets[index])}-{MESH_FORMAT}"'
        result = gzip.compress(coarse_payload(payload), 6, mtime=0), f'{etag[:-1]}-coarse{COARSE_CELL_M * 1000:g}mm"'
        with self.lock:
            self.meshes[key] = result
        return result

    def runtime_model(self, name):
        from robot_platform.simulation.runtime_model import RuntimeCollisionModel

        with self.lock:
            if name not in self.runtime_models:
                profile = self.profiles[name]
                if profile.collision_model is None:
                    raise ValueError('This model has no runtime collision geometry')
                self.runtime_models[name] = RuntimeCollisionModel(profile)
            return self.runtime_models[name]

    def runtime_geometry(self, name):
        runtime = self.runtime_model(name)
        return {
            'revision': runtime.revision,
            'geometry_role': 'enclosing_bounds',
            'bounds': runtime.shapes,
            'keepout_ignored_links': runtime.keepout_ignored_links,
            'source_triangles': runtime.data['source_triangles'],
            'proxy_triangles': runtime.data['proxy_triangles'],
            'geometry_reviewed': False,
        }

    def collisions(self, name, positions, mode='source', scope='all', clearance_m=0.0):
        import math
        from robot_platform.simulation.collision import CollisionScene
        from robot_platform.simulation.collision_groups import CollisionGroups, scope_mask

        scope_mask(scope)
        model = self.model(name)
        model.positions(positions)
        groups = CollisionGroups(model)
        extra = {'mode': mode, 'query_scope': scope, 'whole_robot_checked': scope == 'all'}
        if mode not in {'source', 'runtime', 'keepout'}:
            raise ValueError('Unknown collision mode')
        if type(clearance_m) not in (int, float) or not math.isfinite(clearance_m) or not 0 <= clearance_m <= 1:
            raise ValueError('Inspection gap must be finite and between 0 and 1 metre')
        with self.collision_lock:
            if mode in {'runtime', 'keepout'}:
                from robot_platform.simulation.native_collision import NativeCollisionBackend, library_path

                runtime = self.runtime_model(name)
                key = (name, mode)
                if key not in self.runtime_backends:
                    if mode == 'keepout' and not library_path().is_file():
                        raise ValueError('Keep-out mode requires the native collision library; rebuild it')
                    self.runtime_backends[key] = (
                        NativeCollisionBackend(runtime, geometry_mode='keepout' if mode == 'keepout' else 'cad')
                        if library_path().is_file()
                        else runtime
                    )
                extra.update(
                    self.runtime_backends[key].proximity(
                        positions, threshold=clearance_m if mode == 'keepout' else 0.0, scope=scope
                    )
                )
                scene = runtime.scene
                shape_pairs = sorted(tuple(sorted(p['shapes'])) for p in extra['near_pairs'])
            else:
                if name not in self.scenes:
                    p = model.profile
                    self.scenes[name] = CollisionScene.from_urdf(p.urdf, p.package_root, mesh_resolver=p.resolve_mesh)
                scene = self.scenes[name]
                selected = groups.pairs(scene.shape_metadata, scene._pair_indices, scope)
                shape_pairs = scene._check_pairs(positions, selected, detailed=True)
                extra['eligible_pairs'] = len(selected)
            metadata = {shape['id']: shape for shape in scene.shape_metadata}
        # The revision-bound sweep notes classify individual interfaces for
        # display only. Keep every detected contact, including those interfaces.
        interfaces = (
            {}
            if mode == 'keepout'
            else {
                tuple(sorted(item['shapes'])): item['reason']
                for item in (model.limit_review or {}).get('assembly_interfaces', [])
            }
        )
        contacts = [
            {
                'shapes': [metadata[a], metadata[b]],
                'kind': groups.kind(metadata[a], metadata[b]),
                'provisional_interface': (a, b) in interfaces,
                'reason': interfaces.get((a, b)),
            }
            for a, b in shape_pairs
        ]
        pairs = tuple(sorted({tuple(sorted(shape['link'] for shape in c['shapes'])) for c in contacts}))
        return {
            **extra,
            'positions': positions,
            'pairs': pairs,
            'contacts': contacts,
            'runtime_interlock_active': False,
            'scope': (
                'Sampled keep-out boundaries with explicit link exclusions; not CAD contacts or a hardware interlock.'
                if mode == 'keepout'
                else 'Sampled pose only; no collision exclusions. '
                'Provisional interfaces are labeled, not removed or approved.'
            ),
        }


def create_app(profiles, selected=None, local_only=True, guide=None, index_html=None, mesh_store=None):
    """`guide` is a pose-feed file or a callable returning the same dictionary.

    `index_html` replaces the page at "/"; the viewer itself stays at "/viewer". `mesh_store`: serve the meshes
    converted on the build PC (ModelStore), as on Minerva.
    """
    from aiohttp import web

    store = ModelStore(profiles, mesh_store=mesh_store)
    selected = selected or profiles[0].id
    if selected not in store.profiles:
        raise ValueError(f'Unknown selected model: {selected}')
    static = Path(__file__).parent / 'static'

    @web.middleware
    async def boundary(request, handler):
        # No cross-origin mutation, directory browsing or arbitrary file paths.
        host = request.host.split(':')[0]
        if local_only and host not in {'127.0.0.1', 'localhost'}:
            raise web.HTTPForbidden(text='Viewer is local-only')
        if request.method == 'POST':
            if request.headers.get('Origin') not in (None, f'{request.scheme}://{request.host}'):
                raise web.HTTPForbidden(text='Cross-origin request rejected')
            if request.content_type != 'application/json':
                raise web.HTTPUnsupportedMediaType()
        try:
            response = await handler(request)
        except KeyError:
            response = web.json_response({'error': 'Unknown model or asset'}, status=404)
        except (ValueError, OSError, RuntimeError, ImportError, TypeError, ParseError) as exc:
            response = web.json_response({'error': str(exc)}, status=422)
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers.setdefault('Cache-Control', 'no-cache')
        return response

    app = web.Application(middlewares=[boundary], client_max_size=1024 * 1024)
    executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix='model-viewer')

    async def offline(fn, *args):
        return await asyncio.get_running_loop().run_in_executor(executor, partial(fn, *args))

    async def close(_app):
        executor.shutdown(wait=False, cancel_futures=True)

    app.on_cleanup.append(close)

    async def index(_request):
        return web.FileResponse(index_html or static / 'index.html')

    async def viewer_page(_request):
        return web.FileResponse(static / 'index.html')

    async def catalog(_request):
        return web.json_response(
            {
                'models': [{'id': p.id, 'title': p.title} for p in profiles],
                'selected': selected,
                'calibration_guide': guide is not None,
            }
        )

    async def calibration_guide(_request):
        if guide is None:
            raise KeyError('No calibration guide')
        data = guide() if callable(guide) else read_guide(guide)
        model = await offline(store.model, data['model'])
        if data.get('urdf_sha256') != model.sha256:
            raise ValueError('Calibration guide CAD revision mismatch')
        positions = data['positions']
        if set(positions) != set(model.independent) or any(
            type(q) not in (int, float) or not math.isfinite(q) for q in positions.values()
        ):
            raise ValueError('Invalid calibration guide positions')
        if data.get('live'):
            # Measured positions may exceed provisional viewer limits. Report
            # them honestly; do not clamp a calibration preview to guessed bounds.
            age = time.time() - data['updated_at'] + data['source_age_seconds']
            data['stale'] = data.get('feed_state', 'reading') != 'reading' or not 0 <= age <= 0.75
            data['age_seconds'] = age
        else:
            model.positions(positions)
        return web.json_response(data, headers={'Cache-Control': 'no-store'})

    async def metadata(request):
        model = await offline(store.model, request.match_info['model'])
        return web.json_response(model.metadata())

    def gzipped(request, body, etag):
        headers = {'ETag': etag, 'Cache-Control': 'no-cache'}
        if etag in request.headers.get('If-None-Match', ''):
            return web.Response(status=304, headers=headers)
        if 'gzip' in request.headers.get('Accept-Encoding', ''):
            return web.Response(
                body=body, content_type='application/octet-stream', headers={**headers, 'Content-Encoding': 'gzip'}
            )
        return web.Response(body=gzip.decompress(body), content_type='application/octet-stream', headers=headers)

    async def mesh(request):
        name, index = request.match_info['model'], int(request.match_info['index'])
        if request.query.get('detail') == 'coarse':  # The headset's model (coarse_payload).
            return gzipped(request, *await offline(store.coarse_mesh, name, index))
        if store.mesh_store is not None:
            return gzipped(request, *await offline(store.stored_mesh, name, index))
        payload = await offline(store.mesh, name, index)
        response = web.Response(body=payload, content_type='application/octet-stream', zlib_executor_size=65536)
        response.enable_compression()
        return response

    async def collision(request):
        payload = await request.json()
        if (
            not isinstance(payload, dict)
            or 'positions' not in payload
            or set(payload) - {'positions', 'mode', 'scope', 'clearance_m'}
        ):
            raise ValueError('Expected positions object')
        result = await offline(
            store.collisions,
            request.match_info['model'],
            payload['positions'],
            payload.get('mode', 'source'),
            payload.get('scope', 'all'),
            payload.get('clearance_m', 0.0),
        )
        return web.json_response(result)

    async def runtime_geometry(request):
        return web.json_response(await offline(store.runtime_geometry, request.match_info['model']))

    async def limit_review(request):
        model = await offline(store.model, request.match_info['model'])
        if model.profile.limit_review is None:
            raise KeyError('No limit review')
        return web.FileResponse(model.profile.limit_review, headers={'Content-Type': 'application/json'})

    app.router.add_get('/', index)
    app.router.add_get('/viewer', viewer_page)
    app.router.add_get('/api/models', catalog)
    app.router.add_get('/api/calibration-guide', calibration_guide)
    app.router.add_get('/api/models/{model}', metadata)
    app.router.add_get('/api/models/{model}/limit-review', limit_review)
    app.router.add_get('/api/models/{model}/meshes/{index}', mesh)
    app.router.add_post('/api/models/{model}/collisions', collision)
    app.router.add_get('/api/models/{model}/runtime-model', runtime_geometry)
    app.router.add_static('/static', static, show_index=False)
    return app
