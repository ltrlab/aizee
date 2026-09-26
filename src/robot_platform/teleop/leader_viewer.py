"""Loopback display of pinned OpenRB inputs; no follower or motor commands."""
import argparse
import asyncio
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import urlsplit
import uuid

from .leader_bridge import CachedLeaderSampleStale, LatestLeaderInputs, sample_reply
from .openrb import OpenRBInputs


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_profile(profile):
    profile = copy.deepcopy(profile)
    if not isinstance(profile, dict) or profile.get('schema_version') != 1:
        raise ValueError('Expected leader viewer profile schema_version 1')
    if not isinstance(profile.get('id'), str) or not re.fullmatch(r'[a-z][a-z0-9_-]*', profile['id']):
        raise ValueError('Profile id must be a lowercase identifier')
    leaders = profile.get('leaders')
    if not isinstance(leaders, dict) or not 1 <= len(leaders) <= 8:
        raise ValueError('Configure one to eight leaders')
    serials = set()
    def vector(value):
        return isinstance(value, list) and len(value) == 3 and all(finite(v) for v in value)
    if ('reference_pose_description' in profile and
            (not isinstance(profile['reference_pose_description'], str) or not profile['reference_pose_description'].strip())):
        raise ValueError('Reference pose description must be nonempty text')
    if 'ground_z' in profile and not finite(profile['ground_z']):
        raise ValueError('Ground height must be finite')
    if 'front_direction' in profile:
        front = profile['front_direction']
        if not vector(front) or front[2] != 0 or math.hypot(*front[:2]) == 0:
            raise ValueError('Front direction must be a nonzero horizontal vector')
    if 'fixture' in profile:
        fixture = profile['fixture']
        if not isinstance(fixture, dict) or set(fixture) != {'rods'} or not isinstance(fixture['rods'], list):
            raise ValueError('Fixture must contain a rods list')
        for rod in fixture['rods']:
            if (not isinstance(rod, dict) or set(rod) != {'start_xyz', 'end_xyz', 'radius_m'}
                    or not vector(rod['start_xyz']) or not vector(rod['end_xyz'])
                    or rod['start_xyz'] == rod['end_xyz']
                    or not finite(rod['radius_m']) or rod['radius_m'] <= 0):
                raise ValueError('Fixture rods require distinct finite endpoints and positive radius')
    for name, leader in leaders.items():
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_-]*', name) or not isinstance(leader, dict):
            raise ValueError('Invalid leader name or definition')
        serial = leader.get('usb_serial')
        if not isinstance(serial, str) or not serial.strip() or serial in serials:
            raise ValueError('Leader USB serials must be nonempty and unique')
        serials.add(serial)
        if 'base_rpy' in leader and not vector(leader['base_rpy']):
            raise ValueError('Leader base_rpy must contain three finite angles')
        if leader.get('mount', 'pedestal') not in ('pedestal', 'overhead'):
            raise ValueError('Leader mount must be pedestal or overhead')
        if (not isinstance(leader.get('title'), str) or not leader['title'].strip()
                or not isinstance(leader.get('color'), str) or not re.fullmatch(r'#[0-9a-fA-F]{6}', leader['color'])
                or not vector(leader.get('base_xyz'))):
            raise ValueError('Leader title, hex color and finite base_xyz are required')
        joints = leader.get('joints')
        if not isinstance(joints, list) or len(joints) != 7:
            raise ValueError('Each leader requires exactly seven display joints')
        names = set()
        for joint in joints:
            if not isinstance(joint, dict):
                raise ValueError('Invalid display joint')
            joint_name = joint.get('name')
            if not isinstance(joint_name, str) or not joint_name.strip() or joint_name in names:
                raise ValueError('Display joint names must be nonempty and unique per leader')
            names.add(joint_name)
            for key in ('origin_rpy', 'link_xyz'):
                if key in joint and not vector(joint[key]):
                    raise ValueError(f'Joint {key} must contain three finite values')
            if (not isinstance(joint.get('label'), str) or not joint['label'].strip()
                    or not vector(joint.get('axis')) or math.hypot(*joint['axis']) == 0
                    or not vector(joint.get('offset_xyz')) or not finite(joint.get('length_m')) or joint['length_m'] < 0
                    or not finite(joint.get('zero_rad')) or type(joint.get('direction')) is not int
                    or joint['direction'] not in (-1, 1)):
                raise ValueError('Invalid display joint geometry, zero or direction')
    profile.setdefault('max_age_s', .05)
    profile.setdefault('max_skew_s', .02)
    if (not finite(profile['max_age_s']) or not 0 < profile['max_age_s'] <= .05
            or not finite(profile['max_skew_s']) or not 0 < profile['max_skew_s'] <= profile['max_age_s']):
        raise ValueError('Source age must be <=50 ms and skew no greater than age')
    return profile


class LeaderViewer:
    """One USB owner, age-preserving snapshots, and separate display calibration."""
    def __init__(self, profile, calibration=None, *, inputs=None, rate_hz=100., clock=time.perf_counter, wall_clock=time.time):
        if isinstance(profile, (str, Path)):
            raw = Path(profile).read_bytes()
            profile = json.loads(raw)
        else:
            raw = json.dumps(profile, sort_keys=True, allow_nan=False).encode()
        self.profile = validate_profile(profile)
        if not finite(rate_hz) or not 25 <= rate_hz <= 100:
            raise ValueError('Leader polling rate must be 25-100 Hz')
        self.rate_hz = rate_hz
        self.profile_sha256 = hashlib.sha256(raw).hexdigest()
        self.pins = {name: row['usb_serial'] for name, row in self.profile['leaders'].items()}
        self.clock, self.wall_clock = clock, wall_clock
        self.inputs = inputs if inputs is not None else LatestLeaderInputs(OpenRBInputs(self.profile, clock=clock), rate_hz=rate_hz, clock=clock)
        if self.inputs.pins != self.pins:
            raise ValueError('Input USB identities differ from display profile')
        self.calibration = Path(calibration).resolve() if calibration is not None else None
        self.mapping = {name: [{'name': joint['name'], 'zero_rad': joint['zero_rad'], 'direction': joint['direction']}
                              for joint in leader['joints']] for name, leader in self.profile['leaders'].items()}
        self.lock = threading.RLock()
        self.started, self.closed, self.error = False, False, None
        self.stale_error = None
        self.samples, self.sequence, self.updated_at = {}, 0, None
        self.epoch, self.action_token = uuid.uuid4().hex, secrets.token_hex(32)
        if self.calibration is not None and self.calibration.exists():
            self._load_calibration()

    def _load_calibration(self):
        data = json.loads(self.calibration.read_text())
        if (data.get('schema_version') != 1 or data.get('kind') != 'leader_viewer_display_calibration'
                or data.get('profile_sha256') != self.profile_sha256 or data.get('pins') != self.pins
                or data.get('display_only') is not True or data.get('runtime_approved') is not False
                or not isinstance(data.get('leaders'), dict) or set(data['leaders']) != set(self.pins)):
            raise ValueError('Saved display calibration does not match profile and pinned leaders')
        checked = {}
        for name, expected in self.mapping.items():
            rows = data['leaders'][name]
            if not isinstance(rows, list) or len(rows) != 7:
                raise ValueError('Incomplete saved display calibration')
            for row, joint in zip(rows, expected):
                if (not isinstance(row, dict) or set(row) != {'name', 'zero_rad', 'direction'}
                        or row['name'] != joint['name'] or not finite(row['zero_rad'])
                        or type(row['direction']) is not int or row['direction'] not in (-1, 1)):
                    raise ValueError('Invalid saved display calibration')
            checked[name] = copy.deepcopy(rows)
        self.mapping = checked

    def connect(self):
        with self.lock:
            if self.started or self.closed:
                raise RuntimeError('Start a new viewer session to reconnect')
            self.started = True
            try:
                self.inputs.connect()
                self.poll()
            except CachedLeaderSampleStale:
                # Opening succeeded. Keep the existing sampler; the next fresh
                # frame may restore this read-only display without reconnecting.
                pass
            except Exception as exc:
                self.error = str(exc) or type(exc).__name__
                self.inputs.close()

    def close(self):
        with self.lock:
            self.closed = True
            self.inputs.close()

    def poll(self):
        """Return fresh original LeaderSamples for display or nonce sampling."""
        with self.lock:
            if self.error or not self.started or self.closed:
                raise RuntimeError(self.error or 'Leader viewer is not connected')
            try:
                samples = self.inputs.poll()
                now = self.clock()
                if not isinstance(samples, dict) or set(samples) != set(self.pins):
                    raise ValueError('Incomplete leader frame')
                for name, sample in samples.items():
                    if (len(sample.raw_positions) != 7 or len(sample.positions_rad) != 7
                            or any(type(q) is not int or not -(1 << 31) <= q < (1 << 31) for q in sample.raw_positions)
                            or any(not finite(q) for q in sample.positions_rad)
                            or any(not math.isclose(q, tick * 2 * math.pi / 4096, rel_tol=1e-12, abs_tol=1e-12)
                                   for q, tick in zip(sample.positions_rad, sample.raw_positions))
                            or type(sample.sequence) is not int or not 0 < sample.sequence < (1 << 32)
                            or not finite(sample.sampled_at) or not finite(sample.received_at)
                            or not sample.sampled_at <= sample.received_at <= now
                            or len(sample.joystick) != 4 or any(type(v) is not int for v in sample.joystick)
                            or sample.joystick[3] not in (0, 1, 2)):
                        raise ValueError('Invalid or incomplete leader sample')
                    previous = self.samples.get(name)
                    if previous and sample != previous:
                        if not 0 < (sample.sequence - previous.sequence) & 0xffffffff < 0x80000000:
                            raise ValueError('Leader sample repeated, reversed or restarted')
                stamps = [s.sampled_at for s in samples.values()]
                if max(stamps) - min(stamps) > self.profile['max_skew_s']:
                    raise ValueError('Leader pair skew exceeded the profile')
                if any(now - stamp >= self.profile['max_age_s'] for stamp in stamps):
                    raise CachedLeaderSampleStale('Cached leader source is stale')
                if samples != self.samples:
                    self.sequence += 1
                    self.updated_at = self.wall_clock() - (now - max(s.received_at for s in samples.values()))
                self.samples = dict(samples)
                self.stale_error = None
                return dict(samples)
            except CachedLeaderSampleStale as exc:
                self.stale_error = str(exc)
                raise
            except Exception as exc:
                self.error = str(exc) or type(exc).__name__
                raise RuntimeError(self.error) from exc

    def profile_state(self):
        with self.lock:
            result = copy.deepcopy(self.profile)
            for name, joints in self.mapping.items():
                for output, mapping in zip(result['leaders'][name]['joints'], joints):
                    output.update(mapping)
            return {**result, 'profile_sha256': self.profile_sha256, 'action_token': self.action_token,
                    'rate_hz': self.rate_hz, 'display_only': True, 'runtime_approved': False}

    def state(self):
        with self.lock:
            if self.started and not self.closed and not self.error:
                try:
                    self.poll()
                except (CachedLeaderSampleStale, RuntimeError):
                    pass
            now = self.clock()
            leaders = {}
            for name, sample in self.samples.items():
                joints = [{**mapping, 'label': definition['label'],
                           'value_rad': mapping['direction'] * (raw - mapping['zero_rad'])}
                          for raw, mapping, definition in zip(sample.positions_rad, self.mapping[name], self.profile['leaders'][name]['joints'])]
                leaders[name] = {'usb_serial': self.pins[name], 'sequence': sample.sequence,
                    'age_ms': (now - sample.sampled_at) * 1000, 'raw_ticks': list(sample.raw_positions),
                    'raw_rad': list(sample.positions_rad), 'positions_rad': [j['value_rad'] for j in joints],
                    'joints': joints, 'joystick_present': sample.joystick[3] == 0}
            stamps = [sample.sampled_at for sample in self.samples.values()]
            return {'schema_version': 1, 'status': 'fault' if self.error or self.closed else 'stale' if self.stale_error else 'live' if self.samples else 'waiting',
                    'error': self.error or ('Viewer closed' if self.closed else self.stale_error), 'sequence': self.sequence,
                    'updated_at': self.updated_at, 'age_ms': (now - min(stamps)) * 1000 if stamps else None,
                    'skew_ms': (max(stamps) - min(stamps)) * 1000 if stamps else None, 'leaders': leaders,
                    'skipped_pairs': getattr(self.inputs, 'skipped', None),
                    'last_skip': getattr(self.inputs, 'last_skip', None),
                    'rate_hz': self.rate_hz, 'display_only': True, 'runtime_approved': False}

    def action(self, payload):
        if not isinstance(payload, dict) or set(payload) - {'action', 'leader', 'channel'}:
            raise ValueError('Expected action and optional leader/channel')
        action = payload.get('action')
        if action not in ('reference', 'flip', 'reset', 'save'):
            raise ValueError('Unknown display action')
        leader, channel = payload.get('leader'), payload.get('channel')
        if ('leader' in payload and (not isinstance(leader, str) or leader not in self.pins)
                or 'channel' in payload and (leader is None or type(channel) is not int or not 0 <= channel < 7)
                or action == 'flip' and (leader is None or channel is None)
                or action == 'save' and set(payload) != {'action'}):
            raise ValueError('Invalid leader/channel selection')
        with self.lock:
            selected = [leader] if leader is not None else list(self.pins)
            channels = [channel] if channel is not None else range(7)
            if action == 'reference':
                samples = self.poll()  # Complete, source-fresh frame required.
                for name in selected:
                    for index in channels:
                        self.mapping[name][index]['zero_rad'] = samples[name].positions_rad[index]
            elif action == 'flip':
                self.mapping[leader][channel]['direction'] *= -1
            elif action == 'reset':
                for name in selected:
                    for index in channels:
                        default = self.profile['leaders'][name]['joints'][index]
                        self.mapping[name][index].update(zero_rad=default['zero_rad'], direction=default['direction'])
            elif action == 'save':
                if self.calibration is None:
                    raise ValueError('No display calibration output was selected')
                data = {'schema_version': 1, 'kind': 'leader_viewer_display_calibration',
                        'profile_sha256': self.profile_sha256, 'pins': self.pins, 'leaders': self.mapping,
                        'display_only': True, 'runtime_approved': False}
                self.calibration.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.calibration.with_name(self.calibration.name + '.' + uuid.uuid4().hex + '.tmp')
                try:
                    with temporary.open('x', encoding='utf-8') as stream:
                        json.dump(data, stream, indent=2, allow_nan=False)
                        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
                    os.replace(temporary, self.calibration)
                finally:
                    temporary.unlink(missing_ok=True)
            return {'ok': True, 'state': self.state()}


def create_app(profile, calibration=None, *, inputs=None, rate_hz=100., clock=time.perf_counter, wall_clock=time.time):
    from aiohttp import web
    viewer = LeaderViewer(profile, calibration, inputs=inputs, rate_hz=rate_hz, clock=clock, wall_clock=wall_clock)
    static = Path(__file__).resolve().parents[1] / 'viewer' / 'static'

    @web.middleware
    async def boundary(request, handler):
        if urlsplit('http://' + request.host).hostname not in {'127.0.0.1', 'localhost'}:
            raise web.HTTPForbidden(text='Leader viewer is loopback-only')
        if request.method == 'POST':
            if request.headers.get('Origin') not in (None, f'{request.scheme}://{request.host}'):
                raise web.HTTPForbidden(text='Cross-origin request rejected')
            if request.content_type != 'application/json':
                raise web.HTTPUnsupportedMediaType()
            if not secrets.compare_digest(request.headers.get('X-Leader-Token', ''), viewer.action_token):
                raise web.HTTPForbidden(text='Missing or invalid leader session token')
        try:
            response = await handler(request)
        except CachedLeaderSampleStale as exc:
            # Transient: the sampler still runs. Clients must stop, not reconnect.
            response = web.json_response({'error': str(exc), 'transient': True}, status=409)
        except (ValueError, TypeError, KeyError) as exc:
            response = web.json_response({'error': str(exc)}, status=400)
        except RuntimeError as exc:
            response = web.json_response({'error': str(exc)}, status=409)
        except OSError:
            response = web.json_response({'error': 'Could not save display calibration'}, status=500)
        response.headers.update({'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff'})
        return response

    app = web.Application(middlewares=[boundary], client_max_size=16384)
    async def start(_app):
        await asyncio.to_thread(viewer.connect)
    async def close(_app):
        await asyncio.to_thread(viewer.close)
    app.on_startup.append(start)
    app.on_cleanup.append(close)
    async def state(_request):
        return web.json_response(viewer.state())
    async def profile_info(_request):
        return web.json_response(viewer.profile_state())
    async def action(request):
        return web.json_response(viewer.action(await request.json()))
    async def sample(request):
        return web.json_response(sample_reply(await request.json(), viewer, viewer.epoch, clock=clock))
    app.router.add_get('/api/state', state)
    app.router.add_get('/api/profile', profile_info)
    app.router.add_post('/api/action', action)
    app.router.add_post('/api/sample', sample)
    for route, filename in {'/': 'leaders.html', '/leaders.js': 'leaders.js', '/leaders.css': 'leaders.css',
                            '/vendor/three.module.js': 'vendor/three.module.js', '/vendor/OrbitControls.js': 'vendor/OrbitControls.js'}.items():
        async def asset(_request, filename=filename):
            return web.FileResponse(static / filename)
        app.router.add_get(route, asset)
    return app


def main(argv=None):
    from aiohttp import web
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True, help='Separate display-only calibration JSON')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--rate', type=float, default=100., help='Independent USB polling rate, 25-100 Hz (default: 100)')
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error('Port must be in 1..65535')
    web.run_app(create_app(args.profile, args.calibration, rate_hz=args.rate), host='127.0.0.1', port=args.port)


if __name__ == '__main__':
    main()
