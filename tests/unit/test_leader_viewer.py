import asyncio
import copy
from dataclasses import replace
import json
import math

import pytest

from robot_platform.teleop.leader_bridge import CachedLeaderSampleStale, LeaderSnapshotClient, sample_reply
from robot_platform.teleop.leader_viewer import LeaderViewer, create_app, validate_profile
from robot_platform.teleop.openrb import LeaderSample


def profile():
    return {'schema_version': 1, 'id': 'fixture', 'max_age_s': .05, 'max_skew_s': .02,
            'leaders': {name: {'usb_serial': serial, 'title': name.title(), 'color': '#4488cc',
                'base_xyz': [0., 0., 0.], 'joints': [
                    {'name': f'joint_{i}', 'label': f'Joint {i+1}', 'axis': [0., 0., 1.],
                     'offset_xyz': [0., 0., .1], 'length_m': .1, 'zero_rad': .1, 'direction': 1}
                    for i in range(7)]} for name, serial in [('left', 'L'), ('right', 'R')]}}


class Inputs:
    pins = {'left': 'L', 'right': 'R'}

    def __init__(self, clock):
        self.connects = self.closes = self.polls = 0
        self.failure = None
        self.samples = {name: LeaderSample(tuple(q*2*math.pi/4096 for q in ticks), ticks,
            1, clock()-.01-offset, clock()-.002, (0, 0, 1, 1), 100, 108)
            for name, ticks, offset in [('left', tuple(range(100, 107)), 0.), ('right', tuple(range(200, 207)), .005)]}

    def connect(self):
        self.connects += 1

    def close(self):
        self.closes += 1

    def poll(self):
        self.polls += 1
        if self.failure:
            if isinstance(self.failure, Exception):
                raise self.failure
            raise ValueError(self.failure)
        return dict(self.samples)


def viewer_fixture(calibration=None):
    now = [1000.]
    clock = lambda: now[0]
    source = Inputs(clock)
    viewer = LeaderViewer(profile(), calibration, inputs=source, clock=clock, wall_clock=lambda: 5000.)
    return viewer, source, now


def test_hanging_geometry_is_profile_data_and_keeps_input_mapping_separate():
    data = profile()
    data.update(reference_pose_description='Hang both arms down', ground_z=0,
                front_direction=[0, -1, 0], fixture={'rods': [
                    {'start_xyz': [0, 0, 0], 'end_xyz': [0, 0, 1], 'radius_m': .01}]})
    data['leaders']['left'].update(mount='overhead', base_rpy=[0, .2, -.3])
    data['leaders']['left']['joints'][0].update(link_xyz=[.04, 0, -.1], origin_rpy=[.1, 0, .2])
    source = Inputs(lambda: 1000.)
    viewer = LeaderViewer(data, inputs=source, clock=lambda: 1000.)
    viewer.connect()
    current = viewer.profile_state()
    assert current['leaders']['left']['joints'][0]['link_xyz'] == [.04, 0, -.1]
    assert current['leaders']['left']['base_rpy'] == [0, .2, -.3]
    assert viewer.poll()['left'] == source.samples['left']
    assert current['display_only'] and current['runtime_approved'] is False


@pytest.mark.parametrize('change', ['base_rotation', 'origin_rotation', 'link', 'mount',
                                   'front', 'ground', 'fixture_radius', 'fixture_endpoints'])
def test_bad_optional_geometry_is_rejected(change):
    data = profile()
    leader = data['leaders']['left']
    if change == 'base_rotation': leader['base_rpy'] = [0, math.nan, 0]
    elif change == 'origin_rotation': leader['joints'][0]['origin_rpy'] = [0, 0]
    elif change == 'link': leader['joints'][0]['link_xyz'] = [0, 0, math.inf]
    elif change == 'mount': leader['mount'] = 'unknown'
    elif change == 'front': data['front_direction'] = [0, 0, 1]
    elif change == 'ground': data['ground_z'] = True
    else:
        rod = {'start_xyz': [0, 0, 0], 'end_xyz': [0, 0, 1], 'radius_m': .01}
        data['fixture'] = {'rods': [rod]}
        if change == 'fixture_radius': rod['radius_m'] = -1
        else: rod['end_xyz'] = rod['start_xyz']
    with pytest.raises(ValueError): validate_profile(data)


def test_display_uses_raw_source_angles_and_does_not_freshen_cached_frames():
    viewer, source, now = viewer_fixture()
    assert viewer.state()['status'] == 'waiting' and source.connects == 0
    viewer.connect()
    first = viewer.state()
    now[0] += .01
    second = viewer.state()
    assert first['status'] == second['status'] == 'live'
    assert first['sequence'] == second['sequence'] == 1
    assert first['updated_at'] == second['updated_at']
    assert second['age_ms'] == pytest.approx(first['age_ms'] + 10)
    assert second['skew_ms'] == pytest.approx(5)
    row = second['leaders']['left']
    assert row['raw_ticks'] == list(range(100, 107))
    assert row['raw_rad'][0] == pytest.approx(100*2*math.pi/4096)
    assert row['positions_rad'][0] == pytest.approx(row['raw_rad'][0]-.1)
    assert not row['joystick_present'] and not second['runtime_approved']
    assert first['rate_hz'] == viewer.profile_state()['rate_hz'] == 100


@pytest.mark.parametrize('failure', ['missing', 'partial', 'nonfinite', 'wrong_ticks', 'sequence', 'skew', 'transport', 'timeout', 'board_stale', 'future', 'timestamp_nan'])
def test_source_failures_latch_and_never_reconnect_or_reference_old_data(failure):
    viewer, source, now = viewer_fixture()
    viewer.connect()
    original = copy.deepcopy(viewer.mapping)
    current = source.samples['left']
    if failure == 'missing': del source.samples['right']
    elif failure == 'partial': source.samples['left'] = replace(current, raw_positions=(1,))
    elif failure == 'nonfinite': source.samples['left'] = replace(current, positions_rad=(math.nan,)*7)
    elif failure == 'wrong_ticks': source.samples['left'] = replace(current, positions_rad=(0.,)*7)
    elif failure == 'sequence': source.samples['left'] = replace(current, sequence=0)
    elif failure == 'skew': source.samples['right'] = replace(source.samples['right'], sampled_at=now[0]-.04)
    elif failure == 'transport': source.failure = 'CRC frame error'
    elif failure == 'timeout': source.failure = TimeoutError('USB read timed out')
    elif failure == 'board_stale': source.failure = 'OpenRB servo data is incomplete, faulted or stale'
    elif failure == 'future': source.samples['left'] = replace(current, sampled_at=now[0]+.01)
    elif failure == 'timestamp_nan': source.samples['left'] = replace(current, sampled_at=math.nan)
    state = viewer.state()
    assert state['status'] == 'fault' and state['error']
    with pytest.raises(RuntimeError):
        viewer.action({'action': 'reference'})
    source.failure = None
    source.samples = Inputs(lambda: now[0]).samples
    assert viewer.state()['status'] == 'fault'
    assert source.connects == 1 and viewer.mapping == original


@pytest.mark.parametrize('cached_exception', [False, True])
def test_display_and_input_client_recover_from_cache_age_without_reconnecting(cached_exception):
    viewer, source, now = viewer_fixture()
    viewer.connect()
    initial = viewer.state()
    mappings = copy.deepcopy(viewer.mapping)
    client = LeaderSnapshotClient(lambda request: sample_reply(request, viewer, viewer.epoch, clock=lambda: now[0]),
                                  profile(), clock=lambda: now[0])
    client.poll()
    now[0] += .051
    if cached_exception:
        source.failure = CachedLeaderSampleStale('Cached leader source is stale')
    with pytest.raises(CachedLeaderSampleStale):
        client.poll()
    state = viewer.state()
    assert state['status'] == 'stale' and state['error']
    assert state['sequence'] == initial['sequence'] and state['updated_at'] == initial['updated_at']
    assert state['leaders']['left']['positions_rad'] == initial['leaders']['left']['positions_rad']
    with pytest.raises(CachedLeaderSampleStale):
        viewer.action({'action': 'reference'})
    assert viewer.mapping == mappings and viewer.error is None
    source.failure = None
    source.samples = {name: replace(sample, sequence=2) for name, sample in Inputs(lambda: now[0]).samples.items()}
    recovered = viewer.state()
    assert recovered['status'] == 'live' and recovered['error'] is None
    assert recovered['sequence'] == initial['sequence'] + 1
    assert source.connects == 1 and source.closes == 0
    # The input client did not latch; re-engaging stays an explicit lifecycle action.
    assert client.fault is None
    assert client.poll()['left'].sequence == 2


def test_initial_cache_expiry_does_not_close_a_running_sampler():
    viewer, source, now = viewer_fixture()
    source.failure = CachedLeaderSampleStale('Cached leader source is stale')
    viewer.connect()
    assert viewer.state()['status'] == 'stale'
    source.failure = None
    assert viewer.state()['status'] == 'live'
    assert source.connects == 1 and source.closes == 0


def test_reference_flip_reset_and_save_are_separate_display_calibration(tmp_path):
    output = tmp_path / 'display.json'
    viewer, source, _ = viewer_fixture(output)
    viewer.connect()
    initial_raw = source.samples['left'].positions_rad
    viewer.action({'action': 'reference', 'leader': 'left'})
    assert viewer.state()['leaders']['left']['positions_rad'] == [0.]*7
    assert viewer.mapping['right'][0]['zero_rad'] == .1
    viewer.action({'action': 'flip', 'leader': 'left', 'channel': 0})
    assert viewer.mapping['left'][0] == {'name': 'joint_0', 'zero_rad': initial_raw[0], 'direction': -1}
    assert viewer.profile_state()['leaders']['left']['joints'][0]['direction'] == -1
    viewer.action({'action': 'save'})
    saved = json.loads(output.read_text())
    assert saved['pins'] == source.pins and saved['display_only'] and saved['runtime_approved'] is False
    assert saved['profile_sha256'] == viewer.profile_sha256
    assert not list(tmp_path.glob('*.tmp'))
    loaded = LeaderViewer(profile(), output, inputs=source)
    assert loaded.mapping == viewer.mapping
    assert source.connects == 1  # Loading display configuration never opens USB.
    viewer.action({'action': 'reset', 'leader': 'left', 'channel': 0})
    assert viewer.mapping['left'][0]['zero_rad'] == .1 and viewer.mapping['left'][0]['direction'] == 1
    assert viewer.mapping['left'][1]['zero_rad'] == initial_raw[1]
    assert source.samples['left'].positions_rad == initial_raw


@pytest.mark.parametrize('change', ['profile', 'serial', 'approval', 'names', 'partial'])
def test_saved_calibration_cannot_be_applied_to_another_profile_or_source(tmp_path, change):
    output = tmp_path / 'display.json'
    viewer, source, _ = viewer_fixture(output)
    viewer.action({'action': 'save'})
    data = json.loads(output.read_text())
    if change == 'profile': data['profile_sha256'] = 'other'
    elif change == 'serial': data['pins']['left'] = 'other'
    elif change == 'approval': data['runtime_approved'] = True
    elif change == 'names': data['leaders']['left'][0]['name'] = 'other'
    elif change == 'partial': data['leaders']['left'].pop()
    output.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        LeaderViewer(profile(), output, inputs=source)
    assert source.connects == 0


@pytest.mark.parametrize('payload', [
    {'action': 'enable'}, {'action': 'flip'}, {'action': 'reference', 'leader': 'unknown'},
    {'action': 'reference', 'channel': 0}, {'action': 'flip', 'leader': 'left', 'channel': True},
    {'action': 'flip', 'leader': 'left', 'channel': 7}, {'action': 'save', 'leader': 'left'},
    {'action': 'reference', 'path': 'anything'}, {'action': 'reset', 'leader': None},
])
def test_actions_cannot_address_hardware_or_arbitrary_files(payload):
    viewer, source, _ = viewer_fixture()
    before = copy.deepcopy(viewer.mapping)
    with pytest.raises(ValueError): viewer.action(payload)
    assert viewer.mapping == before and source.connects == 0


@pytest.mark.parametrize('change', ['duplicate_usb', 'six_joints', 'axis', 'direction', 'zero', 'age', 'skew'])
def test_profile_rejects_incomplete_geometry_identities_or_loose_freshness(change):
    data = profile()
    if change == 'duplicate_usb': data['leaders']['right']['usb_serial'] = 'L'
    elif change == 'six_joints': data['leaders']['left']['joints'].pop()
    elif change == 'axis': data['leaders']['left']['joints'][0]['axis'] = [0, 0, 0]
    elif change == 'direction': data['leaders']['left']['joints'][0]['direction'] = True
    elif change == 'zero': data['leaders']['left']['joints'][0]['zero_rad'] = math.inf
    elif change == 'age': data['max_age_s'] = .1
    elif change == 'skew': data['max_skew_s'] = .06
    with pytest.raises(ValueError): validate_profile(data)


def test_nonce_sampling_shares_usb_owner_and_preserves_source_age_and_raw_pose():
    viewer, source, now = viewer_fixture()
    viewer.connect()
    viewer.action({'action': 'reference', 'leader': 'left'})
    replies = []
    def exchange(request):
        reply = sample_reply(request, viewer, viewer.epoch, clock=lambda: now[0])
        replies.append(reply)
        return reply
    client = LeaderSnapshotClient(exchange, profile(), clock=lambda: now[0])
    received = client.poll()
    assert received['left'].raw_positions == source.samples['left'].raw_positions
    assert received['left'].positions_rad[0] != 0.  # Display zeros never change teleop input.
    assert replies[0]['leaders']['left']['age_s'] == pytest.approx(.01)
    now[0] += .005
    with pytest.raises(ValueError, match='repeated'):
        client.poll()
    assert replies[1]['leaders']['left']['age_s'] == pytest.approx(.015)
    assert source.connects == 1


def test_http_boundaries_actions_sampling_and_fixed_static_routes(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    now = [1000.]
    source = Inputs(lambda: now[0])
    async def run():
        app = create_app(profile(), tmp_path / 'display.json', inputs=source, clock=lambda: now[0])
        async with TestClient(TestServer(app)) as client:
            response = await client.get('/api/profile')
            data = await response.json()
            assert response.headers['Cache-Control'] == 'no-store'
            headers = {'X-Leader-Token': data['action_token']}
            assert (await client.get('/api/state', headers={'Host': 'evil.example'})).status == 403
            assert (await client.post('/api/action', json={'action': 'reference'})).status == 403
            assert (await client.post('/api/action', json={'action': 'reference'}, headers={**headers, 'Origin': 'https://evil.example'})).status == 403
            assert (await client.post('/api/action', data='action=reference', headers=headers)).status == 415
            assert (await client.post('/api/action', json={'action': 'enable'}, headers=headers)).status == 400
            action = await client.post('/api/action', json={'action': 'reference', 'leader': 'left'}, headers=headers)
            assert action.status == 200 and (await action.json())['state']['leaders']['left']['positions_rad'] == [0.]*7
            nonce = 'a'*32
            sample = await client.post('/api/sample', json={'type': 'sample', 'nonce': nonce}, headers=headers)
            value = await sample.json()
            assert sample.status == 200 and value['nonce'] == nonce and value['protocol_version'] == 1
            assert value['leaders']['left']['age_s'] == pytest.approx(.01)
            assert (await client.post('/api/sample', json={'type': 'enable', 'nonce': nonce}, headers=headers)).status == 400
            assert (await client.get('/vendor/three.module.js')).status == 200
            assert (await client.get('/profile.json')).status == 404
            assert (await client.get('/vendor/not-allowlisted')).status == 404
            now[0] += .1
            state = await (await client.get('/api/state')).json()
            assert state['status'] == 'stale' and state['age_ms'] > 100
            assert (await client.post('/api/action', json={'action': 'reference'}, headers=headers)).status == 409
            assert (await client.post('/api/sample', json={'type': 'sample', 'nonce': nonce}, headers=headers)).status == 409
            source.samples = {name: replace(sample, sequence=2) for name, sample in Inputs(lambda: now[0]).samples.items()}
            recovered = await (await client.get('/api/state')).json()
            assert recovered['status'] == 'live' and recovered['error'] is None
            sample = await client.post('/api/sample', json={'type': 'sample', 'nonce': 'b'*32}, headers=headers)
            assert sample.status == 200 and (await sample.json())['leaders']['left']['sequence'] == 2
        assert source.connects == 1 and source.closes == 1
    asyncio.run(run())


def test_connect_failure_remains_visible_without_auto_reconnect():
    viewer, source, _ = viewer_fixture()
    source.failure = 'Missing servo data'
    viewer.connect()
    assert viewer.state()['status'] == 'fault'
    source.failure = None
    assert viewer.state()['status'] == 'fault'
    with pytest.raises(RuntimeError): viewer.connect()
    assert source.connects == source.closes == 1
