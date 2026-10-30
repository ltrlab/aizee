"""The head's setup in Helm (helm.head_setup) on simulated servos left as the bench left the real ones: read limp,
set the reference (both servos centred; both offsets restored if either fails), flip and accept directions, sweep
and save the limits (with the servos' own limits just outside), stale limits after a new reference, the 3D view's
neck from the readings, and Helm's endpoints for each step."""
import asyncio
import json
from pathlib import Path
import time

import pytest

from robot_platform.helm import head_setup as head_setup_module
from robot_platform.helm.head_setup import MARGIN_RAD, RAD_PER_STEP, HeadSetup
from robot_platform.neck import head
from robot_platform.neck.feetech import FeetechBus
from robot_platform.neck.sim import SimulatedServo, realtime_bus

ROOT = Path(__file__).resolve().parents[2]
PROFILE = head.load_profile(ROOT / head.PROFILE)
ASSET = 'minerva-production-01'


@pytest.fixture(autouse=True)
def quick_reader(monkeypatch):
    monkeypatch.setattr(head_setup_module, 'PERIOD_S', .01)


def bench_servos():
    """Pan id 1 and tilt id 2 with the offsets and limits the bench found (set straight into memory)."""
    pan, tilt = SimulatedServo(1, position=2224), SimulatedServo(2, position=1307)
    for servo, (low, high), offset in ((pan, (1050, 3730), 2047), (tilt, (360, 2145), -219)):
        servo.set('Min_Position_Limit', low)
        servo.set('Max_Position_Limit', high)
        servo.set('Homing_Offset', offset)
    return pan, tilt


@pytest.fixture
def rig(tmp_path):
    pan, tilt = bench_servos()
    bus = realtime_bus([pan, tilt])
    setup = HeadSetup(tmp_path / 'operator', ASSET, tmp_path / 'runs', profile=PROFILE,
                      open_bus=lambda: FeetechBus(bus, timeout_s=.005))
    yield setup, pan, tilt
    setup.close()


def settle(setup, readings=3):
    """Wait for `readings` new polls (a held end needs two readings that agree)."""
    start = setup.sequence
    deadline = time.monotonic() + 3
    while setup.sequence < start + readings:
        assert time.monotonic() < deadline, 'the reader stopped polling'
        time.sleep(.005)


def hand(setup, servo, steps):
    servo.move_by_hand(steps)
    settle(setup)


def test_the_reader_reads_both_servos_limp_and_steps_wait_for_a_reference(rig):
    setup, pan, tilt = rig
    assert setup.snapshot()['reading'] is False
    with pytest.raises(ValueError, match='read the head first'):
        setup.set_reference()
    setup.start()
    settle(setup)
    state = setup.snapshot()
    assert state['fresh'] and [s['steps'] for s in state['servos']] == [2224, 1307]
    assert all(s['torque'] is False and s['alarms'] == [] for s in state['servos'])  # Never enabled.
    assert state['reference'] is None and state['rows'] == [] and setup.joints() is None
    for step in (lambda: setup.limit('confirm', 'head_pan'), lambda: setup.direction('accept', 'head_pan'),
                 setup.save_limits):
        with pytest.raises(ValueError, match="reference first"):
            step()
    setup.stop()
    assert not setup.snapshot()['reading'] and not setup.fresh()


def test_setting_the_reference_centres_both_servos_and_the_view_follows_them(rig, tmp_path):
    setup, pan, tilt = rig
    setup.start()
    settle(setup)
    document = setup.set_reference()
    assert [pan.get('Present_Position'), tilt.get('Present_Position')] == [2048, 2048]
    assert pan.get('Lock') == tilt.get('Lock') == 1 and pan.eeprom_writes_locked == 0
    saved = json.loads((tmp_path / 'operator' / f'{ASSET}-head-reference.json').read_text())
    assert saved == document and [m['uid'] for m in saved['motors']] == ['5A68010635/1', '5A68010635/2']
    assert all(m['sign'] == 1 and not m['direction_verified'] for m in saved['motors'])
    settle(setup)
    assert setup.joints() == pytest.approx({'neck_z': 0, 'neck_y': 0})
    hand(setup, pan, 2248)  # 200 steps one way: the 3D view's neck_z follows.
    assert setup.joints()['neck_z'] == pytest.approx(200 * RAD_PER_STEP)
    state = setup.snapshot()
    assert [row['name'] for row in state['rows']] == ['head_pan', 'head_tilt']
    assert state['rows'][0]['current_rad'] == pytest.approx(200 * RAD_PER_STEP) and state['rows'][0]['arm'] == 'head'


def test_a_centring_that_fails_part_way_puts_both_offsets_back(rig, tmp_path):
    setup, pan, tilt = rig
    tilt.set('Homing_Offset', 0)
    tilt.set('Present_Position', 0)  # Exactly 0 cannot be centred (the offset holds ±2047).
    setup.start()
    settle(setup)
    with pytest.raises(Exception, match='turn the joint a little'):
        setup.set_reference()
    assert pan.get('Homing_Offset') == 2047 and pan.get('Present_Position') == 2224  # The pan's was centred: undone.
    assert tilt.get('Homing_Offset') == 0 and setup.reference is None
    assert not (tmp_path / 'operator' / f'{ASSET}-head-reference.json').exists()


def test_directions_flip_and_accept_in_the_reference_and_carry_over_to_a_new_one(rig):
    setup, pan, tilt = rig
    setup.start()
    settle(setup)
    setup.set_reference()
    hand(setup, pan, 2148)
    setup.direction('flip', 'head_pan')
    assert setup.joints()['neck_z'] == pytest.approx(-100 * RAD_PER_STEP)  # The view turns the other way now.
    setup.direction('accept', 'head_pan')
    setup.direction('accept', 'head_tilt')
    motors = {m['name']: m for m in setup.reference['motors']}
    assert (motors['head_pan']['sign'], motors['head_pan']['direction_verified']) == (-1, True)
    assert setup.snapshot()['reference']['directions_verified'] is True
    with pytest.raises(ValueError, match='flip or accept'):
        setup.direction('swap', 'head_pan')
    hand(setup, pan, 2048)
    again = {m['name']: m for m in setup.set_reference()['motors']}  # The same servos: still flipped and checked.
    assert (again['head_pan']['sign'], again['head_pan']['direction_verified']) == (-1, True)


def test_the_limits_sweep_saves_both_joints_and_sets_the_servos_own_limits(rig, tmp_path):
    setup, pan, tilt = rig
    setup.start()
    settle(setup)
    setup.set_reference()
    for servo, ends in ((pan, (1648, 2548)), (tilt, (1748, 2448))):
        for steps in (*ends, 2048):
            hand(setup, servo, steps)
    setup.limit('confirm', 'head_pan')
    with pytest.raises(ValueError, match='Confirm Head tilt first'):
        setup.save_limits()
    setup.limit('confirm', 'head_tilt')
    setup.save_limits()
    saved = json.loads((tmp_path / 'operator' / f'{ASSET}-head-limits.json').read_text())
    assert saved['reference_id'] == setup.reference['id']
    joints = saved['joints']
    assert (joints['head_pan']['min_steps'], joints['head_pan']['max_steps']) == (1648, 2548)
    assert (joints['head_tilt']['min_steps'], joints['head_tilt']['max_steps']) == (1748, 2448)
    margin = round(MARGIN_RAD / RAD_PER_STEP + .5)
    assert (pan.get('Min_Position_Limit'), pan.get('Max_Position_Limit')) == (1648 - margin, 2548 + margin)
    assert (tilt.get('Min_Position_Limit'), tilt.get('Max_Position_Limit')) == (1748 - margin, 2448 + margin)
    assert pan.eeprom_writes_locked == tilt.eeprom_writes_locked == 0
    state = setup.snapshot()
    assert state['limits']['matches_reference'] and state['pending'] == 0
    assert state['rows'][0]['saved'] == pytest.approx({'min_rad': -400 * RAD_PER_STEP, 'max_rad': 500 * RAD_PER_STEP})
    # A new reference makes them stale: they name the old one, and the cards show no saved range.
    setup.set_reference()
    state = setup.snapshot()
    assert state['limits']['matches_reference'] is False and state['rows'][0]['saved'] is None
    replaced = list((tmp_path / 'runs').glob('head-setup-*/replaced-*'))
    assert replaced, 'the replaced reference is kept'


class Teleop:
    def __init__(self):
        self.active = False

    def guide(self):
        return None

    def snapshot(self):
        return {'phase': 'stopped'}

    def stop(self):
        pass


def test_helm_serves_each_step_and_refuses_to_read_while_a_session_holds_the_motors(rig, tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.cli import SimulationLink
    from robot_platform.helm.server import create_app
    from robot_platform.helm.session import HelmSession
    setup, pan, tilt = rig
    helm = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'helm-runs',
                       limits=tmp_path / 'limits.json')
    teleop = Teleop()

    async def run():
        from robot_platform.helm.display_model import DisplayModel
        from robot_platform.models import ModelProfile, UrdfModel
        from robot_platform.ops.release import HEAD_MODEL
        display = DisplayModel(helm.model, UrdfModel(ModelProfile.load(ROOT / HEAD_MODEL)))
        app = create_app(helm, SimulationLink(), teleop=teleop, token='secret', head_setup=setup, display=display)
        async with TestClient(TestServer(app)) as client:
            post = lambda path, payload: client.post(path, json=payload, headers={'X-Helm-Token': 'secret'})
            state = await (await client.get('/api/helm/state')).json()
            assert state['head_setup']['available'] and not state['head_setup']['reading']
            teleop.active = True  # A session holds the motors: no reading the head now.
            refused = await post('/api/helm/head/read', {'on': True})
            assert refused.status == 422 and 'once teleop has ended' in (await refused.json())['error']
            teleop.active = False
            assert (await (await post('/api/helm/head/read', {'on': True})).json())['head_setup']['reading']
            await asyncio.get_running_loop().run_in_executor(None, settle, setup)
            state = await (await post('/api/helm/head/reference', {})).json()
            assert state['head_setup']['reference'] is not None and pan.get('Present_Position') == 2048
            # The 3D view's head follows each reading, each a pose of its own, even on a still pose.
            first = await (await client.get('/api/calibration-guide')).json()
            pan.move_by_hand(2148)
            await asyncio.get_running_loop().run_in_executor(None, settle, setup)
            moved = await (await client.get('/api/calibration-guide')).json()
            assert moved['stream_id'] != first['stream_id'] and moved['view'] == 'keep'
            assert moved['positions']['neck_z'] == pytest.approx(100 * RAD_PER_STEP)
            assert 'read by its setup, the servos limp' in moved['badge']
            pan.move_by_hand(2048)
            await asyncio.get_running_loop().run_in_executor(None, settle, setup)
            state = await (await post('/api/helm/head/direction', {'action': 'flip', 'joint': 'head_tilt'})).json()
            assert state['head_setup']['rows'][1]['direction']['sign'] == -1
            for servo, ends in ((pan, (1748, 2348)), (tilt, (1848, 2248))):
                for steps in (*ends, 2048):
                    servo.move_by_hand(steps)
                    await asyncio.get_running_loop().run_in_executor(None, settle, setup)
            for motor in ('head_pan', 'head_tilt'):
                state = await (await post('/api/helm/head/limits', {'action': 'confirm', 'motor': motor})).json()
            assert state['head_setup']['pending'] == 2
            saved = await post('/api/helm/head/limits/save', {})
            assert saved.status == 200 and saved.headers['X-Helm-Saved'].endswith(f'{ASSET}-head-limits.json')
            assert (await saved.json())['head_setup']['limits']['matches_reference']
            assert (await (await post('/api/helm/head/read', {'on': False})).json())['head_setup']['reading'] is False
            bad = await post('/api/helm/head/limits', {'action': 'confirm'})
            assert bad.status == 422
    try:
        asyncio.run(run())
    finally:
        helm.close()


def test_helm_without_the_head_board_says_where_the_head_is_set_up(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from robot_platform.helm.cli import SimulationLink
    from robot_platform.helm.server import create_app
    from robot_platform.helm.session import HelmSession
    helm = HelmSession(ROOT / 'config/devices/minerva-joint-calibration.json', runs=tmp_path / 'helm-runs',
                       limits=tmp_path / 'limits.json')

    async def run():
        async with TestClient(TestServer(create_app(helm, SimulationLink(), token='secret'))) as client:
            assert (await (await client.get('/api/helm/state')).json())['head_setup'] is None
            refused = await client.post('/api/helm/head/read', json={'on': True}, headers={'X-Helm-Token': 'secret'})
            assert refused.status == 422 and 'Helm on Minerva' in (await refused.json())['error']
    try:
        asyncio.run(run())
    finally:
        helm.close()
