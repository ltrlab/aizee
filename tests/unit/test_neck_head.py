"""The head on the arms' pipeline: its settings from the arms' derivation, its receiver's configuration, and a
session on the real receiver core (the harness) under the direct policy."""
import math
from pathlib import Path

import pytest

from robot_platform.control.approach import OVERTRAVEL_RAD
from robot_platform.control.guarded import GuardedControllerClient, GuardRejected
from robot_platform.models import ModelProfile, UrdfModel
from robot_platform.neck import head

ROOT = Path(__file__).resolve().parents[2]
PROFILE = head.load_profile(ROOT / 'config/devices/minerva-head.json')
MODEL = UrdfModel(ModelProfile.load(ROOT / PROFILE['model_profile']))


def derived(**changes):
    reference = head.virtual_reference(PROFILE)
    for motor in reference['motors']:
        motor.update(changes.get(motor['name'], {}))
    return reference, head.derive(PROFILE, reference, head.virtual_limits(reference), MODEL)


def test_the_head_derives_through_the_arms_receiver_derivation():
    reference, result = derived()
    config = result['config']
    assert [j['name'] for j in config['joints']] == ['head_pan', 'head_tilt']
    assert config['identity']['collision_policy'] == 'disabled-v1' and config['interpolation_ms'] == 40
    assert config['direct'] == {'command_ms': 60, 'velocity_timeout_ms': 50, 'limit_reaction_ms': 20,
                                'position_only': False}
    assert 'collision' not in config
    pan = config['joints'][0]
    centre = 2048 * head.RAD_PER_STEP
    assert pan['lower'] == pytest.approx(centre - 1 - OVERTRAVEL_RAD) and pan['upper'] == pytest.approx(
        centre + 1 + OVERTRAVEL_RAD)
    assert (pan['hold_kp'], pan['hold_kd'], pan['max_kp'], pan['max_kd']) == (32., 32., 64., 64.)
    assert result['joint_map']['neck_z'] == {'actuator': 'head_pan', 'scale': 1.0, 'offset': -centre}
    assert result['initial'] == pytest.approx([centre, centre])
    # A reversed servo maps to its CAD joint through the sign, as an arm motor's direction does.
    _, flipped = derived(head_tilt={'sign': -1})
    assert flipped['joint_map']['neck_y'] == {'actuator': 'head_tilt', 'scale': -1.0, 'offset': centre}


def test_the_head_service_configuration_follows_the_receiver_rules():
    reference, result = derived()
    virtual = head.service_config(PROFILE, reference, result, mode='virtual_bus')
    assert virtual['bus'] == {'device': '', 'usb_serial': None, 'baud': 1000000} and not virtual['reference_positions']
    assert {(s['direction'], s['zero_steps']) for s in virtual['servos']} == {(1.0, 0.0)}  # Raw servo radians.
    with pytest.raises(ValueError, match='not pinned'):
        head.service_config(PROFILE, reference, result, mode='teleop_direct')
    pinned = dict(PROFILE, bus={'device': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46083062-if00',
                                'usb_serial': '5A46083062', 'baud': 1000000})
    live = head.service_config(pinned, reference, result, mode='teleop_direct')
    assert [r['raw_position_rad'] for r in live['reference_positions']] == pytest.approx([math.pi, math.pi])
    # Every joint's range stays inside one turn of the servo (the receiver refuses any other).
    for joint in live['controller']['joints']:
        assert 0 <= joint['lower'] / head.RAD_PER_STEP and joint['upper'] / head.RAD_PER_STEP <= 4095


def test_a_head_session_holds_and_follows_direct_updates_on_the_receiver_core():
    from robot_platform.simulation.control_harness import NativeControlHarness
    reference, result = derived()
    config = result['config']
    harness = NativeControlHarness(config, result['initial'])
    clock = {'ms': 0}

    def exchange(request):
        clock['ms'] += 10
        return harness.step(clock['ms'], request)['reply']

    try:
        client = GuardedControllerClient(exchange, group=config['group'], identity=config['identity'],
                                         joint_names=('head_pan', 'head_tilt'))
        client.acquire()
        client.idle()
        assert client.arm()['state'] == 'holding'
        centre = result['initial'][0]
        offered = client.freshness_offer()
        joints = [{'position': centre + .1, 'velocity': 0., 'effort': 0., 'kp': 40., 'kd': 30.},
                  {'position': centre, 'velocity': 0., 'effort': 0., 'kp': 32., 'kd': 32.}]
        reply = client.direct_update(joints, offered, valid_for_ms=40)
        assert reply['accepted'] and reply['status']['state'] in ('holding', 'executing')
        # Gains past the head's ceilings (the servo's register range is far wider) are refused whole.
        too_stiff = [dict(joints[0], kp=100.), joints[1]]
        with pytest.raises(GuardRejected, match='invalid_command'):
            client.direct_update(too_stiff, reply, valid_for_ms=40)
    finally:
        harness.close()
