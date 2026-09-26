"""The live receiver for one arm: its service configuration and its start/stop sequence (faked hardware)."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_platform.control.commissioning import wire_ranges
from robot_platform.control.live_receiver import (LiveReceiver, fault_detail, outside_limits, reference_offsets,
                                                  service_config)

ROOT = Path(__file__).resolve().parents[2]
DEVICE = json.loads((ROOT / 'config/devices/minerva-production-01.json').read_text(encoding='utf-8'))
REFERENCE = json.loads((ROOT / 'config/devices/minerva-reference-20260924.json').read_text(encoding='utf-8'))
LEFT = [m for m in REFERENCE['motors'] if m['name'].startswith('left_')]
MODELS = dict(zip([m['name'] for m in LEFT], ['ROBSTRIDE03', 'ROBSTRIDE04', 'ROBSTRIDE03', 'ROBSTRIDE02',
                                               'ROBSTRIDE02', 'ROBSTRIDE00', 'ROBSTRIDE00']))
RIGHT_J1 = {'right_arm_j1': 'ROBSTRIDE03'}


def controller(names):
    return {'group': 'g', 'identity': {'model': 'm', 'calibration': 'c', 'collision_policy': 'p'},
            'lease_ms': 150, 'feedback_timeout_ms': 30, 'settled_velocity': .15,
            'collision': {'world_revision': 'w', 'scope': 'inter_branch', 'max_verdict_ms': 80, 'reaction_ms': 10},
            'joints': [{'name': name, 'lower': -1., 'upper': 1., 'max_velocity': 1., 'max_effort': 10.,
                        'max_temperature': 70.} for name in names]}


def config(**limits):
    """limits: {motor: (lower, upper)} overrides."""
    service = service_config(controller([m['name'] for m in LEFT]), DEVICE, dict(REFERENCE, motors=LEFT), MODELS,
                             bus='canL', tolerance_rad=.1)
    for joint in service['controller']['joints']:
        joint['lower'], joint['upper'] = limits.get(joint['name'], (joint['lower'], joint['upper']))
    return service


def test_the_service_drives_one_pinned_arm_with_its_watchdog_identities_and_references():
    service = config()
    assert service['mode'] == 'teleop' and service['host_id'] == 0xAA
    assert service['buses'] == [{'interface': 'canL', 'usb_serial': DEVICE['buses'][0]['usb_serial']}]
    assert [m['name'] for m in service['motors']] == [m['name'] for m in LEFT]
    assert [m['id'] for m in service['motors']] == [m['id'] for m in LEFT]
    assert {m['bus'] for m in service['motors']} == {0}
    assert service['motors'][1]['ranges'] == wire_ranges('ROBSTRIDE04')
    assert service['motors'][1]['ranges']['effort'] == 120
    assert service['watchdog_identities'][0] == {'uid': list(bytes.fromhex(LEFT[0]['uid'])),
                                                 'firmware': [int(p) for p in LEFT[0]['firmware'].split('.')]}
    assert all(r['raw_position_rad'] == 0. and r['tolerance_rad'] == .1 for r in service['reference_positions'])
    assert service['conflicting_services'] == DEVICE['conflicting_services']
    with pytest.raises(ValueError, match='one arm'):
        service_config(controller(['left_arm_j1', 'right_arm_j1']), DEVICE, REFERENCE, MODELS | RIGHT_J1,
                       bus='canL', tolerance_rad=.1)


def scan(offsets=None, **change):
    offsets = offsets or {}
    report = {'success': True,
              'motors': [{'name': m['name'], 'position_rad': offsets.get(m['name'], .02)} for m in LEFT]}
    report.update(change)
    return report


class Process:
    def __init__(self, exits=None):
        self.exits, self.terminated, self.returncode = exits, False, None

    def poll(self):
        return self.returncode if self.returncode is not None else self.exits

    def terminate(self):
        self.terminated, self.returncode = True, 0

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -9


def feedback(position=.02, **change):
    return {'position': position, 'velocity': 0., 'effort': 0., 'temperature': 30., 'sequence': 3,
            'sampled_at_ms': 10, 'fault': False, **change}


class Transport:
    STATES, REASON, FEEDBACK = ('starting', 'disarmed'), 'ok', {}

    def __init__(self, endpoint, *, timeout_ms):
        self.endpoint, self.last, self.closed, self.states = endpoint, None, False, list(self.STATES)

    def request(self, request):
        if self.states[0] == 'starting':
            self.states.pop(0)
            raise TimeoutError('not bound yet')
        self.last = {'reply': {}, 'status': {'state': self.states[0], 'reason': self.REASON},
                     'feedback': [self.FEEDBACK.get(m['name'], feedback()) for m in LEFT], 'watchdog': 'ready'}
        return self.last

    def exchange(self, request):
        return {'accepted': True, 'request': request}

    def close(self):
        self.closed = True


def receiver(tmp_path, *, capture=None, process=None, commands=None, transport=Transport, service=None):
    commands = [] if commands is None else commands
    spawned = []

    def command(args, **options):
        commands.append(args)
        return SimpleNamespace(returncode=0, stdout='', stderr='')

    def spawn(args, **options):
        spawned.append(args)
        return process or Process()

    labels = {'left_arm_j2': 'Left J2'}
    return LiveReceiver(service or config(), device=DEVICE, reference=REFERENCE,
                        binary='/release/bin/guarded_controller', directory=tmp_path, labels=labels,
                        capture=capture or (lambda device, reference: scan()), command=command, spawn=spawn,
                        transport_factory=transport), commands, spawned


def test_start_scans_stopped_then_brings_the_link_up_and_waits_for_a_disarmed_arm(tmp_path):
    seen = []
    live, commands, spawned = receiver(tmp_path, capture=lambda device, reference: seen.append((device, reference))
                                       or scan())
    assert live.start() is live
    device, reference = seen[0]
    assert [b['interface'] for b in device['buses']] == ['canL'] and len(reference['motors']) == 7
    assert commands == [['sudo', '-n', DEVICE['can_up_helper'], 'canL']]
    assert spawned[0][1:] == [str(tmp_path / 'live-canL.json'), '--allow-powered-teleop', '--confirm-reference-pose']
    assert json.loads((tmp_path / 'live-canL.json').read_text())['mode'] == 'teleop'
    assert live.last['status']['state'] == 'disarmed' and live.exchange({'type': 'status'})['accepted']
    process, transport = live.process, live.transport
    live.close()
    assert process.terminated and transport.closed and commands[-1] == ['sudo', '-n', '/usr/sbin/ip', 'link', 'set',
                                                                        'canL', 'down']
    # The receiver's last status stays behind as evidence of how the session ended.
    assert json.loads((tmp_path / 'live-canL-last.json').read_text())['status']['state'] == 'disarmed'
    live.close()  # Idempotent.
    assert len(commands) == 2


def test_an_arm_out_of_its_reference_pose_is_named_and_nothing_starts(tmp_path):
    live, commands, spawned = receiver(tmp_path, capture=lambda d, r: scan({'left_arm_j2': .35}))
    with pytest.raises(ValueError, match=r'Not in the reference pose: Left J2 is \+0\.35 rad away \(tolerance 0\.1'):
        live.start()
    assert commands == [] and spawned == []
    live, commands, spawned = receiver(tmp_path, capture=lambda d, r: scan(success=False, error='Not answering: x'))
    with pytest.raises(RuntimeError, match='stopped check of the arm failed: Not answering'):
        live.start()
    assert commands == [] and spawned == []
    assert reference_offsets(scan({'left_gripper': -.021}), config()) == []


def test_a_service_that_dies_while_starting_is_reported_and_the_link_goes_down(tmp_path):
    (tmp_path / 'live-canL.log').write_text('Error: "watchdog startup verification failed"')
    live, commands, spawned = receiver(tmp_path, process=Process(exits=1))
    with pytest.raises(RuntimeError, match='stopped while starting: .*watchdog startup verification failed'
                                           '.*hang still'):
        live.start()
    assert commands[-1][-1] == 'down' and live.process is None and not live.can_up


def test_a_joint_resting_at_a_receiver_limit_is_named_before_anything_starts(tmp_path):
    # A closed gripper rests on its swept end: 16-bit feedback one step past it faults the receiver.
    service = config(left_gripper=(-.0018, 5.2))
    live, commands, spawned = receiver(tmp_path, service=service,
                                       capture=lambda d, r: scan({'left_gripper': -.0017}))
    with pytest.raises(ValueError, match=r"receiver's joint limits.*left_gripper reads -0\.0017 rad "
                                         r'\(limits -0\.0018 to \+5\.2000\)'):
        live.start()
    assert commands == [] and spawned == []
    assert outside_limits({'left_gripper': -.01}, config(left_gripper=(-.05, 5.2))['controller'], margin=.002) == []


def test_a_startup_fault_names_what_the_receiver_saw_and_keeps_its_last_status(tmp_path):
    class Faulted(Transport):
        STATES, REASON = ('faulted',), 'limit_violation'
        FEEDBACK = {'left_gripper': feedback(-.0021), 'left_arm_j2': feedback(velocity=1.5)}

    live, commands, spawned = receiver(tmp_path, transport=Faulted, service=config(left_gripper=(-.0018, 5.2)))
    with pytest.raises(RuntimeError, match=r'faulted while starting: limit_violation · Left J2 moving at \+1\.50 '
                                           r'rad/s \(limit 1\); left_gripper at -0\.0021 rad, outside its limits '
                                           r'-0\.0018 to \+5\.2000'):
        live.start()
    assert commands[-1][-1] == 'down' and spawned
    saved = json.loads((tmp_path / 'live-canL-last.json').read_text())
    assert saved['status'] == {'state': 'faulted', 'reason': 'limit_violation'}
    controller_config = config()['controller']
    assert fault_detail({'feedback': [feedback() for _ in LEFT], 'watchdog': 'failed'},
                        controller_config).startswith('the watchdog audit failed: every motor must answer promptly')
    assert fault_detail({'feedback': [feedback(fault=True)]}, controller_config) == 'left_arm_j1 reports a drive fault'
    assert fault_detail(None, controller_config) == ''
    # Called from fault handlers: an unexpected response shape never raises.
    assert fault_detail({'feedback': [{'sequence': 1}, 'garbled']}, controller_config) == ''
