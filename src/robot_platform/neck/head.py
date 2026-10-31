"""The head on the arms' receiver pipeline: its profile, its receiver's settings and the receiver itself.

Minerva's head is two Feetech STS3250s (head_pan drives the CAD joint neck_z, head_tilt drives neck_y) on its own
receiver, bin/head_controller: the arms' receiver core behind a Feetech bus. Its settings come from the arms'
derivation (control.guarded_config.derive_receiver) with collision checks off, since the arms keep out of
everywhere the head can reach. It reads files in the arms' formats:
- the head profile (config/devices/minerva-head.json): bus, servo roster, receiver, live, motion and sweep blocks,
  read as a teleop profile;
- the head reference: per servo its identity (the adapter's USB serial and its id, so `uid`; `firmware`), its raw
  reading at the reference pose (head straight ahead and level, centred to read mid-turn) and its CAD direction;
- its joint limits and servo calibration, as the arms' (min_rad/max_rad; the servo sweep's settings);
- its camera alignment (helm.head_alignment): the head camera's mount measured on the table's tags, for the head's
  pose feed (neck.head_pose), display only.
The receiver works in raw servo radians (steps x 2 pi / 4096), as the arms' works in raw motor radians.
"""
import json
import math
from pathlib import Path
import subprocess

from robot_platform.control.live_receiver import LiveReceiver, _die_with_parent

from .feetech import RESOLUTION

RAD_PER_STEP = 2 * math.pi / RESOLUTION
BUS = 'head'
VIRTUAL_FIRMWARE = '3.10'
PROFILE = 'config/devices/minerva-head.json'


def files(directory, asset_id):
    """The head's measured files, beside the arms' servo calibration (the operator's files)."""
    return {kind: Path(directory) / f'{asset_id}-head-{kind}.json'
            for kind in ('reference', 'limits', 'servo', 'camera')}


def load_document(path):
    """A saved head file, or None when there is none yet."""
    path = Path(path)
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None


def load_profile(path):
    profile = json.loads(Path(path).read_text(encoding='utf-8'))
    if profile.get('schema') != 1 or profile.get('kind') != 'head_profile':
        raise ValueError(f'{Path(path).name} is not a head profile')
    if profile['receiver'].get('collision_mode') != 'disabled' or not profile['receiver'].get('direct'):
        raise ValueError('The head receiver runs collision checks off, under the direct policy')
    names = [motor['name'] for motor in profile['motors']]
    if not names or len(set(names)) != len(names) or len({m['id'] for m in profile['motors']}) != len(names):
        raise ValueError('The head profile needs servos with distinct names and ids')
    return profile


def virtual_reference(profile):
    """A nominal reference for the simulated head: each servo centred (reading mid-turn at the reference pose),
    turning with its CAD joint."""
    return {'id': 'virtual-head', 'motors': [
        {'name': m['name'], 'joint': m['joint'], 'bus': BUS, 'id': m['id'], 'uid': f"virtual/{m['id']}",
         'firmware': VIRTUAL_FIRMWARE, 'raw_reference_rad': RESOLUTION // 2 * RAD_PER_STEP, 'sign': 1,
         'direction_verified': False} for m in profile['motors']]}


def virtual_limits(reference, span_rad=1.0):
    """Provisional limits for the simulated head: `span_rad` either side of the reference."""
    return {'joints': {m['name']: {'min_rad': m['raw_reference_rad'] - span_rad,
                                   'max_rad': m['raw_reference_rad'] + span_rad} for m in reference['motors']}}


def roster(profile, reference):
    """(reference, steps, directions) in the arms' shapes for derive_receiver: CAD = sign x raw + offset, with
    the CAD zero at the reference pose."""
    by_name = {m['name']: m for m in reference['motors']}
    if set(by_name) != {m['name'] for m in profile['motors']}:
        raise ValueError('The head reference must cover exactly the profile\'s servos')
    motors, steps, directions = [], [], {}
    for servo in profile['motors']:
        measured = by_name[servo['name']]
        if measured['id'] != servo['id'] or measured['joint'] != servo['joint']:
            raise ValueError(f"{servo['name']}: the reference names another servo id or joint")
        raw, sign = float(measured['raw_reference_rad']), int(measured['sign'])
        motors.append({'name': servo['name'], 'bus': BUS, 'id': servo['id'], 'uid': measured['uid'],
                       'firmware': measured['firmware'], 'raw_reference_rad': raw})
        steps.append({'motor': servo['name'], 'joint': servo['joint'], 'label': servo['label']})
        directions[servo['joint']] = {'motor': servo['name'], 'sign': sign, 'offset_rad': -sign * raw,
                                      'direction_verified': measured.get('direction_verified') is True}
    return {'id': reference['id'], 'motors': motors}, steps, directions


def derive(profile, reference, limits, model, servo=None):
    """The head receiver's settings, from the arms' derivation (collision checks off, no backend)."""
    from robot_platform.control.guarded_config import derive_receiver
    motors, steps, directions = roster(profile, reference)
    return derive_receiver(reference=motors, steps=steps, directions=directions, transmissions=[], model=model,
                           limits=limits, profile=profile, backend=None, servo=servo)


REST_MARGIN_RAD = .1  # A head resting this far outside its saved limits is not where its setup left it.


def read_rest(profile, limits, *, open_bus=None):
    """Where the head rests, its servos limp ({motor: raw rad}), read just before its receiver starts. The head cannot
    stay at its reference (it droops, the user 2026-10-30), so a session starts where it rests: the receiver's start
    check takes this pose, and the session raises the head from it and lowers it back there. A reading outside the
    saved limits (by more than REST_MARGIN_RAD) starts nothing."""
    if open_bus is None:
        from .probe import open_bus as open_head_bus
        open_bus = lambda: open_head_bus(profile['bus']['device'], profile['bus']['baud'])  # noqa: E731
    bus = open_bus()
    try:
        values = bus.sync_read([m['id'] for m in profile['motors']], ['Present_Position'])
    finally:
        bus.transport.close()
    rest = {}
    for motor in profile['motors']:
        raw = values[motor['id']]['Present_Position'] * RAD_PER_STEP
        saved = limits['joints'][motor['name']]
        if not saved['min_rad'] - REST_MARGIN_RAD <= raw <= saved['max_rad'] + REST_MARGIN_RAD:
            raise ValueError(f"{motor['label']} rests at {raw:.3f} rad, outside its saved limits "
                             f"{saved['min_rad']:.3f}…{saved['max_rad']:.3f}: set the head up again in Limits → Head")
        rest[motor['name']] = raw
    return rest


def service_config(profile, reference, derived, *, mode, start=None):
    """bin/head_controller's configuration (rust head_config::HeadServiceConfig). `mode`: virtual_bus, observe
    or teleop_direct; the physical ones need the profile's bus pinned by its adapter's USB serial. `start`
    ({motor: raw rad}, read_rest): the pose the receiver checks the head is still at before it starts, where it
    rests; else its reference."""
    if mode not in ('virtual_bus', 'observe', 'teleop_direct'):
        raise ValueError('The head receiver runs virtual_bus, observe or teleop_direct')
    live = profile['live']
    by_name = {m['name']: m for m in reference['motors']}
    start = start or {name: m['raw_reference_rad'] for name, m in by_name.items()}
    bus = profile['bus']
    if mode == 'virtual_bus':
        bus = {'device': '', 'usb_serial': None, 'baud': bus['baud']}
    elif not bus.get('device') or not bus.get('usb_serial'):
        raise ValueError('The head bus is not pinned yet: fill in its adapter\'s /dev/serial/by-id path and USB serial')
    servos = [{'name': m['name'], 'id': m['id'], 'model': m['model'], 'firmware': by_name[m['name']]['firmware'],
               'direction': 1.0, 'zero_steps': 0.0, 'full_load_nm': m['full_load_nm']} for m in profile['motors']]
    references = [] if mode != 'teleop_direct' else [
        {'name': m['name'], 'raw_position_rad': float(start[m['name']]),
         'tolerance_rad': live['reference_tolerance_rad']} for m in profile['motors']]
    return {'mode': mode, 'endpoint': live['endpoint'], 'period_us': live['period_us'],
            'maximum_lateness_us': live['maximum_lateness_us'], 'bus': dict(bus), 'servos': servos,
            'controller': derived['config'], 'reference_positions': references}


class HeadReceiver(LiveReceiver):
    """The head's receiver, owned by this process: bin/head_controller with its configuration. Exchange-compatible
    with the harness and the arms' receiver. The service itself checks the servos' identity and, for head teleop,
    the reference pose before it opens its socket, so there is no separate prescan or bus to bring up."""

    def __init__(self, config, *, binary, directory, labels=None, **options):
        # The arms' receiver names itself after its CAN buses (config['buses']); the head's servos share one Feetech
        # bus of its own (config['bus']), so it starts with none and keeps its own configuration for its service.
        super().__init__(dict(config, buses=[]), device={'buses': []}, reference={'motors': []}, binary=binary,
                         directory=directory, labels=labels, **options)
        self.config, self.buses, self.label = config, [], 'head'

    def _prescan(self):
        return None

    def _link(self, *, up):
        return None

    def _spawn(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / 'live-head.json'
        path.write_text(json.dumps(self.config, indent=1, allow_nan=False) + '\n', encoding='utf-8')
        self.log = (self.directory / 'live-head.log').open('ab')
        flags = ['--allow-powered-teleop', '--confirm-reference-pose'] if self.config['mode'] == 'teleop_direct' else []
        self.process = self.spawn([str(self.binary), str(path), *flags], stdout=self.log, stderr=subprocess.STDOUT,
                                  preexec_fn=_die_with_parent)

    def _await_disarmed(self, timeout_s):
        # The arms' wait, against the head's servos instead of a CAN roster.
        self.config = dict(self.config, motors=self.config['servos'])
        try:
            return super()._await_disarmed(timeout_s)
        finally:
            self.config.pop('motors')
