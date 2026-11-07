"""Finite, pinned-SSH camera settings and previews; never a motor connection.

The deployed legacy camera REP is the only mutation path. Its get_ctrls reply
omits menu entries and inactive flags, so a read-only V4L2 query supplements it.
The helper is sent as source to the verified robot; no deployment is required.
"""

import base64
import json
from pathlib import Path
import re
import threading
import time

ROLES = {
    'left_wrist': {'control_port': 5573, 'frame_port': 5563, 'camera_id': 'left_wrist', 'usb_id': '05a3:9230'},
    'right_wrist': {'control_port': 5574, 'frame_port': 5565, 'camera_id': 'right_wrist', 'usb_id': '32e4:9230'},
    'head': {'frame_port': 5564, 'camera_id': 'scene_cam'},
}
MAX_REPLY = 3 * 1024 * 1024


class CameraWriteUncertain(RuntimeError):
    """Legacy REP has no request TTL: a timed-out write may apply later."""


def role_config(role):
    if role not in ROLES:
        raise ValueError('Expected camera role left_wrist, right_wrist or head')
    return ROLES[role]


def parse_controls(text):
    """Parse actual V4L2 ranges, flags and sparse menu IDs; never infer menus."""
    result, current = {}, None
    for line in text.splitlines():
        match = re.match(r'\s*(\w+)\s+0x[\da-fA-F]+\s+\(([^)]+)\)\s*:\s*(.*)', line)
        if match:
            name, kind, tail = match.groups()
            current = {'type': kind}
            for key, value in re.findall(r'(min|max|step|default|value)=(-?\d+)', tail):
                current[key] = int(value)
            current['flags'] = tail.split('flags=', 1)[-1].strip() if 'flags=' in tail else ''
            if kind == 'menu':
                current['options'] = []
            result[name] = current
        elif current is not None and current.get('type') == 'menu':
            match = re.match(r'\s*(-?\d+)\s*:\s*(.+)', line)
            if match:
                current['options'].append({'value': int(match[1]), 'label': match[2].strip()})
    return result


def verified_controls(reported, actual):
    if not isinstance(reported, dict) or not reported:
        raise RuntimeError('Camera returned no controls; check v4l2-ctl and the active video device')
    result = {}
    for name, info in reported.items():
        physical = actual.get(name)
        if not isinstance(info, dict) or not physical:
            raise RuntimeError('V4L2 device does not match camera control response')
        # Values may change in automatic mode; identity comparison uses static fields.
        for key in ('type', 'min', 'max', 'step', 'default'):
            if info.get(key) != physical.get(key):
                raise RuntimeError('V4L2 device control ranges do not match camera response')
        control = dict(physical, value=info.get('value'))
        flags = physical.get('flags', '')
        reason = None
        if physical.get('type') not in ('int', 'bool', 'menu'):
            reason = 'Unsupported control type'
        elif any(flag in flags for flag in ('inactive', 'read-only', 'disabled', 'grabbed')):
            reason = 'Camera reports control ' + flags
        elif physical.get('type') == 'menu' and not physical.get('options'):
            reason = 'Valid menu entries were not reported'
        elif physical.get('type') == 'int' and (type(physical.get('step')) is not int or physical['step'] < 1):
            reason = 'Camera did not report a positive step'
        control.update(writable=reason is None, reason=reason)
        result[name] = control
    return result


def validate_value(controls, name, value):
    if not isinstance(name, str) or name not in controls:
        raise ValueError('Unknown camera control')
    control = controls[name]
    if not control.get('writable'):
        raise ValueError(control.get('reason') or 'Camera control is read-only')
    if type(value) is not int:
        raise ValueError('Camera control value must be an integer')
    kind = control['type']
    if kind == 'menu':
        if value not in [option['value'] for option in control['options']]:
            raise ValueError('Value is not a reported camera menu entry')
    elif kind == 'bool':
        if value not in (0, 1):
            raise ValueError('Boolean camera control must be 0 or 1')
    else:
        low, high, step = control.get('min'), control.get('max'), control.get('step')
        if any(type(item) is not int for item in (low, high, step)) or step <= 0:
            raise ValueError('Camera did not report valid range and step')
        if not low <= value <= high or (value - low) % step:
            raise ValueError(f'Value must be in {low}..{high}, step {step} from {low}')


def _devices(usb_id):
    """Read sysfs identities only; do not open another capture stream."""
    result = []
    for node in sorted(Path('/sys/class/video4linux').glob('video*')):
        parent = (node / 'device').resolve()
        for candidate in (parent, *parent.parents):
            try:
                actual = (
                    (candidate / 'idVendor').read_text().strip().lower()
                    + ':'
                    + (candidate / 'idProduct').read_text().strip().lower()
                )
            except OSError:
                continue
            if actual == usb_id:
                result.append('/dev/' + node.name)
            break
    if len(result) > 8:
        raise RuntimeError('Ambiguous camera USB identity: too many video devices')
    return result


def _device_controls(config, reported):
    import shutil
    import subprocess

    if shutil.which('v4l2-ctl') is None:
        raise RuntimeError('v4l2-ctl is not installed on the robot; install v4l-utils')
    matches, errors = [], []
    for device in _devices(config['usb_id']):
        try:
            reply = subprocess.run(
                ['v4l2-ctl', '--device', device, '--list-ctrls-menus'], capture_output=True, text=True, timeout=1.0
            )
            if reply.returncode:
                errors.append(reply.stderr.strip()[:200])
                continue
            controls = verified_controls(reported, parse_controls(reply.stdout))
            matches.append((device, controls))
        except (ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
            errors.append(str(exc))
    if len(matches) != 1:
        raise RuntimeError(
            'Camera USB identity/control match is missing or ambiguous' + (': ' + '; '.join(errors) if errors else '')
        )
    return matches[0]


def _request(config, payload):
    import msgpack
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.MAXMSGSIZE, 65536)
        socket.connect('tcp://127.0.0.1:' + str(config['control_port']))
        socket.send(msgpack.packb(payload, use_bin_type=True))
        if not socket.poll(2200):
            raise TimeoutError('Camera control service did not reply within 2.2 seconds')
        reply = msgpack.unpackb(socket.recv(), raw=False)
        if not isinstance(reply, dict) or reply.get('ok') is not True:
            raise RuntimeError(
                str(reply.get('error') or reply.get('msg') or 'Camera rejected request')
                if isinstance(reply, dict)
                else 'Malformed camera reply'
            )
        return reply
    finally:
        socket.close(0)
        context.term()


def _snapshot_role(role):
    config = role_config(role)
    state = dict(
        available=False, controls={}, controls_supported='control_port' in config, preview_supported=True, error=None
    )
    if 'control_port' not in config:
        state['error'] = 'Head camera exposure controls are not supported by this camera service'
        return state
    try:
        # Check the binary explicitly: legacy get_ctrls returns {} on a missing binary.
        import shutil

        if shutil.which('v4l2-ctl') is None:
            raise RuntimeError('v4l2-ctl is not installed on the robot; install v4l-utils')
        reported = _request(config, {'op': 'get_ctrls'})['ctrls']
        device, controls = _device_controls(config, reported)
        state.update(available=True, controls=controls, device=device)
    except Exception as exc:
        state['error'] = str(exc)
    return state


def _preview(role):
    import math
    import msgpack
    import zmq

    config = role_config(role)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVHWM, 1)
        socket.setsockopt(zmq.MAXMSGSIZE, 2 * 1024 * 1024)
        socket.setsockopt(zmq.SUBSCRIBE, b'')
        socket.connect('tcp://127.0.0.1:' + str(config['frame_port']))
        if not socket.poll(2500):
            raise TimeoutError('Camera preview did not arrive within 2.5 seconds')
        frames = socket.recv_multipart()
        header = msgpack.unpackb(frames[0], raw=False)
        if header.get('camera_id') != config['camera_id']:
            raise ValueError('Camera preview identity mismatch')
        stamp = header.get('timestamp')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or not -0.1 <= time.time() - stamp <= 2.0:
            raise ValueError('Camera preview timestamp is missing or stale')
        parts = header.get('parts', [])
        if (
            len(parts) != len(frames) - 1
            or parts.count('color') != 1
            or header.get('color', {}).get('format', '').lower() != 'jpeg'
        ):
            raise ValueError('Camera preview is not a valid color JPEG packet')
        jpeg = frames[1 + parts.index('color')]
        if not 4 <= len(jpeg) <= 2 * 1024 * 1024 or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
            raise ValueError('Camera returned an invalid JPEG')
        return {'jpeg': base64.b64encode(jpeg).decode('ascii'), 'timestamp': stamp}
    finally:
        socket.close(0)
        context.term()


def remote_operation(request):
    """Executed on the pinned robot; all ports/operations are fixed here."""
    operation = request.get('operation')
    if operation == 'snapshot':
        return {'cameras': {role: _snapshot_role(role) for role in ROLES}}
    role = request.get('role')
    config = role_config(role)
    if operation == 'frame':
        return _preview(role)
    if operation != 'set':
        raise ValueError('Unknown camera operation')
    state = _snapshot_role(role)
    if not state['available']:
        raise ValueError(state['error'])
    name, value = request.get('name'), request.get('value')
    validate_value(state['controls'], name, value)
    try:
        _request(config, {'op': 'set_ctrl', 'name': name, 'value': value})
        updated = _snapshot_role(role)
        readback = updated['controls'].get(name, {}).get('value')
        if not updated['available'] or type(readback) is not int or readback != value:
            raise RuntimeError(
                'Camera write was sent but readback did not confirm it: ' + str(updated['error'] or readback)
            )
        return {'camera': updated, 'name': name, 'value': value, 'readback': readback}
    except Exception as exc:
        raise CameraWriteUncertain(str(exc)) from exc


def remote_main(request):
    import signal

    def expired(*_):
        raise TimeoutError('Camera operation exceeded its 10 second robot budget')

    signal.signal(signal.SIGALRM, expired)
    signal.alarm(10)
    try:
        print(json.dumps({'ok': True, 'result': remote_operation(request)}), flush=True)
    except Exception as exc:
        print(
            json.dumps({'ok': False, 'error': str(exc), 'write_uncertain': isinstance(exc, CameraWriteUncertain)}),
            flush=True,
        )
    finally:
        signal.alarm(0)


class CameraSettings:
    """Call from a worker thread. One finite SSH process, no persistent camera owner."""

    def __init__(self, profile, *, address, open_ssh=None):
        self.profile, self.address, self.open_ssh = Path(profile), address, open_ssh
        self._lock = threading.Lock()
        self.write_fault = None

    def _call(self, request):
        from robot_platform.ops.ssh import connect, open_command

        if not self._lock.acquire(blocking=False):
            raise ValueError('Another camera request is running; wait for it to finish')
        client = channel = None
        submitted_write = False
        try:
            if request.get('operation') == 'set' and self.write_fault:
                raise ValueError(self.write_fault)
            client = (self.open_ssh or connect)(self.profile, address=self.address(), timeout=3.0)
            script = Path(__file__).read_text(encoding='utf-8') + '\nremote_main(' + repr(request) + ')\n'
            stdin, out, err = open_command(client, 'python3 -', timeout=3.0)
            channel = out.channel
            channel.settimeout(3.0)
            # A failed send/EOF can still leave a complete script at the peer.
            submitted_write = request.get('operation') == 'set'
            stdin.write(script)
            stdin.flush()
            channel.shutdown_write()
            deadline, stdout, stderr = time.monotonic() + 12.0, bytearray(), bytearray()
            while True:
                if channel.recv_ready():
                    stdout.extend(channel.recv(65536))
                if channel.recv_stderr_ready():
                    stderr.extend(channel.recv_stderr(4096))
                if len(stdout) > MAX_REPLY or len(stderr) > 65536:
                    raise RuntimeError('Camera helper exceeded its output limit')
                if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError('Camera SSH operation timed out')
                time.sleep(0.01)
            if channel.recv_exit_status():
                raise RuntimeError('Camera helper failed: ' + stderr.decode(errors='replace')[:1000])
            reply = json.loads(stdout)
            if reply.get('write_uncertain') is True:
                raise CameraWriteUncertain(reply.get('error') or 'Camera write was not confirmed')
            if reply.get('ok') is False and reply.get('write_uncertain') is False:
                # The helper rejected validation before attempting any mutation.
                submitted_write = False
                raise ValueError(reply.get('error') or 'Camera helper failed')
            if reply.get('ok') is not True or 'result' not in reply:
                raise RuntimeError('Malformed camera helper reply')
            return reply['result']
        except Exception as exc:
            if submitted_write or isinstance(exc, CameraWriteUncertain):
                self.write_fault = (
                    'Camera write outcome is uncertain; session start and further changes are blocked. '
                    'Restart the camera service, verify its settings, then restart Helm. Details: ' + str(exc)
                )
                raise CameraWriteUncertain(self.write_fault) from exc
            if isinstance(exc, (ValueError, RuntimeError)):
                raise
            raise RuntimeError('Camera SSH request failed: ' + str(exc)) from exc
        finally:
            if channel is not None:
                channel.close()
            if client is not None:
                client.close()
            self._lock.release()

    def snapshot(self):
        try:
            return self._call({'operation': 'snapshot'})
        except Exception as exc:
            return {
                'cameras': {
                    role: {
                        'available': False,
                        'controls': {},
                        'error': str(exc),
                        'controls_supported': 'control_port' in config,
                        'preview_supported': True,
                    }
                    for role, config in ROLES.items()
                }
            }

    def set(self, role, name, value):
        role_config(role)
        if not isinstance(name, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', name) or type(value) is not int:
            raise ValueError('Expected a control name and integer value')
        return self._call({'operation': 'set', 'role': role, 'name': name, 'value': value})

    def frame(self, role):
        role_config(role)
        response = self._call({'operation': 'frame', 'role': role})
        return base64.b64decode(response['jpeg'], validate=True)
