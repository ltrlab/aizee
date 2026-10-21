"""Minerva's UPS: its battery state for Helm, and a clean shutdown before the battery runs out.

On the Jetson, once, from the active release (it copies itself to /usr/local/lib/minerva-ups and starts the
system service minerva-ups; AIZEE's old aizee-ups-monitor is stopped and disabled):
    cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.ups install
`remove` undoes it; `status` and `read` (one reading) need no sudo. `run --dry-run` rehearses the service with
its own state and event files, never signalling a process or powering off.

The UPS is a Waveshare UPS Power Module (C): a 3S lithium pack, read through an INA219 on I2C bus 7 at 0x41.
On this module a positive shunt voltage is the pack charging (measured 2026-10-21: +1.38 A while the voltage
rose). The service reads it once a second without touching the chip's configuration, and writes
/run/minerva-ups/state.json for Helm. On battery at the shutdown level it announces the shutdown there, lets
Helm end any session (the loop lowers the arm first), signals a loop still running after the grace period
(robot_platform.helm.edge lowers the arm on SIGTERM too), then powers the Jetson off. The charger coming back
cancels it. Transitions go to /var/log/minerva-ups/events.jsonl, which survives the power cut the journal does
not (docs/development/UPS_MONITOR.md).

Standard library only: the installed copy runs without a release (python3 -I).
"""
import argparse
from collections import deque
import ctypes
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import signal
import subprocess
import sys
import time

VERSION = 1
UNIT = 'minerva-ups.service'
OLD_UNIT = 'aizee-ups-monitor.service'  # AIZEE's ZMQ publisher for its status display; it never shut down.
UNITS = PurePosixPath('/etc/systemd/system')
LIB = PurePosixPath('/usr/local/lib/minerva-ups')
STATE = '/run/minerva-ups/state.json'
EVENTS = '/var/log/minerva-ups/events.jsonl'
CONFIG = 'minerva-ups.json'
SESSION = 'robot_platform.helm.edge'  # The command line of a Helm session's loop on the robot.


def release_config():
    """The release's UPS profile beside this module, or None for the installed copy."""
    parents = Path(__file__).resolve().parents
    path = parents[3] / 'config' / 'devices' / CONFIG if len(parents) > 3 else None
    return path if path is not None and path.exists() else None


# A lithium-ion cell's resting voltage against its charge (%), at a low rate: the usual published 18650 curve.
OCV = ((3.27, 0.), (3.61, 5.), (3.69, 10.), (3.71, 15.), (3.73, 20.), (3.75, 25.), (3.77, 30.), (3.79, 35.),
       (3.80, 40.), (3.82, 45.), (3.84, 50.), (3.85, 55.), (3.87, 60.), (3.91, 65.), (3.95, 70.), (3.98, 75.),
       (4.02, 80.), (4.08, 85.), (4.11, 90.), (4.15, 95.), (4.20, 100.))

DEFAULTS = dict(i2c_bus=7, i2c_address=0x41, shunt_ohm=.01, charging_sign=1, cells=3, pack_ohm=.2,
                capacity_wh=28., period_s=1., samples=5, on_battery_a=.1, on_battery_s=3., power_back_s=5.,
                charging_a=.05, average_s=20., floor_average_s=5., low_percent=25., low_clear_percent=28.,
                shutdown_percent=10., floor_cell_v=3.35, notice_s=30., grace_s=60., terminate_s=30.,
                sensor_error_s=10., battery_sample_s=60.)
BOUNDS = dict(i2c_bus=(0, 31), i2c_address=(0x03, 0x77), shunt_ohm=(.001, 1.), cells=(1, 8), pack_ohm=(0., 2.),
              capacity_wh=(1., 1000.), period_s=(.2, 10.), samples=(1, 32), on_battery_a=(.01, 5.),
              on_battery_s=(0., 60.), power_back_s=(0., 120.), charging_a=(.01, 5.), average_s=(1., 300.),
              floor_average_s=(1., 60.), low_percent=(0., 100.), low_clear_percent=(0., 100.),
              shutdown_percent=(0., 100.), floor_cell_v=(2.5, 4.), notice_s=(0., 600.), grace_s=(0., 900.),
              terminate_s=(0., 600.), sensor_error_s=(1., 600.), battery_sample_s=(5., 3600.))
INTEGERS = {'i2c_bus', 'i2c_address', 'cells', 'samples', 'charging_sign'}


def load_config(path=None, overrides=None):
    """The UPS profile (config/devices/minerva-ups.json) over the defaults; raises ValueError when out of bounds."""
    config = dict(DEFAULTS)
    if path is not None:
        block = json.loads(Path(path).read_text(encoding='utf-8'))
        if not isinstance(block, dict):
            raise ValueError('The UPS profile is a JSON object')
        block = {key: value for key, value in block.items() if key != 'note'}
        if set(block) - set(DEFAULTS):
            raise ValueError(f'Unknown UPS settings: {sorted(set(block) - set(DEFAULTS))}')
        config.update(block)
    config.update(overrides or {})
    for key, value in config.items():
        if key in INTEGERS:
            if type(value) is not int:
                raise ValueError(f'UPS {key} is a whole number')
        elif type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'UPS {key} is a number')
        else:
            config[key] = float(value)
        low, high = BOUNDS.get(key, (None, None))
        if low is not None and not low <= config[key] <= high:
            raise ValueError(f'UPS {key} is between {low:g} and {high:g}')
    if config['charging_sign'] not in (-1, 1):
        raise ValueError('UPS charging_sign is 1 or -1: the shunt voltage sign while the pack charges')
    if not config['shutdown_percent'] < config['low_percent'] <= config['low_clear_percent']:
        raise ValueError('UPS shutdown_percent < low_percent <= low_clear_percent')
    if not config['notice_s'] <= config['grace_s']:
        raise ValueError('UPS notice_s is at most grace_s')
    return config


def charge_percent(cell_v):
    """A cell's charge (%) from its resting voltage (OCV)."""
    if cell_v <= OCV[0][0]:
        return 0.
    for (v0, p0), (v1, p1) in zip(OCV, OCV[1:]):
        if cell_v <= v1:
            return p0 + (p1 - p0) * (cell_v - v0) / (v1 - v0)
    return 100.


# -- the sensor ------------------------------------------------------------------------------------------------

I2C_RDWR, I2C_M_RD = 0x0707, 0x0001
SHUNT_REGISTER, BUS_REGISTER = 1, 2


class _Msg(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16), ('len', ctypes.c_uint16),
                ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _RdWr(ctypes.Structure):
    _fields_ = [('msgs', ctypes.POINTER(_Msg)), ('nmsgs', ctypes.c_uint32)]


def i2c_registers(bus):
    """read(address, register) -> the 16-bit register, as one combined transaction (pointer write, repeated
    start, two-byte read): atomic even while another program reads the same chip."""
    import fcntl
    fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)

    def read(address, register):
        pointer, data = (ctypes.c_uint8 * 1)(register), (ctypes.c_uint8 * 2)()
        msgs = (_Msg * 2)(_Msg(address, 0, 1, pointer), _Msg(address, I2C_M_RD, 2, data))
        fcntl.ioctl(fd, I2C_RDWR, _RdWr(msgs, 2))
        return data[0] << 8 | data[1]
    read.close = lambda: os.close(fd)
    return read


def decode_bus(raw):
    """INA219 bus voltage register -> volts (bits 15-3, 4 mV)."""
    return (raw >> 3) * .004


def decode_shunt(raw):
    """INA219 shunt voltage register -> millivolts (two's complement, 10 uV)."""
    return (raw - 65536 if raw > 32767 else raw) * .01


class Ina219:
    """The UPS's current monitor, read without writing its configuration or calibration."""

    def __init__(self, config, registers=None, sleep=time.sleep):
        self.config, self.sleep = config, sleep
        self.registers = registers or i2c_registers(config['i2c_bus'])

    def read(self):
        """(pack volts, current in A with charging positive), each the mean of `samples` quick readings."""
        c, volts, shunt = self.config, 0., 0.
        for index in range(c['samples']):
            if index:
                self.sleep(.01)
            shunt += decode_shunt(self.registers(c['i2c_address'], SHUNT_REGISTER))
            volts += decode_bus(self.registers(c['i2c_address'], BUS_REGISTER))
        count = c['samples']
        return volts / count, c['charging_sign'] * shunt / count / 1000. / c['shunt_ohm']


# -- the battery's state --------------------------------------------------------------------------------------

class Monitor:
    """The pack's state from each reading: on battery or on power (debounced), its charge, and its level."""

    def __init__(self, config):
        self.c = config
        self.source = None          # 'external' or 'battery', once known.
        self._pending = None        # (source, since): a change waiting out its debounce.
        self.battery_since = None   # Wall clock.
        self.level, self.reason = 'ok', None
        self._rest = deque()        # (t, resting cell volts) for the charge estimate.
        self._loaded = deque()      # (t, loaded cell volts) for the floor.
        self.reading = None         # The latest good (volts, amps).
        self.percent = None
        self.error, self.error_since = None, None
        self.stint = None           # Energy drawn on battery since it started: {'ah', 'wh', 't'}.

    def _average(self, history, t, window):
        while history and history[0][0] < t - window:
            history.popleft()
        return sum(value for _, value in history) / len(history) if history else None

    def observe(self, t, wall, reading=None, error=None):
        """(state, [(event, fields)]) after one reading or one failed read at monotonic `t`."""
        c, events = self.c, []
        if error is not None:
            if self.error is None:
                self.error_since = t
                events.append(('sensor_error', {'error': error}))
            self.error = error
            return self.state(wall, t), events
        if self.error is not None:
            events.append(('sensor_recovered', {'after_s': round(t - self.error_since, 1)}))
            self.error = self.error_since = None
        volts, amps = reading
        previous, self.reading = self.reading, reading
        cells = c['cells']
        self._rest.append((t, (volts - amps * c['pack_ohm']) / cells))
        self._loaded.append((t, volts / cells))
        if self.stint is not None and amps < 0:
            dt = min(t - self.stint['t'], 5 * c['period_s'])
            self.stint['ah'] += -amps * dt / 3600
            self.stint['wh'] += -amps * volts * dt / 3600
        if self.stint is not None:
            self.stint['t'] = t
        # On battery: the pack discharging by at least on_battery_a for on_battery_s; back on power after
        # power_back_s of charge or idle current. The first reading decides at once.
        seen = 'battery' if amps <= -c['on_battery_a'] else 'external'
        if self.source is None:
            self._switch(seen, t, wall, events, volts, amps)
        elif seen == self.source:
            self._pending = None
        else:
            if self._pending is None or self._pending[0] != seen:
                self._pending = (seen, t, previous)  # With the old source's last reading.
            wait = c['on_battery_s'] if seen == 'battery' else c['power_back_s']
            if t - self._pending[1] >= wait:
                self._switch(seen, t, wall, events, volts, amps)
        self._grade(t, wall, events)
        return self.state(wall, t), events

    def _switch(self, source, t, wall, events, volts, amps):
        before = self._pending[2] if self._pending else None
        previous, self.source, self._pending = self.source, source, None
        fields = {'voltage_v': round(volts, 3), 'current_a': round(amps, 3)}
        if before is not None:  # The step across the switch measures the pack's resistance: dV / dI.
            fields.update(before_v=round(before[0], 3), before_a=round(before[1], 3))
        if source == 'battery':
            self.battery_since, self.stint = wall, {'ah': 0., 'wh': 0., 't': t}
            events.append(('on_battery', fields))
        else:
            if previous == 'battery':
                fields.update(on_battery_s=round(wall - self.battery_since, 1),
                              used_ah=round(self.stint['ah'], 4), used_wh=round(self.stint['wh'], 3))
                events.append(('power_back', fields))
            else:
                events.append(('on_power', fields))
            self.battery_since, self.stint = None, None
            self.level, self.reason = 'ok', None

    def _grade(self, t, wall, events):
        c = self.c
        rest = self._average(self._rest, t, c['average_s'])
        loaded = self._average(self._loaded, t, c['floor_average_s'])
        self.percent = charge_percent(rest)
        if self.source != 'battery':
            return
        level, reason = self.level, self.reason
        if self.percent <= c['shutdown_percent']:
            level, reason = 'critical', f"about {self.percent:.0f}% charge, at or under {c['shutdown_percent']:g}%"
        elif loaded <= c['floor_cell_v']:
            level, reason = 'critical', f"{loaded:.2f} V a cell under load, at or under {c['floor_cell_v']:g} V"
        elif self.level == 'critical':
            pass  # Critical holds until the power comes back: the shutdown is under way.
        elif self.percent <= c['low_percent']:
            level, reason = 'low', f"about {self.percent:.0f}% charge"
        elif self.level == 'low' and self.percent < c['low_clear_percent']:
            pass
        else:
            level, reason = 'ok', None
        if level != self.level:
            events.append((f'level_{level}', {'percent': round(self.percent, 1), 'cell_v': round(loaded, 3),
                                              'reason': reason}))
        self.level, self.reason = level, reason

    def state(self, wall, t):
        c = self.c
        state = {'version': VERSION, 'updated_at': round(wall, 3), 'sensor': 'ok', 'error': None,
                 'source': self.source, 'level': self.level, 'reason': self.reason,
                 'battery_since': self.battery_since,
                 'thresholds': {'low_percent': c['low_percent'], 'shutdown_percent': c['shutdown_percent'],
                                'floor_cell_v': c['floor_cell_v']}}
        if self.error is not None and t - self.error_since >= c['sensor_error_s']:
            state.update(sensor='error', error=self.error)
        if self.reading is None:
            state['sensor'] = 'error' if self.error is not None else 'starting'
            return state
        volts, amps = self.reading
        power = volts * abs(amps)
        charge = ('discharging' if self.source == 'battery' else 'charging' if amps >= c['charging_a']
                  else 'full' if self.percent >= 95 else 'on_power')
        minutes = None
        if self.source == 'battery' and power >= 1.:
            remaining_wh = max(0., self.percent - c['shutdown_percent']) / 100 * c['capacity_wh']
            minutes = int(remaining_wh / power * 60)
        state.update(voltage_v=round(volts, 3), current_a=round(amps, 3), power_w=round(power, 2),
                     cell_v=round(volts / c['cells'], 3), percent=int(round(self.percent)), charge=charge,
                     minutes_left=minutes)
        return state


class Shutdown:
    """The steps from the shutdown level to power-off: notice for Helm, the grace period, SIGTERM to a loop still
    running, power-off. Power back before power-off cancels it."""

    def __init__(self, config):
        self.c = config
        self.reset()

    def reset(self):
        self.since = self.since_wall = self.reason = self.asked_at = None
        self.stage, self.signalled = None, set()

    def step(self, t, wall, state, sessions):
        """([action], [(event, fields)]): actions are ('terminate', pid) and ('poweroff',)."""
        c, actions, events = self.c, [], []
        if self.since is None:
            if state.get('source') == 'battery' and state.get('level') == 'critical':
                self.since, self.since_wall, self.reason, self.stage = t, wall, state.get('reason'), 'pending'
                events.append(('shutdown_pending', {'reason': self.reason, 'sessions': sorted(sessions),
                                                     'percent': state.get('percent'),
                                                     'voltage_v': state.get('voltage_v')}))
            return actions, events
        if self.stage == 'powering_off':
            if t - self.asked_at >= 30.:  # Still running: systemd did not take the request.
                self.asked_at = t
                events.append(('poweroff_retry', {'after_s': round(t - self.since, 1)}))
                actions.append(('poweroff',))
            return actions, events
        if state.get('source') == 'external':
            events.append(('shutdown_cancelled', {'after_s': round(t - self.since, 1), 'charge': state.get('charge')}))
            self.reset()
            return actions, events
        elapsed = t - self.since
        if (elapsed >= c['notice_s'] and not sessions) or elapsed >= c['grace_s'] + c['terminate_s']:
            self.stage, self.asked_at = 'powering_off', t
            events.append(('poweroff', {'after_s': round(elapsed, 1), 'sessions_left': sorted(sessions),
                                        'voltage_v': state.get('voltage_v'), 'percent': state.get('percent')}))
            actions.append(('poweroff',))
        elif elapsed >= c['grace_s']:
            for pid in sorted(set(sessions) - self.signalled):
                self.signalled.add(pid)
                actions.append(('terminate', pid))
                events.append(('session_terminated', {'pid': pid, 'after_s': round(elapsed, 1)}))
            self.stage = 'terminating'
        return actions, events

    def snapshot(self):
        if self.since is None:
            return None
        c = self.c
        return {'stage': self.stage, 'reason': self.reason, 'since': round(self.since_wall, 3),
                'notice_s': c['notice_s'], 'grace_s': c['grace_s'],
                'poweroff_by': round(self.since_wall + c['grace_s'] + c['terminate_s'], 3)}


# -- the robot -------------------------------------------------------------------------------------------------

class Host:
    """The robot's files, processes and services. Tests root it in a directory and record what it would run."""

    def __init__(self, root='/', systemctl=None, signal_process=None, power_off=None):
        self.root = Path(root)
        self._systemctl, self._signal, self._power_off = systemctl, signal_process, power_off

    def path(self, absolute):
        return self.root.joinpath(*PurePosixPath(absolute).parts[1:])

    def systemctl(self, *args, check=True):
        if self._systemctl is not None:
            return self._systemctl(*args)
        result = subprocess.run(['systemctl', *args], capture_output=True, text=True)
        if check and result.returncode != 0:
            raise RuntimeError(f"systemctl {' '.join(args)}: {(result.stderr or result.stdout).strip()}")
        return result.stdout

    def unit_state(self, unit):
        """{'LoadState', 'ActiveState', 'UnitFileState'} of a unit, as systemctl shows them."""
        text = self.systemctl('show', '-p', 'LoadState', '-p', 'ActiveState', '-p', 'UnitFileState', unit,
                              check=False)
        return dict(line.split('=', 1) for line in text.splitlines() if '=' in line)

    def sessions(self):
        """Processes running a Helm session's loop."""
        found = []
        try:
            entries = list(self.path('/proc').iterdir())
        except OSError:
            return found
        for entry in entries:
            if entry.name.isdigit():
                try:
                    argv = (entry / 'cmdline').read_bytes().split(b'\0')
                except OSError:
                    continue
                if SESSION.encode() in argv:
                    found.append(int(entry.name))
        return found

    def terminate(self, pid):
        if self._signal is not None:
            return self._signal(pid, signal.SIGTERM)
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass  # It ended on its own.

    def poweroff(self):
        if self._power_off is not None:
            return self._power_off()
        os.sync()
        subprocess.run(['systemctl', 'poweroff'], check=False)


def write_state(path, state):
    """Replace the state file atomically: Helm never reads half a write."""
    path = Path(path)
    partial = path.with_name(path.name + '.partial')
    partial.write_text(json.dumps(state, separators=(',', ':')) + '\n', encoding='utf-8')
    os.replace(partial, path)


class EventLog:
    """events.jsonl: a line per transition, flushed to disk at once (the next line may come after a power cut)."""

    LIMIT = 5 << 20

    def __init__(self, path):
        self.path = Path(path)

    def write(self, wall, event, fields=None):
        record = {'at': datetime.fromtimestamp(wall, timezone.utc).isoformat(timespec='seconds'),
                  't': round(wall, 3), 'event': event, **(fields or {})}
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            if self.path.exists() and self.path.stat().st_size > self.LIMIT:
                os.replace(self.path, self.path.with_name(self.path.name + '.1'))
            with open(self.path, 'a', encoding='utf-8') as handle:
                handle.write(json.dumps(record, separators=(',', ':')) + '\n')
                handle.flush()
                os.fsync(handle.fileno())
        except OSError as exc:
            print(f'minerva-ups: could not log {event}: {exc}', file=sys.stderr, flush=True)
        print(f"minerva-ups: {event} {json.dumps(fields or {})}", flush=True)


def shutdown_announced(path=STATE, *, now=None, fresh_s=10.):
    """Why a session may not start: the UPS service's announced shutdown, from a state it wrote in the last
    `fresh_s` seconds (by this computer's clock); None otherwise, also with no service or a stale file."""
    try:
        state = json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    now = time.time() if now is None else now
    updated = state.get('updated_at') if isinstance(state, dict) else None
    if type(updated) not in (int, float) or not 0 <= now - updated <= fresh_s:
        return None
    shutdown = state.get('shutdown')
    if not isinstance(shutdown, dict):
        return None
    return (f"Minerva is shutting down: its UPS battery is low ({shutdown.get('reason') or 'critical'}). "
            'Plug in the UPS charger, then start again once the shutdown is cancelled.')


def run(config, *, sensor, host, state_path, events_path, dry_run=False, duration_s=None,
        clock=time.monotonic, wall=time.time, sleep=time.sleep):
    """The service: read, grade, write the state, log transitions, and act on the shutdown steps."""
    monitor, shutdown, log = Monitor(config), Shutdown(config), EventLog(events_path)
    Path(state_path).parent.mkdir(parents=True, exist_ok=True)
    log.write(wall(), 'started', {'version': VERSION, 'dry_run': dry_run, 'pid': os.getpid(),
                                  'config': {k: config[k] for k in sorted(config)}})
    started = next_at = clock()
    sampled = None
    while duration_s is None or clock() - started < duration_s:
        t, now = clock(), wall()
        try:
            state, events = monitor.observe(t, now, reading=sensor.read())
        except OSError as exc:
            state, events = monitor.observe(t, now, error=str(exc) or type(exc).__name__)
        sessions = host.sessions()
        actions, steps = shutdown.step(t, now, state, sessions)
        state['shutdown'] = shutdown.snapshot()
        state['sessions'] = len(sessions)
        state['dry_run'] = dry_run
        try:
            write_state(state_path, state)
        except OSError as exc:
            print(f'minerva-ups: could not write {state_path}: {exc}', file=sys.stderr, flush=True)
        for event, fields in events + steps:
            log.write(now, event, fields)
        if state.get('source') == 'battery':
            if sampled is None or t - sampled >= config['battery_sample_s']:
                sampled = t
                log.write(now, 'battery_sample', {key: state.get(key) for key in
                                                  ('voltage_v', 'current_a', 'power_w', 'percent', 'minutes_left')}
                          | {'used_wh': round(monitor.stint['wh'], 3) if monitor.stint else None})
        else:
            sampled = None
        for action in actions:
            if dry_run:
                log.write(now, 'dry_run_skipped', {'action': list(action)})
            elif action[0] == 'terminate':
                host.terminate(action[1])
            elif action[0] == 'poweroff':
                host.poweroff()
        next_at += config['period_s']
        delay = next_at - clock()
        if delay < 0:
            next_at, delay = clock(), 0.
        sleep(delay)
    return monitor, shutdown


# -- installing ------------------------------------------------------------------------------------------------

def unit_text():
    return f"""[Unit]
Description=Minerva UPS monitor: battery state for Helm, clean shutdown before the battery runs out
Documentation=file://{LIB}/README
After=local-fs.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 -I -B {LIB}/ups.py run --config {LIB}/{CONFIG}
Restart=always
RestartSec=5
RuntimeDirectory=minerva-ups
RuntimeDirectoryMode=0755
LogsDirectory=minerva-ups
LogsDirectoryMode=0755
# Off the control CPUs (4-5) the teleop loop and receiver keep to (robot_platform.ops.cpu_partition).
CPUAffinity=0 1 2 3
Nice=10
ProtectSystem=full
ProtectHome=read-only
PrivateTmp=yes
NoNewPrivileges=yes

[Install]
WantedBy=multi-user.target
"""


README = """Minerva UPS monitor (robot_platform.ops.ups), installed from a release by
    cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.ups install
These files are root's copies: a root service must not run files the robot's user can change. Install again
to update them. State: /run/minerva-ups/state.json; events: /var/log/minerva-ups/events.jsonl.
"""


def _write(path, data, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    partial.write_bytes(data)
    os.chmod(partial, mode)
    os.replace(partial, path)


def install(host=None, source=None, config_path=None, *, wait_s=20., clock=time.monotonic, sleep=time.sleep):
    host = host or Host()
    source = Path(source or __file__)
    config_path = config_path or release_config()
    if config_path is None:
        raise RuntimeError(f'Install from a release: config/devices/{CONFIG} is not beside this module')
    config_path = Path(config_path)
    load_config(config_path)  # Refuse a bad profile before anything changes.
    lib = host.path(LIB)
    _write(lib / 'ups.py', source.read_bytes())
    _write(lib / CONFIG, config_path.read_bytes())
    _write(lib / 'README', README.encode())
    _write(host.path(UNITS / UNIT), unit_text().encode())
    old = host.unit_state(OLD_UNIT)
    if old.get('LoadState') == 'loaded':
        host.systemctl('stop', OLD_UNIT)
        host.systemctl('disable', OLD_UNIT)
    host.systemctl('daemon-reload')
    host.systemctl('enable', UNIT)
    host.systemctl('restart', UNIT)  # Picks up a reinstall's new files too.
    deadline = clock() + wait_s
    while True:
        report = status(host)
        state = report['state']
        if state is not None and state.get('age_s') is not None and state['age_s'] < 5 and state['sensor'] == 'ok':
            return report
        if clock() > deadline:
            raise RuntimeError(f'{UNIT} started but wrote no fresh reading in {wait_s:g} s: '
                               f'journalctl -u {UNIT} -n 30. Status: {json.dumps(report)}')
        sleep(.5)


def remove(host=None):
    host = host or Host()
    if host.unit_state(UNIT).get('LoadState') == 'loaded':
        host.systemctl('disable', '--now', UNIT)
    unit = host.path(UNITS / UNIT)
    if unit.exists():
        unit.unlink()
    shutil.rmtree(host.path(LIB), ignore_errors=True)
    host.systemctl('daemon-reload')
    return status(host)  # The event log stays; aizee-ups-monitor stays off.


def status(host=None, now=None):
    host = host or Host()
    now = time.time() if now is None else now
    report = {'service': host.unit_state(UNIT), 'old_service': host.unit_state(OLD_UNIT),
              'installed': host.path(LIB / 'ups.py').exists(), 'state': None, 'events': []}
    try:
        state = json.loads(host.path(STATE).read_text(encoding='utf-8'))
        updated = state.get('updated_at')
        state['age_s'] = round(now - updated, 1) if type(updated) in (int, float) else None
        report['state'] = state
    except (OSError, ValueError):
        pass
    try:
        report['events'] = host.path(EVENTS).read_text(encoding='utf-8').splitlines()[-5:]
    except OSError:
        pass
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python3 -m robot_platform.ops.ups', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('run', 'read', 'install', 'remove', 'status'))
    parser.add_argument('--config', type=Path, help=f"UPS profile (default the release's config/devices/{CONFIG})")
    parser.add_argument('--state', default=None, help=f'state file (run; default {STATE})')
    parser.add_argument('--events', default=None, help=f'event log (run; default {EVENTS})')
    parser.add_argument('--dry-run', action='store_true', help='run: never signal a loop or power off')
    parser.add_argument('--for', dest='duration_s', type=float, help='run: stop after this many seconds')
    parser.add_argument('--set', dest='overrides', action='append', default=[], metavar='KEY=VALUE',
                        help='dry run only: override a UPS setting, e.g. shutdown_percent=99')
    args = parser.parse_args(argv)
    config_path = args.config or release_config()
    try:
        overrides = {}
        for item in args.overrides:
            key, _, value = item.partition('=')
            overrides[key] = json.loads(value)
        if overrides and not (args.action == 'run' and args.dry_run):
            raise ValueError('--set rehearses with --dry-run only; the service reads its installed profile')
        if args.action in ('install', 'remove'):
            if getattr(os, 'geteuid', lambda: -1)() != 0:
                print('install and remove change systemd units: run them with sudo.', file=sys.stderr)
                return 2
            if not sys.flags.dont_write_bytecode:
                print('Run as sudo python3 -B -m robot_platform.ops.ups, so no root-owned bytecode is left in the '
                      'release.', file=sys.stderr)
                return 2
            report = install(config_path=config_path) if args.action == 'install' else remove()
        elif args.action == 'status':
            report = status()
        else:
            config = load_config(config_path, overrides)
            sensor = Ina219(config)
            if args.action == 'read':
                volts, amps = sensor.read()
                report = {'voltage_v': round(volts, 3), 'current_a': round(amps, 3),
                          'cell_v': round(volts / config['cells'], 3),
                          'percent': round(charge_percent((volts - amps * config['pack_ohm']) / config['cells']))}
            else:
                if args.dry_run and (args.state is None or args.events is None):
                    raise ValueError('A dry run writes its own --state and --events, never the service\'s')
                signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
                run(config, sensor=sensor, host=Host(), state_path=args.state or STATE,
                    events_path=args.events or EVENTS, dry_run=args.dry_run, duration_s=args.duration_s)
                return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
