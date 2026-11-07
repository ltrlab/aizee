"""Minerva's UPS monitor on the robot: readings, the battery's state, the shutdown steps, and installing it."""

import json
from pathlib import Path
import subprocess
import sys

import pytest

from robot_platform.ops import ups

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/devices/minerva-ups.json'


def config(**overrides):
    return ups.load_config(PROFILE, overrides)


def bus_raw(volts):
    return int(round(volts / 0.004)) << 3 | 2  # CNVR set, as the chip reports a fresh conversion.


def shunt_raw(millivolts):
    value = int(round(millivolts / 0.01))
    return value + 65536 if value < 0 else value


def test_registers_decode_as_the_live_pack_read():
    # 2026-10-21, charging: 10.492 V and +13.80 mV across the 0.01-ohm shunt.
    assert ups.decode_bus(bus_raw(10.492)) == pytest.approx(10.492)
    assert ups.decode_shunt(shunt_raw(13.8)) == pytest.approx(13.8)
    assert ups.decode_shunt(shunt_raw(-14.2)) == pytest.approx(-14.2)
    assert ups.decode_shunt(0x8000) == pytest.approx(-327.68)


def test_the_sensor_averages_quick_readings_and_reads_charging_positive():
    reads = []

    def registers(address, register):
        reads.append((address, register))
        return {ups.SHUNT_REGISTER: shunt_raw(13.8), ups.BUS_REGISTER: bus_raw(10.492)}[register]

    sensor = ups.Ina219(config(), registers=registers, sleep=lambda s: None)
    volts, amps = sensor.read()
    assert volts == pytest.approx(10.492) and amps == pytest.approx(1.38)
    assert len(reads) == 10 and {address for address, _ in reads} == {0x41}  # Five pairs; never a write.
    flipped = ups.Ina219(config(charging_sign=-1), registers=registers, sleep=lambda s: None)
    assert flipped.read()[1] == pytest.approx(-1.38)


def test_charge_follows_the_lithium_curve():
    assert ups.charge_percent(3.0) == 0.0 and ups.charge_percent(4.3) == 100.0
    assert ups.charge_percent(3.69) == pytest.approx(10.0) and ups.charge_percent(3.75) == pytest.approx(25.0)
    assert ups.charge_percent(3.72) == pytest.approx(17.5)
    values = [ups.charge_percent(3.2 + i * 0.01) for i in range(110)]
    assert values == sorted(values)


def test_the_shipped_profile_loads_and_bad_ones_are_refused(tmp_path):
    c = config()
    assert (c['i2c_bus'], c['i2c_address'], c['cells'], c['charging_sign']) == (7, 0x41, 3, 1)
    assert c['shutdown_percent'] < c['low_percent'] <= c['low_clear_percent']
    for bad, message in (
        ({'shutdown_percent': 30.0}, 'shutdown_percent < low_percent'),
        ({'charging_sign': 0}, 'charging_sign'),
        ({'i2c_address': 65.0}, 'whole number'),
        ({'notice_s': 90.0}, 'notice_s is at most grace_s'),
        ({'floor_cell_v': 1.0}, 'between'),
        ({'cells': True}, 'whole number'),
        ({'pack_ohm': float('nan')}, 'number'),
    ):
        with pytest.raises(ValueError, match=message):
            ups.load_config(PROFILE, bad)
    unknown = tmp_path / 'ups.json'
    unknown.write_text(json.dumps({'shunt': 0.01}))
    with pytest.raises(ValueError, match='Unknown UPS settings'):
        ups.load_config(unknown)


class Feed:
    """Readings at one a second through a Monitor and a Shutdown, as the service runs them."""

    def __init__(self, **overrides):
        self.c = config(**overrides)
        self.monitor, self.shutdown = ups.Monitor(self.c), ups.Shutdown(self.c)
        self.t, self.events, self.actions, self.sessions = 0.0, [], [], []

    def run(self, seconds, volts=None, amps=None, error=None):
        for _ in range(int(seconds)):
            self.t += 1.0
            reading = None if error else (volts, amps)
            state, events = self.monitor.observe(self.t, 1e9 + self.t, reading=reading, error=error)
            actions, steps = self.shutdown.step(self.t, 1e9 + self.t, state, list(self.sessions))
            state['shutdown'] = self.shutdown.snapshot()
            self.events += [(self.t, name, fields) for name, fields in events + steps]
            self.actions += [(self.t, action) for action in actions]
            self.state = state
        return self.state

    def names(self):
        return [name for _, name, _ in self.events]


def test_unplugging_the_charger_is_on_battery_after_three_seconds_and_back_after_five():
    feed = Feed()
    state = feed.run(2, 12.0, 1.38)
    assert (state['source'], state['charge'], state['level']) == ('external', 'charging', 'ok')
    assert feed.names() == ['on_power']
    state = feed.run(3, 11.4, -1.4)
    assert state['source'] == 'external'  # Not yet: discharging since 2 s ago; a short dip is not a power cut.
    state = feed.run(1, 11.4, -1.4)
    assert state['source'] == 'battery' and state['charge'] == 'discharging' and feed.names()[-1] == 'on_battery'
    assert {'before_v': 12.0, 'before_a': 1.38, 'voltage_v': 11.4}.items() <= feed.events[-1][2].items()
    # Resting cell voltage (11.4 + 1.4 x 0.2) / 3 = 3.893 V: about 63 %; 28 Wh x (63 - 10) % at 15.96 W.
    state = feed.run(20, 11.4, -1.4)
    assert state['percent'] == 63 and state['power_w'] == pytest.approx(15.96)
    assert state['minutes_left'] == pytest.approx(28 * (state['percent'] - 10) / 100 / 15.96 * 60, abs=1.5)
    feed.run(5, 11.6, 0.9)
    assert feed.state['source'] == 'battery'  # Charging for 4 s: not yet back.
    state = feed.run(1, 11.6, 0.9)
    assert state['source'] == 'external' and feed.names()[-1] == 'power_back'
    back = feed.events[-1][2]
    assert back['on_battery_s'] == pytest.approx(26.0) and back['used_wh'] == pytest.approx(
        11.4 * 1.4 * 20 / 3600, abs=1e-3
    )


def test_low_warns_with_hysteresis_and_critical_starts_the_shutdown():
    feed = Feed()
    feed.run(25, 11.4, -1.4)
    # 3.75 V resting (25 %) is low; the 20 s average moves there over the window.
    feed.run(25, 3 * 3.75 - 1.4 * 0.2, -1.4)
    assert feed.state['level'] == 'low' and 'level_low' in feed.names()
    feed.run(25, 3 * 3.76 - 1.4 * 0.2, -1.4)  # About 27.5 %: under the 28 % that clears it.
    assert feed.state['level'] == 'low'
    feed.run(25, 3 * 3.775 - 1.4 * 0.2, -1.4)  # About 31 %.
    assert feed.state['level'] == 'ok' and feed.names()[-1] == 'level_ok'
    feed.run(25, 3 * 3.68 - 1.4 * 0.2, -1.4)  # Under 10 %.
    assert feed.state['level'] == 'critical' and 'charge' in feed.state['reason']
    assert 'shutdown_pending' in feed.names() and feed.state['shutdown']['stage'] == 'pending'


def test_the_cell_floor_starts_a_shutdown_whatever_the_charge_estimate_says():
    feed = Feed(shutdown_percent=0.0, pack_ohm=1.0)  # The estimate reads high through a large resistance.
    feed.run(10, 11.4, -1.4)
    feed.run(6, 3 * 3.34, -1.4)
    assert feed.state['level'] == 'critical' and 'a cell under load' in feed.state['reason']
    assert feed.names().count('shutdown_pending') == 1


def test_charging_readings_never_count_toward_the_battery_decision():
    # The robot on 2026-10-21: charging at 11.104 V, +1.386 A reads about 5 % through the assumed 0.2-ohm pack. If the
    # real resistance is 0.1 ohm, unplugging gives 10.825 V at -1.4 A: about 13 % on battery, above the shutdown level.
    feed = Feed()
    feed.run(30, 11.104, 1.386)
    assert feed.state['percent'] == 5
    feed.run(40, 10.825, -1.4)
    assert feed.state['source'] == 'battery' and feed.state['percent'] == 13
    assert 'shutdown_pending' not in feed.names() and feed.state['level'] == 'low'


def test_a_charge_estimate_needs_settled_battery_readings_before_a_shutdown():
    feed = Feed()
    feed.run(10, 11.0, 1.2)
    feed.run(4, 3 * 3.6 - 1.4 * 0.2, -1.4)  # On battery from the 4th reading, at about 3 %.
    switched = next(t for t, name, _ in feed.events if name == 'on_battery')
    feed.run(20, 3 * 3.6 - 1.4 * 0.2, -1.4)
    pending = next(t for t, name, _ in feed.events if name == 'shutdown_pending')
    assert pending - switched == pytest.approx(10.0)  # settle_s of battery readings first.
    floor = Feed(shutdown_percent=0.0, pack_ohm=1.0)
    floor.run(10, 11.0, 1.2)
    floor.run(20, 3 * 3.3, -1.4)
    switched = next(t for t, name, _ in floor.events if name == 'on_battery')
    pending = next(t for t, name, _ in floor.events if name == 'shutdown_pending')
    assert pending - switched == pytest.approx(5.0)  # The floor's own 5 s window, of battery readings only.


def test_the_service_looks_for_sessions_only_when_a_shutdown_is_near(tmp_path):
    readings = iter([(11.0, 1.2)] * 20 + [(11.4, -1.4)] * 20 + [(3 * 3.6 - 0.28, -1.4)] * 40)

    class Sensor:
        def read(self):
            return next(readings)

    class Counting(Host):
        scans = 0

        def sessions(self):
            Counting.scans += 1
            return []

    host, clock = Counting(tmp_path), {'t': 0.0}
    ups.run(
        config(),
        sensor=Sensor(),
        host=host,
        state_path=tmp_path / 's.json',
        events_path=tmp_path / 'e.jsonl',
        duration_s=40,
        clock=lambda: clock['t'],
        wall=lambda: 1.8e9 + clock['t'],
        sleep=lambda s: clock.__setitem__('t', clock['t'] + s),
    )
    assert Counting.scans == 0 and json.loads((tmp_path / 's.json').read_text())['sessions'] is None
    ups.run(
        config(),
        sensor=Sensor(),
        host=host,
        state_path=tmp_path / 's.json',
        events_path=tmp_path / 'e.jsonl',
        duration_s=40,
        clock=lambda: clock['t'],
        wall=lambda: 1.8e9 + clock['t'],
        sleep=lambda s: clock.__setitem__('t', clock['t'] + s),
    )
    assert 0 < Counting.scans < 40  # From the shutdown level on.


def critical(feed, sessions=()):
    feed.sessions = list(sessions)
    feed.run(5, 11.4, -1.4)
    feed.run(25, 3 * 3.6 - 1.4 * 0.2, -1.4)
    start = next(t for t, name, _ in feed.events if name == 'shutdown_pending')
    return start


def test_with_no_session_the_jetson_powers_off_after_the_notice():
    feed = Feed()
    start = critical(feed)
    feed.run(40, 3 * 3.6 - 1.4 * 0.2, -1.4)
    assert feed.actions == [(start + 30.0, ('poweroff',))]
    poweroff = next(fields for _, name, fields in feed.events if name == 'poweroff')
    assert poweroff['sessions_left'] == [] and feed.state['shutdown']['stage'] == 'powering_off'


def test_a_session_ending_after_the_notice_lets_the_power_off_come_at_once():
    feed = Feed()
    start = critical(feed, sessions=[42])
    feed.run(39, 3 * 3.6 - 1.4 * 0.2, -1.4)
    assert feed.actions == []  # Helm is ending the session: the arm is being lowered.
    feed.sessions = []
    feed.run(5, 3 * 3.6 - 1.4 * 0.2, -1.4)
    assert [action for _, action in feed.actions] == [('poweroff',)] and feed.actions[0][0] > start + 30


def test_a_loop_still_running_after_the_grace_period_gets_sigterm_then_the_power_goes_off():
    feed = Feed()
    start = critical(feed, sessions=[42, 43])
    feed.run(120, 3 * 3.6 - 1.4 * 0.2, -1.4)
    assert feed.actions == [
        (start + 60.0, ('terminate', 42)),
        (start + 60.0, ('terminate', 43)),
        (start + 90.0, ('poweroff',)),
        (start + 120.0, ('poweroff',)),
    ]
    assert feed.names().count('session_terminated') == 2 and 'poweroff_retry' in feed.names()
    poweroff = next(fields for _, name, fields in feed.events if name == 'poweroff')
    assert poweroff['sessions_left'] == [42, 43]


def test_the_charger_coming_back_cancels_the_shutdown():
    feed = Feed()
    critical(feed, sessions=[42])
    feed.run(10, 11.0, 1.2)
    assert 'shutdown_cancelled' in feed.names() and feed.state['shutdown'] is None
    assert feed.state['level'] == 'ok' and feed.actions == []
    feed.sessions = []
    feed.run(60, 11.0, 1.2)
    assert feed.actions == []


def test_a_sensor_that_stops_answering_is_reported_and_a_shutdown_under_way_continues():
    feed = Feed()
    feed.run(3, 11.4, -1.4)
    state = feed.run(5, error='[Errno 121] Remote I/O error')
    assert state['sensor'] == 'ok' and state['voltage_v'] == pytest.approx(11.4)  # Brief: the last reading stands.
    state = feed.run(6, error='[Errno 121] Remote I/O error')
    assert state['sensor'] == 'error' and 'Remote I/O' in state['error']
    feed.run(2, 11.4, -1.4)
    assert feed.names().count('sensor_error') == 1 and 'sensor_recovered' in feed.names()
    start = critical(feed)
    feed.run(10, error='bus stuck')
    feed.run(25, error='bus stuck')
    assert [action for _, action in feed.actions] == [('poweroff',)] and feed.actions[0][0] == start + 30


class Host(ups.Host):
    def __init__(self, root, units=None):
        super().__init__(root, systemctl=self.record, signal_process=self.signal, power_off=self.power)
        self.calls, self.signals, self.powered = [], [], 0
        self.units = units or {}

    def record(self, *args):
        self.calls.append(args)
        if args[0] == 'show':
            return ''.join(f'{key}={value}\n' for key, value in self.units.get(args[-1], {}).items())
        if args == ('restart', ups.UNIT):
            state = self.path(ups.STATE)
            state.parent.mkdir(parents=True, exist_ok=True)
            state.write_text(json.dumps({'updated_at': 2e9, 'sensor': 'ok', 'source': 'external'}))
        return ''

    def signal(self, pid, signum):
        self.signals.append((pid, signum))

    def power(self):
        self.powered += 1


def test_the_service_writes_its_state_logs_transitions_and_acts(tmp_path):
    readings = iter([(10.49, 1.38)] * 3 + [(11.4, -1.4)] * 10 + [(3 * 3.6 - 0.28, -1.4)] * 100)

    class Sensor:
        def read(self):
            return next(readings)

    host = Host(tmp_path)
    proc = tmp_path / 'proc' / '42'
    proc.mkdir(parents=True)
    (proc / 'cmdline').write_bytes(b'python3\0-m\0robot_platform.helm.edge\0')
    clock = {'t': 0.0}

    def sleep(seconds):
        clock['t'] += seconds

    state_path, events_path = tmp_path / 'run' / 'state.json', tmp_path / 'log' / 'events.jsonl'
    ups.run(
        config(),
        sensor=Sensor(),
        host=host,
        state_path=state_path,
        events_path=events_path,
        duration_s=95,
        clock=lambda: clock['t'],
        wall=lambda: 1.8e9 + clock['t'],
        sleep=sleep,
    )
    state = json.loads(state_path.read_text())
    assert state['source'] == 'battery' and state['shutdown']['stage'] == 'terminating' and state['sessions'] == 1
    assert host.signals == [(42, ups.signal.SIGTERM)] and host.powered == 0
    events = [json.loads(line) for line in events_path.read_text().splitlines()]
    names = [event['event'] for event in events]
    assert names[:3] == ['started', 'on_power', 'on_battery'] and 'shutdown_pending' in names
    assert 'battery_sample' in names and events[0]['config']['shutdown_percent'] == 10.0
    assert not list(state_path.parent.glob('*.partial'))


def test_a_dry_run_signals_nothing_and_never_powers_off(tmp_path):
    readings = iter([(11.4, -1.4)] * 5 + [(3 * 3.6 - 0.28, -1.4)] * 120)

    class Sensor:
        def read(self):
            return next(readings)

    host, clock = Host(tmp_path), {'t': 0.0}
    (tmp_path / 'proc').mkdir()
    ups.run(
        config(),
        sensor=Sensor(),
        host=host,
        state_path=tmp_path / 's.json',
        events_path=tmp_path / 'e.jsonl',
        dry_run=True,
        duration_s=120,
        clock=lambda: clock['t'],
        wall=lambda: 1.8e9 + clock['t'],
        sleep=lambda s: clock.__setitem__('t', clock['t'] + s),
    )
    assert host.signals == [] and host.powered == 0
    events = [json.loads(line) for line in (tmp_path / 'e.jsonl').read_text().splitlines()]
    assert {'event': 'dry_run_skipped', 'action': ['poweroff']}.items() <= next(
        e for e in events if e['event'] == 'dry_run_skipped'
    ).items()
    assert json.loads((tmp_path / 's.json').read_text())['dry_run'] is True


def test_a_session_start_is_refused_only_while_a_fresh_state_announces_a_shutdown(tmp_path):
    path = tmp_path / 'state.json'
    assert ups.shutdown_announced(path) is None  # No service.
    path.write_text(json.dumps({'updated_at': 1000.0, 'shutdown': None}))
    assert ups.shutdown_announced(path, now=1001.0) is None
    path.write_text(json.dumps({'updated_at': 1000.0, 'shutdown': {'reason': 'about 9% charge'}}))
    assert 'about 9% charge' in ups.shutdown_announced(path, now=1002.0)
    assert ups.shutdown_announced(path, now=1030.0) is None  # Stale: the service stopped.
    path.write_text('{"updated_at": 10')
    assert ups.shutdown_announced(path, now=11.0) is None


def test_install_copies_root_files_retires_the_old_monitor_and_waits_for_a_reading(tmp_path):
    host = Host(tmp_path, units={ups.OLD_UNIT: {'LoadState': 'loaded', 'ActiveState': 'active'}})
    report = ups.install(host, config_path=PROFILE, wait_s=1.0, clock=lambda: 0.0, sleep=lambda s: None)
    lib = host.path(ups.LIB)
    assert (lib / 'ups.py').read_bytes() == Path(ups.__file__).read_bytes()
    assert (lib / ups.CONFIG).read_bytes() == PROFILE.read_bytes() and (lib / 'README').exists()
    unit = host.path(ups.UNITS / ups.UNIT).read_text()
    assert f'ExecStart=/usr/bin/python3 -I -B {ups.LIB}/ups.py run --config {ups.LIB}/{ups.CONFIG}' in unit
    assert 'CPUAffinity=0 1 2 3' in unit and 'RuntimeDirectory=minerva-ups' in unit
    assert ('stop', ups.OLD_UNIT) in host.calls and ('disable', ups.OLD_UNIT) in host.calls
    order = [call for call in host.calls if call[0] != 'show']
    assert order == [
        ('stop', ups.OLD_UNIT),
        ('disable', ups.OLD_UNIT),
        ('daemon-reload',),
        ('enable', ups.UNIT),
        ('restart', ups.UNIT),
    ]
    assert report['installed'] and report['state']['sensor'] == 'ok'


def test_install_says_so_when_the_service_writes_no_reading(tmp_path):
    host = Host(tmp_path)
    host.record = lambda *args: ''
    host._systemctl = host.record
    ticks = iter(range(100))
    with pytest.raises(RuntimeError, match='wrote no fresh reading'):
        ups.install(host, config_path=PROFILE, wait_s=3.0, clock=lambda: next(ticks), sleep=lambda s: None)
    bad = tmp_path / 'bad.json'
    bad.write_text(json.dumps({'shutdown_percent': 50.0}))
    with pytest.raises(ValueError):
        ups.install(Host(tmp_path / 'other'), config_path=bad)
    assert not (tmp_path / 'other' / 'usr').exists()  # A bad profile changes nothing.


def test_remove_takes_the_service_away_and_keeps_the_event_log(tmp_path):
    host = Host(tmp_path, units={ups.UNIT: {'LoadState': 'loaded'}})
    ups.install(host, config_path=PROFILE, wait_s=1.0, clock=lambda: 0.0, sleep=lambda s: None)
    events = host.path(ups.EVENTS)
    events.parent.mkdir(parents=True)
    events.write_text('{"event":"started"}\n')
    report = ups.remove(host)
    assert ('disable', '--now', ups.UNIT) in host.calls
    assert not host.path(ups.UNITS / ups.UNIT).exists() and not host.path(ups.LIB).exists()
    assert events.exists() and report['events'] == ['{"event":"started"}']


def test_the_installed_copy_runs_on_its_own(tmp_path):
    copy = tmp_path / 'lib' / 'ups.py'
    copy.parent.mkdir()
    copy.write_bytes(Path(ups.__file__).read_bytes())
    result = subprocess.run(
        [sys.executable, '-I', '-B', str(copy), '--help'], capture_output=True, text=True, timeout=60, cwd=tmp_path
    )
    assert result.returncode == 0 and 'minerva-ups' in result.stdout
    refused = subprocess.run(
        [sys.executable, '-I', '-B', str(copy), 'run', '--set', 'shutdown_percent=99', '--config', str(PROFILE)],
        capture_output=True,
        text=True,
        timeout=60,
        cwd=tmp_path,
    )
    assert refused.returncode == 1 and '--dry-run only' in refused.stderr
