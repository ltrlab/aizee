"""Feetech STS driver for the neck: packets, encodings, strict replies, the simulated bus and the read-only probe."""

import time

import pytest

from robot_platform.neck import feetech
from robot_platform.neck.feetech import (
    PING,
    READ,
    SYNC_READ,
    WRITE,
    FeetechBus,
    FeetechError,
    NoReply,
    ServoAlarm,
    from_signed,
    instruction,
    parse_status,
    to_signed,
)
from robot_platform.neck.probe import probe, watch
from robot_platform.neck.sim import SimulatedBus, SimulatedServo


def bus_with(*servos, echo=False):
    sim = SimulatedBus(servos, echo=echo)
    return sim, FeetechBus(sim, timeout_s=0.005, echo=echo)


def test_instruction_packets_match_the_protocol():
    assert instruction(1, PING) == bytes.fromhex('ff ff 01 02 01 fb')
    assert instruction(1, READ, bytes((0x38, 2))) == bytes.fromhex('ff ff 01 04 02 38 02 be')
    # Goal_Position (42) = 2048 on servo 1: ~(1 + 5 + 3 + 42 + 0 + 8) & 0xFF = 0xC4.
    assert instruction(1, WRITE, bytes((42, 0x00, 0x08))) == bytes.fromhex('ff ff 01 05 03 2a 00 08 c4')


def test_sign_magnitude_registers():
    assert to_signed(0x8005, 15) == -5 and to_signed(0x0005, 15) == 5
    assert from_signed(-5, 15) == 0x8005 and from_signed(5, 15) == 5
    assert from_signed(-100, 11) == 0x800 | 100 and to_signed(0x800 | 100, 11) == -100
    with pytest.raises(ValueError):
        from_signed(2048, 11)
    position = feetech.REGISTERS['Goal_Position']
    assert feetech.decode(position, feetech.encode(position, -300)) == -300


def test_replies_are_checked_strictly():
    good = bytes.fromhex('ff ff 01 04 00 00 08') + bytes((feetech.checksum(bytes.fromhex('01 04 00 00 08')),))
    assert parse_status(good, 1, 2) == (0, b'\x00\x08')
    with pytest.raises(FeetechError, match='malformed'):
        parse_status(good, 2, 2)  # Another servo's reply.
    with pytest.raises(FeetechError, match='checksum'):
        parse_status(good[:-1] + bytes((good[-1] ^ 1,)), 1, 2)
    with pytest.raises(NoReply):
        parse_status(good[:-2], 1, 2)


def test_bus_reads_servos_and_finds_only_those_present():
    sim, bus = bus_with(SimulatedServo(1, position=1000), SimulatedServo(2, position=3000))
    assert [bus.ping(i) for i in (1, 2, 3)] == [True, True, False]
    assert bus.read(1, 'Model_Number') == 2825
    assert bus.read_many(2, ['Present_Position', 'Present_Voltage', 'Present_Temperature']) == {
        'Present_Position': 3000,
        'Present_Voltage': 120,
        'Present_Temperature': 30,
    }
    assert bus.sync_read([2, 1], ['Present_Position']) == {2: {'Present_Position': 3000}, 1: {'Present_Position': 1000}}


def test_writes_move_the_simulated_servo_and_eeprom_needs_permission():
    sim, bus = bus_with(SimulatedServo(1, position=1000), SimulatedServo(2, position=1000))
    with pytest.raises(PermissionError):
        bus.write(1, 'ID', 5)
    with pytest.raises(ValueError, match='read-only'):
        bus.write(1, 'Present_Position', 0)
    bus.write(1, 'Goal_Position', 2000)
    sim.advance(0.1)
    assert bus.read(1, 'Present_Position') == 1000  # Torque off: the goal alone does not move it.
    bus.write(1, 'Torque_Enable', 1)
    sim.advance(0.1)
    assert bus.read(1, 'Present_Position') == 1000 + feetech_speed(0.1)
    sim.advance(1)
    assert bus.read(1, 'Present_Position') == 2000
    bus.sync_write('Torque_Enable', {1: 1, 2: 1})
    bus.sync_write('Goal_Position', {1: 1500, 2: 1600})
    sim.advance(1)
    assert bus.sync_read([1, 2], ['Present_Position']) == {1: {'Present_Position': 1500}, 2: {'Present_Position': 1600}}
    bus.write(2, 'Min_Position_Limit', 0, allow_eeprom=True)
    assert sim.servos[2].eeprom_writes_locked == 1  # Locked: a real servo would not keep it over a power cycle.


def feetech_speed(dt):
    from robot_platform.neck.sim import NO_LOAD_SPEED

    return round(NO_LOAD_SPEED * dt)


def test_motion_honours_limits_speed_and_acceleration():
    servo = SimulatedServo(1, position=1000)
    for name, value in (
        ('Torque_Enable', 1),
        ('Max_Position_Limit', 1500),
        ('Goal_Position', 3000),
        ('Goal_Velocity', 500),
    ):
        servo.set(name, value)
    servo.advance(0.5)
    assert servo.get('Present_Position') == 1250
    servo.advance(10)
    assert servo.get('Present_Position') == 1500  # Clamped to the position limit.
    slow = SimulatedServo(1, position=0)
    for name, value in (('Torque_Enable', 1), ('Goal_Position', 2000), ('Acceleration', 10)):
        slow.set(name, value)
    slow.advance(0.1)
    assert 0 < slow.velocity <= 1000 * 0.1 + 1e-9  # 10 x 100 steps/s^2 for 0.1 s.


def test_faults_raise_and_name_the_servo():
    silent, corrupt, hot = SimulatedServo(1), SimulatedServo(2), SimulatedServo(3)
    silent.silent, corrupt.corrupt, hot.error_bits = True, True, 0x04
    sim, bus = bus_with(silent, corrupt, hot)
    assert not bus.ping(1)
    with pytest.raises(NoReply, match='Servo 1'):
        bus.read(1, 'Present_Position')
    with pytest.raises(FeetechError, match='checksum') as caught:
        bus.read(2, 'Present_Position')
    assert not isinstance(caught.value, NoReply)
    with pytest.raises(ServoAlarm, match='overheat') as alarm:
        bus.read(3, 'Present_Temperature')
    assert alarm.value.alarms == ['overheat'] and alarm.value.params == b'\x1e'
    with pytest.raises(NoReply, match='no reply from servo 1'):
        bus.sync_read([1, 3], ['Present_Position'])  # Servo 3's reply arrives where servo 1's should be.


def test_an_echoing_adapter_is_detected_and_its_echo_consumed():
    sim, bus = bus_with(SimulatedServo(1, position=1234), echo=True)
    bus.echo = False
    assert bus.detect_echo() is True and bus.echo
    assert bus.read(1, 'Present_Position') == 1234
    assert bus.sync_read([1], ['Present_Position']) == {1: {'Present_Position': 1234}}
    quiet_sim, quiet = bus_with(SimulatedServo(1))
    assert quiet.detect_echo() is False


def test_probe_and_watch_only_read():
    sim, bus = bus_with(SimulatedServo(1, position=2048), SimulatedServo(2, position=1024))
    report = probe(bus, range(0, 6))
    assert [s['id'] for s in report['servos']] == [1, 2]
    first = report['servos'][0]
    assert (first['model'], first['voltage_v'], first['temperature_c'], first['torque_enabled']) == (
        'sts3250',
        12.0,
        30,
        False,
    )
    assert first['position']['steps'] == 2048 and abs(first['position']['rad'] - 3.1416) < 1e-4
    now = [0.0]
    hand = iter([1900, 2300, 2100])

    def sleep(seconds):
        sim.servos[1].move_by_hand(next(hand, 2100))
        now[0] += seconds

    seen = watch(bus, [1, 2], hz=10, seconds=0.35, emit=lambda line: None, clock=lambda: now[0], sleep=sleep)
    assert seen[1] == (1900, 2300) and seen[2] == (1024, 1024)
    assert set(sim.instructions()) <= {PING, READ, SYNC_READ}


def test_setup_gives_the_only_servo_its_id_and_relocks():
    from robot_platform.neck import setup

    sim, bus = bus_with(SimulatedServo(1, position=1500))
    assert setup.set_id(bus, 2, ids=range(0, 6)) == (1, 2)
    assert not bus.ping(1) and bus.ping(2)
    assert bus.read(2, 'ID') == 2 and bus.read(2, 'Lock') == 1
    assert sim.servos[2].eeprom_writes_locked == 0  # Written unlocked, so it persists.
    two_sim, two = bus_with(SimulatedServo(1), SimulatedServo(2))
    with pytest.raises(setup.SetupError, match='exactly one servo'):
        setup.set_id(two, 3, ids=range(0, 6))


class SlowEeprom:
    """A bus whose servo, after an EEPROM write, is busy storing it for `busy_s`, as the neck's STS3250s were when
    renumbered: its acknowledgement (from the id it was written at) and its reply to anything asked meanwhile come
    only then, in that order."""

    def __init__(self, sim, busy_s):
        self.sim, self.busy_s, self.until, self.held = sim, busy_s, 0.0, bytearray()

    def reset_input_buffer(self):
        self.sim.reset_input_buffer()  # Only what has arrived: the held replies have not.

    def write(self, request):
        busy = time.perf_counter() < self.until
        self.sim.write(request)
        if request[4] == WRITE and request[5] < feetech.EEPROM_END and self.sim.pending:
            self.sim.pending.clear()
            body = bytes((request[2], 2, 0))
            self.held.extend(feetech.HEADER + body + bytes((feetech.checksum(body),)))
            self.until = time.perf_counter() + self.busy_s
        elif busy:
            self.held.extend(self.sim.pending)
            self.sim.pending.clear()

    def read(self, size):
        if self.held and time.perf_counter() >= self.until:
            self.sim.pending[:0] = self.held
            self.held.clear()
        return self.sim.read(size)


def test_an_eeprom_write_waits_for_its_own_late_acknowledgement():
    """With only the short window, the id write's late acknowledgement (from the old id) was read as the reply to
    the ping of the new id: "malformed reply ff ff 0b 02 00 f2" renumbering servo 11 to 1."""
    from robot_platform.neck import setup

    sim = SimulatedBus([SimulatedServo(1, position=1500)])
    bus = FeetechBus(SlowEeprom(sim, 0.007), timeout_s=0.005)
    assert setup.set_id(bus, 2, ids=range(0, 6)) == (1, 2)
    assert bus.read(2, 'ID') == 2 and bus.read(2, 'Lock') == 1
    bus.write(2, 'Lock', 0)  # RAM writes keep the short window.
    started = time.perf_counter()
    with pytest.raises(NoReply, match='100 ms'):  # Silence on an EEPROM write: the long window, then NoReply.
        sim.servos[2].silent = True
        bus.write(2, 'Max_Position_Limit', 3000, allow_eeprom=True)
    assert time.perf_counter() - started >= 0.1


def test_setup_centres_the_reading_and_sets_limits_on_a_limp_servo():
    from robot_platform.neck import setup

    sim, bus = bus_with(SimulatedServo(1, position=1000))
    offset = setup.centre(bus, 1)
    assert offset == 1000 - 2048 and bus.read(1, 'Present_Position') == 2048
    assert bus.read(1, 'Homing_Offset') == offset and bus.read(1, 'Lock') == 1
    sim.servos[1].move_by_hand(2100)  # Turning it now reads relative to the new centre.
    assert bus.read(1, 'Present_Position') == 2100
    assert setup.set_limits(bus, 1, 1300, 2800) == (1300, 2800)
    bus.write(1, 'Torque_Enable', 1)
    with pytest.raises(setup.SetupError, match='holds torque'):
        setup.centre(bus, 1)


def test_setup_commands_only_write_with_confirm(capsys):
    from robot_platform.neck.cli import main

    assert main(['set-id', '--port', 'COM_UNUSED', '--to', '2']) == 0
    assert 'Would give the only servo on the bus id 2' in capsys.readouterr().out


def test_the_head_reference_and_limits_files_feed_its_derivation():
    from datetime import datetime, timezone
    from pathlib import Path
    from robot_platform.models import ModelProfile, UrdfModel
    from robot_platform.neck import head, setup

    root = Path(__file__).resolve().parents[2]
    profile = head.load_profile(root / head.PROFILE)
    now = datetime(2026, 10, 25, tzinfo=timezone.utc)
    sim, bus = bus_with(SimulatedServo(1, position=2048), SimulatedServo(2, position=2050))
    reference = setup.reference_document(
        bus, profile, asset_id='minerva-test', usb_serial='5A46083062', device='/dev/serial/by-id/x', now=now
    )
    pan, tilt = reference['motors']
    assert (pan['uid'], pan['firmware'], pan['raw_reference_steps'], tilt['raw_reference_steps']) == (
        '5A46083062/1',
        '3.10',
        2048,
        2050,
    )
    assert set(sim.instructions()) <= {PING, READ}  # Saving the reference only reads the servos.
    again = setup.reference_document(
        bus, profile, asset_id='minerva-test', usb_serial='5A46083062', device='/dev/serial/by-id/x', now=now
    )
    assert again['id'] != reference['id']  # Limits name their reference: two at one moment must differ.
    limits = setup.limits_document({1: (1500, 2600), 2: (1800, 2300)}, profile, reference, now=now)
    assert limits['joints']['head_pan']['min_rad'] == pytest.approx(1500 * head.RAD_PER_STEP)
    model = UrdfModel(ModelProfile.load(root / profile['model_profile']))
    joints = {j['name']: j for j in head.derive(profile, reference, limits, model)['config']['joints']}
    assert joints['head_tilt']['upper'] == pytest.approx(2300 * head.RAD_PER_STEP + 0.02)
    with pytest.raises(setup.SetupError, match='swept only'):
        setup.limits_document({1: (2040, 2060), 2: (1800, 2300)}, profile, reference, now=now)
    with pytest.raises(setup.SetupError, match='does not contain its reference'):
        setup.limits_document({1: (2100, 2600), 2: (1800, 2300)}, profile, reference, now=now)
    sim.servos[2].move_by_hand(1000)
    with pytest.raises(setup.SetupError, match='centre it'):
        setup.reference_document(bus, profile, asset_id='minerva-test', usb_serial='5A46083062', device='x', now=now)
