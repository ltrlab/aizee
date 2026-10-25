"""Feetech STS driver for the neck: packets, encodings, strict replies, the simulated bus and the read-only probe."""
import pytest

from robot_platform.neck import feetech
from robot_platform.neck.feetech import (PING, READ, SYNC_READ, WRITE, FeetechBus, FeetechError, NoReply,
                                         ServoAlarm, from_signed, instruction, parse_status, to_signed)
from robot_platform.neck.probe import probe, watch
from robot_platform.neck.sim import SimulatedBus, SimulatedServo


def bus_with(*servos, echo=False):
    sim = SimulatedBus(servos, echo=echo)
    return sim, FeetechBus(sim, timeout_s=.005, echo=echo)


def test_instruction_packets_match_the_protocol():
    assert instruction(1, PING) == bytes.fromhex("ff ff 01 02 01 fb")
    assert instruction(1, READ, bytes((0x38, 2))) == bytes.fromhex("ff ff 01 04 02 38 02 be")
    # Goal_Position (42) = 2048 on servo 1: ~(1 + 5 + 3 + 42 + 0 + 8) & 0xFF = 0xC4.
    assert instruction(1, WRITE, bytes((42, 0x00, 0x08))) == bytes.fromhex("ff ff 01 05 03 2a 00 08 c4")


def test_sign_magnitude_registers():
    assert to_signed(0x8005, 15) == -5 and to_signed(0x0005, 15) == 5
    assert from_signed(-5, 15) == 0x8005 and from_signed(5, 15) == 5
    assert from_signed(-100, 11) == 0x800 | 100 and to_signed(0x800 | 100, 11) == -100
    with pytest.raises(ValueError):
        from_signed(2048, 11)
    position = feetech.REGISTERS["Goal_Position"]
    assert feetech.decode(position, feetech.encode(position, -300)) == -300


def test_replies_are_checked_strictly():
    good = bytes.fromhex("ff ff 01 04 00 00 08") + bytes((feetech.checksum(bytes.fromhex("01 04 00 00 08")),))
    assert parse_status(good, 1, 2) == (0, b"\x00\x08")
    with pytest.raises(FeetechError, match="malformed"):
        parse_status(good, 2, 2)  # Another servo's reply.
    with pytest.raises(FeetechError, match="checksum"):
        parse_status(good[:-1] + bytes((good[-1] ^ 1,)), 1, 2)
    with pytest.raises(NoReply):
        parse_status(good[:-2], 1, 2)


def test_bus_reads_servos_and_finds_only_those_present():
    sim, bus = bus_with(SimulatedServo(1, position=1000), SimulatedServo(2, position=3000))
    assert [bus.ping(i) for i in (1, 2, 3)] == [True, True, False]
    assert bus.read(1, "Model_Number") == 2825
    assert bus.read_many(2, ["Present_Position", "Present_Voltage", "Present_Temperature"]) == {
        "Present_Position": 3000, "Present_Voltage": 120, "Present_Temperature": 30}
    assert bus.sync_read([2, 1], ["Present_Position"]) == {2: {"Present_Position": 3000}, 1: {"Present_Position": 1000}}


def test_writes_move_the_simulated_servo_and_eeprom_needs_permission():
    sim, bus = bus_with(SimulatedServo(1, position=1000), SimulatedServo(2, position=1000))
    with pytest.raises(PermissionError):
        bus.write(1, "ID", 5)
    with pytest.raises(ValueError, match="read-only"):
        bus.write(1, "Present_Position", 0)
    bus.write(1, "Goal_Position", 2000)
    sim.advance(.1)
    assert bus.read(1, "Present_Position") == 1000  # Torque off: the goal alone does not move it.
    bus.write(1, "Torque_Enable", 1)
    sim.advance(.1)
    assert bus.read(1, "Present_Position") == 1000 + feetech_speed(.1)
    sim.advance(1)
    assert bus.read(1, "Present_Position") == 2000
    bus.sync_write("Torque_Enable", {1: 1, 2: 1})
    bus.sync_write("Goal_Position", {1: 1500, 2: 1600})
    sim.advance(1)
    assert bus.sync_read([1, 2], ["Present_Position"]) == {1: {"Present_Position": 1500}, 2: {"Present_Position": 1600}}
    bus.write(2, "Min_Position_Limit", 0, allow_eeprom=True)
    assert sim.servos[2].eeprom_writes_locked == 1  # Locked: a real servo would not keep it over a power cycle.


def feetech_speed(dt):
    from robot_platform.neck.sim import NO_LOAD_SPEED
    return round(NO_LOAD_SPEED * dt)


def test_motion_honours_limits_speed_and_acceleration():
    servo = SimulatedServo(1, position=1000)
    for name, value in (("Torque_Enable", 1), ("Max_Position_Limit", 1500), ("Goal_Position", 3000),
                        ("Goal_Velocity", 500)):
        servo.set(name, value)
    servo.advance(.5)
    assert servo.get("Present_Position") == 1250
    servo.advance(10)
    assert servo.get("Present_Position") == 1500  # Clamped to the position limit.
    slow = SimulatedServo(1, position=0)
    for name, value in (("Torque_Enable", 1), ("Goal_Position", 2000), ("Acceleration", 10)):
        slow.set(name, value)
    slow.advance(.1)
    assert 0 < slow.velocity <= 1000 * .1 + 1e-9  # 10 x 100 steps/s^2 for 0.1 s.


def test_faults_raise_and_name_the_servo():
    silent, corrupt, hot = SimulatedServo(1), SimulatedServo(2), SimulatedServo(3)
    silent.silent, corrupt.corrupt, hot.error_bits = True, True, 0x04
    sim, bus = bus_with(silent, corrupt, hot)
    assert not bus.ping(1)
    with pytest.raises(NoReply, match="Servo 1"):
        bus.read(1, "Present_Position")
    with pytest.raises(FeetechError, match="checksum") as caught:
        bus.read(2, "Present_Position")
    assert not isinstance(caught.value, NoReply)
    with pytest.raises(ServoAlarm, match="overheat") as alarm:
        bus.read(3, "Present_Temperature")
    assert alarm.value.alarms == ["overheat"] and alarm.value.params == b"\x1e"
    with pytest.raises(NoReply, match="no reply from servo 1"):
        bus.sync_read([1, 3], ["Present_Position"])  # Servo 3's reply arrives where servo 1's should be.


def test_an_echoing_adapter_is_detected_and_its_echo_consumed():
    sim, bus = bus_with(SimulatedServo(1, position=1234), echo=True)
    bus.echo = False
    assert bus.detect_echo() is True and bus.echo
    assert bus.read(1, "Present_Position") == 1234
    assert bus.sync_read([1], ["Present_Position"]) == {1: {"Present_Position": 1234}}
    quiet_sim, quiet = bus_with(SimulatedServo(1))
    assert quiet.detect_echo() is False


def test_probe_and_watch_only_read():
    sim, bus = bus_with(SimulatedServo(1, position=2048), SimulatedServo(2, position=1024))
    report = probe(bus, range(0, 6))
    assert [s["id"] for s in report["servos"]] == [1, 2]
    first = report["servos"][0]
    assert (first["model"], first["voltage_v"], first["temperature_c"], first["torque_enabled"]) == (
        "sts3250", 12.0, 30, False)
    assert first["position"]["steps"] == 2048 and abs(first["position"]["rad"] - 3.1416) < 1e-4
    now = [0.0]
    hand = iter([1900, 2300, 2100])

    def sleep(seconds):
        sim.servos[1].move_by_hand(next(hand, 2100))
        now[0] += seconds

    seen = watch(bus, [1, 2], hz=10, seconds=.35, emit=lambda line: None, clock=lambda: now[0], sleep=sleep)
    assert seen[1] == (1900, 2300) and seen[2] == (1024, 1024)
    assert set(sim.instructions()) <= {PING, READ, SYNC_READ}
