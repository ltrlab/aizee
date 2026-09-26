import struct
import pytest

from robot_platform.control.mechanical_zero import StoppedMaintenance, maintenance_frame


def roster():
    return [{"name": f"joint{i}", "id": i + 4, "bus": "canTest",
             "uid": "0102030405060708", "firmware": "0.2.3.34"} for i in range(2)]


class Bus:
    def __init__(self):
        self.events = []
        self.q = {4: 1.5, 5: -2.}
        self.fail = None
        self.run_mode = 0
        self.mode = 0
        self.velocity = 0.
        self.bad_zero = False

    def __call__(self, bus, frame, kind, parameter):
        arb, data = frame
        motor = arb & 255
        self.events.append((motor, kind, data))
        if self.fail == (motor, kind):
            raise TimeoutError("missing motor")
        if kind == "identity":
            return (motor << 8) | 0xFE, bytes(range(1, 9))
        if kind == "read":
            value = {0x7005: self.run_mode, 0x7019: self.q[motor], 0x701B: self.velocity}[parameter]
            data = struct.pack("<H", parameter) + bytes(2)
            data += bytes([value, 0, 0, 0]) if parameter == 0x7005 else struct.pack("<f", value)
            return (17 << 24) | (motor << 8) | 0xAA, data
        if kind == "zero" and not self.bad_zero:
            self.q[motor] = 0.
        data = bytes([0, 0xC4, 0x56, 0, 2, 3, 34, 0]) if kind == "version" else bytes([128, 0, 128, 0, 128, 0, 0, 250])
        return (2 << 24) | (self.mode << 22) | (motor << 8) | 0xAA, data


def test_complete_roster_verified_before_write_and_save_magic():
    bus = Bus()
    report = StoppedMaintenance(bus, sleep=lambda _: None).run(
        roster(), {}, reference_confirmed=True, save=True)
    assert report["success"] and not report["enable_requested"]
    assert not report["power_cycle_verified"]
    first_zero = next(i for i, e in enumerate(bus.events) if e[1] == "zero")
    assert any(e[:2] == (5, "version") for e in bus.events[:first_zero])
    assert [e[2] for e in bus.events if e[1] == "zero"] == [bytes([1]) + bytes(7)] * 2
    assert [e[2] for e in bus.events if e[1] == "save"] == [bytes(range(1, 9))] * 2
    assert all(e[1] in {"identity", "stop", "version", "read", "zero", "save"} for e in bus.events)


@pytest.mark.parametrize("failure", ["missing", "firmware", "enabled", "pp", "moving"])
def test_preflight_failure_never_zeroes_or_saves(failure):
    bus, motors = Bus(), roster()
    if failure == "missing": bus.fail = (5, "identity")
    if failure == "firmware": motors[1]["firmware"] = "0.0.0.0"
    if failure == "enabled": bus.mode = 2
    if failure == "pp": bus.run_mode = 1
    if failure == "moving": bus.velocity = .21
    result = StoppedMaintenance(bus, sleep=lambda _: None).run(motors, {}, reference_confirmed=True, save=True)
    assert not result["success"]
    assert not any(e[1] in ("zero", "save") for e in bus.events)
    assert len(result["final_stops"]) == 2


@pytest.mark.parametrize("failure", ["timeout", "bad_zero"])
def test_ambiguous_write_is_recorded_never_retried_and_stops_other_writes(failure):
    bus = Bus()
    if failure == "timeout": bus.fail = (4, "zero")
    if failure == "bad_zero": bus.bad_zero = True
    result = StoppedMaintenance(bus, sleep=lambda _: None).run(roster(), {}, reference_confirmed=True, save=True)
    assert not result["success"]
    assert result["motors"][0]["zero_requested"]
    assert len([e for e in bus.events if e[1] == "zero"]) == 1
    assert not any(e[:2] == (5, "zero") for e in bus.events)


def test_confirmation_and_frame_allowlist():
    bus = Bus()
    with pytest.raises(ValueError):
        StoppedMaintenance(bus).run(roster(), {})
    assert not bus.events
    for kind in ("enable", "control", "clear_fault", "write"):
        with pytest.raises(ValueError): maintenance_frame(4, 0xAA, kind)
    with pytest.raises(ValueError): maintenance_frame(4, 0xAA, "read", 0x700B)


def test_velocity_outlier_requires_a_new_complete_quiet_interval():
    bus = Bus()
    velocities = iter([0., 0., .271, 0., 0., 0., 0., 0., 0.])
    reads = []
    def exchange(interface, frame, kind, parameter):
        if kind == 'read' and parameter == 0x701B:
            bus.velocity = next(velocities)
            reads.append(bus.velocity)
        return bus(interface, frame, kind, parameter)
    result = StoppedMaintenance(exchange, sleep=lambda _: None).stationary(roster()[0])
    assert len(result) == 6 and len(reads) == 9
    assert not any(e[1] in ('zero', 'save') for e in bus.events)


def test_resume_verifies_full_roster_and_does_not_repeat_acknowledged_zero():
    bus = Bus()
    previous = {'motors': [{**m, 'zero_requested': i == 0,
                           **({'zero_reply': {'mode': 0}} if i == 0 else {})}
                          for i, m in enumerate(roster())]}
    bus.q[4] = 0.
    result = StoppedMaintenance(bus, sleep=lambda _: None).run(roster(), {},
        reference_confirmed=True, save=True, previous=previous)
    assert result['success'] and result['motors'][0]['zero_already_acknowledged']
    assert [(m,k) for m,k,_ in bus.events if k == 'zero'] == [(5, 'zero')]
    assert [(m,k) for m,k,_ in bus.events if k == 'save'] == [(4, 'save'), (5, 'save')]
    previous['motors'][0].pop('zero_reply')
    fresh = Bus()
    with pytest.raises(ValueError, match='Ambiguous'):
        StoppedMaintenance(fresh).run(roster(), {}, reference_confirmed=True, previous=previous)
    assert not fresh.events


def test_journal_marks_write_intent_before_sending_and_ack_after_reply():
    bus, result, checkpoints = Bus(), {}, []
    def checkpoint():
        checkpoints.append(([e[:2] for e in bus.events], [dict(m) for m in result['motors']]))
    StoppedMaintenance(bus, sleep=lambda _: None).run(roster(), result,
        reference_confirmed=True, save=True, checkpoint=checkpoint)
    intent = next((tx, rows) for tx, rows in checkpoints if rows[0].get('zero_requested'))
    assert (4,'zero') not in intent[0] and 'zero_reply' not in intent[1][0]
    ack = next((tx, rows) for tx, rows in checkpoints if rows[0].get('zero_reply'))
    assert (4,'zero') in ack[0]


def test_save_identity_response_is_pinned_and_not_persistence_evidence():
    motor = roster()[0]
    def exchange(bus, frame, kind, parameter):
        return (motor['id'] << 8) | 0xFE, bytes.fromhex(motor['uid'])
    response = StoppedMaintenance(exchange).request(motor, 'save')
    assert response['kind'] == 'identity' and not response['persistence_verified']
    with pytest.raises(RuntimeError, match='Wrong UID'):
        StoppedMaintenance(lambda *a: ((motor['id'] << 8) | 0xFE, bytes(8))).request(motor,'save')
    # A UID frame can never pass a stop or a zero readback check.
    with pytest.raises(RuntimeError): StoppedMaintenance(exchange).request(motor, 'zero')


def test_unacknowledged_save_is_never_retried_and_requires_power_cycle_verification():
    bus = Bus(); bus.fail = (4, 'save')
    result = StoppedMaintenance(bus, sleep=lambda _: None).run(roster(), {}, reference_confirmed=True, save=True)
    assert result['success'] and not result['power_cycle_verified']
    assert result['motors'][0]['save_response_missing']
    assert len([e for e in bus.events if e[:2] == (4,'save')]) == 1
    # Explicit resumed verification cannot turn missing acknowledgement into
    # success evidence or send either flash/zero command again.
    bus.events.clear()
    resumed = StoppedMaintenance(bus, sleep=lambda _: None).run(roster(), {},
        reference_confirmed=True, save=True, previous=result)
    assert resumed['success'] and resumed['motors'][0]['save_response_missing']
    assert not resumed['power_cycle_verified']
    assert not any(e[1] in ('zero', 'save') for e in bus.events)


def test_only_idempotent_diagnostics_may_repeat_once_on_timeout():
    bus = Bus(); attempts = []
    def exchange(*args):
        attempts.append(args[2])
        if len(attempts) == 1: raise TimeoutError('lost read')
        return bus(*args)
    assert StoppedMaintenance(exchange, sleep=lambda _: None).request(roster()[0], 'read', 0x7019) == 1.5
    assert attempts == ['read', 'read']


def test_a_faulted_or_running_drive_is_named_with_its_mode_and_latched_faults():
    bus = Bus()
    bus.mode = 2
    reader = StoppedMaintenance(bus, sleep=lambda _: None)
    with pytest.raises(RuntimeError, match='fault-free Reset: joint0 reports mode Run$'):
        reader.request(roster()[0], 'stop')
    bus.mode = 0
    faulted = lambda bus_name, frame, kind, parameter: ((2 << 24) | (0b000010 << 16) | (5 << 8) | 0xAA, bytes(8))
    with pytest.raises(RuntimeError, match='joint1 reports mode Reset; latched faults: overcurrent'):
        StoppedMaintenance(faulted, sleep=lambda _: None).request(roster()[1], 'stop')
