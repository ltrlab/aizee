"""Read-only look at the neck's bus servos: which serial ports are there, which servo ids answer and what each
holds (model, firmware, id, baud, limits, offset, mode, gains, torque, lock, position, voltage, temperature,
alarms), and a position watch for hand sweeps with the torque off.

Only PING, READ and SYNC_READ go out: nothing is written, so torque, ids, limits and gains stay as they are.
"""
import math
import time

from .feetech import BAUD_RATES, MODELS, REGISTERS, FeetechBus, alarms_of, steps_to_rad

EEPROM = [name for name, r in REGISTERS.items() if r.address < 40]
RAM = [name for name, r in REGISTERS.items() if r.address >= 40]
BAUD_OF = {code: baud for baud, code in BAUD_RATES.items()}


def list_ports():
    from serial.tools import list_ports as ports
    return [{"device": p.device, "vid": f"{p.vid:04x}" if p.vid is not None else None,
             "pid": f"{p.pid:04x}" if p.pid is not None else None, "serial_number": p.serial_number,
             "manufacturer": p.manufacturer, "description": p.description} for p in ports.comports()]


def open_bus(port, baud=1_000_000, timeout_s=.02):
    """Open `port` and find out whether the adapter echoes requests (a ping to an unused id)."""
    import serial
    connection = serial.Serial(port, baud, timeout=.005, write_timeout=.05)
    try:
        bus = FeetechBus(connection, timeout_s=timeout_s)
        bus.detect_echo()
        return bus
    except BaseException:
        connection.close()
        raise


def servo_report(bus, servo_id):
    values = bus.read_many(servo_id, EEPROM)
    values.update(bus.read_many(servo_id, RAM))
    return {
        "id": servo_id,
        "model": MODELS.get(values["Model_Number"], f"unknown ({values['Model_Number']})"),
        "firmware": f"{values['Firmware_Major_Version']}.{values['Firmware_Minor_Version']}",
        "baud": BAUD_OF.get(values["Baud_Rate"], f"code {values['Baud_Rate']}"),
        "position": {"steps": values["Present_Position"], "rad": round(steps_to_rad(values["Present_Position"]), 4)},
        "position_limits_steps": [values["Min_Position_Limit"], values["Max_Position_Limit"]],
        "homing_offset_steps": values["Homing_Offset"],
        "operating_mode": values["Operating_Mode"],
        "torque_enabled": bool(values["Torque_Enable"]),
        "eeprom_locked": bool(values["Lock"]),
        "gains": {"P": values["P_Coefficient"], "D": values["D_Coefficient"], "I": values["I_Coefficient"]},
        "voltage_v": values["Present_Voltage"] / 10,
        "temperature_c": values["Present_Temperature"],
        "alarms": alarms_of(values["Status"]),
        "registers": values,
    }


def probe(bus, ids=range(0, 21)):
    """Ping each id (one at a time) and report every servo that answers."""
    found = [servo_id for servo_id in ids if bus.ping(servo_id)]
    return {"echo": bus.echo, "ids_scanned": [min(ids), max(ids)], "servos": [servo_report(bus, i) for i in found]}


def watch(bus, ids, *, hz=10, seconds=30, emit=print, clock=time.perf_counter, sleep=time.sleep):
    """Positions at `hz` for `seconds` (Ctrl-C ends it early) with the range seen so far: turn each joint by hand
    to both ends with the torque off. Returns {id: (min_steps, max_steps)}."""
    if not 0 < hz <= 50 or not seconds > 0:
        raise ValueError("Watch at 0..50 Hz for a positive time")
    seen, period, end = {}, 1 / hz, clock() + seconds
    try:
        while clock() < end:
            started = clock()
            positions = {i: v["Present_Position"] for i, v in bus.sync_read(ids, ["Present_Position"]).items()}
            for servo_id, steps in positions.items():
                low, high = seen.get(servo_id, (steps, steps))
                seen[servo_id] = (min(low, steps), max(high, steps))
            emit("  ".join(f"id {i}: {s:5d} ({math.degrees(steps_to_rad(s)):7.2f} deg) "
                           f"range {seen[i][0]}..{seen[i][1]}" for i, s in positions.items()))
            sleep(max(0, period - (clock() - started)))
    except KeyboardInterrupt:
        pass
    return seen

