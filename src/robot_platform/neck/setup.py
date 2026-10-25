"""One-off setup of the neck's servos, with the user present: give a servo its id, centre its reading at the
reference pose, and set its position limits.

Each step writes the servo's EEPROM, which persists across power cycles, so each runs only on an explicit command,
with the torque off and exactly the expected servo answering. Each verifies what it wrote: a value the servo does
not read back as intended is put back and the step fails. The EEPROM is locked again afterwards, so later gain
writes (the receiver's) never persist.
"""
from .feetech import BROADCAST, FeetechError, RESOLUTION

CENTRE = RESOLUTION // 2  # 2048 steps: the reading at the reference pose, mid-turn, so the range never wraps.


class SetupError(RuntimeError):
    """The servo is not in the state a setup step needs, or did not take what was written."""


def scan(bus, ids=range(0, 254)):
    return [servo_id for servo_id in ids if servo_id != BROADCAST and bus.ping(servo_id)]


def only_servo(bus, ids=range(0, 254)):
    """The id of the single servo on the bus; anything else (none, several) fails."""
    found = scan(bus, ids)
    if len(found) != 1:
        raise SetupError(f"Expected exactly one servo on the bus, found {found or 'none'}: connect one at a time")
    return found[0]


def _torque_off(bus, servo_id):
    if bus.read(servo_id, "Torque_Enable"):
        raise SetupError(f"Servo {servo_id} holds torque: disable it first (setup writes only a limp servo)")


def _locked(bus, servo_id, writes):
    """Unlock the EEPROM, apply `writes` [(register, value)], lock it again even if a write failed."""
    bus.write(servo_id, "Lock", 0)
    try:
        for name, value in writes:
            bus.write(servo_id, name, value, allow_eeprom=True)
    finally:
        bus.write(servo_id, "Lock", 1)
    if bus.read(servo_id, "Lock") != 1:
        raise SetupError(f"Servo {servo_id}: the EEPROM did not lock again")


def set_id(bus, new_id, ids=range(0, 254)):
    """Give the only servo on the bus `new_id`. Returns (old id, new id)."""
    if not 1 <= new_id <= 253:
        raise ValueError("Servo ids are 1..253")
    old = only_servo(bus, ids)
    if old == new_id:
        return old, new_id
    _torque_off(bus, old)
    bus.write(old, "Lock", 0)
    try:
        bus.write(old, "ID", new_id, allow_eeprom=True)
    except FeetechError:
        pass  # The servo may acknowledge from its new id; the pings below decide.
    if not bus.ping(new_id) or bus.ping(old):
        raise SetupError(f"Servo {old} did not move to id {new_id}")
    bus.write(new_id, "Lock", 1)
    if bus.read(new_id, "Lock") != 1:
        raise SetupError(f"Servo {new_id}: the EEPROM did not lock again")
    return old, new_id


def centre(bus, servo_id, target=CENTRE):
    """With the joint held at its reference pose, set the servo's Homing_Offset so it reads `target` there
    (Present_Position = actual - Homing_Offset). Returns the offset written."""
    _torque_off(bus, servo_id)
    previous = bus.read(servo_id, "Homing_Offset")
    _locked(bus, servo_id, [("Homing_Offset", 0)])
    actual = bus.read(servo_id, "Present_Position")
    offset = actual - target
    if abs(offset) > 2047:  # The offset holds +-2047 (sign-magnitude): only a reading of 0 lands outside.
        _locked(bus, servo_id, [("Homing_Offset", previous)])
        raise SetupError(f"Servo {servo_id} reads {actual}: turn the joint a little and centre again")
    try:
        _locked(bus, servo_id, [("Homing_Offset", offset)])
        reading = bus.read(servo_id, "Present_Position")
    except FeetechError:
        _locked(bus, servo_id, [("Homing_Offset", previous)])
        raise
    if abs(reading - target) > 2:
        _locked(bus, servo_id, [("Homing_Offset", previous)])
        raise SetupError(f"Servo {servo_id} read {reading} after centring, not {target}: offset {previous} restored")
    return offset


def set_limits(bus, servo_id, low, high):
    """The servo's own position limits (steps): it refuses goals outside them, a backstop under the receiver's."""
    if not 0 <= low < high <= RESOLUTION - 1:
        raise ValueError("Position limits are steps, 0 <= low < high <= 4095")
    _torque_off(bus, servo_id)
    _locked(bus, servo_id, [("Min_Position_Limit", low), ("Max_Position_Limit", high)])
    read = (bus.read(servo_id, "Min_Position_Limit"), bus.read(servo_id, "Max_Position_Limit"))
    if read != (low, high):
        raise SetupError(f"Servo {servo_id} reads limits {read}, not {(low, high)}")
    return read
