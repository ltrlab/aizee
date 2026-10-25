"""Feetech STS-series serial bus servos (the neck's two STS3250s) over a half-duplex TTL adapter.

Protocol ("protocol 0", little-endian): an instruction is FF FF id length instruction params... checksum and a
status reply is FF FF id length error params... checksum, where length counts the bytes after it and
checksum = ~(id + length + instruction-or-error + params) & 0xFF.

Every reply is checked for header, id, length and checksum; a missing, short, malformed or wrong-id reply raises
`FeetechError` and nothing is retried or reopened here, so a caller latches a stop rather than reusing old data.
EEPROM (addresses below 40) is written only on request (`allow_eeprom=True`): ids, baud rates, limits and gains
persist in the servo across power cycles.
"""
from dataclasses import dataclass
import math
import time

HEADER = b"\xff\xff"
BROADCAST = 0xFE
PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
RESOLUTION = 4096  # Steps per turn (12-bit magnetic encoder).
MODELS = {2825: "sts3250", 777: "sts3215"}
BAUD_RATES = {1_000_000: 0, 500_000: 1, 250_000: 2, 128_000: 3, 115_200: 4, 57_600: 5, 38_400: 6, 19_200: 7}
EEPROM_END = 40  # Addresses below this persist; writing them needs the lock open (Lock = 0).
# Status error bits, as the servo reports them in every reply.
ERRORS = {0x01: "input voltage", 0x02: "angle sensor", 0x04: "overheat", 0x08: "overcurrent", 0x20: "overload"}


@dataclass(frozen=True)
class Register:
    address: int
    size: int
    writable: bool = True
    sign_bit: int | None = None  # Sign-magnitude registers: the bit that holds the sign.


# The STS/SMS control table (shared by the STS3215 and STS3250); sources in docs/development/NECK_FEETECH.md.
REGISTERS = {
    "Firmware_Major_Version": Register(0, 1, False), "Firmware_Minor_Version": Register(1, 1, False),
    "Model_Number": Register(3, 2, False), "ID": Register(5, 1), "Baud_Rate": Register(6, 1),
    "Return_Delay_Time": Register(7, 1), "Response_Status_Level": Register(8, 1),
    "Min_Position_Limit": Register(9, 2), "Max_Position_Limit": Register(11, 2),
    "Max_Temperature_Limit": Register(13, 1), "Max_Voltage_Limit": Register(14, 1),
    "Min_Voltage_Limit": Register(15, 1), "Max_Torque_Limit": Register(16, 2), "Phase": Register(18, 1),
    "Unloading_Condition": Register(19, 1), "LED_Alarm_Condition": Register(20, 1),
    "P_Coefficient": Register(21, 1), "D_Coefficient": Register(22, 1), "I_Coefficient": Register(23, 1),
    "Minimum_Startup_Force": Register(24, 2), "CW_Dead_Zone": Register(26, 1), "CCW_Dead_Zone": Register(27, 1),
    "Protection_Current": Register(28, 2), "Angular_Resolution": Register(30, 1),
    "Homing_Offset": Register(31, 2, sign_bit=11), "Operating_Mode": Register(33, 1),
    "Protective_Torque": Register(34, 1), "Protection_Time": Register(35, 1), "Overload_Torque": Register(36, 1),
    "Velocity_P_Coefficient": Register(37, 1), "Over_Current_Protection_Time": Register(38, 1),
    "Velocity_I_Coefficient": Register(39, 1),
    "Torque_Enable": Register(40, 1), "Acceleration": Register(41, 1),
    "Goal_Position": Register(42, 2, sign_bit=15), "Goal_Time": Register(44, 2),
    "Goal_Velocity": Register(46, 2, sign_bit=15), "Torque_Limit": Register(48, 2), "Lock": Register(55, 1),
    "Present_Position": Register(56, 2, False, sign_bit=15), "Present_Velocity": Register(58, 2, False, sign_bit=15),
    "Present_Load": Register(60, 2, False, sign_bit=10), "Present_Voltage": Register(62, 1, False),
    "Present_Temperature": Register(63, 1, False), "Status": Register(65, 1, False), "Moving": Register(66, 1, False),
    "Present_Current": Register(69, 2, False),
}


class FeetechError(RuntimeError):
    """A reply was missing, malformed, from the wrong servo or flagged an error; the bus state is unknown."""


class NoReply(FeetechError):
    """Nothing (or not enough) came back before the reply deadline."""


class ServoAlarm(FeetechError):
    """A well-formed reply carried status error bits (`alarms`); its data, if any, is still in `params`."""
    def __init__(self, servo_id, alarms, params):
        super().__init__(f"Servo {servo_id} reports {', '.join(alarms)}")
        self.servo_id, self.alarms, self.params = servo_id, alarms, params


def checksum(body):
    return ~sum(body) & 0xFF


def instruction(servo_id, code, params=b""):
    if not 0 <= servo_id <= 0xFE or len(params) > 250:
        raise ValueError("Servo id must be 0..254 and parameters at most 250 bytes")
    body = bytes((servo_id, len(params) + 2, code)) + bytes(params)
    return HEADER + body + bytes((checksum(body),))


def to_signed(raw, sign_bit):
    """Sign-magnitude to int: bit `sign_bit` is the sign, the bits below it the magnitude."""
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if raw >> sign_bit & 1 else magnitude


def from_signed(value, sign_bit):
    if abs(value) >= 1 << sign_bit:
        raise ValueError(f"{value} does not fit a {sign_bit}-bit magnitude")
    return (1 << sign_bit) | -value if value < 0 else value


def decode(register, data):
    raw = int.from_bytes(data, "little")
    return to_signed(raw, register.sign_bit) if register.sign_bit is not None else raw


def encode(register, value):
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError("Register values are integers")
    raw = from_signed(value, register.sign_bit) if register.sign_bit is not None else value
    if not 0 <= raw < 1 << (8 * register.size):
        raise ValueError(f"{value} does not fit {register.size} byte(s)")
    return raw.to_bytes(register.size, "little")


def alarms_of(error):
    unknown = error & ~sum(ERRORS) & 0xFF
    return [name for bit, name in ERRORS.items() if error & bit] + ([f"unknown bits 0x{unknown:02x}"] if unknown else [])


def parse_status(data, servo_id, length):
    """One status packet with `length` parameter bytes from `servo_id`: (error, params)."""
    size = length + 6
    if len(data) != size:
        raise NoReply(f"Servo {servo_id}: expected a {size}-byte reply, got {len(data)} bytes")
    if data[:2] != HEADER or data[2] != servo_id or data[3] != length + 2:
        raise FeetechError(f"Servo {servo_id}: malformed reply {data.hex(' ')}")
    if checksum(data[2:-1]) != data[-1]:
        raise FeetechError(f"Servo {servo_id}: reply checksum mismatch")
    return data[4], data[5:-1]


def span(names):
    """The contiguous address range covering registers `names`: (first address, byte count)."""
    registers = [REGISTERS[name] for name in names]
    start = min(r.address for r in registers)
    return start, max(r.address + r.size for r in registers) - start


class FeetechBus:
    """One half-duplex bus on an open transport: `write(bytes)`, `read(n)` returning at most n bytes within its
    own short timeout, and `reset_input_buffer()`, as pyserial's Serial has. Requests go strictly one at a time.

    `echo`: the adapter loops each request back onto RX before the reply (`detect_echo` finds out once); the echo
    must then match the request exactly."""
    def __init__(self, transport, *, timeout_s=.02, echo=False, clock=time.perf_counter):
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= .5:
            raise ValueError("Reply timeout must be in (0, 0.5] s")
        self.transport, self.timeout_s, self.echo, self.clock = transport, timeout_s, echo, clock

    def _read_exact(self, size, deadline):
        data = bytearray()
        while len(data) < size and self.clock() < deadline:
            data.extend(self.transport.read(size - len(data)))
        return bytes(data)

    def _send(self, request):
        """Send `request`, consume its echo if the adapter has one, and return the reply deadline."""
        self.transport.reset_input_buffer()
        self.transport.write(request)
        deadline = self.clock() + self.timeout_s
        if self.echo:
            echoed = self._read_exact(len(request), deadline)
            if echoed != request:
                raise FeetechError(f"Adapter echo mismatch: sent {request.hex(' ')}, read {echoed.hex(' ')}")
        return deadline

    def _transact(self, servo_id, code, params, reply_length):
        deadline = self._send(instruction(servo_id, code, params))
        reply = self._read_exact(reply_length + 6, deadline)
        if not reply:
            raise NoReply(f"Servo {servo_id}: no reply within {self.timeout_s * 1000:.0f} ms")
        error, data = parse_status(reply, servo_id, reply_length)
        if error:
            raise ServoAlarm(servo_id, alarms_of(error), data)
        return data

    def detect_echo(self, probe_id=0xFD):
        """Whether the adapter echoes requests: ping an id no servo should hold and see what comes back.
        Sets and returns `echo`; anything but silence or the exact request raises."""
        request = instruction(probe_id, PING)
        self.transport.reset_input_buffer()
        self.transport.write(request)
        heard = self._read_exact(len(request) + 6, self.clock() + self.timeout_s)
        if heard and heard != request:
            raise FeetechError(f"Unexpected bytes after pinging unused id {probe_id}: {heard.hex(' ')}")
        self.echo = heard == request
        return self.echo

    def ping(self, servo_id):
        """True if `servo_id` answers, False on silence; a malformed or alarmed reply still raises."""
        if servo_id == BROADCAST:
            raise ValueError("Ping one id at a time: replies to a broadcast ping can collide")
        try:
            self._transact(servo_id, PING, b"", 0)
        except NoReply:
            return False
        return True

    def read_block(self, servo_id, address, count):
        if servo_id == BROADCAST or not 0 <= address < 256 or not 0 < count <= 128:
            raise ValueError("Read one servo, address 0..255, 1..128 bytes")
        return self._transact(servo_id, READ, bytes((address, count)), count)

    def read(self, servo_id, name):
        register = REGISTERS[name]
        return decode(register, self.read_block(servo_id, register.address, register.size))

    def read_many(self, servo_id, names):
        """Several registers of one servo in a single read of the span covering them."""
        start, count = span(names)
        block = self.read_block(servo_id, start, count)
        return {name: decode(REGISTERS[name], block[REGISTERS[name].address - start:][:REGISTERS[name].size])
                for name in names}

    def write(self, servo_id, name, value, *, allow_eeprom=False):
        """Write one register and wait for the servo's acknowledgement (none from a broadcast write)."""
        register = REGISTERS[name]
        if not register.writable:
            raise ValueError(f"{name} is read-only")
        if register.address < EEPROM_END and not allow_eeprom:
            raise PermissionError(f"{name} is in EEPROM: pass allow_eeprom=True to change it persistently")
        params = bytes((register.address,)) + encode(register, value)
        if servo_id == BROADCAST:
            self._send(instruction(servo_id, WRITE, params))
            return
        self._transact(servo_id, WRITE, params, 0)

    def sync_read(self, servo_ids, names):
        """Registers `names` (one contiguous span) from several servos in one request; each answers in turn, in
        the order asked: {servo_id: {name: value}}."""
        ids = list(servo_ids)
        if not ids or len(set(ids)) != len(ids) or BROADCAST in ids:
            raise ValueError("Sync read needs distinct servo ids")
        start, count = span(names)
        deadline = self._send(instruction(BROADCAST, SYNC_READ, bytes((start, count)) + bytes(ids)))
        size = count + 6
        replies = self._read_exact(size * len(ids), deadline)
        values = {}
        for index, servo_id in enumerate(ids):
            reply = replies[index * size:(index + 1) * size]
            if len(reply) < 3 or reply[2] != servo_id:  # Silence, or the next servo's reply in its place.
                raise NoReply(f"Sync read of {', '.join(names)}: no reply from servo {servo_id}")
            error, block = parse_status(reply, servo_id, count)
            if error:
                raise ServoAlarm(servo_id, alarms_of(error), block)
            values[servo_id] = {name: decode(REGISTERS[name], block[REGISTERS[name].address - start:]
                                             [:REGISTERS[name].size]) for name in names}
        return values

    def sync_write(self, name, values):
        """One RAM register on several servos in one request; servos do not acknowledge it."""
        register = REGISTERS[name]
        if not register.writable or register.address < EEPROM_END:
            raise ValueError(f"Sync write is for writable RAM registers, not {name}")
        if not values or BROADCAST in values:
            raise ValueError("Sync write needs servo ids")
        params = bytes((register.address, register.size))
        for servo_id, value in values.items():
            params += bytes((servo_id,)) + encode(register, value)
        self._send(instruction(BROADCAST, SYNC_WRITE, params))


def steps_to_rad(steps):
    return steps * (2 * math.pi / RESOLUTION)
