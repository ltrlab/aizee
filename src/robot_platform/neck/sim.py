"""Simulated STS3250s on one bus that speak the byte protocol, for tests and rehearsal; no hardware.

`SimulatedBus` is a transport (`write`, `read`, `reset_input_buffer`) holding servos by id. Each servo keeps the
STS register memory and a simple motion model, not the real servo's dynamics: with torque on, Present_Position slews
toward Goal_Position (clamped to its position limits) at Goal_Velocity, or its no-load speed when that is 0, within
its Acceleration; with torque off it stays where a hand puts it (`move_by_hand`). Requests are logged, so a test can
prove a tool only read. Faults per servo: `silent`, `corrupt` (bad reply checksum), `error_bits`.
"""

import time

from .feetech import BROADCAST, HEADER, PING, READ, REGISTERS, SYNC_READ, SYNC_WRITE, WRITE, checksum, decode, encode

NO_LOAD_SPEED = 2900  # Steps/s at 12 V (~255 deg/s): a stand-in until the real servo is measured.


class SimulatedServo:
    def __init__(self, servo_id, *, position=2048, model=2825, firmware=(3, 10), voltage_dv=120, temperature_c=30):
        self.memory = bytearray(128)
        self.silent, self.corrupt, self.error_bits = False, False, 0
        self.velocity = 0.0  # Steps/s, the motion model's state.
        self.eeprom_writes_locked = 0  # EEPROM writes made while Lock was closed (a real servo would not keep them).
        defaults = {
            'Firmware_Major_Version': firmware[0],
            'Firmware_Minor_Version': firmware[1],
            'Model_Number': model,
            'ID': servo_id,
            'Baud_Rate': 0,
            'Return_Delay_Time': 0,
            'Response_Status_Level': 1,
            'Min_Position_Limit': 0,
            'Max_Position_Limit': 4095,
            'Max_Temperature_Limit': 70,
            'Max_Voltage_Limit': 140,
            'Min_Voltage_Limit': 40,
            'Max_Torque_Limit': 1000,
            'P_Coefficient': 32,
            'D_Coefficient': 32,
            'I_Coefficient': 0,
            'Protection_Current': 500,
            'Angular_Resolution': 1,
            'Operating_Mode': 0,
            'Torque_Enable': 0,
            'Torque_Limit': 1000,
            'Lock': 1,
            'Present_Position': position,
            'Goal_Position': position,
            'Present_Voltage': voltage_dv,
            'Present_Temperature': temperature_c,
        }
        for name, value in defaults.items():
            self.set(name, value)

    @property
    def servo_id(self):
        return self.memory[REGISTERS['ID'].address]

    def get(self, name):
        register = REGISTERS[name]
        return decode(register, self.memory[register.address : register.address + register.size])

    def set(self, name, value):
        register = REGISTERS[name]
        self.memory[register.address : register.address + register.size] = encode(register, value)

    def write(self, address, data):
        if address < 40 and self.get('Lock'):
            self.eeprom_writes_locked += 1
        offset = self.get('Homing_Offset')
        self.memory[address : address + len(data)] = data
        # Present_Position = actual - Homing_Offset (as Feetech servos report it), so a new offset shifts the reading.
        shift = offset - self.get('Homing_Offset')
        if shift:
            self.set('Present_Position', (self.get('Present_Position') + shift) % 4096)

    def move_by_hand(self, position):
        """An outside hand turns the output; a servo holding torque does not give way."""
        if not self.get('Torque_Enable'):
            self.set('Present_Position', int(position))
            self.velocity = 0.0

    def advance(self, dt):
        if not self.get('Torque_Enable') or dt <= 0:
            self.velocity = 0.0
            self.set('Moving', 0)
            return
        low, high = self.get('Min_Position_Limit'), self.get('Max_Position_Limit')
        goal = self.get('Goal_Position')
        if low < high:
            goal = min(max(goal, low), high)
        position = self.get('Present_Position')
        speed = abs(self.get('Goal_Velocity')) or NO_LOAD_SPEED
        accel = self.get('Acceleration') * 100 or float('inf')  # Units of 100 steps/s^2; 0 is unlimited.
        error = goal - position
        # Fastest speed that can still stop at the goal, capped by the commanded speed.
        stoppable = (2 * accel * abs(error)) ** 0.5 if accel != float('inf') else speed
        wanted = min(speed, stoppable, abs(error) / dt) * (1 if error > 0 else -1)
        step = accel * dt
        self.velocity = min(max(wanted, self.velocity - step), self.velocity + step) if step != float('inf') else wanted
        moved = position + self.velocity * dt
        if (goal - moved) * error <= 0:
            moved, self.velocity = goal, 0.0
        self.set('Present_Position', int(round(moved)))
        self.set('Present_Velocity', int(round(self.velocity)))
        self.set('Moving', int(self.velocity != 0))


class SimulatedBus:
    """The transport side of a bus of `SimulatedServo`s. `clock=None` freezes time (call `advance`); a clock makes
    the servos move in real time between requests. `echo` loops each request back first, as some adapters do."""

    def __init__(self, servos, *, clock=None, echo=False):
        self.servos = {servo.servo_id: servo for servo in servos}
        self.clock, self.echo = clock, echo
        self.last = clock() if clock else None
        self.pending = bytearray()
        self.log = []  # (instruction, id, params) of every well-formed request.

    def advance(self, dt):
        for servo in self.servos.values():
            servo.advance(dt)

    def _tick(self):
        if self.clock:
            now = self.clock()
            self.advance(now - self.last)
            self.last = now

    def reset_input_buffer(self):
        self.pending.clear()

    def close(self):
        """As a serial port closes: nothing to release here."""

    def read(self, size):
        out = bytes(self.pending[:size])
        del self.pending[:size]
        return out

    def _reply(self, servo, params=b''):
        if servo.silent:
            return
        body = bytes((servo.servo_id, len(params) + 2, servo.error_bits)) + bytes(params)
        self.pending.extend(HEADER + body + bytes(((checksum(body) + servo.corrupt) & 0xFF,)))

    def write(self, request):
        self._tick()
        if self.echo:
            self.pending.extend(request)
        if len(request) < 6 or request[:2] != HEADER or request[3] != len(request) - 4:
            return  # A real servo ignores what it cannot frame.
        if checksum(request[2:-1]) != request[-1]:
            return
        servo_id, code, params = request[2], request[4], bytes(request[5:-1])
        self.log.append((code, servo_id, params))
        servo = self.servos.get(servo_id)
        if code == PING and servo:
            self._reply(servo)
        elif code == READ and servo and len(params) == 2:
            self._reply(servo, servo.memory[params[0] : params[0] + params[1]])
        elif code == WRITE and len(params) >= 2:
            for target in list(self.servos.values()) if servo_id == BROADCAST else [servo] if servo else []:
                target.write(params[0], params[1:])
            self.servos = {s.servo_id: s for s in self.servos.values()}  # A written ID answers at once.
            if servo:
                self._reply(servo)
        elif code == SYNC_READ and servo_id == BROADCAST and len(params) >= 3:
            address, count = params[0], params[1]
            for target_id in params[2:]:
                if target_id in self.servos:
                    self._reply(self.servos[target_id], self.servos[target_id].memory[address : address + count])
        elif code == SYNC_WRITE and servo_id == BROADCAST and len(params) >= 2:
            address, count = params[0], params[1]
            for offset in range(2, len(params) - count, count + 1):
                target = self.servos.get(params[offset])
                if target:
                    target.write(address, params[offset + 1 : offset + 1 + count])

    def instructions(self):
        """The instruction codes received, in order (a read-only tool shows only PING and READ)."""
        return [code for code, _, _ in self.log]


def realtime_bus(servos, echo=False):
    return SimulatedBus(servos, clock=time.perf_counter, echo=echo)
