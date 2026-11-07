"""Gripper force feedback: the follower gripper's grip, felt in the leader's trigger as a wall.

The robot loop decides when each gripper holds something, from its measured effort (the squeeze, in its closing
direction), only while the arm follows the leaders (TELEOP):
- A grip starts once the squeeze has stayed past a grip threshold, with the gripper still, for a hold time.
- It ends once the squeeze has stayed near nothing for a while (the operator let go), or the gripper moves fast
  (it opened, or the object slipped).

While a gripper holds something, its trigger meets a wall: the trigger reading that maps to the gripper's measured
position (teleop.absolute_mapping.reading_for), which is where the object stopped the jaws. The operator PC renders
the wall as a spring from its own trigger readings, at its sample rate (teleop.leader_bridge):
- There is no push at the wall, and the push grows as the trigger is squeezed past it, reaching the operator's
  maximum at `wall_deg` degrees.
- So the push grows with the squeeze the robot applies (both grow with the trigger past the wall), and it can
  never push the trigger open past the object.
- The robot only says where the wall is, and moves it as the jaws move.
- Until 2026-10-24 the loop pushed directly: on a lightly held trigger, the push opened it and let the grasp go,
  over and over (the pulses).

The PC turns the push into a goal current on the trigger servo, in the direction the leader profile says opens it,
and nothing else. The other servos stay torque-off, so feedback can only push the trigger open, never pull the
operator's hand closed. The firmware caps the current, zeroes it at 200 ms without a command, then disables it at
1 s (firmware/openrb_leader).

Measured on Minerva (docs/robots/minerva/GRIPPER_FORCE_FEEDBACK.md), closing effort negative on both grippers:
- A held cube squeezes 1.8-2.0 Nm with the gripper still.
- The gripper's own friction leaves 0.30-0.35 Nm at rest.
- A free close starts stuck for 30-100 ms.
- One degree of trigger moves the jaws about 0.10 rad (the jaws' 5.2 rad over the trigger's 0.88-0.94 rad sweep),
  about 2.5 Nm of squeeze against an object, so a held cube (about 2 Nm) sits about 0.8 degrees past the wall.
- The operator holds close to the wall: in live-20261024T172641Z the median was 0.08-0.15 degrees past it
  (0.40-0.47 Nm), only 16-28 mA at a 0.8-degree wall of 150 mA. Asked for more, the default is stiffer:
  the boards' 200 mA at 0.5 degrees (about 1.3 Nm), found within 70 ms. One encoder count of trigger
  (0.088 degrees) is then about 35 mA.
"""

import math
import struct

FF_CMD = 0xCC
FF_DISABLE = -32768  # INT16_MIN: that servo's torque off, freely backdrivable.
FF_HW_CAP_MA = 200  # Firmware cap per servo; the leader rail needs its 5 V supply for more than one or two servos.
SERVOS = 7

DEFAULTS = dict(
    enabled=False,
    closing_effort_sign=-1,
    grip_nm=0.5,
    hold_ms=70.0,
    wall_deg=0.5,
    cap_ma=200.0,
    still_rad_s=0.3,
    slip_rad_s=1.0,
    release_nm=0.15,
    release_ms=250.0,
)
BOUNDS = dict(
    grip_nm=(0.1, 3.0),
    hold_ms=(0.0, 500.0),
    wall_deg=(0.2, 20.0),
    cap_ma=(0.0, FF_HW_CAP_MA),
    still_rad_s=(0.0, 2.0),
    slip_rad_s=(0.1, 5.0),
    release_nm=(0.0, 3.0),
    release_ms=(0.0, 2000.0),
)
# What the operator sets live from Helm and Helm saves: the strength (0-1) and the feel.
SETTINGS = ('strength', 'cap_ma', 'grip_nm', 'hold_ms', 'wall_deg')
SLIP_S = 0.03  # Moving faster than slip_rad_s for this long ends a grip.
SQUEEZE_S = 0.03  # The squeeze's low-pass time constant: drive noise, not the grip's own changes.
MAX_STEP_S = 0.1  # A late cycle counts as at most this long.


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 255 if crc & 128 else (crc << 1) & 255
    return crc


def ff_frame(currents):
    """The firmware's FF_CURRENT command: 0xCC, the servo count, int16 mA each (FF_DISABLE turns one off), CRC8."""
    if len(currents) != SERVOS:
        raise ValueError(f'FF_CURRENT carries {SERVOS} servo currents')
    values = []
    for value in currents:
        if type(value) is not int or not (value == FF_DISABLE or -FF_HW_CAP_MA <= value <= FF_HW_CAP_MA):
            raise ValueError(f'A leader servo current is an integer within +-{FF_HW_CAP_MA} mA, or FF_DISABLE')
        values.append(value)
    body = bytes((FF_CMD, SERVOS)) + struct.pack('<7h', *values)
    return body + bytes((crc8(body),))


def validate_config(block):
    """The teleop profile's optional 'grip_feedback' block over the defaults; raises ValueError when out of bounds."""
    config = dict(DEFAULTS)
    if block is None:
        return config
    if not isinstance(block, dict) or set(block) - set(DEFAULTS):
        raise ValueError(f'grip_feedback takes only {sorted(DEFAULTS)}')
    config.update(block)
    if type(config['enabled']) is not bool:
        raise ValueError('grip_feedback.enabled is true or false')
    if config['closing_effort_sign'] not in (-1, 1) or type(config['closing_effort_sign']) is not int:
        raise ValueError('grip_feedback.closing_effort_sign is -1 or 1')
    for key, (low, high) in BOUNDS.items():
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'grip_feedback.{key} is between {low:g} and {high:g}')
        config[key] = float(value)
    return config


def trigger_config(profile):
    """Each leader's trigger servo from the leader profile: {leader: (channel 1-7, open sign +-1)}. A leader
    without a 'trigger' entry gets no force feedback."""
    triggers = {}
    for name, item in profile.get('leaders', {}).items():
        trigger = item.get('trigger') if isinstance(item, dict) else None
        if trigger is None:
            continue
        channel, sign = (
            (trigger.get('channel'), trigger.get('open_sign')) if isinstance(trigger, dict) else (None, None)
        )
        if type(channel) is not int or not 1 <= channel <= SERVOS or type(sign) is not int or sign not in (-1, 1):
            raise ValueError(f'Leader {name}: trigger needs a channel 1-{SERVOS} and an open_sign of -1 or 1')
        triggers[name] = (channel, sign)
    return triggers


def trigger_currents(strength_ma, trigger):
    """The seven servo currents for one board: the trigger's resist current in its opening direction, every other
    servo off. `strength_ma` is a magnitude; a negative or non-finite one is refused."""
    channel, sign = trigger
    if type(strength_ma) not in (int, float) or not math.isfinite(strength_ma) or strength_ma < 0:
        raise ValueError('A trigger resist strength is a finite number of mA, not negative')
    currents = [FF_DISABLE] * SERVOS
    currents[channel - 1] = sign * int(round(min(float(strength_ma), FF_HW_CAP_MA)))
    return currents


def validate_settings(payload):
    """The operator's live settings, any of them: strength 0-1, cap_ma 0-200 mA, and the feel (grip_nm, hold_ms,
    wall_deg) within the profile's bounds."""
    if not isinstance(payload, dict) or set(payload) - set(SETTINGS):
        raise ValueError('Force feedback settings are ' + ', '.join(SETTINGS))
    bounds = dict(BOUNDS, strength=(0.0, 1.0))
    labels = {
        'strength': 'Force feedback strength is between 0 and 1',
        'cap_ma': f'The maximum trigger force is 0-{FF_HW_CAP_MA} mA',
    }
    result = {}
    for key, value in payload.items():
        low, high = bounds[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(labels.get(key, f'{key} is between {low:g} and {high:g}'))
        result[key] = float(value)
    return result


def past_wall(wall, reading, opens):
    """How far (rad) a trigger reading has been squeezed past its wall: toward closing, opposite `opens` (the
    reading direction that opens it); never negative. Readings a whole turn apart are the same angle."""
    return max(0.0, opens * math.remainder(wall - reading, 2 * math.pi))


class GripFeedback:
    """Each gripper's grip from the receiver's latest feedback rows, and while it grips a wall for its trigger; the
    push the trigger feels (as the loop estimates it) and its share of the maximum (Helm's force bars), kept even
    while feedback is off."""

    GRIPPERS = {'left_gripper': 'left', 'right_gripper': 'right'}

    def __init__(self, config):
        self.config = validate_config(config)
        self.enabled = self.config['enabled']
        self.strength = 1.0  # The operator's scale on the push, 0-1, set live from Helm.
        sides = self.GRIPPERS.values()
        self.squeezes = dict.fromkeys(sides, 0.0)  # Low-passed, Nm.
        self.gripping = dict.fromkeys(sides, False)
        self.pushes = dict.fromkeys(sides, 0.0)  # mA, strength included.
        self.levels = dict.fromkeys(sides, 0.0)  # The push against the maximum, before the strength.
        self._held_s = dict.fromkeys(sides, 0.0)  # Still and squeezing past the threshold, so far.
        self._release_s = dict.fromkeys(sides, 0.0)  # A grip's squeeze near nothing, so far.
        self._slip_s = dict.fromkeys(sides, 0.0)  # A grip's gripper moving fast, so far.

    def set(self, *, enabled=None, **settings):
        """Switch feedback on or off, and set any of SETTINGS live."""
        if enabled is not None:
            if type(enabled) is not bool:
                raise ValueError('Force feedback is switched on or off')
            self.enabled = enabled
        values = validate_settings({key: value for key, value in settings.items() if value is not None})
        self.strength = values.pop('strength', self.strength)
        self.config.update(values)

    def reading(self, row):
        """A gripper's squeeze (its effort in the closing direction, Nm, never negative) and speed (rad/s), or None
        when the row carries no usable reading."""
        effort, velocity = row.get('effort'), row.get('velocity')
        if (
            type(effort) not in (int, float)
            or type(velocity) not in (int, float)
            or not math.isfinite(effort)
            or not math.isfinite(velocity)
        ):
            return None
        return max(0.0, self.config['closing_effort_sign'] * effort), abs(velocity)

    def stiffness(self):
        """The wall's spring at full strength, mA per radian of trigger past it: the maximum at wall_deg."""
        return self.config['cap_ma'] / math.radians(self.config['wall_deg'])

    def update(self, rows, *, engaged, dt=0.01, walls=None):
        """{leader: wall or 0.} for this cycle, `dt` seconds after the last, or None when feedback is off (the PC
        then turns the trigger servos off). 0. leaves a trigger free.

        A wall is {'wall': trigger reading (rad), 'k': mA per rad past it, 'cap': mA}, strength included.
        `walls` is {leader: (wall, reading, opens)} from the loop's routes:
        - the reading that maps to the gripper's measured position (where the object stopped the jaws);
        - the trigger's latest reading;
        - the reading direction that opens the trigger.
        A gripping leader missing there gets no wall. The pushes and bars follow the grip either way."""
        c = self.config
        dt = min(max(float(dt), 0.0), MAX_STEP_S) if math.isfinite(dt) else MAX_STEP_S
        follow = dt / (SQUEEZE_S + dt)
        release = min(c['release_nm'], 0.75 * c['grip_nm'])
        k = self.stiffness()
        walls = walls or {}
        by_name = {row.get('name'): row for row in rows or [] if isinstance(row, dict)}
        specs = {}
        for name, side in self.GRIPPERS.items():
            row = by_name.get(name)
            reading = self.reading(row) if row is not None else None
            squeeze, speed = reading if reading is not None else (0.0, math.inf)
            self.squeezes[side] += follow * (squeeze - self.squeezes[side])
            held = self.squeezes[side]
            if reading is None:  # No reading, or a faulted drive: never felt.
                self.gripping[side], self._held_s[side] = False, 0.0
            elif not self.gripping[side]:
                still = held >= c['grip_nm'] and speed <= c['still_rad_s']
                self._held_s[side] = self._held_s[side] + dt if still else 0.0
                if still and self._held_s[side] >= c['hold_ms'] / 1000.0 - 1e-9:
                    self.gripping[side] = True
                    self._release_s[side] = self._slip_s[side] = 0.0
            else:
                self._release_s[side] = self._release_s[side] + dt if held < release else 0.0
                self._slip_s[side] = self._slip_s[side] + dt if speed > c['slip_rad_s'] else 0.0
                if self._release_s[side] >= c['release_ms'] / 1000.0 - 1e-9 or self._slip_s[side] >= SLIP_S - 1e-9:
                    self.gripping[side], self._held_s[side] = False, 0.0
            wall = walls.get(side) if self.gripping[side] else None
            past = past_wall(*wall) if wall is not None else 0.0
            self.levels[side] = min(1.0, past / math.radians(c['wall_deg']))
            self.pushes[side] = self.strength * min(c['cap_ma'], k * past)
            specs[side] = (
                {'wall': float(wall[0]), 'k': k * self.strength, 'cap': c['cap_ma'] * self.strength}
                if wall is not None and engaged
                else 0.0
            )
        if not self.enabled:
            return None
        return specs

    def settings(self):
        return dict({key: self.config[key] for key in SETTINGS if key != 'strength'}, strength=self.strength)

    def snapshot(self):
        return dict(
            enabled=self.enabled,
            strength=self.strength,
            cap_ma=self.config['cap_ma'],
            currents_ma={side: round(value, 1) for side, value in self.pushes.items()},
            levels={side: round(value, 3) for side, value in self.levels.items()},
            squeeze_nm={side: round(value, 3) for side, value in self.squeezes.items()},
            gripping=dict(self.gripping),
            settings=self.settings(),
            config={key: self.config[key] for key in ('still_rad_s', 'slip_rad_s', 'release_nm', 'release_ms')},
        )
