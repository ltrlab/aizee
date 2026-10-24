"""Gripper force feedback: the follower gripper's grip, felt back in the leader's trigger.

The robot loop turns each gripper's measured effort into a resist strength (mA, never negative), only while the arm
follows the leaders (TELEOP). The squeeze is the effort in the gripper's closing direction. The trigger pushes only
while the gripper holds something: the squeeze has stayed past a grip threshold, with the gripper still, for a hold
time. It then pushes at once with a contact push, a share of the maximum force, and rises with the squeeze to the
maximum at the full-force squeeze, smoothed. The grip lasts until the squeeze falls well under the threshold (the
push fading out on the way) or the gripper moves fast again (it opened or the object slipped), so speed blips and
dips inside a hold never pulse it.
The operator tunes all of it live in Helm (Settings, Grippers).

The operator PC turns that strength into a goal current on the leader's trigger servo, in the direction the leader
profile says opens it, and nothing else: the other servos stay torque-off, so feedback can only push the trigger
open against a squeeze, never pull the operator's hand closed. The firmware caps the current and zeroes it at
200 ms without a command, then disables it at 1 s (firmware/openrb_leader).

Measured on Minerva (2026-10-20 to 10-24, docs/development/GRIPPER_FORCE_FEEDBACK.md), closing effort negative on
both grippers (receiver joint space):
- A held cube squeezes 1.8-2.0 Nm, a hard squeeze up to 3.7 Nm, with the gripper still (|v| p90 under 0.1 rad/s).
- At rest the gripper's own friction leaves 0.30-0.35 Nm of squeeze, for seconds at a time.
- A free close starts with the gripper stuck for 30-100 ms before it breaks free and moves at 0.7-2 rad/s.
A plain deadband with a speed gate (until 2026-10-24) read each of those starts as a grip, and flickered on the rest
level: the pulses the operator felt. The grip threshold sits above the rest level and the hold time outlasts most
starts; the defaults follow from replaying the recorded sessions.
"""
import math
import struct

FF_CMD = 0xCC
FF_DISABLE = -32768  # INT16_MIN: that servo's torque off, freely backdrivable.
FF_HW_CAP_MA = 200   # Firmware cap per servo; the leader rail needs its 5 V supply for more than one or two servos.
SERVOS = 7

DEFAULTS = dict(enabled=False, closing_effort_sign=-1, grip_nm=.5, full_nm=1.2, contact_push=.4, hold_ms=100.,
                smoothing_ms=40., cap_ma=150., still_rad_s=.3, slip_rad_s=1.)
BOUNDS = dict(grip_nm=(.1, 3.), full_nm=(.2, 5.), contact_push=(0., 1.), hold_ms=(0., 500.), smoothing_ms=(0., 300.),
              cap_ma=(0., FF_HW_CAP_MA), still_rad_s=(0., 2.), slip_rad_s=(.1, 5.))
# What the operator sets live from Helm and Helm saves: the strength (0-1) and the feel.
SETTINGS = ('strength', 'cap_ma', 'grip_nm', 'full_nm', 'contact_push', 'hold_ms', 'smoothing_ms')
RELEASE_SHARE = .75  # A grip ends once the squeeze stays under this share of the threshold...
RELEASE_S = .03      # ...or the gripper moves faster than slip_rad_s, for this long.
SQUEEZE_S = .03      # The squeeze's low-pass time constant: drive noise, not the grip's own changes.
MAX_STEP_S = .1      # A late cycle counts as at most this long.


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
        channel, sign = (trigger.get('channel'), trigger.get('open_sign')) if isinstance(trigger, dict) else (None, None)
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
    """The operator's live settings, any of them: strength 0-1, cap_ma 0-200 mA, and the feel (grip_nm, full_nm,
    contact_push, hold_ms, smoothing_ms) within the profile's bounds."""
    if not isinstance(payload, dict) or set(payload) - set(SETTINGS):
        raise ValueError('Force feedback settings are ' + ', '.join(SETTINGS))
    bounds = dict(BOUNDS, strength=(0., 1.))
    labels = {'strength': 'Force feedback strength is between 0 and 1',
              'cap_ma': f'The maximum trigger force is 0-{FF_HW_CAP_MA} mA'}
    result = {}
    for key, value in payload.items():
        low, high = bounds[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(labels.get(key, f'{key} is between {low:g} and {high:g}'))
        result[key] = float(value)
    return result


class GripFeedback:
    """Per-gripper resist strengths from the receiver's latest feedback rows, and each gripper's grip as a share of
    the maximum force (Helm's force bars), kept even while feedback is off."""

    GRIPPERS = {'left_gripper': 'left', 'right_gripper': 'right'}

    def __init__(self, config):
        self.config = validate_config(config)
        self.enabled = self.config['enabled']
        self.strength = 1.  # The operator's scale on the push, 0-1, set live from Helm.
        sides = self.GRIPPERS.values()
        self.output = dict.fromkeys(sides, 0.)
        self.levels = dict.fromkeys(sides, 0.)
        self.squeezes = dict.fromkeys(sides, 0.)  # Low-passed, Nm.
        self.gripping = dict.fromkeys(sides, False)
        self._held_s = dict.fromkeys(sides, 0.)     # Still and squeezing past the threshold, so far.
        self._release_s = dict.fromkeys(sides, 0.)  # A grip looking released, so far.

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
        if (type(effort) not in (int, float) or type(velocity) not in (int, float)
                or not math.isfinite(effort) or not math.isfinite(velocity)):
            return None
        return max(0., self.config['closing_effort_sign'] * effort), abs(velocity)

    def update(self, rows, *, engaged, dt=.01):
        """{leader: mA} for this cycle, `dt` seconds after the last, or None when feedback is off (the PC then turns
        the trigger servos off). The force bars' levels follow the grip either way: a picture of it with the
        triggers free."""
        c = self.config
        dt = min(max(float(dt), 0.), MAX_STEP_S) if math.isfinite(dt) else MAX_STEP_S
        tau = c['smoothing_ms'] / 1000.
        fall = dt / (tau + dt) if tau + dt > 0 else 1.
        rise = dt / (tau / 2 + dt) if tau + dt > 0 else 1.  # Contact lands twice as fast as it lets go.
        follow = dt / (SQUEEZE_S + dt)
        full = max(c['full_nm'], c['grip_nm'] + .05)
        by_name = {row.get('name'): row for row in rows or [] if isinstance(row, dict)}
        for name, side in self.GRIPPERS.items():
            row = by_name.get(name)
            reading = self.reading(row) if row is not None else None
            squeeze, speed = reading if reading is not None else (0., math.inf)
            self.squeezes[side] += follow * (squeeze - self.squeezes[side])
            held = self.squeezes[side]
            if reading is None:  # No reading, or a faulted drive: never felt.
                self.gripping[side], self._held_s[side] = False, 0.
            elif not self.gripping[side]:
                still = held >= c['grip_nm'] and speed <= c['still_rad_s']
                self._held_s[side] = self._held_s[side] + dt if still else 0.
                if still and self._held_s[side] >= c['hold_ms'] / 1000. - 1e-9:
                    self.gripping[side], self._release_s[side] = True, 0.
            else:
                letting_go = held < RELEASE_SHARE * c['grip_nm'] or speed > c['slip_rad_s']
                self._release_s[side] = self._release_s[side] + dt if letting_go else 0.
                if self._release_s[side] >= RELEASE_S - 1e-9:
                    self.gripping[side], self._held_s[side] = False, 0.
            # The push as a share of the maximum: the contact push at the threshold, the whole of it at full_nm,
            # and under the threshold fading out to nothing where the grip ends (no push left stuck on).
            release = RELEASE_SHARE * c['grip_nm']
            level = (0. if not self.gripping[side]
                     else c['contact_push'] * max(0., (held - release) / (c['grip_nm'] - release))
                     if held < c['grip_nm']
                     else c['contact_push'] + (1. - c['contact_push']) * min(1., (held - c['grip_nm']) / (full - c['grip_nm'])))
            target = c['cap_ma'] * self.strength * level if self.enabled and engaged else 0.
            self.output[side] += (rise if target > self.output[side] else fall) * (target - self.output[side])
            if self.output[side] < 1.:  # The boards take whole mA.
                self.output[side] = 0.
            # The bar: the grip against the maximum force, before the strength (which scales only the push).
            self.levels[side] += (rise if level > self.levels[side] else fall) * (level - self.levels[side])
            if self.levels[side] < .002:
                self.levels[side] = 0.
        if not self.enabled:
            return None
        return {side: round(value, 1) for side, value in self.output.items()}

    def settings(self):
        return dict({key: self.config[key] for key in SETTINGS if key != 'strength'}, strength=self.strength)

    def snapshot(self):
        return dict(enabled=self.enabled, strength=self.strength, cap_ma=self.config['cap_ma'],
                    currents_ma={side: round(value, 1) for side, value in self.output.items()},
                    levels={side: round(value, 3) for side, value in self.levels.items()},
                    squeeze_nm={side: round(value, 3) for side, value in self.squeezes.items()},
                    gripping=dict(self.gripping), settings=self.settings(),
                    config={key: self.config[key] for key in ('still_rad_s', 'slip_rad_s')})
