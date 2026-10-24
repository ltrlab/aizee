"""Gripper force feedback: the follower gripper's squeeze, felt back in the leader's trigger.

The robot loop turns each gripper's measured effort into a resist strength (mA, never negative): only the effort
that squeezes (its closing direction), only while the gripper has nearly stopped (a hold, not a fast move), only
above a deadband, scaled and capped, smoothed, and only while the arm follows the leaders (TELEOP). The operator PC
turns that strength into a goal current on the leader's trigger servo, in the direction the leader profile says
opens it, and nothing else: the other servos stay torque-off, so feedback can only push the trigger open against
a squeeze, never pull the operator's hand closed. The firmware caps the current and zeroes it at 200 ms without
a command, then disables it at 1 s (firmware/openrb_leader).

Measured on Minerva (2026-10-20, docs/development/GRIPPER_FORCE_FEEDBACK.md): a held cube squeezes at -1.1 to
-2.2 Nm; free air reads within about +-0.3 Nm (p95 0.17-0.33), with brief fast-move spikes to 0.8 Nm; closing effort
is negative on both grippers (receiver joint space). The defaults follow from that, made much more sensitive at the
user's request (2026-10-24): the deadband sits just above the free-air noise and 400 mA/Nm feeds a light grip back
(0.5 Nm: 80 mA) and reaches a 150 mA maximum by about 0.7 Nm. The operator's strength scales it; 25 % is the old feel.
"""
import math
import struct

FF_CMD = 0xCC
FF_DISABLE = -32768  # INT16_MIN: that servo's torque off, freely backdrivable.
FF_HW_CAP_MA = 200   # Firmware cap per servo; the leader rail needs its 5 V supply for more than one or two servos.
SERVOS = 7

DEFAULTS = dict(enabled=False, closing_effort_sign=-1, deadband_nm=.3, gain_ma_per_nm=400., cap_ma=150.,
                still_rad_s=.3, smoothing=.4)
BOUNDS = dict(deadband_nm=(0., 3.), gain_ma_per_nm=(0., 1000.), cap_ma=(0., FF_HW_CAP_MA), still_rad_s=(0., 2.),
              smoothing=(.01, 1.))


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
    """The operator's saved settings: {strength 0-1, cap_ma 0-200}, either or both."""
    if not isinstance(payload, dict) or set(payload) - {'strength', 'cap_ma'}:
        raise ValueError('Force feedback settings are strength and cap_ma')
    result = {}
    for key, high, label in (('strength', 1., 'Force feedback strength is between 0 and 1'),
                             ('cap_ma', float(FF_HW_CAP_MA), f'The maximum trigger force is 0-{FF_HW_CAP_MA} mA')):
        if key in payload:
            value = payload[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= high:
                raise ValueError(label)
            result[key] = float(value)
    return result


class GripFeedback:
    """Per-gripper resist strengths from the receiver's latest feedback rows, smoothed cycle to cycle, and each
    gripper's squeeze as a fraction of the maximum force (Helm's force bars), kept even while feedback is off."""

    GRIPPERS = {'left_gripper': 'left', 'right_gripper': 'right'}

    def __init__(self, config):
        self.config = validate_config(config)
        self.enabled = self.config['enabled']
        self.strength = 1.  # The operator's scale on the gain, 0-1, set live from Helm.
        self.output = {leader: 0. for leader in self.GRIPPERS.values()}
        self.levels = {leader: 0. for leader in self.GRIPPERS.values()}
        self.squeezes = {leader: 0. for leader in self.GRIPPERS.values()}

    def set(self, *, enabled=None, strength=None, cap_ma=None):
        if enabled is not None:
            if type(enabled) is not bool:
                raise ValueError('Force feedback is switched on or off')
            self.enabled = enabled
        settings = validate_settings({key: value for key, value in (('strength', strength), ('cap_ma', cap_ma))
                                      if value is not None})
        self.strength = settings.get('strength', self.strength)
        self.config['cap_ma'] = settings.get('cap_ma', self.config['cap_ma'])

    def squeeze(self, row):
        """The effort a gripper squeezes with while (nearly) stopped, in Nm; 0 while it moves or pushes open."""
        c = self.config
        effort, velocity = row.get('effort'), row.get('velocity')
        if (type(effort) not in (int, float) or type(velocity) not in (int, float)
                or not math.isfinite(effort) or not math.isfinite(velocity) or abs(velocity) > c['still_rad_s']):
            return 0.
        return max(0., c['closing_effort_sign'] * effort)

    def update(self, rows, *, engaged):
        """{leader: mA} for this cycle, or None when feedback is off (the PC then turns the trigger servos off).
        The force bars' levels follow the squeeze either way: a picture of the grip even with the triggers free."""
        c = self.config
        by_name = {row.get('name'): row for row in rows or [] if isinstance(row, dict)}
        for name, leader in self.GRIPPERS.items():
            row = by_name.get(name)
            squeeze = self.squeeze(row) if row is not None else 0.
            felt = min(c['cap_ma'], max(0., squeeze - c['deadband_nm']) * c['gain_ma_per_nm'] * self.strength)
            target = felt if self.enabled and engaged else 0.
            # Smooth both ways: no step when contact starts, no buzz from a noisy reading.
            self.output[leader] += c['smoothing'] * (target - self.output[leader])
            if self.output[leader] < .5:
                self.output[leader] = 0.
            # The bar: the robot's squeeze against the force that reaches the maximum at full strength (the
            # strength scales only what the trigger pushes).
            excess = max(0., squeeze - c['deadband_nm']) * c['gain_ma_per_nm']
            level = min(1., excess / c['cap_ma']) if c['cap_ma'] > 0 else (1. if excess > 0 else 0.)
            self.levels[leader] += c['smoothing'] * (level - self.levels[leader])
            self.squeezes[leader] = squeeze
        if not self.enabled:
            return None
        return {leader: round(value, 1) for leader, value in self.output.items()}

    def snapshot(self):
        return dict(enabled=self.enabled, strength=self.strength, cap_ma=self.config['cap_ma'],
                    currents_ma={leader: round(value, 1) for leader, value in self.output.items()},
                    levels={leader: round(value, 3) for leader, value in self.levels.items()},
                    squeeze_nm={leader: round(value, 3) for leader, value in self.squeezes.items()},
                    config={k: self.config[k] for k in ('deadband_nm', 'gain_ma_per_nm', 'still_rad_s')})
