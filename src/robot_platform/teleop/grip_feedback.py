"""Gripper force feedback: the follower gripper's squeeze, felt back in the leader's trigger.

The robot loop turns each gripper's measured effort into a resist strength (mA, never negative): only the effort
that squeezes (its closing direction), only while the gripper has nearly stopped (a hold, not a fast move), only
above a deadband, scaled and capped, smoothed, and only while the arm follows the leaders (TELEOP). The operator PC
turns that strength into a goal current on the leader's trigger servo, in the direction the leader profile says
opens it, and nothing else: the other servos stay torque-off, so feedback can only push the trigger open against
a squeeze, never pull the operator's hand closed. The firmware caps the current and zeroes it at 200 ms without
a command, then disables it at 1 s (firmware/openrb_leader).

Measured on Minerva (2026-10-20, docs/development/GRIPPER_FORCE_FEEDBACK.md): a held cube squeezes at -1.1 to
-2.2 Nm; free air reads within about +-0.3 Nm, with brief fast-move spikes to 0.8 Nm; closing effort is negative
on both grippers (receiver joint space). The defaults below follow from that.
"""
import math
import struct

FF_CMD = 0xCC
FF_DISABLE = -32768  # INT16_MIN: that servo's torque off, freely backdrivable.
FF_HW_CAP_MA = 200   # Firmware cap per servo; the leader rail needs its 5 V supply for more than one or two servos.
SERVOS = 7

DEFAULTS = dict(enabled=False, closing_effort_sign=-1, deadband_nm=.5, gain_ma_per_nm=100., cap_ma=150.,
                still_rad_s=.3, smoothing=.4)
BOUNDS = dict(deadband_nm=(0., 3.), gain_ma_per_nm=(0., 400.), cap_ma=(0., FF_HW_CAP_MA), still_rad_s=(0., 2.),
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


class GripFeedback:
    """Per-gripper resist strengths from the receiver's latest feedback rows, smoothed cycle to cycle."""

    GRIPPERS = {'left_gripper': 'left', 'right_gripper': 'right'}

    def __init__(self, config):
        self.config = validate_config(config)
        self.enabled = self.config['enabled']
        self.strength = 1.  # The operator's scale on the gain, 0-1, set live from Helm.
        self.output = {leader: 0. for leader in self.GRIPPERS.values()}

    def set(self, *, enabled=None, strength=None):
        if enabled is not None:
            if type(enabled) is not bool:
                raise ValueError('Force feedback is switched on or off')
            self.enabled = enabled
        if strength is not None:
            if type(strength) not in (int, float) or not math.isfinite(strength) or not 0 <= strength <= 1:
                raise ValueError('Force feedback strength is between 0 and 1')
            self.strength = float(strength)

    def squeeze(self, row):
        """The effort a gripper squeezes with while (nearly) stopped, in Nm; 0 while it moves or pushes open."""
        c = self.config
        effort, velocity = row.get('effort'), row.get('velocity')
        if (type(effort) not in (int, float) or type(velocity) not in (int, float)
                or not math.isfinite(effort) or not math.isfinite(velocity) or abs(velocity) > c['still_rad_s']):
            return 0.
        return max(0., c['closing_effort_sign'] * effort)

    def update(self, rows, *, engaged):
        """{leader: mA} for this cycle, or None when feedback is off (the PC then turns the trigger servos off)."""
        if not self.enabled:
            self.output = {leader: 0. for leader in self.output}
            return None
        c = self.config
        by_name = {row.get('name'): row for row in rows or [] if isinstance(row, dict)}
        for name, leader in self.GRIPPERS.items():
            target = 0.
            row = by_name.get(name)
            if engaged and row is not None:
                excess = self.squeeze(row) - c['deadband_nm']
                target = min(c['cap_ma'], max(0., excess) * c['gain_ma_per_nm'] * self.strength)
            # Smooth both ways: no step when contact starts, no buzz from a noisy reading.
            self.output[leader] += c['smoothing'] * (target - self.output[leader])
            if self.output[leader] < .5:
                self.output[leader] = 0.
        return {leader: round(value, 1) for leader, value in self.output.items()}

    def snapshot(self):
        return dict(enabled=self.enabled, strength=self.strength, currents_ma=dict(self.output),
                    config={k: self.config[k] for k in ('deadband_nm', 'gain_ma_per_nm', 'cap_ma', 'still_rad_s')})
