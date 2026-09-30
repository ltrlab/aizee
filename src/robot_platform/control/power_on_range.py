"""Set each drive's power-on position range to −π…π (zero_sta 1) and save it, every drive disabled.

A RobStride drive reports its angle within one turn when it powers up: 0…2π by
factory default (zero_sta 0), so a joint resting just below its zero reads a
whole turn high. −π…π centres that window on the zero, where every joint rests
at power-up. This writes that one fixed value and saves it: no enable, zero,
motion or other parameter, and no write or save is ever repeated automatically.
Every drive is read before any is written. The setting takes effect at the next
power-up, so a power cycle and a stopped scan (Minerva Check) verify it.

    python3 -m robot_platform.control.power_on_range --confirm   (on the robot, from the active release)
"""
import argparse
import json
import math
import os
import signal
import time

from .joint_capture import POWER_ON_RANGE
from .mechanical_zero import StoppedMaintenance

POSITION = 0x7019


def power_on_exchange(exchange):
    """The session's exchange, limited to stopped reads, the fixed power-on range write and the save."""
    def call(bus, frame, kind, parameter):
        if kind not in {'identity', 'stop', 'version', 'read', 'power_on_range', 'save'}:
            raise ValueError('The power-on range permits its one write and the save only')
        if kind == 'read' and parameter not in {0x7005, POSITION, POWER_ON_RANGE}:
            raise ValueError('Parameter is not allowlisted')
        return exchange(bus, frame, kind, parameter)
    return call


def set_power_on_range(reader, motors, result, *, checkpoint=lambda: None, sleep=time.sleep):
    """Write zero_sta 1 and save it on each drive that reads 0; `result['motors']` records every step."""
    rows = result['motors'] = []
    try:
        for motor in motors:  # Every drive answers before any is written.
            rows.append({'name': motor['name'], 'zero_sta_before': reader.request(motor, 'read', POWER_ON_RANGE),
                         'position_before_rad': reader.request(motor, 'read', POSITION)})
        checkpoint()
        for motor, row in zip(motors, rows):
            if row['zero_sta_before'] == 1:
                row['already_set'] = True
                continue
            if row['zero_sta_before'] != 0:
                raise RuntimeError(f"{motor['name']}: zero_sta reads {row['zero_sta_before']}, neither 0 nor 1")
            row['write_requested'] = True
            checkpoint()
            try:
                row['write_reply'] = reader.request(motor, 'power_on_range')
            except TimeoutError:
                row['write_response_missing'] = True  # Its readback decides.
            row['zero_sta_written'] = reader.request(motor, 'read', POWER_ON_RANGE)
            if row['zero_sta_written'] != 1:
                raise RuntimeError(f"{motor['name']}: zero_sta reads {row['zero_sta_written']} after the write; "
                                   'nothing was saved')
            row['save_requested'] = True
            checkpoint()
            try:
                row['save_reply'] = reader.request(motor, 'save')
            except TimeoutError:
                row['save_response_missing'] = True  # The power cycle shows whether it was kept.
            checkpoint()
            sleep(.25)
            row['zero_sta_after'] = reader.request(motor, 'read', POWER_ON_RANGE)
            row['position_after_rad'] = reader.request(motor, 'read', POSITION)
            change = row['position_after_rad'] - row['position_before_rad']
            # The drive may re-express its angle in the new window at once: a whole turn, and nothing else.
            if min(abs(change), abs(abs(change) - 2 * math.pi)) > .01:
                raise RuntimeError(f"{motor['name']}: its position changed by {change:+.4f} rad across the write")
            if row['zero_sta_after'] != 1:
                raise RuntimeError(f"{motor['name']}: zero_sta reads {row['zero_sta_after']} after the save")
        result['written'] = [row['name'] for row in rows if row.get('write_requested')]
        result['power_cycle_verified'] = False
    finally:
        stops = result['final_stops'] = {}
        for motor in motors:
            try:
                stops[motor['name']] = reader.request(motor, 'stop')  # Still fault-free Reset.
            except Exception as exc:
                stops[motor['name']] = {'error': str(exc)}
        checkpoint()
    failed = [name for name, reply in stops.items() if 'error' in reply]
    if failed:
        raise RuntimeError('Final stopped replies failed: ' + ', '.join(failed))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--confirm', action='store_true', help='write and save zero_sta 1 on every drive')
    args = parser.parse_args(argv)
    if not args.confirm:
        parser.error('Writes each drive\'s flash: confirm with --confirm')
    from robot_platform.ops.probe import EVIDENCE, identity
    from .joint_capture import run_capture
    # A dropped SSH session must not kill the run between CAN up and CAN down.
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    device, reference = identity()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    path = EVIDENCE / f"power-on-range-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}.json"
    with path.open('x', encoding='utf-8') as output:
        def save(result):
            output.seek(0)
            output.write(json.dumps(result, indent=1, allow_nan=False) + '\n')
            output.truncate()
            output.flush()
            os.fsync(output.fileno())

        def maintain(exchange, motors, result):
            set_power_on_range(StoppedMaintenance(power_on_exchange(exchange)), motors, result,
                               checkpoint=lambda: save(result))

        report = run_capture(device, reference, maintain=maintain)
        save(report)
    print(json.dumps({'success': report.get('success'), 'error': report.get('error'), 'evidence': str(path),
                      'written': report.get('written'),
                      'motors': [{k: row.get(k) for k in ('name', 'zero_sta_before', 'zero_sta_after', 'already_set',
                                                          'write_response_missing', 'save_response_missing',
                                                          'position_before_rad', 'position_after_rad')}
                                 for row in report.get('motors') or []]}, allow_nan=False))
    return 0 if report.get('success') else 1


if __name__ == '__main__':
    raise SystemExit(main())
