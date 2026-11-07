"""robot-platform neck: the neck's Feetech bus servos.

Read only: ports, probe, watch. One-off setup, writing the servo's EEPROM only with --confirm: set-id (one servo
on the bus), centre (the joint held at its reference pose), limits. The head's files for Helm (read only from the
servos): save-reference (at the reference pose, after centring) and sweep-limits (each joint turned by hand).
"""

import argparse
import json
import sys

from .feetech import FeetechError
from .setup import SetupError


def _ids(text):
    ids = set()
    for part in text.split(','):
        first, _, last = part.partition('-')
        ids.update(range(int(first), int(last or first) + 1))
    if not ids or min(ids) < 0 or max(ids) > 253:
        raise argparse.ArgumentTypeError('ids are 0..253, e.g. 1,2 or 0-20')
    return sorted(ids)


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='robot-platform neck', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('ports', help='list serial ports with their USB ids')
    commands = (
        ('probe', 'find servos and print what each holds (JSON); writes nothing'),
        ('watch', 'print positions while you turn the joints by hand; writes nothing'),
        ('set-id', 'give the only servo on the bus a new id (EEPROM; needs --confirm)'),
        ('centre', 'with the joint at its reference pose, make it read mid-turn (EEPROM; needs --confirm)'),
        ('limits', "set a servo's own position limits in steps (EEPROM; needs --confirm)"),
        ('save-reference', "save the head's reference: identities and readings at the reference pose"),
        ('sweep-limits', "save the head's joint limits while you turn each joint to both ends by hand"),
    )
    for name, text in commands:
        command = sub.add_parser(name, help=text)
        command.add_argument('--port', required=True, help='serial device, e.g. /dev/ttyACM0 or COM5')
        command.add_argument('--baud', type=int, default=1_000_000)
        if name in ('probe', 'watch'):
            command.add_argument('--ids', type=_ids, default=_ids('0-20' if name == 'probe' else '1,2'))
        elif name in ('save-reference', 'sweep-limits'):
            command.add_argument('--directory', required=True, help="Helm's operator files (beside the servo file)")
            command.add_argument('--asset', default='minerva-production-01')
        else:
            command.add_argument('--confirm', action='store_true', help='write it (without: say what would happen)')
    sub.choices['watch'].add_argument('--hz', type=float, default=10)
    sub.choices['watch'].add_argument('--seconds', type=float, default=60)
    sub.choices['set-id'].add_argument('--to', type=int, required=True, help='the new id, 1..253')
    for name in ('centre', 'limits'):
        sub.choices[name].add_argument('--id', type=int, required=True)
    sub.choices['limits'].add_argument('--low', type=int, required=True)
    sub.choices['limits'].add_argument('--high', type=int, required=True)
    sub.choices['save-reference'].add_argument('--usb-serial', help="the adapter's USB serial, if ports cannot tell")
    sub.choices['sweep-limits'].add_argument('--seconds', type=float, default=60)
    args = parser.parse_args(argv)
    try:
        if args.command == 'ports':
            from .probe import list_ports

            print(json.dumps(list_ports(), indent=2))
            return 0
        writes = args.command in ('set-id', 'centre', 'limits')
        if writes and not args.confirm:
            what = {
                'set-id': f'give the only servo on the bus id {getattr(args, "to", "")}',
                'centre': f'centre servo {getattr(args, "id", "")} at its current pose',
                'limits': f'limit servo {getattr(args, "id", "")} to steps '
                f'{getattr(args, "low", "")}..{getattr(args, "high", "")}',
            }[args.command]
            print(f'Would {what} (written to its EEPROM, kept across power cycles). Add --confirm to do it.')
            return 0
        from .probe import open_bus, probe, usb_serial_of, watch

        bus = open_bus(args.port, args.baud)
        try:
            if args.command in ('save-reference', 'sweep-limits'):
                print(json.dumps(_save_file(args, bus, usb_serial_of), indent=2))
            elif args.command == 'probe':
                print(json.dumps(probe(bus, args.ids), indent=2))
            elif args.command == 'watch':
                seen = watch(bus, args.ids, hz=args.hz, seconds=args.seconds)
                print(json.dumps({'range_steps': {str(i): list(r) for i, r in seen.items()}}, indent=2))
            else:
                from . import setup

                if args.command == 'set-id':
                    old, new = setup.set_id(bus, args.to)
                    print(json.dumps({'id': {'was': old, 'now': new}}))
                elif args.command == 'centre':
                    offset = setup.centre(bus, args.id)
                    print(
                        json.dumps(
                            {'id': args.id, 'homing_offset': offset, 'reads': bus.read(args.id, 'Present_Position')}
                        )
                    )
                else:
                    print(
                        json.dumps({'id': args.id, 'limits_steps': setup.set_limits(bus, args.id, args.low, args.high)})
                    )
        finally:
            bus.transport.close()
        return 0
    except (FeetechError, SetupError, OSError, ValueError) as error:
        print(f'neck {args.command}: {error}', file=sys.stderr)
        return 1


def _save_file(args, bus, usb_serial_of):
    """The head's reference or limits file, where Helm's Tune reads it (neck.head.files)."""
    from datetime import datetime, timezone
    from robot_platform.ops.release import ROOT
    from . import head, setup
    from .probe import watch

    profile = head.load_profile(ROOT / head.PROFILE)
    paths = head.files(args.directory, args.asset)
    now = datetime.now(timezone.utc)
    if args.command == 'save-reference':
        document = setup.reference_document(
            bus,
            profile,
            asset_id=args.asset,
            device=args.port,
            now=now,
            usb_serial=args.usb_serial or usb_serial_of(args.port),
        )
        path = paths['reference']
    else:
        reference = head.load_document(paths['reference'])
        if reference is None:
            raise setup.SetupError("Save the head's reference first (neck save-reference)")
        print(
            'Turn each head joint slowly to both ends by hand (the torque is off); Ctrl-C ends early.', file=sys.stderr
        )
        seen = watch(
            bus,
            [m['id'] for m in profile['motors']],
            seconds=args.seconds,
            emit=lambda line: print(line, file=sys.stderr),
        )
        document = setup.limits_document(seen, profile, reference, now=now)
        path = paths['limits']
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=1) + '\n', encoding='utf-8')
    return {
        'saved': str(path),
        **({'joints': document['joints']} if 'joints' in document else {'motors': document['motors']}),
    }


if __name__ == '__main__':
    sys.exit(main())
