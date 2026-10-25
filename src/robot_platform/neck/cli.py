"""robot-platform neck: the neck's Feetech bus servos, read-only for now (ports, probe, watch)."""
import argparse
import json
import sys

from .feetech import FeetechError


def _ids(text):
    ids = set()
    for part in text.split(","):
        first, _, last = part.partition("-")
        ids.update(range(int(first), int(last or first) + 1))
    if not ids or min(ids) < 0 or max(ids) > 253:
        raise argparse.ArgumentTypeError("ids are 0..253, e.g. 1,2 or 0-20")
    return sorted(ids)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-platform neck", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("ports", help="list serial ports with their USB ids")
    for name, text in (("probe", "find servos and print what each holds (JSON); writes nothing"),
                       ("watch", "print positions while you turn the joints by hand; writes nothing")):
        command = sub.add_parser(name, help=text)
        command.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0 or COM5")
        command.add_argument("--baud", type=int, default=1_000_000)
        command.add_argument("--ids", type=_ids, default=_ids("0-20" if name == "probe" else "1,2"))
    sub.choices["watch"].add_argument("--hz", type=float, default=10)
    sub.choices["watch"].add_argument("--seconds", type=float, default=60)
    args = parser.parse_args(argv)
    try:
        if args.command == "ports":
            from .probe import list_ports
            print(json.dumps(list_ports(), indent=2))
            return 0
        from .probe import open_bus, probe, watch
        bus = open_bus(args.port, args.baud)
        try:
            if args.command == "probe":
                print(json.dumps(probe(bus, args.ids), indent=2))
            else:
                seen = watch(bus, args.ids, hz=args.hz, seconds=args.seconds)
                print(json.dumps({"range_steps": {str(i): list(r) for i, r in seen.items()}}, indent=2))
        finally:
            bus.transport.close()
        return 0
    except (FeetechError, OSError, ValueError) as error:
        print(f"neck {args.command}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
