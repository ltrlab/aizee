"""Time the OpenRB leader boards read-only: where a leader sample's age comes from.

Sends only IDENT and FRESH_POLL (0xA6); never setup, centering or force feedback.
Three phases, each SECONDS long:
  solo      each board alone, polled back to back: PC round trip, on-board read time, rate
  pair      both boards polled together, as OpenRBInputs.poll does
  sampler   Helm's LatestLeaderInputs at SAMPLER_HZ, read by a 100 Hz consumer like the robot loop:
            the sample age that consumer sees, and the rate samples actually arrive

Usage: python scripts/probe_leader_timing.py [--profile config/devices/gello-dual-viewer.json] [--seconds 10]
Writes build/leader-timing/<UTC stamp>/timing.json and prints a summary.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[1]


def percentiles(values, scale=1.):
    """p1/p50/p90/p99/max of `values` times `scale`, rounded; None when empty."""
    if not values:
        return None
    ordered = sorted(v * scale for v in values)
    pick = lambda q: round(ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1)))], 3)
    return {'n': len(ordered), 'p1': pick(.01), 'p50': pick(.5), 'p90': pick(.9), 'p99': pick(.99),
            'max': round(ordered[-1], 3)}


def summarize_polls(polls, skipped, seconds):
    """`polls`: (round_trip_s, board_read_ms, sequence_delta) per complete back-to-back poll; `skipped`: reasons
    for polls the board reported incomplete or too old."""
    return {'rate_hz': round(len(polls) / seconds, 1),
            'round_trip_ms': percentiles([p[0] for p in polls], 1000),
            'board_read_ms': percentiles([p[1] for p in polls]),
            'usb_and_host_ms': percentiles([p[0] * 1000 - p[1] for p in polls]),
            'sequence_steps': sorted({p[2] for p in polls}),
            'skipped': {reason: skipped.count(reason) for reason in sorted(set(skipped))}}


def solo(reader, seconds, clock=time.perf_counter):
    from robot_platform.teleop.openrb import LeaderSampleSkipped
    polls, skipped, previous = [], [], None
    end = clock() + seconds
    while clock() < end:
        started = clock()
        try:
            sample = reader.poll()
        except LeaderSampleSkipped as exc:
            skipped.append(str(exc))
            continue
        received = clock()
        step = sample.sequence - previous.sequence if previous else 1
        polls.append((received - started, (sample.board_reply_ms - sample.board_sample_ms) & 0xffffffff, step))
        previous = sample
    return summarize_polls(polls, skipped, seconds)


def pair(inputs, seconds, clock=time.perf_counter):
    from robot_platform.teleop.openrb import LeaderSampleSkipped
    times, skipped = [], 0
    end = clock() + seconds
    while clock() < end:
        started = clock()
        try:
            inputs.poll()
        except LeaderSampleSkipped:  # A late, skewed or incomplete pair.
            skipped += 1
            continue
        times.append(clock() - started)
    return {'rate_hz': round(len(times) / seconds, 1), 'round_trip_ms': percentiles(times, 1000), 'skipped': skipped}


def sampler(inputs, seconds, rate_hz, clock=time.perf_counter):
    from robot_platform.teleop.leader_bridge import CachedLeaderSampleStale, LatestLeaderInputs
    latest = LatestLeaderInputs(inputs, rate_hz=rate_hz, clock=clock)
    latest.connect()
    ages, stale, first, last = [], 0, None, None
    try:
        due = clock()
        end = due + seconds
        while clock() < end:
            due += .01
            time.sleep(max(0., due - clock()))
            try:
                samples = latest.poll()
            except CachedLeaderSampleStale:
                stale += 1
                continue
            now = clock()
            ages.append(max(now - s.sampled_at for s in samples.values()))
            sequences = {name: s.sequence for name, s in samples.items()}
            first = first or sequences
            last = sequences
    finally:
        skipped = latest.skipped
        latest.close()
    return {'requested_hz': rate_hz, 'consumer_hz': 100, 'age_ms': percentiles(ages, 1000), 'stale_reads': stale,
            'usb_skipped': skipped,
            'sample_rate_hz': {name: round((last[name] - first[name]) / seconds, 1) for name in last} if last else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--profile', default=str(ROOT / 'config/devices/gello-dual-viewer.json'))
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    from robot_platform.control.scheduling import prefer_responsive
    from robot_platform.teleop.leader_bridge import SAMPLER_HZ
    from robot_platform.teleop.openrb import OpenRBInputs
    profile = json.loads(Path(args.profile).read_text(encoding='utf-8'))
    # As the leader-link process does (1 ms timer, no throttling), so the sampler runs as it does in a session.
    result = {'profile': profile.get('id'), 'seconds': args.seconds, 'scheduling': prefer_responsive(),
              'measured_at': datetime.now(timezone.utc).isoformat()}
    inputs = OpenRBInputs(profile)
    inputs.connect()
    try:
        result['solo'] = {name: solo(reader, args.seconds) for name, reader in inputs.readers.items()}
        for reader in inputs.readers.values():
            reader.previous = None  # The pair phase checks its own sequence order from here.
        result['pair'] = pair(inputs, args.seconds)
    finally:
        inputs.close()
    result['sampler'] = sampler(OpenRBInputs(profile), args.seconds, SAMPLER_HZ)  # It connects and closes them.
    directory = ROOT / 'build/leader-timing' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    directory.mkdir(parents=True)
    (directory / 'timing.json').write_text(json.dumps(result, indent=1) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=1))
    print('Saved', directory / 'timing.json')


if __name__ == '__main__':
    main()
