"""Camera noise from frame stacks, as the cameras deliver it (ZMQ JPEG). Runs on the Jetson with its python3 (OpenCV,
numpy, pyzmq, msgpack); reads only, except the controls given with --set or --sweep, which it restores afterwards.

    python3 camera_noise_stack.py --label baseline [--frames 120] [--roles left_wrist right_wrist head]
    python3 camera_noise_stack.py --label gain --roles left_wrist right_wrist --sweep gain=0,30,60
    python3 camera_noise_stack.py --label panel5600 --set white_balance_automatic=0 \\
        --sweep white_balance_temperature=4000,4800,5600,6400

Each stack is N consecutive frames of a still scene. Per pixel: the temporal mean and standard deviation (8-bit DN, per
channel and luma); noise against brightness in 16 bins (pixels that clip in any frame left out); the noise left in
the N-frame average, measured from the even and odd halves (white noise gives sigma / sqrt(N)); frame timing; flicker
(the frames' mean and rolling-shutter row bands); and the centre's colour (R/G and B/G of the middle half of the image,
neutral at 1 under a grey or white target or a light panel's diffuser) with the corners' fall-off (vignetting).

--set and --sweep apply to every role given that has controls (the wrist cameras; the head has none), through the
camera node's control REP as Helm's Settings -> Cameras does. Run only while no session or recording runs: a changed
gain or white balance would be recorded. Writes /tmp/minerva-calib/noise/<stamp>-<label>/[<value>/]<role>/.
"""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import msgpack
import numpy as np
import zmq

STREAMS = {'left_wrist': (5563, 5573), 'right_wrist': (5565, 5574), 'head': (5564, None)}
BINS = np.arange(0, 257, 16)


def control(port, payload):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(f'tcp://127.0.0.1:{port}')
        socket.send(msgpack.packb(payload, use_bin_type=True))
        if not socket.poll(2200):
            raise TimeoutError('camera control did not reply')
        reply = msgpack.unpackb(socket.recv(), raw=False)
        if not isinstance(reply, dict) or reply.get('ok') is not True:
            raise RuntimeError(f'camera control refused {payload}: {reply}')
        return reply
    finally:
        socket.close(0)
        context.term()


def value_of(entry):
    return entry['value'] if isinstance(entry, dict) else entry


def controls(role):
    port = STREAMS[role][1]
    return None if port is None else control(port, {'op': 'get_ctrls'})['ctrls']


def grab(role, frames, skip=5):
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    try:
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVHWM, 4000)
        socket.setsockopt(zmq.SUBSCRIBE, b'')
        socket.connect(f'tcp://127.0.0.1:{STREAMS[role][0]}')
        packets, deadline = [], time.time() + 60
        while len(packets) < frames + skip and time.time() < deadline:
            if not socket.poll(3000):
                raise TimeoutError(f'{role}: no frame within 3 s')
            parts = socket.recv_multipart()
            header = msgpack.unpackb(parts[0], raw=False)
            jpeg = parts[1 + header['parts'].index('color')]
            packets.append(
                (
                    float(header['timestamp']),
                    bytes(jpeg),
                    {k: v for k, v in header.items() if k not in ('color', 'depth')},
                )
            )
        return packets[skip:]
    finally:
        socket.close(0)
        context.term()


def centre_colour(mean):
    """R/G and B/G of the image's middle half, and each corner patch's luma against the centre's."""
    h, w = mean.shape[:2]
    middle = mean[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4, :3].reshape(-1, 3).mean(axis=0)
    luma = 0.114 * mean[..., 0] + 0.587 * mean[..., 1] + 0.299 * mean[..., 2]
    ph, pw = h // 8, w // 8
    centre = luma[h // 2 - ph // 2 : h // 2 + ph // 2, w // 2 - pw // 2 : w // 2 + pw // 2].mean()
    corners = [luma[:ph, :pw].mean(), luma[:ph, -pw:].mean(), luma[-ph:, :pw].mean(), luma[-ph:, -pw:].mean()]
    b, g, r = (float(v) for v in middle)
    return {
        'bgr': [round(b, 2), round(g, 2), round(r, 2)],
        'r_over_g': round(r / g, 4) if g else None,
        'b_over_g': round(b / g, 4) if g else None,
        'corner_over_centre': [round(float(c / centre), 3) if centre else None for c in corners],
    }


def analyse(packets, out):
    out.mkdir(parents=True, exist_ok=True)
    stamps = np.array([p[0] for p in packets])
    first = cv2.imdecode(np.frombuffer(packets[0][1], np.uint8), cv2.IMREAD_COLOR)
    h, w = first.shape[:2]
    n = len(packets)
    total, square, even = np.zeros((h, w, 4)), np.zeros((h, w, 4)), np.zeros((h, w, 4))
    lo, hi = np.full((h, w, 3), 255, np.uint8), np.zeros((h, w, 3), np.uint8)
    frame_mean, rows = [], []
    for i, (_, jpeg, _) in enumerate(packets):
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        np.minimum(lo, bgr, out=lo)
        np.maximum(hi, bgr, out=hi)
        x = bgr.astype(np.float64)
        y = 0.114 * x[..., 0] + 0.587 * x[..., 1] + 0.299 * x[..., 2]
        v = np.concatenate([x, y[..., None]], axis=2)
        total += v
        square += v * v
        if i % 2 == 0:
            even += v
        frame_mean.append(float(y.mean()))
        rows.append(y.mean(axis=1))
    mean = total / n
    std = np.sqrt(np.maximum(square / n - mean * mean, 0) * n / (n - 1))
    # Even/odd halves: var(d) = sigma^2 (1/n_even + 1/n_odd), so the N-frame average's sigma is
    # |d| sqrt(n_even n_odd) / n. White noise gives sigma / sqrt(N); flicker or drift gives more.
    n_even, n_odd = (n + 1) // 2, n // 2
    residual = np.abs(even / n_even - (total - even) / n_odd) * math.sqrt(n_even * n_odd) / n
    clipped = (hi == 255).any(axis=2) | (lo == 0).any(axis=2)
    names = ('B', 'G', 'R', 'Y')
    bins = []
    for k in range(len(BINS) - 1):
        a, b = BINS[k], BINS[k + 1]
        row = {'from': int(a), 'to': int(b)}
        for c, name in enumerate(names):
            sel = (mean[..., c] >= a) & (mean[..., c] < b) & ~clipped
            if sel.sum() < 200:
                continue
            s = std[..., c][sel]
            row[name] = {
                'pixels': int(sel.sum()),
                'sigma_median': round(float(np.median(s)), 3),
                'sigma_p90': round(float(np.percentile(s, 90)), 3),
            }
        bins.append(row)
    ok = ~clipped
    overall = {}
    for c, name in enumerate(names):
        s = std[..., c][ok]
        if not s.size:
            continue
        overall[name] = {
            'sigma_median': round(float(np.median(s)), 3),
            'sigma_mean': round(float(s.mean()), 3),
            'mean_level': round(float(mean[..., c][ok].mean()), 2),
            # The median |d| of a normal variable is 0.6745 sigma.
            'stack_sigma_measured': round(float(np.median(residual[..., c][ok])) / 0.6745, 4),
            'stack_sigma_white': round(float(np.median(s)) / math.sqrt(n), 4),
        }
    rows = np.array(rows)
    band = rows - rows.mean(axis=0)
    floor = float(np.median(std[..., 3][ok])) / math.sqrt(w) if ok.any() else 0.0
    intervals = np.diff(stamps)
    fm = np.array(frame_mean)
    summary = {
        'frames': n,
        'size': [w, h],
        'jpeg_kb_mean': round(float(np.mean([len(p[1]) for p in packets])) / 1024, 1),
        'fps': round(1 / float(np.median(intervals)), 2) if len(intervals) else None,
        'interval_ms': {
            'median': round(float(np.median(intervals)) * 1e3, 2),
            'p99': round(float(np.percentile(intervals, 99)) * 1e3, 2),
            'max': round(float(intervals.max()) * 1e3, 2),
            'gaps_over_1p5x': int((intervals > 1.5 * np.median(intervals)).sum()),
        },
        'clipped_fraction': round(float(clipped.mean()), 4),
        'overall': overall,
        'bins': bins,
        'flicker': {
            'frame_mean_luma': round(float(fm.mean()), 2),
            'frame_mean_std': round(float(fm.std()), 4),
            'frame_mean_ptp': round(float(np.ptp(fm)), 3),
            'row_band_std': round(float(band.std()), 4),
            'row_band_noise_floor': round(floor, 4),
        },
        'centre': centre_colour(mean),
        'header': packets[0][2],
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=1))
    cv2.imwrite(str(out / 'mean.png'), np.clip(mean[..., :3] + 0.5, 0, 255).astype(np.uint8))
    cv2.imwrite(str(out / 'std_x20.png'), np.clip(std[..., 3] * 20, 0, 255).astype(np.uint8))
    return summary


def line(role, summary):
    o, f, c = summary['overall'].get('Y', {}), summary['flicker'], summary['centre']
    return (
        f'{role:12s} {summary["fps"]} fps  luma {o.get("mean_level", float("nan")):.1f}  sigma Y '
        f'{o.get("sigma_median", float("nan")):.2f}  stack {o.get("stack_sigma_measured", float("nan")):.3f} '
        f'(white {o.get("stack_sigma_white", float("nan")):.3f})  rows {f["row_band_std"]:.3f}/'
        f'{f["row_band_noise_floor"]:.3f}  R/G {c["r_over_g"]}  B/G {c["b_over_g"]}  '
        f'clipped {summary["clipped_fraction"]:.3f}'
    )


def assignments(items):
    return [(name, value) for name, value in (item.split('=', 1) for item in items)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--label', required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--roles', nargs='+', default=list(STREAMS), choices=list(STREAMS))
    parser.add_argument('--set', nargs='*', default=[], help='name=value for the whole run (restored afterwards)')
    parser.add_argument('--sweep', help='name=v1,v2,...: one stack per value (restored afterwards)')
    parser.add_argument('--settle', type=float, default=1.5, help='seconds after a change before the stack')
    args = parser.parse_args()
    root = Path('/tmp/minerva-calib/noise') / (time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()) + '-' + args.label)
    root.mkdir(parents=True, exist_ok=True)
    adjustable = [role for role in args.roles if STREAMS[role][1] is not None]
    before = {role: controls(role) for role in args.roles}
    (root / 'controls.json').write_text(json.dumps(before, indent=1))
    fixed = assignments(args.set)
    sweep_name, sweep_values = (
        (args.sweep.split('=', 1)[0], args.sweep.split('=', 1)[1].split(',')) if args.sweep else (None, [None])
    )
    # In the order applied; restored in reverse (a white balance temperature before automatic white balance).
    touched = [name for name, _ in fixed] + ([sweep_name] if sweep_name else [])
    for role in adjustable:
        missing = [name for name in touched if name not in before[role]]
        if missing:
            raise SystemExit(f'{role} has no controls {missing}')
    try:
        for name, value in fixed:
            for role in adjustable:
                control(STREAMS[role][1], {'op': 'set_ctrl', 'name': name, 'value': int(value)})
        for value in sweep_values:
            out = root / (f'{sweep_name}-{value}' if sweep_name else '')
            if sweep_name:
                for role in adjustable:
                    control(STREAMS[role][1], {'op': 'set_ctrl', 'name': sweep_name, 'value': int(value)})
            if touched:
                time.sleep(args.settle)
                out.mkdir(parents=True, exist_ok=True)
                (out / 'controls_set.json').write_text(json.dumps({r: controls(r) for r in adjustable}, indent=1))
            if sweep_name:
                print(f'## {sweep_name}={value}', flush=True)
            for role in args.roles:
                print(line(role, analyse(grab(role, args.frames), out / role)), flush=True)
    finally:
        for role in adjustable:
            for name in reversed(touched):
                control(STREAMS[role][1], {'op': 'set_ctrl', 'name': name, 'value': int(value_of(before[role][name]))})
        if touched:
            after = {role: controls(role) for role in adjustable}
            wrong = [(r, n) for r in adjustable for n in touched if value_of(after[r][n]) != value_of(before[r][n])]
            print('restored' if not wrong else f'RESTORE MISMATCH {wrong}', flush=True)
    print(root)


if __name__ == '__main__':
    main()
