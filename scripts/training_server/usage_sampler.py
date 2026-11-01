"""Sample the CPU use of our processes grouped by session (one session per generation worker or training run), the
load average and the GPUs, every INTERVAL seconds, while the named tmux session lives:

    usage_sampler.py NAME [INTERVAL]      (writes $AIZEE_SIM/logs/NAME-usage.log)
"""
import os
from pathlib import Path
import subprocess
import sys
import time

name = sys.argv[1]
session = name.replace('.', '_')  # tmux turns a dot in a session name into an underscore.
interval = float(sys.argv[2]) if len(sys.argv) > 2 else 15.
uid, hz = os.getuid(), os.sysconf('SC_CLK_TCK')


def snapshot():
    out = {}
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            if d.stat().st_uid != uid:
                continue
            raw = (d / 'stat').read_text()
            comm = raw[raw.index('(') + 1:raw.rindex(')')]
            fields = raw[raw.rindex(')') + 2:].split()
            out[int(d.name)] = (int(fields[3]), comm, int(fields[11]) + int(fields[12]))  # session, comm, CPU ticks
        except (OSError, ValueError, IndexError):
            pass
    return out


def alive():
    return subprocess.run(['tmux', 'has-session', '-t', '=' + session], capture_output=True).returncode == 0


log = open(Path(os.environ.get('AIZEE_SIM', Path.home() / 'aizee-sim')) / f'logs/{name}-usage.log', 'a', encoding='utf-8')
before, t0 = snapshot(), time.time()
while alive():
    time.sleep(interval)
    now, t1 = snapshot(), time.time()
    sessions = {}
    for pid, (sid, comm, ticks) in now.items():
        prev = before.get(pid)
        # A process born during the interval counts all its ticks.
        delta = ticks - prev[2] if prev else ticks
        entry = sessions.setdefault(sid, [0, set()])
        entry[0] += delta
        entry[1].add(comm)
    busy = {sid: (ticks / hz / (t1 - t0), sorted(names)) for sid, (ticks, names) in sessions.items() if ticks > 0}
    gpu = subprocess.run(['nvidia-smi', '--query-gpu=index,memory.used,utilization.gpu',
                          '--format=csv,noheader,nounits'], capture_output=True, text=True).stdout.strip()
    load = ' '.join(open('/proc/loadavg').read().split()[:3])
    avail = next((int(l.split()[1]) / 2**20 for l in open('/proc/meminfo') if l.startswith('MemAvailable:')), 0.)
    parts = ' '.join(f"{sid}:{cores:.2f}[{','.join(names)[:70]}]"
                     for sid, (cores, names) in sorted(busy.items(), key=lambda item: -item[1][0]))
    total = sum(cores for cores, _ in busy.values())
    log.write(f"{time.strftime('%H:%M:%S')} load {load} avail {avail:.1f}G ours {total:.2f} cores | gpu {gpu.replace(chr(10), ' / ')} | {parts}\n")
    log.flush()
    before, t0 = now, t1
