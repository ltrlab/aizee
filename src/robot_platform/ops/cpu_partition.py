"""Split Minerva's Jetson CPUs between the arms' control, the head's receiver and the cameras (see control.scheduling).

On the Jetson, once, from the active release:
    cd ~/minerva/current/src && sudo python3 -B -m robot_platform.ops.cpu_partition install
`remove` undoes it; `status` (no sudo) reports what is in place. -B: a root run must not leave root-owned
bytecode in the release, which the deploy, running as the robot's user, could then not prune.

The camera services shared all six cores with the teleop loop and its receiver, and their load stretched the
loop's cycles until a command expired (2026-10-17). Holding both arms enabled with the camera threads on CPUs 0-3
and the loop and receiver on 4-5, 104 s had no missed cycle and the slowest took 8.05 ms; shared, 77 s missed 60
and the slowest took 14.0 ms.

install gives each camera service a CPUAffinity drop-in, restarts the running ones and checks where they now run,
and only then writes the partition file the loop reads, so the loop never keeps to its CPUs while the cameras can
still use them; remove takes the file away first for the same reason. Both refuse while a Helm session runs on
the robot: restarting a camera interrupts its stream.

The split also keeps CPU 3 for the head's receiver alone (2026-11-03), run real-time there: install gives the
service manager of the user who ran sudo (user@UID.service, which runs Helm and so every session) a LimitRTPRIO
drop-in, and restarts it, with Helm, to take it. Beside the cameras on 0-3 the receiver's 10 ms ticks started up to
15 ms late, and one late tick stopped the head mid-session. Installing again moves an older split to this one.
"""
import argparse
from itertools import combinations
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys

from robot_platform.control.scheduling import PARTITION, REALTIME_PRIORITY, cpu_partition, head_cpus

CONTROL = (4, 5)
HEAD = (3,)
BACKGROUND = (0, 1, 2)
SERVICES = ('aizee-scene-cam.service', 'minerva-left-wrist.service', 'minerva-right-wrist.service')
DROP_IN = '50-minerva-cpu-partition.conf'
REALTIME_DROP_IN = '50-minerva-realtime.conf'
UNITS = PurePosixPath('/etc/systemd/system')
ONLINE = PurePosixPath('/sys/devices/system/cpu/online')
SESSION = 'robot_platform.helm.edge'  # The command line of a Helm session's loop on the robot.


def cpu_list(text):
    """'0-3,5' -> {0, 1, 2, 3, 5}"""
    cpus = set()
    for part in text.strip().split(','):
        first, _, last = part.partition('-')
        cpus.update(range(int(first), int(last or first) + 1))
    return cpus


def drop_in(cpus=BACKGROUND):
    return ('# robot_platform.ops.cpu_partition: this camera keeps off the control CPUs.\n'
            '[Service]\nCPUAffinity=\nCPUAffinity=' + ' '.join(map(str, cpus)) + '\n')


def realtime_drop_in(priority=REALTIME_PRIORITY):
    return ("# robot_platform.ops.cpu_partition: this user's services may run the head's receiver real-time.\n"
            f'[Service]\nLimitRTPRIO={priority}\n')


def manager(user):
    """The service manager of user `user` (a uid): Helm's, so every session's."""
    return f'user@{user}.service'


class Host:
    """The robot's files, processes and services. Tests root it in a directory and record systemctl."""

    def __init__(self, root='/', systemctl=None):
        self.root = Path(root)
        self._systemctl = systemctl

    def path(self, absolute):
        return self.root.joinpath(*PurePosixPath(absolute).parts[1:])

    def systemctl(self, *args):
        if self._systemctl is not None:
            return self._systemctl(*args)
        return subprocess.run(['systemctl', *args], check=True, capture_output=True, text=True).stdout

    def main_pid(self, unit):
        return int(self.systemctl('show', '-p', 'MainPID', '--value', unit).strip() or 0)

    def sessions(self):
        """Processes running a Helm session's loop."""
        found = []
        for entry in self.path('/proc').iterdir():
            if entry.name.isdigit():
                try:
                    argv = (entry / 'cmdline').read_bytes().split(b'\0')
                except OSError:
                    continue
                if SESSION.encode() in argv:
                    found.append(int(entry.name))
        return found

    def cpus(self, pid):
        """{thread: the CPUs it may run on} for every thread of `pid`."""
        result = {}
        try:
            tasks = list((self.path('/proc') / str(pid) / 'task').iterdir())
        except FileNotFoundError:
            return result  # The process has ended (a camera that failed to start is restarted by systemd).
        for task in tasks:
            try:
                status = (task / 'status').read_text()
            except OSError:
                continue  # That thread has ended.
            for line in status.splitlines():
                if line.startswith('Cpus_allowed_list:'):
                    result[int(task.name)] = cpu_list(line.split(':', 1)[1])
        return result

    def realtime_limit(self, pid):
        """Process `pid`'s soft RLIMIT_RTPRIO, None where it cannot be read."""
        try:
            text = (self.path('/proc') / str(pid) / 'limits').read_text()
        except OSError:
            return None
        for line in text.splitlines():
            if line.startswith('Max realtime priority'):
                return int(line.split()[3])  # Max realtime priority  <soft>  <hard>
        return None


def _write(path, text, mode=0o644):
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(path.name + '.partial')
    partial.write_text(text, encoding='utf-8')
    os.chmod(partial, mode)
    os.replace(partial, path)


def _quiet(host):
    sessions = host.sessions()
    if sessions:
        raise RuntimeError(f'A Helm session is running on this robot (process {sessions[0]}); stop it in Helm '
                           'first: restarting a camera interrupts its stream.')


def _restart(host):
    """Reload the units, restart the running cameras; {unit: CPUs of each thread of its new process, or None}."""
    host.systemctl('daemon-reload')
    placed = {}
    for unit in SERVICES:
        host.systemctl('try-restart', unit)  # Only a running service restarts; a stopped one keeps stopped.
        pid = host.main_pid(unit)
        placed[unit] = host.cpus(pid) if pid else None
    return placed


def install(host=None, control=CONTROL, background=BACKGROUND, head=HEAD, *, user):
    """`user`: the robot user's uid, whose services (Helm) may then run the head's receiver real-time."""
    host = host or Host()
    _quiet(host)
    online = cpu_list(host.path(ONLINE).read_text())
    groups = (control, background, head)
    if not set().union(*map(set, groups)) <= online or any(set(a) & set(b) for a, b in combinations(groups, 2)):
        raise RuntimeError(f'CPUs {sorted(online)} are online here: control {list(control)}, background '
                           f'{list(background)} and head {list(head)} must be online and apart. Check the power mode '
                           '(nvpmodel).')
    for unit in SERVICES + (manager(user),):
        if host.systemctl('show', '-p', 'LoadState', '--value', unit).strip() != 'loaded':
            raise RuntimeError(f'{unit} is not installed here.')
    partition = host.path(PARTITION)
    if partition.exists():
        partition.unlink()  # An older split's: the loop keeps to no CPUs while the cameras move.
    for unit in SERVICES:
        _write(host.path(UNITS / (unit + '.d') / DROP_IN), drop_in(background))
    placed = _restart(host)
    stray = {unit: sorted(set().union(*threads.values())) for unit, threads in placed.items()
             if threads and any(cpus != set(background) for cpus in threads.values())}
    if stray:
        raise RuntimeError(f'Cameras still run outside CPUs {list(background)}: {stray}. The loop was left '
                           'unpinned; run remove, then check the units with systemctl cat.')
    _write(partition, json.dumps({'control': list(control), 'background': list(background), 'head': list(head),
                                  'services': list(SERVICES)}, indent=2) + '\n')
    cpu_partition(partition), head_cpus(partition)  # The loop and the head read it back the same way.
    # The manager takes its limit as it starts: restarted, it starts Helm again, which keeps to the new split.
    _write(host.path(UNITS / (manager(user) + '.d') / REALTIME_DROP_IN), realtime_drop_in())
    host.systemctl('daemon-reload')
    host.systemctl('restart', manager(user))
    report = status(host, user=user)
    if report['realtime']['limit'] != REALTIME_PRIORITY:
        raise RuntimeError(f"{manager(user)} restarted without the real-time limit ({report['realtime']['limit']}): "
                           "the head's receiver keeps its CPU but runs at normal priority.")
    return report


def remove(host=None, *, user):
    host = host or Host()
    _quiet(host)
    partition = host.path(PARTITION)
    if partition.exists():
        partition.unlink()  # First: the loop stops keeping to its CPUs before the cameras may use them.
    for path in [host.path(UNITS / (unit + '.d') / DROP_IN) for unit in SERVICES] + [
            host.path(UNITS / (manager(user) + '.d') / REALTIME_DROP_IN)]:
        if path.exists():
            path.unlink()
            if not any(path.parent.iterdir()):
                path.parent.rmdir()
    _restart(host)
    host.systemctl('restart', manager(user))  # Helm and the manager without the limit.
    return status(host, user=user)


def status(host=None, *, user=None):
    host = host or Host()
    partition = cpu_partition(host.path(PARTITION))
    head = head_cpus(host.path(PARTITION))
    report = {'partition': None if partition is None else {'control': sorted(partition[0]),
                                                           'background': sorted(partition[1]),
                                                           'head': None if head is None else sorted(head)},
              'cameras': {}, 'sessions': {}}
    if user is not None:
        pid = host.main_pid(manager(user))
        report['realtime'] = {'drop_in': host.path(UNITS / (manager(user) + '.d') / REALTIME_DROP_IN).exists(),
                              'limit': host.realtime_limit(pid) if pid else None}
    for unit in SERVICES:
        pid = host.main_pid(unit)
        threads = host.cpus(pid) if pid else {}
        report['cameras'][unit] = {'drop_in': host.path(UNITS / (unit + '.d') / DROP_IN).exists(), 'pid': pid or None,
                                   'cpus': sorted(set().union(*threads.values())) if threads else None}
    for pid in host.sessions():
        threads = host.cpus(pid)
        report['sessions'][pid] = sorted(set().union(*threads.values())) if threads else None
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python3 -m robot_platform.ops.cpu_partition', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=('install', 'remove', 'status'))
    args = parser.parse_args(argv)
    if args.action != 'status' and getattr(os, 'geteuid', lambda: -1)() != 0:
        print('install and remove change systemd units: run them with sudo.', file=sys.stderr)
        return 2
    if args.action != 'status' and not sys.flags.dont_write_bytecode:
        print('Run as sudo python3 -B -m robot_platform.ops.cpu_partition, so no root-owned bytecode is left '
              'in the release.', file=sys.stderr)
        return 2
    # The robot user's services get the real-time limit: the user who ran sudo, or for status the one asking.
    user = os.environ.get('SUDO_UID') or (str(os.getuid()) if args.action == 'status' and hasattr(os, 'getuid')
                                          else None)
    if user is None or not user.isdigit() or user == '0':
        print("Run it with sudo from the robot's user, whose services then may run the head real-time.",
              file=sys.stderr)
        return 2
    try:
        report = {'install': install, 'remove': remove, 'status': status}[args.action](user=int(user))
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
