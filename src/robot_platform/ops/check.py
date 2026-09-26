"""Minerva Check: the comprehensive go/no-go for live operation, run from the operator PC.

Stages run in order. A check whose prerequisite failed or was skipped is
skipped too, never guessed:
  1. Operator PC: calibration profile, motor-to-model directions, joint limits,
     leader ranges and zero, routes, the teleop receiver configuration, the
     leader arms, the ARM64 builds and the route to the tether.
  2. Link and Jetson: pinned SSH over the tether, the deployed release (intact,
     matching this PC, smoke-tested), the Jetson's health and leader-link round
     trips at the control rate.
  3. Motors, stopped: CAN system setup, adapters and services, then the stopped
     reader's scan of every motor (identity, firmware, fault-free Reset and one
     position each; CAN ends down) and every position inside its swept range.
  4. Live joint sweep: appended once the live pipeline is proven (pending).

Nothing here enables a motor. The verdict is GO when nothing failed or was
skipped; warnings are listed. Every run is saved as evidence.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import secrets
import socket
import time

from .deploy import BASE, smoke
from .release import ROOT, check_binaries, collect, release_drift
from .ssh import DEFAULT_PROFILE, TETHER, connect, run

CURRENT = f'{BASE}/current'
PROBE = f'cd {CURRENT} && PYTHONPATH=src python3 -u -m robot_platform.ops.probe'
DEFAULT_TELEOP = ROOT / 'config/devices/minerva-teleop.json'
DEFAULT_RUNS = ROOT / 'calibration_runs'
TETHER_NETWORK = '10.42.0.'
POSE_TOLERANCE_RAD = .05
OPENRB = (0x2F5D, 0x2202)
PASS, WARN, FAIL, SKIP, PENDING, RUNNING = 'pass', 'warn', 'fail', 'skip', 'pending', 'running'
STAGES = {'pc': 'Operator PC', 'jetson': 'Link and Jetson', 'motors': 'Motors, stopped',
          'sweep': 'Live joint sweep'}
CHECKS = []

RELEASE_STATE = '''
import json, os, sys
root = os.path.realpath(sys.argv[1])
sys.path.insert(0, os.path.join(root, "src"))
try:
    from robot_platform.ops.release import MANIFEST, verify_tree
    manifest = json.load(open(os.path.join(root, MANIFEST)))
except Exception as exc:
    print(json.dumps({"error": f"{type(exc).__name__}: {exc}"}))
else:
    print(json.dumps({"root": root, "problems": verify_tree(root), "manifest": manifest}))
'''


def check(stage, key, title, requires=()):
    """Register a check. It returns (status, detail) or (status, detail, data); raising fails it."""
    def register(function):
        CHECKS.append({'stage': stage, 'id': key, 'title': title, 'requires': tuple(requires), 'run': function})
        return function
    return register


class Context:
    """Options plus what earlier checks found (plan, limits, SSH client, probe results)."""

    def __init__(self, *, root, profile, teleop, runs, limits, address, motors, link_seconds, leaders_busy,
                 open_ssh, open_leaders, serve_link):
        self.root, self.profile, self.teleop = Path(root), Path(profile).resolve(), Path(teleop).resolve()
        self.runs, self.limits_path = Path(runs).resolve(), limits
        self.address, self.motors, self.link_seconds = address, motors, link_seconds
        self.leaders_busy = leaders_busy or (lambda: None)
        self.open_ssh, self.open_leaders, self.serve_link = open_ssh, open_leaders, serve_link
        self.client = None
        self.facts = {}

    def label(self, motor):
        return self.facts.get('labels', {}).get(motor, motor)

    def probe(self, name, *, timeout=60.):
        status, out, err = run(self.client, f'{PROBE} {name}', timeout=timeout)
        if 'No module named robot_platform.ops.probe' in err:
            raise RuntimeError('The deployed release predates these checks. Deploy: scripts/deploy_minerva_arms.sh')
        try:
            report = json.loads(out.strip().splitlines()[-1])
        except (ValueError, IndexError):
            raise RuntimeError(f'The {name} probe did not answer: {err.strip()[-300:] or f"exit {status}"}')
        if not report.get('ok'):
            raise RuntimeError(f"The {name} probe failed on the robot: {report.get('error')}")
        return report


def _ago(iso):
    try:
        seconds = time.time() - datetime.fromisoformat(iso).timestamp()
    except (TypeError, ValueError):
        return 'at an unknown time'
    for unit, size in (('day', 86400), ('h', 3600), ('min', 60)):
        if seconds >= size:
            count = int(seconds // size)
            return f"{count} {unit}{'s' if unit == 'day' and count > 1 else ''} ago"
    return 'just now'


def _names(ctx, motors):
    return ', '.join(ctx.label(motor) for motor in motors)


# -- Stage 1: operator PC -------------------------------------------------------

@check('pc', 'calibration', 'Calibration profile')
def _calibration(ctx):
    from robot_platform.control.joint_calibration import load_plan, validate_preview_transmissions
    plan, _paths, device, reference, model, _pose, bindings = load_plan(ctx.profile)
    transmissions = validate_preview_transmissions(plan.get('preview_transmissions', []), plan['steps'],
                                                   reference, model)
    labels = {step['motor']: step['label'] for step in plan['steps']}
    labels.update({t['motor']: t['motor'].replace('_', ' ').capitalize() for t in transmissions})
    ctx.facts.update(plan=plan, device=device, reference=reference, model=model, bindings=bindings,
                     transmissions=transmissions, labels=labels, asset_id=device['asset_id'])
    return PASS, (f"{device['asset_id']}: {len(reference['motors'])} motors on the power-cycle-verified "
                  f"mechanical reference {reference['id']}")


def direction_session(runs, asset_id, bindings):
    """Helm's direction session for this calibration revision (read-only), or None."""
    for directory in (runs / f'{asset_id}-directions', runs / f"{asset_id}-directions-{bindings['plan'][:8]}"):
        path = directory / 'session.json'
        if path.is_file():
            state = json.loads(path.read_text(encoding='utf-8'))
            if state.get('bindings') == bindings:
                return state
    return None


@check('pc', 'directions', 'Motor-to-model directions', requires=('calibration',))
def _directions(ctx):
    state = direction_session(ctx.runs, ctx.facts['asset_id'], ctx.facts['bindings'])
    if state is None or 'preview_mapping' not in state:
        return FAIL, 'No direction session for this calibration: open Helm and confirm the directions in Limits'
    mapping = state['preview_mapping']
    steps = ctx.facts['plan']['steps']
    open_ = [step['motor'] for step in steps if mapping.get(step['joint'], {}).get('direction_verified') is not True]
    if open_:
        return FAIL, f'Not confirmed yet ({len(steps) - len(open_)}/{len(steps)}): {_names(ctx, open_)}. Confirm them in Helm Limits.'
    ctx.facts['directions'] = mapping
    return PASS, f'All {len(steps)} arm joints confirmed against the model'


@check('pc', 'limits', 'Joint limits', requires=('calibration',))
def _limits(ctx):
    from robot_platform.helm.limits import load_limits
    asset_id, reference = ctx.facts['asset_id'], ctx.facts['reference']
    path = Path(ctx.limits_path) if ctx.limits_path else ctx.profile.parent / f'{asset_id}-joint-limits.json'
    document, error = load_limits(path, asset_id=asset_id, reference=reference,
                                  reference_sha256=ctx.facts['bindings']['reference'])
    if error:
        return FAIL, f'{path.name} was not accepted: {error}'
    if document is None:
        return FAIL, 'No saved joint limits: sweep every joint in Helm Limits and save'
    missing = [m['name'] for m in reference['motors'] if m['name'] not in document['joints']]
    if missing:
        return FAIL, f'No limits saved for {_names(ctx, missing)}: sweep them in Helm Limits'
    ctx.facts['limits'] = document
    return PASS, f"All {len(document['joints'])} motors, saved {_ago(document.get('updated_at'))}"


@check('pc', 'leader_ranges', 'Leader ranges and zero')
def _leader_ranges(ctx):
    from robot_platform.helm.leaders import LeaderWorkspace
    from robot_platform.helm.teleop import load_profile
    teleop = load_profile(ctx.teleop)
    workspace = LeaderWorkspace(ctx.teleop)  # Reads the saved files only; opens no USB port.
    ctx.facts.update(teleop=teleop, leader_profile=workspace.profile, leader_ranges=workspace.saved)
    if workspace.saved_error:
        return FAIL, f'{workspace.path.name} was not accepted: {workspace.saved_error}'
    missing = workspace.missing(teleop['routes'])
    if missing:
        return FAIL, f"Missing in {workspace.path.name}: {', '.join(missing)}. Sweep and save them in Helm Leaders."
    zero = workspace.saved['reference']
    return PASS, f"All {len(teleop['routes'])} routed channels swept; leader zero set {_ago(zero.get('captured_at'))}"


@check('pc', 'routes', 'Leader-to-robot routes', requires=('calibration', 'leader_ranges'))
def _routes(ctx):
    from robot_platform.helm.teleop import load_route_edits, merge_routes, routes_path
    teleop, asset_id = ctx.facts['teleop'], ctx.facts['asset_id']
    routes = merge_routes(teleop['routes'], load_route_edits(routes_path(ctx.runs, asset_id), teleop, asset_id))
    motors = [m['name'] for m in ctx.facts['reference']['motors']]
    unrouted = [motor for motor in motors if motor not in routes]
    unverified = [motor for motor in motors if motor in routes and routes[motor].get('verified') is not True]
    if unrouted:
        return FAIL, f'No leader route for {_names(ctx, unrouted)} in {ctx.teleop.name}'
    if unverified:
        return FAIL, f'Direction not confirmed for {_names(ctx, unverified)}: confirm it in Helm Teleop'
    ctx.facts['routes'] = routes
    return PASS, f'All {len(motors)} motors routed with confirmed directions'


@check('pc', 'receiver', 'Teleop receiver configuration', requires=('directions', 'limits', 'routes'))
def _receiver(ctx):
    from types import SimpleNamespace
    from robot_platform.control.guarded_config import derive_receiver
    # The derivation reads only identity revisions from the collision backend, so
    # skip the ~8 s CAD collision load here; the release smoke test loads the model.
    revisions = SimpleNamespace(policy_revision='check', runtime_model=SimpleNamespace(revision='check',
                                                                                        world_revision='check'))
    derived = derive_receiver(reference=ctx.facts['reference'], steps=ctx.facts['plan']['steps'],
                              directions=ctx.facts['directions'], transmissions=ctx.facts['transmissions'],
                              model=ctx.facts['model'], limits=ctx.facts['limits'], profile=ctx.facts['teleop'],
                              backend=revisions)
    rows = derived['report']
    share = {row['motor']: (row['usable'][1] - row['usable'][0]) / (row['swept'][1] - row['swept'][0])
             for row in rows}
    narrowest = min(share, key=share.get)
    mode = ctx.facts['teleop']['receiver'].get('collision_mode', 'enforced')
    detail = (f'{len(rows)} joints derive limits, margins and stopping reserves; collision {mode}; '
              f'narrowest usable range {ctx.label(narrowest)} at {share[narrowest]:.0%} of its swept travel')
    return PASS, detail, {'report': rows}


def _leader_outliers(profile, document, samples, routes):
    """Routed leader channels whose reading lies outside its saved sweep: [(leader, label, beyond_rad)]."""
    from robot_platform.teleop.leader_ranges import range_table, wrap
    table = range_table(document, profile)
    outliers = []
    for route in routes.values():
        leader, index = route['leader'], route['channel'] - 1
        saved = table.get(leader, [None] * 8)[index]
        if saved is None or leader not in samples:
            continue
        center, span = saved
        beyond = abs(wrap(samples[leader].positions_rad[index] - center)) - span / 2
        if beyond > POSE_TOLERANCE_RAD:
            label = profile['leaders'][leader]['joints'][index].get('label', f'channel {index + 1}')
            outliers.append((leader, label, beyond))
    return outliers


@check('pc', 'leaders', 'Leader arms', requires=('leader_ranges',))
def _leaders(ctx):
    from serial.tools import list_ports
    from robot_platform.teleop.openrb import LeaderSampleSkipped
    profile = ctx.facts['leader_profile']
    ports = [port for port in list_ports.comports() if (port.vid, port.pid) == OPENRB]
    absent = [item['title'] for item in profile['leaders'].values()
              if sum(port.serial_number == item['usb_serial'] for port in ports) != 1]
    if absent:
        return FAIL, f"Not connected by USB: {', '.join(absent)}"
    busy = ctx.leaders_busy()
    if busy:
        return PASS, f'Both present by USB serial; in use by {busy}, so not read now'
    try:
        inputs = ctx.open_leaders(profile)
    except Exception as exc:
        return FAIL, f'Present but could not be opened ({exc}). Close any other program using them.'
    fresh, last, skipped = 0, None, None
    try:
        for _ in range(12):
            try:
                last = inputs.poll()
                fresh += 1
            except LeaderSampleSkipped as exc:
                skipped = str(exc)
            time.sleep(.02)
    finally:
        inputs.close()
    if fresh < 10:
        return FAIL, f'Only {fresh}/12 fresh leader pairs' + (f' (last: {skipped})' if skipped else '')
    ctx.facts['leaders_readable'] = True
    outliers = _leader_outliers(profile, ctx.facts['leader_ranges'], last, ctx.facts['teleop']['routes'])
    if outliers:
        return WARN, ('Reading, but outside the saved sweep: '
                      + ', '.join(f'{leader} {label.lower()} by {beyond:.2f} rad' for leader, label, beyond in outliers)
                      + '. Re-sweep it in Helm Leaders if it really moves that far.')
    return PASS, f"Both present and reading: {fresh}/12 fresh pairs, every routed channel inside its sweep"


@check('pc', 'builds', 'ARM64 builds')
def _builds(ctx):
    files = collect(ctx.root)
    check_binaries(files, ctx.root)
    built = max(path.stat().st_mtime for key, path in files.items() if key.startswith(('bin/', 'lib/')))
    return PASS, f"Receiver, harness and collision library built {_ago(datetime.fromtimestamp(built, timezone.utc).isoformat())}, newer than their Rust sources"


def route_source(address):
    """The local address this PC would use to reach `address`, or None (no packet is sent)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((address, 9))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


@check('pc', 'tether', 'Tether route')
def _tether(ctx):
    if not ctx.address.startswith(TETHER_NETWORK):
        return FAIL, f'Control must use the USB Ethernet tether ({TETHER}), not {ctx.address}'
    local = route_source(ctx.address)
    if local is None or not local.startswith(TETHER_NETWORK):
        return FAIL, (f'This PC has no tether address (would send from {local or "nowhere"}). '
                      'Plug in the USB Ethernet adapter and check it has 10.42.0.50.')
    ctx.facts['local_address'] = local
    return PASS, f'{local} → {ctx.address} over the USB Ethernet tether'


# -- Stage 2: link and Jetson ---------------------------------------------------

@check('jetson', 'ssh', 'Pinned SSH', requires=('tether',))
def _ssh(ctx):
    started = time.perf_counter()
    ctx.client = ctx.open_ssh(ctx.profile, address=ctx.address)
    return PASS, f'{ctx.address}: host key and machine id verified in {(time.perf_counter() - started) * 1000:.0f} ms'


@check('jetson', 'release', 'Deployed release', requires=('ssh',))
def _release(ctx):
    status, out, err = run(ctx.client, f'python3 - {CURRENT}', stdin=RELEASE_STATE)
    try:
        state = json.loads(out.strip().splitlines()[-1])
    except (ValueError, IndexError):
        state = {'error': err.strip()[-300:] or f'exit {status}'}
    if 'error' in state:
        return FAIL, f"No usable release at {CURRENT} ({state['error']}). Deploy: scripts/deploy_minerva_arms.sh"
    manifest = state['manifest']
    ctx.facts['release'] = manifest
    if state['problems']:
        return FAIL, (f"{manifest['id']} was modified on the robot ({'; '.join(state['problems'][:3])}). "
                      'Deploy again: scripts/deploy_minerva_arms.sh')
    return PASS, f"{manifest['id']}: all {len(manifest['files'])} files match its manifest"


@check('jetson', 'current', 'Release matches this PC', requires=('release',))
def _current(ctx):
    manifest = ctx.facts['release']
    drift = release_drift(manifest, ctx.root)
    if drift:
        shown = ', '.join(item.split(' ', 1)[1].rsplit('/', 1)[-1] for item in drift[:4])
        more = f' and {len(drift) - 4} more' if len(drift) > 4 else ''
        return FAIL, (f"The robot runs {manifest['id']}; {len(drift)} file(s) differ from this PC ({shown}{more}). "
                      'Deploy: scripts/deploy_minerva_arms.sh'), {'drift': drift}
    source = manifest['source']
    return PASS, (f"Same files as this PC · built from {source['commit'][:7]}"
                  + (' with uncommitted changes' if source['dirty'] else ''))


@check('jetson', 'smoke', 'Release smoke test', requires=('release',))
def _smoke(ctx):
    report = smoke(ctx.client, CURRENT)
    if not report.get('ok'):
        return FAIL, f"The smoke test failed on the robot: {report.get('error')}"
    return PASS, (f"Collision model loads in {report['load_ms']} ms, keep-out query p50 "
                  f"{report['keepout_query_ms']['p50']} ms, receiver harness {report['receiver_harness']}"), report


def health_findings(report, offset_s):
    """[(status, text)] for the Jetson's health report."""
    findings = []
    disk, memory = report['disk_free_gb'], report['memory_available_gb']
    findings.append((FAIL if disk < 1 else WARN if disk < 5 else PASS, f'{disk:g} GB disk free'))
    findings.append((FAIL if memory < .5 else WARN if memory < 1.5 else PASS, f'{memory:g} GB memory available'))
    temperatures = report['temperatures_c']
    if temperatures:
        zone = max(temperatures, key=temperatures.get)
        hottest = temperatures[zone]
        findings.append((FAIL if hottest >= 90 else WARN if hottest >= 75 else PASS, f'hottest {hottest:g} °C ({zone})'))
    if offset_s is not None:
        findings.append((WARN if abs(offset_s) > 2 else PASS, f'clock {abs(offset_s):.1f} s from this PC'
                         + ('' if abs(offset_s) <= 2 else ': evidence timestamps will not line up')))
    mode = ' · '.join(filter(None, (report.get('power_mode'), report.get('governor'))))
    if mode:
        findings.append((PASS, mode))
    findings.append((PASS, f"load {report['load'][0]:g} on {report['cpus']} CPUs"))
    return findings


@check('jetson', 'health', 'Jetson health', requires=('release',))
def _health(ctx):
    before = time.time()
    report = ctx.probe('health', timeout=30.)
    after = time.time()
    ctx.facts['health'] = report
    # The probe reads its clock between our send and receive, so this bounds the offset.
    offset = report['time'] - (before + after) / 2 if after - before < 5 else None
    findings = health_findings(report, offset)
    order = (FAIL, WARN, PASS)
    status = min((status for status, _ in findings), key=order.index)
    return status, ' · '.join(text for _, text in findings), report


def serve_link(profile, key, robot, *, local, real):
    """The PC end of the leader link, as a session runs it: its own responsive process."""
    from robot_platform.teleop.leader_link import LeaderLinkProcess
    return LeaderLinkProcess(profile, key, robot, local=local, real=real).start()


@check('jetson', 'link', 'Leader link at 25 Hz', requires=('release', 'leader_ranges'))
def _link(ctx):
    profile, key = ctx.facts['leader_profile'], secrets.token_bytes(32)
    stdin, stdout, stderr = ctx.client.exec_command(f'{PROBE} link', timeout=ctx.link_seconds + 20)
    stdin.write(f'{key.hex()}\n{json.dumps(profile)}\n{ctx.link_seconds}\n{ctx.address}\n')
    stdin.flush()
    ready = stdout.readline().split()
    if len(ready) != 2 or ready[0] != 'ready':
        raise RuntimeError(f"The link probe did not start: {' '.join(ready) or stderr.read().decode()[-300:]}")
    real = ctx.facts.get('leaders_readable', False)  # Free and healthy: test the whole path with them.
    try:
        server = ctx.serve_link(profile, key, (ctx.address, int(ready[1])),
                                local=(ctx.facts.get('local_address', '0.0.0.0'), 0), real=real)
    except Exception:
        stdin.channel.close()  # The robot side gives up after 10 s without a hello.
        raise
    try:
        line = stdout.readline()
    finally:
        pc = server.close()
    try:
        report = json.loads(line)
    except ValueError:
        raise RuntimeError(f'The link probe ended without a result: {stderr.read().decode()[-300:]}')
    if not report.get('ok'):
        return FAIL, f"The robot's leader input failed: {report.get('error')}"
    report['pc'] = pc
    cycles, fresh, age = report['cycles'], report['fresh'], report['age_ms']
    share = fresh / cycles if cycles else 0
    status = PASS if share >= .98 and (age['p99'] or math.inf) < report['max_age_ms'] else WARN if share >= .9 else FAIL
    detail = (f"{fresh}/{cycles} fresh pairs from {'the real leaders' if real else 'synthetic leaders (network only)'}"
              f" · sample age p50 {age['p50']} ms, p99 {age['p99']} ms (limit {report['max_age_ms']} ms)"
              f" · longest gap {report['longest_skip_run']} cycle(s)")
    if report.get('last_skip') and status != PASS:
        detail += f" · last skip: {report['last_skip']}"
    if pc.get('failure') or pc.get('error'):
        status, detail = FAIL, detail + f" · PC side: {pc.get('failure') or pc.get('error')}"
    return status, detail, report


# -- Stage 3: motors, stopped ---------------------------------------------------

@check('motors', 'setup', 'CAN system setup', requires=('health',))
def _setup(ctx):
    setup = ctx.facts['health']['setup']
    missing = [text for key, text in (('helper', 'the CAN helpers'), ('sudo', 'the sudo rule for them'),
                                      ('udev_rule', 'the adapter serial pinning rule')) if not setup[key]]
    if missing:
        return FAIL, f"Missing {', '.join(missing)}. Run scripts/setup_minerva_system.sh once (it needs sudo)."
    return PASS, 'CAN helpers, their sudo rule and the adapter serial pinning are installed'


@check('motors', 'adapters', 'CAN adapters', requires=('health',))
def _adapters(ctx):
    problems = []
    for name, row in ctx.facts['health']['adapters'].items():
        if not row['present']:
            problems.append(f'{name} is missing: check its USB-CAN adapter cable')
        elif row.get('serial') != row['expected_serial']:
            problems.append(f"{name} is adapter {row.get('serial')}, expected {row['expected_serial']}")
        elif row.get('up'):
            problems.append(f'{name} is already up: another program owns it (Helm Limits connected?)')
    if problems:
        return FAIL, '; '.join(problems)
    return PASS, f"{' and '.join(ctx.facts['health']['adapters'])} present on their pinned adapters and free"


@check('motors', 'services', 'No other motor owner', requires=('health',))
def _services(ctx):
    health = ctx.facts['health']
    running = [name for name, pid in health['services'].items() if pid != 0]
    if running:
        return FAIL, f"Running (or unreadable): {', '.join(running)}. Stop them before operating."
    if health['receiver_listening'] is not False:
        return FAIL, 'A guarded receiver is already listening on the robot (port 5590)'
    return PASS, 'No motor service or receiver is running'


def scan_findings(ctx, report):
    """(status, detail) for the stopped reader's report."""
    reference = ctx.facts['reference']
    rows = report.get('motors') or []
    after = report.get('after') or {}
    cleaned = (not report.get('cleanup_errors') and set(after) == {m['bus'] for m in reference['motors']}
               and all('UP' not in row.get('flags', ['UP']) for row in after.values()))
    if report.get('enable_requested') is not False or report.get('parameter_writes_requested') is not False:
        return FAIL, 'The reader did not confirm a stopped-only scan'
    if not report.get('success'):
        silent = report.get('unresponsive') or []
        if silent:
            buses = {}
            for motor in reference['motors']:
                buses.setdefault(motor['bus'], []).append(motor['name'])
            states = {bus: (row.get('linkinfo') or {}).get('info_data', {}).get('state')
                      for bus, row in (report.get('before_cleanup') or {}).items()}
            unacknowledged = states and all(state in ('ERROR-PASSIVE', 'BUS-OFF') for state in states.values())
            if len(silent) == len(reference['motors']):
                detail = ('No motor answered' + (f", and nothing on {' or '.join(sorted(states))} acknowledged a "
                                                 'frame' if unacknowledged else '')
                          + '. Motor power is off or the foot switch (e-stop) is pressed.')
            elif any(set(names) <= set(silent) for names in buses.values()):
                bus = next(bus for bus, names in buses.items() if set(names) <= set(silent))
                detail = (f"Every motor on {bus} is silent ({_names(ctx, silent)}): check that arm's power "
                          'and CAN cable.')
            else:
                detail = f'Not answering: {_names(ctx, silent)}. Check their plugs and power; power-cycling is fine.'
        else:
            error = report.get('error', 'the stopped reader failed')
            detail = error + (' A latched fault clears only with a power cycle.' if 'latched faults' in error else '')
        if not cleaned and report.get('after') is not None:
            detail += ' CAN links were not confirmed down afterwards: check them before anything else.'
        return FAIL, detail
    if not cleaned:
        return FAIL, 'The scan finished but CAN links were not confirmed down afterwards'
    if report.get('identity_verified') is not True or len(rows) != len(reference['motors']):
        return FAIL, 'The scan did not cover every motor'
    return PASS, (f'All {len(rows)} motors answered: identity, firmware and fault-free Reset confirmed, '
                  'one position each; CAN links down again')


@check('motors', 'scan', 'Stopped motor scan', requires=('setup', 'adapters', 'services'))
def _scan(ctx):
    if not ctx.motors:
        return SKIP, 'Skipped on request: the motors were not read'
    report = ctx.probe('motors', timeout=90.)
    status, detail = scan_findings(ctx, report)
    if status == PASS:
        ctx.facts['positions'] = {row['name']: row['position_rad'] for row in report['motors']}
    return status, detail, report


@check('motors', 'pose', 'Positions inside the swept ranges', requires=('scan', 'limits'))
def _pose(ctx):
    limits, positions = ctx.facts['limits']['joints'], ctx.facts['positions']
    outside = []
    for motor, q in positions.items():
        low, high = limits[motor]['min_rad'], limits[motor]['max_rad']
        beyond = max(low - q, q - high)
        if beyond > POSE_TOLERANCE_RAD:
            outside.append(f'{ctx.label(motor)} at {q:+.2f} rad, {beyond:.2f} beyond its swept {low:+.2f}…{high:+.2f}')
    if outside:
        return FAIL, ('Outside the swept range: ' + '; '.join(outside) + '. Move it into range by hand, or re-sweep '
                      'it in Limits. Far outside can mean its zero shifted: do not operate until that is explained.')
    raw = {m['name']: m['raw_reference_rad'] for m in ctx.facts['reference']['motors']}
    farthest = max(positions, key=lambda motor: abs(positions[motor] - raw[motor]))
    return PASS, (f'All {len(positions)} inside their swept ranges; farthest from the reference pose: '
                  f'{ctx.label(farthest)} by {abs(positions[farthest] - raw[farthest]):.2f} rad')


# -- Stage 4: live joint sweep --------------------------------------------------

@check('sweep', 'sweep', 'Automated live joint sweep')
def _sweep(_ctx):
    return PENDING, ('Added once the live pipeline is proven: with the foot switch in hand, each follower joint '
                     'moves slowly through its range under the guarded receiver while tracking, effort and '
                     'temperature are checked.')


# -- Runner ------------------------------------------------------------------------

def _now():
    return datetime.now(timezone.utc).isoformat()


def verdict(report):
    statuses = [c['status'] for stage in report['stages'] for c in stage['checks']]
    failed = [c['title'] for stage in report['stages'] for c in stage['checks'] if c['status'] == FAIL]
    warned = statuses.count(WARN)
    if failed:
        return 'no_go', f"NO GO: {len(failed)} check{'s' if len(failed) > 1 else ''} failed ({', '.join(failed)})"
    if SKIP in statuses:
        return 'incomplete', 'INCOMPLETE: some checks were skipped, so this is not a go'
    return 'go', ('GO' + (f' with {warned} warning{"s" if warned > 1 else ""}' if warned else ': every check passed')
                  + '. The live joint sweep is not part of the check yet.')


def run_check(*, root=ROOT, profile=DEFAULT_PROFILE, teleop=DEFAULT_TELEOP, runs=DEFAULT_RUNS, limits=None,
              address=TETHER, motors=True, link_seconds=5., leaders_busy=None, publish=None, save=True,
              open_ssh=connect, open_leaders=None, serve_link=serve_link):
    """Run every check in order; publish(report) after each change. Returns the final report."""
    if open_leaders is None:
        def open_leaders(leader_profile):
            from robot_platform.teleop.openrb import OpenRBInputs
            inputs = OpenRBInputs(leader_profile)
            inputs.connect()
            return inputs
    ctx = Context(root=root, profile=profile, teleop=teleop, runs=runs, limits=limits, address=address,
                  motors=motors, link_seconds=link_seconds, leaders_busy=leaders_busy, open_ssh=open_ssh,
                  open_leaders=open_leaders, serve_link=serve_link)
    publish = publish or (lambda report: None)
    report = {'schema': 1, 'kind': 'minerva_check', 'asset_id': None, 'address': address,
              'started_at': _now(), 'finished_at': None, 'state': 'running', 'verdict': None, 'summary': None,
              'stages': [{'id': key, 'title': title, 'checks': [
                  {'id': c['id'], 'title': c['title'], 'status': PENDING, 'detail': '', 'seconds': None}
                  for c in CHECKS if c['stage'] == key]} for key, title in STAGES.items()],
              'evidence': None}
    entries = {c['id']: c for stage in report['stages'] for c in stage['checks']}
    started = time.perf_counter()
    try:
        for spec in CHECKS:
            entry = entries[spec['id']]
            blocked = [entries[key]['title'] for key in spec['requires'] if entries[key]['status'] in (FAIL, SKIP)]
            if blocked:
                entry.update(status=SKIP, detail=f"Needs {', '.join(blocked)}")
                publish(deepcopy(report))
                continue
            entry['status'] = RUNNING
            publish(deepcopy(report))
            began = time.perf_counter()
            try:
                result = spec['run'](ctx)
            except Exception as exc:
                result = (FAIL, str(exc) or type(exc).__name__)
            entry.update(status=result[0], detail=result[1], seconds=round(time.perf_counter() - began, 2))
            if len(result) > 2:
                entry['data'] = result[2]
            report['asset_id'] = ctx.facts.get('asset_id', report['asset_id'])
            publish(deepcopy(report))
    finally:
        if ctx.client is not None:
            ctx.client.close()
    report.update(state='finished', finished_at=_now(), seconds=round(time.perf_counter() - started, 1))
    report['verdict'], report['summary'] = verdict(report)
    if save:
        directory = Path(runs).resolve() / 'minerva-check'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
        path.write_text(json.dumps(report, indent=1, allow_nan=False) + '\n', encoding='utf-8')
        report['evidence'] = str(path)
    publish(deepcopy(report))
    return report


def latest_report(runs=DEFAULT_RUNS):
    """The most recent saved check report, or None."""
    directory = Path(runs).resolve() / 'minerva-check'
    for path in sorted(directory.glob('*.json'), reverse=True):
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        if report.get('kind') == 'minerva_check':
            report['evidence'] = str(path)
            return report
    return None
