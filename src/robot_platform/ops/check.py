"""Minerva Check: whether the robot's systems are ready for live operation, run from the operator PC.

It checks the robot only, not how it is controlled: no leader arms, leader
ranges, routes or leader link (the control interface may change; checks
before teleop are separate). Stages run in order. A check whose prerequisite
failed or was skipped is skipped too, never guessed:
  1. Operator PC: calibration profile, motor-to-model directions, joint limits,
     servo calibration, the receiver configuration, the ARM64 builds and the
     route to the tether.
  2. Link and Jetson: pinned SSH over the tether, the deployed release (intact,
     matching this PC, smoke-tested), each arm's live receiver configuration
     validated by the deployed receiver, and the Jetson's health.
  3. Motors, stopped: CAN system setup, adapters and services, then the stopped
     reader's scan of every motor (identity, firmware, fault-free Reset and one
     position each; CAN ends down) and every position inside its swept range.
  4. Live joint sweep, one per arm, attended: each arm joint at its settings,
     and each gripper's travel.

Nothing here enables a motor. The verdict is GO when nothing failed or was
skipped; warnings are listed. Every run is saved as evidence.
"""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import secrets
import time

from .deploy import BASE, smoke
from .release import ROOT, check_binaries, collect, release_drift
from .ssh import DEFAULT_PROFILE, TETHER, connect, route_source, run

CURRENT = f'{BASE}/current'
PROBE = f'cd {CURRENT} && PYTHONPATH=src python3 -u -m robot_platform.ops.probe'
DEFAULT_TELEOP = ROOT / 'config/devices/minerva-teleop.json'
DEFAULT_RUNS = ROOT / 'calibration_runs'
TETHER_NETWORK = '10.42.0.'
POSE_TOLERANCE_RAD = .05
PASS, WARN, FAIL, SKIP = 'pass', 'warn', 'fail', 'skip'
PENDING = 'pending'  # Not built yet (the live sweep); QUEUED and RUNNING only appear during a run.
QUEUED, RUNNING = 'queued', 'running'
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

    def __init__(self, *, root, profile, teleop, runs, limits, address, motors, open_ssh):
        self.root, self.profile, self.teleop = Path(root), Path(profile).resolve(), Path(teleop).resolve()
        self.runs, self.limits_path = Path(runs).resolve(), limits
        self.address, self.motors, self.open_ssh = address, motors, open_ssh
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


def _teleop(ctx):
    """The teleop profile: the receiver's settings (gains, limits, collision policy and live block)."""
    if 'teleop' not in ctx.facts:
        from robot_platform.helm.teleop import load_profile
        ctx.facts['teleop'] = load_profile(ctx.teleop)
    return ctx.facts['teleop']


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


@check('pc', 'servo', 'Servo calibration', requires=('calibration',))
def _servo(ctx):
    """The live servo sweep's saved settings: every arm joint swept with the motor it has now. Grippers are not
    swept (they squeeze what they hold); the live joint sweep opens and closes them instead."""
    from robot_platform.control.guarded_config import motion_settings
    from robot_platform.control.servo_calibration import coverage, load_servo, servo_path
    path = servo_path(ctx.profile, ctx.facts['asset_id'])
    document = load_servo(path)
    ctx.facts['servo'] = document
    motion = _teleop(ctx)['motion']
    grippers = {motor['name'] for motor in ctx.facts['reference']['motors']
                if motion_settings(motion, motor['name']).get('gripper')}
    calibrated, missing, changed = coverage(document, ctx.facts['reference'], skip=grippers)
    if changed:
        ctx.facts['servo'] = None  # The receiver would refuse it; derive from the profile instead.
        return FAIL, ('Swept with another motor or firmware: ' + ', '.join(ctx.label(m) for m in changed)
                      + ". Sweep them again in Helm's Tune tab.")
    if missing:
        return WARN, (f'{len(calibrated)} of {len(calibrated) + len(missing)} arm joints calibrated; not swept yet: '
                      + ', '.join(ctx.label(m) for m in missing)
                      + ". Run the servo sweep in Helm's Tune tab; until then they use the teleop profile's settings.")
    return PASS, (f"All {len(calibrated)} arm joints calibrated by the servo sweep "
                  f"({document.get('updated_at', '')[:10]}); grippers are checked by their travel in the live sweep")


def _derive(ctx, arm=None, collision_bound=None):
    """The teleop receiver derived from this PC's calibration: the whole robot, or one arm; `collision_bound`
    'motion' for both arms live."""
    from types import SimpleNamespace
    from robot_platform.control.guarded_config import arm_subset, derive_receiver
    reference, steps, transmissions = ctx.facts['reference'], ctx.facts['plan']['steps'], ctx.facts['transmissions']
    if arm is not None:
        reference, steps, transmissions = arm_subset(reference, steps, transmissions, arm)
    # The derivation reads only identity revisions from the collision backend, so
    # skip the ~8 s CAD collision load here; the release smoke test loads the model.
    revisions = SimpleNamespace(policy_revision='check', runtime_model=SimpleNamespace(revision='check',
                                                                                        world_revision='check'))
    return derive_receiver(reference=reference, steps=steps, directions=ctx.facts['directions'],
                           transmissions=transmissions, model=ctx.facts['model'], limits=ctx.facts['limits'],
                           profile=_teleop(ctx), backend=revisions, servo=ctx.facts.get('servo'),
                           collision_bound=collision_bound)


@check('pc', 'receiver', 'Teleop receiver configuration', requires=('directions', 'limits'))
def _receiver(ctx):
    derived = _derive(ctx)
    rows = derived['report']
    share = {row['motor']: (row['usable'][1] - row['usable'][0]) / (row['swept'][1] - row['swept'][0])
             for row in rows}
    narrowest = min(share, key=share.get)
    mode = _teleop(ctx)['receiver'].get('collision_mode', 'enforced')
    detail = (f'{len(rows)} joints derive limits, margins and stopping reserves; collision {mode}; '
              f'narrowest usable range {ctx.label(narrowest)} at {share[narrowest]:.0%} of its swept travel')
    return PASS, detail, {'report': rows}


@check('pc', 'builds', 'ARM64 builds')
def _builds(ctx):
    files = collect(ctx.root)
    check_binaries(files, ctx.root)
    built = max(path.stat().st_mtime for key, path in files.items() if key.startswith(('bin/', 'lib/')))
    return PASS, f"Receiver, harness and collision library built {_ago(datetime.fromtimestamp(built, timezone.utc).isoformat())}, newer than their Rust sources"


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


@check('jetson', 'live_config', 'Live receiver configuration', requires=('release', 'receiver'))
def _live_config(ctx):
    from robot_platform.control.commissioning import motor_models
    from robot_platform.control.live_receiver import service_config
    live = _teleop(ctx).get('live')
    if live is None:
        return WARN, 'The teleop profile has no live block, so live teleop is not available'
    models = motor_models(ctx.facts['device'], ctx.root)
    directory = f'/tmp/minerva-check-{secrets.token_hex(4)}'
    problems = []
    try:
        # Each arm on its bus, and both arms on both buses under the two-arm (motion) collision bound.
        for arm in ('left', 'right', 'both'):
            derived = _derive(ctx, None, 'motion') if arm == 'both' else _derive(ctx, arm)
            reference = {'motors': [m for m in ctx.facts['reference']['motors']
                                    if m['name'] in {j['name'] for j in derived['config']['joints']}]}
            used = {m['bus'] for m in reference['motors']}
            config = service_config(derived['config'], ctx.facts['device'], reference, models,
                                    buses=[b['interface'] for b in ctx.facts['device']['buses'] if b['interface'] in used],
                                    tolerance_rad=live['reference_tolerance_rad'],
                                    period_us=live['period_us'], maximum_lateness_us=live['maximum_lateness_us'])
            path = f'{directory}/{arm}.json'
            status, _out, err = run(ctx.client, f'mkdir -p {directory} && cat > {path}', stdin=json.dumps(config))
            if status:
                raise RuntimeError(f'Could not stage the {arm} arm configuration: {err.strip()}')
            # --validate parses and checks the configuration only: no socket, CAN or motor.
            status, out, err = run(ctx.client, f'{CURRENT}/bin/guarded_controller {path} --validate', timeout=30)
            if status or 'configuration valid' not in out:
                problems.append(f'{"both arms" if arm == "both" else arm + " arm"}: '
                                f'{(err or out).strip()[-200:] or f"exit {status}"}')
    finally:
        run(ctx.client, f'rm -rf {directory}')
    if problems:
        return FAIL, 'The deployed receiver rejects: ' + '; '.join(problems)
    return PASS, (f"Both arms' teleop-mode configurations pass the deployed receiver's validation: one bus each, "
                  f"and both arms on both buses under the two-arm collision bound; watchdog identity and reference "
                  f"position for every motor, collision policy, {live['reference_tolerance_rad']:g} rad reference "
                  f"tolerance")


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
        ctx.facts['power_on'] = {row['name']: row.get('zero_sta') for row in report['motors']}
    return status, detail, report


@check('motors', 'power_on', 'Power-on position range', requires=('scan',))
def _power_on(ctx):
    """zero_sta, read while stopped: a drive left at the factory's 0…2π reads a whole turn high when its joint
    rests just below zero at power-up, which stops operation until someone turns it back and power-cycles."""
    ranges = ctx.facts['power_on']
    wide = [motor for motor, value in ranges.items() if value == 0]
    unknown = [motor for motor, value in ranges.items() if value not in (0, 1)]
    if wide or unknown:
        return WARN, ((f'{_names(ctx, wide)} start{"s" if len(wide) == 1 else ""} within 0…2π (zero_sta 0): a joint '
                       'resting just below its zero at power-up reads a whole turn high until it is turned back '
                       'and the motors are power-cycled. −π…π (zero_sta 1) avoids it. ' if wide else '')
                      + (f'{_names(ctx, unknown)} did not report a power-on range.' if unknown else '')).strip()
    return PASS, f'All {len(ranges)} drives start within −π…π of their zero'


@check('motors', 'pose', 'Positions inside the swept ranges', requires=('scan', 'limits'))
def _pose(ctx):
    from robot_platform.control.live_receiver import TURN_ADVICE, turned, whole_turns
    limits, positions = ctx.facts['limits']['joints'], ctx.facts['positions']
    outside, wrapped = [], []
    for motor, q in positions.items():
        low, high = limits[motor]['min_rad'], limits[motor]['max_rad']
        beyond = max(low - q, q - high)
        if beyond <= POSE_TOLERANCE_RAD:
            continue
        turns = whole_turns(q, low, high, tolerance=POSE_TOLERANCE_RAD)
        if turns:
            wrapped.append(turned(motor, q, turns, {motor: ctx.label(motor)}))
        else:
            outside.append(f'{ctx.label(motor)} at {q:+.2f} rad, {beyond:.2f} beyond its swept {low:+.2f}…{high:+.2f}')
    if outside or wrapped:
        return FAIL, ' '.join(
            ([f"{'; '.join(wrapped)}. {TURN_ADVICE}"] if wrapped else [])
            + (['Outside the swept range: ' + '; '.join(outside) + '. Move it into range by hand, or re-sweep it '
                'in Limits. Far outside can mean its zero shifted: do not operate until that is explained.']
               if outside else []))
    raw = {m['name']: m['raw_reference_rad'] for m in ctx.facts['reference']['motors']}
    farthest = max(positions, key=lambda motor: abs(positions[motor] - raw[motor]))
    return PASS, (f'All {len(positions)} inside their swept ranges; farthest from the reference pose: '
                  f'{ctx.label(farthest)} by {abs(positions[farthest] - raw[farthest]):.2f} rad')


@check('motors', 'live_pose', "Live start: reference pose, clear of the receiver's limits",
       requires=('scan', 'receiver'))
def _live_pose(ctx):
    """What a live session's own pre-scan checks, for each arm: the receiver faults on a reading
    outside its limits even disarmed, so a joint at a limit (a closed gripper on its swept end)
    would stop the session at once."""
    from robot_platform.control.live_receiver import (FEEDBACK_MARGIN_RAD, TURN_ADVICE, outside_limits, turned,
                                                      whole_turns)
    positions = ctx.facts['positions']
    raw = {m['name']: m['raw_reference_rad'] for m in ctx.facts['reference']['motors']}
    tolerance = _teleop(ctx).get('live', {}).get('reference_tolerance_rad', .1)
    blocked, wrapped, away = [], [], []
    for arm in ('left', 'right'):
        controller = _derive(ctx, arm)['config']
        names = [joint['name'] for joint in controller['joints']]
        for motor, q, lower, upper in outside_limits({name: positions[name] for name in names}, controller,
                                                     margin=FEEDBACK_MARGIN_RAD):
            turns = whole_turns(q, lower, upper)
            if turns:
                wrapped.append(turned(motor, q, turns, {motor: ctx.label(motor)}))
            else:
                blocked.append(f'{ctx.label(motor)} reads {q:+.4f} rad (receiver limits {lower:+.4f} to {upper:+.4f})')
        off = [f'{ctx.label(name)} {positions[name] - raw[name]:+.2f} rad' for name in names
               if abs(positions[name] - raw[name]) > tolerance]
        if off:
            away.append(f'{arm} arm: ' + ', '.join(off))
    if blocked or wrapped:
        return FAIL, ' '.join(
            ([f"{'; '.join(wrapped)}: the live receiver would fault at once. {TURN_ADVICE}"] if wrapped else [])
            + (["At or next to the live receiver's limits, which would fault it at once: " + '; '.join(blocked)
                + '. Re-sweep that joint in Limits.'] if blocked else []))
    if away:
        return WARN, ('Away from the reference pose, which a live session requires at its start: '
                      + '; '.join(away) + f' (tolerance {tolerance:g} rad). Let that arm hang straight down.')
    return PASS, (f"Both arms within {tolerance:g} rad of their reference pose and clear of the live receiver's "
                  'limits')


# -- Stage 4: live joint sweep --------------------------------------------------

LIVE_SWEEP = ('Run it from the Check tab after the stopped checks pass, with the foot switch in hand: each '
              'joint of the {arm} arm moves out and back for friction, then across its range at its settings, '
              'judged on lag, effort, heat and against its saved servo sweep. Its gripper opens from its swept '
              'min to its swept max and closes again.')


@check('sweep', 'sweep_left', 'Live joint sweep: left arm')
def _sweep_left(_ctx):
    return PENDING, LIVE_SWEEP.format(arm='left')


@check('sweep', 'sweep_right', 'Live joint sweep: right arm')
def _sweep_right(_ctx):
    return PENDING, LIVE_SWEEP.format(arm='right')


def live_sweep_entry(arm, results, *, fault=None):
    """(status, detail, data) for a live joint sweep's results ({motor: engine summary})."""
    if fault:
        return FAIL, f'The live sweep of the {arm} arm stopped: {fault}', {'results': results}
    checked = {motor: result.get('check') for motor, result in (results or {}).items()}
    if not checked or any(check is None for check in checked.values()):
        missing = [motor for motor, check in checked.items() if check is None]
        return FAIL, (f'The live sweep of the {arm} arm did not finish'
                      + (f' ({len(missing)} joint{"s" if len(missing) != 1 else ""} not checked)' if missing else '')
                      ), {'results': results}
    failed = {motor: check for motor, check in checked.items() if not check['passed']}
    if failed:
        return FAIL, '; '.join(f"{(results[motor].get('label') or motor)}: {', '.join(check['reasons'])}"
                               for motor, check in failed.items()), {'results': results}
    # A gripper's check is its travel (servo_sweep._verify_gripper): it is never servo-swept.
    uncalibrated = [motor for motor, check in checked.items() if check['calibrated'] is False]
    joints = [motor for motor, check in checked.items() if check.get('travel') is None]
    parts = []
    if joints:
        worst = max(joints, key=lambda motor: checked[motor]['lag'] or 0.)
        parts.append(f"All {len(joints)} joints followed within half their tracking bound; worst lag "
                     f"{checked[worst]['lag']:.3f} rad ({results[worst].get('label') or worst})")
    for motor, check in checked.items():
        if check.get('travel') is not None:
            travel = check['travel']
            parts.append(f"{results[motor].get('label') or motor} opened and closed its whole travel "
                         f"({travel['open']['seconds']:g} s, {travel['close']['seconds']:g} s)")
    detail = '; '.join(parts)
    if uncalibrated:
        return WARN, detail + f'; {len(uncalibrated)} not yet servo-swept, judged on the profile settings', \
            {'results': results}
    return PASS, detail + ', no more lag or friction than when swept', {'results': results}


def record_live(report, arm, results, *, fault=None, save=True):
    """Put a live sweep's outcome into a check report (the one it followed), recompute its verdict and
    save it over the same file."""
    entry = next(c for stage in report['stages'] for c in stage['checks'] if c['id'] == f'sweep_{arm}')
    status, detail, data = live_sweep_entry(arm, results, fault=fault)
    entry.update(status=status, detail=detail, data=data, recorded_at=_now())
    report['verdict'], report['summary'] = verdict(report)
    if save and report.get('evidence'):
        Path(report['evidence']).write_text(json.dumps(report, indent=1, allow_nan=False, default=float) + '\n',
                                            encoding='utf-8')
    return report


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
    warnings = f' with {warned} warning{"s" if warned > 1 else ""}' if warned else ''
    live = [c for stage in report['stages'] if stage['id'] == 'sweep' for c in stage['checks']]
    pending = [c['title'].split(': ')[-1] for c in live if c['status'] == PENDING]
    if pending:
        return 'incomplete', (f'INCOMPLETE: the stopped checks passed{warnings}. Run the live joint sweep on the '
                              + ' and the '.join(pending) + ' (attended, motors move) to complete Minerva Check')
    return 'go', 'GO' + (warnings if warned else ': every check passed, including the live joint sweeps')


def run_check(*, root=ROOT, profile=DEFAULT_PROFILE, teleop=DEFAULT_TELEOP, runs=DEFAULT_RUNS, limits=None,
              address=TETHER, motors=True, publish=None, save=True, open_ssh=connect):
    """Run every check in order; publish(report) after each change. Returns the final report."""
    ctx = Context(root=root, profile=profile, teleop=teleop, runs=runs, limits=limits, address=address,
                  motors=motors, open_ssh=open_ssh)
    publish = publish or (lambda report: None)
    report = {'schema': 1, 'kind': 'minerva_check', 'asset_id': None, 'address': address,
              'started_at': _now(), 'finished_at': None, 'state': 'running', 'verdict': None, 'summary': None,
              'stages': [{'id': key, 'title': title, 'checks': [
                  {'id': c['id'], 'title': c['title'], 'status': QUEUED, 'detail': '', 'seconds': None}
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


def outdated(report):
    """The titles of today's checks a saved report does not have (saved before they were added)."""
    present = {c['id'] for stage in report.get('stages', []) for c in stage.get('checks', [])}
    return [c['title'] for c in CHECKS if c['id'] not in present]


def latest_report(runs=DEFAULT_RUNS):
    """The most recent saved check report, or None. A report saved before checks were added is shown
    as outdated, whatever it said then: its GO was against fewer checks. The file is left as it was."""
    directory = Path(runs).resolve() / 'minerva-check'
    for path in sorted(directory.glob('*.json'), reverse=True):
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        if report.get('kind') == 'minerva_check':
            report['evidence'] = str(path)
            missing = outdated(report)
            if missing and report.get('state') == 'finished':
                report.update(verdict='outdated', summary=(
                    f"OUTDATED: saved before Minerva Check had {', '.join(missing)}. Run it again; "
                    f"it said then: {report.get('summary') or report.get('verdict')}"))
            return report
    return None
