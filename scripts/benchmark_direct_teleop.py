"""Measure direct updates on the native harness or Linux virtual CAN, never physical CAN.

Synthetic 14-joint targets isolate receiver/transport cost. This is not leader USB
latency, physical tracking accuracy, or independent scheduler evidence in harness mode.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time

from dev import source_identity
from guarded_vcan_bench import Motors, virtual_config
from robot_platform.control.guarded import GuardedControllerClient, GuardRejected
from robot_platform.control.guarded_transport import GuardedZmqTransport
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.teleop.latency import next_scheduled_slot, summarize


MANIFEST_REQUIRED = (
    'scripts/benchmark_direct_teleop.py', 'scripts/dev.py', 'scripts/guarded_vcan_bench.py',
    'src/robot_platform/control/guarded.py', 'src/robot_platform/control/guarded_transport.py',
    'src/robot_platform/simulation/control_harness.py', 'src/robot_platform/teleop/latency.py',
)


def verified_source(manifest=None, *, root=None):
    """Verify a packaged candidate's explicit file set when its .git is absent."""
    if manifest is None:
        return source_identity()
    root = (root or Path(__file__).resolve().parents[1]).resolve()
    document = json.loads(Path(manifest).read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('Source manifest must be a JSON object')
    identity, files = document.get('source_identity'), document.get('files')
    if (not isinstance(identity, dict) or identity.get('scope') != 'verified_candidate_bundle'
            or not re.fullmatch(r'[0-9a-f]{40}|[0-9a-f]{64}', str(identity.get('commit', '')))
            or not isinstance(files, dict) or not set(MANIFEST_REQUIRED) <= files.keys()):
        raise ValueError('Source manifest requires candidate scope, commit and all benchmark/runtime files')
    if any(files[name] is None for name in MANIFEST_REQUIRED):
        raise ValueError('Source manifest required runtime files cannot be deleted')
    required = {p.relative_to(root).as_posix() for base, pattern in
                ((root/'src', '*.py'), (root/'config', '*'), (root/'scripts', '*.py'))
                for p in base.rglob(pattern) if p.is_file() and '__pycache__' not in p.parts}
    if not required <= files.keys():
        raise ValueError('Source manifest omits candidate runtime/config files: ' +
                         ', '.join(sorted(required-files.keys())[:5]))
    digest = hashlib.sha256()
    for name in sorted(files):
        if not isinstance(name, str) or '\\' in name:
            raise ValueError('Source manifest paths must use relative forward-slash names')
        path = (root/name).resolve()
        if Path(name).is_absolute() or '..' in Path(name).parts or not path.is_relative_to(root):
            raise ValueError('Source manifest path escapes candidate: ' + name)
        expected = files[name]
        digest.update(name.encode())
        if expected is None:
            if path.exists():
                raise ValueError('Source manifest deletion is present: ' + name)
            digest.update(b'<deleted>')
        else:
            if not isinstance(expected, str) or not re.fullmatch(r'[0-9a-fA-F]{64}', expected):
                raise ValueError('Invalid source file digest: ' + name)
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected.lower():
                raise ValueError('Source manifest hash mismatch: ' + name)
            digest.update(bytes.fromhex(actual))
    if digest.hexdigest() != identity.get('content_sha256'):
        raise ValueError('Source manifest content fingerprint mismatch')
    return {**identity, 'manifest_sha256': hashlib.sha256(Path(manifest).read_bytes()).hexdigest()}


def verify_runtime_imports(root):
    """A verified candidate must not silently use an older editable installation."""
    root = Path(root).resolve()
    package_root = (root/'src/robot_platform').resolve()
    scripts = {'dev', 'guarded_vcan_bench', 'benchmark_direct_teleop', 'benchmark_direct_loop'}
    for name, module in tuple(sys.modules.items()):
        raw_path = getattr(module, '__file__', None)
        path = Path(raw_path).resolve() if raw_path else None
        if name == 'robot_platform' or name.startswith('robot_platform.'):
            valid = path is not None and path.is_relative_to(package_root)
        elif name in scripts:
            valid = path == (root/'scripts'/f'{name}.py').resolve()
        elif name == '__main__' and path is not None and path.stem in scripts:
            valid = path == (root/'scripts'/path.name).resolve()
        else:
            continue
        if not valid:
            raise ValueError(f'{name} was imported outside the verified candidate source directories; '
                             'set PYTHONPATH to its src and use its scripts')


def observation(snapshot):
    value = snapshot()
    return deepcopy({key: value.get(key) for key in
                     ('status', 'hardware_enabled', 'feedback', 'timing', 'tx_frames', 'rx_frames')})


def quiet_observation(controller, snapshot, seconds, *, wait=time.sleep, clock=time.perf_counter):
    """Observe a no-request interval; only vcan has independently advancing ticks."""
    before = observation(snapshot)
    exchanges, started = controller.exchanges, clock()
    wait(seconds)
    quiet_requests = controller.exchanges-exchanges
    controller.poll()
    after = observation(snapshot)
    ticks_before, ticks_after = before.get('timing') or {}, after.get('timing') or {}
    return dict(before=before, after=after, requested_wait_ms=seconds*1000,
                observed_client_ms=(clock()-started)*1000, quiet_requests=quiet_requests,
                receiver_elapsed_ms=after['status']['now_ms']-before['status']['now_ms'],
                receiver_tick_delta=(ticks_after['ticks']-ticks_before['ticks']
                                     if 'ticks' in ticks_before and 'ticks' in ticks_after else None))


def qualify_vcan_faults(controller, cfg, snapshot, emulator, checks, *,
                        wait=time.sleep, clock=time.perf_counter):
    """Explicit synthetic trials after timing; no automatic recovery in the live path.

    Source validity here is its encoded receiver budget, not a USB-reader test.
    Quiet intervals report observation bounds, not an inferred exact stop time.
    """
    if snapshot().get('mode') != 'virtual_can':
        raise ValueError('Fault qualification requires an explicitly virtual CAN receiver')

    def check(name, passed, **evidence):
        checks[name] = dict(passed=bool(passed), **evidence)
        if not passed:
            raise RuntimeError('Virtual CAN fault qualification failed: ' + name)

    def arm_trial():
        # Each trial begins from a deliberate emergency-stop/reset sequence.
        # Its calls are excluded from the steady-state latency measurements.
        controller.emergency_stop()
        wait(.02)
        controller.acquire()
        controller.reset()
        controller.arm()
        return controller.freshness_offer()

    def commands():
        return [dict(position=f['position'], velocity=0., effort=0.,
                     kp=j['hold_kp'], kd=j['hold_kd'])
                for f, j in zip(snapshot()['feedback'], cfg['joints'])]

    def rejected(name, request, allowed, **extra):
        before = observation(snapshot)
        reason = None
        try:
            controller._call(request)  # Deliberate raw replay injection in the fixture only.
        except GuardRejected as exc:
            reason = exc.reason
        after = observation(snapshot)
        unchanged = all(before['status'].get(key) == after['status'].get(key)
                        for key in ('joints', 'lease', 'command_expires_at_ms'))
        check(name, reason in allowed and unchanged, rejection=reason, before=before, after=after,
              authority_and_target_unchanged=unchanged, **extra)

    def quiet(seconds):
        return quiet_observation(controller, snapshot, seconds, wait=wait, clock=clock)

    def independent(evidence):
        return evidence['quiet_requests'] == 0 and (evidence['receiver_tick_delta'] or 0) > 1

    offered = arm_trial()
    request = dict(type='direct_update', authority=controller._authority(), identity=controller.identity,
                   offer=offered['offer']['id'], valid_for_ms=cfg['direct']['command_ms'],
                   joints=commands(), ramp_efforts=[])
    controller._call(request)
    rejected('command_replay', request, {'wrong_lease', 'sequence'})
    replay = deepcopy(request)
    replay['authority'] = controller._authority()
    rejected('consumed_offer', replay, {'invalid_offer'})

    offered = arm_trial()
    source_budget_ms = min(10, cfg['direct']['command_ms'])
    evidence = quiet(source_budget_ms/1000+.01)
    request = dict(type='direct_update', authority=controller._authority(), identity=controller.identity,
                   offer=offered['offer']['id'], valid_for_ms=source_budget_ms,
                   joints=commands(), ramp_efforts=[])
    rejected('expired_source_validity', request, {'offer_expired'},
             source_valid_for_ms=source_budget_ms, quiet=evidence)
    check('source_interval_independent', independent(evidence), **evidence)

    offered = arm_trial()
    controller.direct_update(commands(), offered, valid_for_ms=cfg['direct']['command_ms'])
    controller.stop()
    evidence = quiet(.02)
    status = evidence['after']['status']
    check('requested_stop', independent(evidence) and status['state'] == 'stopped'
          and status['reason'] == 'requested_stop' and status['lease'] is None
          and evidence['after']['hardware_enabled'] is True, **evidence)

    arm_trial()
    emulator.freeze = True
    try:
        evidence = quiet(cfg['feedback_timeout_ms']/1000+.02)
        status = evidence['after']['status']
        check('stale_feedback', independent(evidence) and status['state'] == 'faulted'
              and status['reason'] == 'feedback_stale' and status['lease'] is None
              and evidence['after']['hardware_enabled'] is False, **evidence)
    finally:
        emulator.freeze = False
    wait(.02)

    offered = arm_trial()
    controller.direct_update(commands(), offered, valid_for_ms=cfg['direct']['command_ms'])
    controller.emergency_stop()
    immediate = observation(snapshot)
    evidence = quiet(.02)
    status = immediate['status']
    check('emergency_stop', status['state'] == 'faulted' and status['reason'] == 'emergency_stop'
          and status['lease'] is None and immediate['hardware_enabled'] is False
          and all(not motor['enabled'] for motor in emulator.state.values()),
          immediate=immediate, **evidence)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('harness', 'vcan'), default='harness')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hz', type=int, choices=(50, 100, 200), default=100)
    parser.add_argument('--cycles', type=int, default=1000)
    parser.add_argument('--source-manifest', type=Path,
                        help='Verified candidate file manifest when running a release without .git')
    args = parser.parse_args()
    if args.output.exists() or not 10 <= args.cycles <= 45000:
        parser.error('Use a new output directory and 10-45000 cycles')
    if args.backend == 'vcan' and (sys.platform != 'linux' or args.binary is None):
        parser.error('vcan requires Linux and --binary pointing to guarded_controller')
    try:
        source = verified_source(args.source_manifest)
        if args.source_manifest is not None:
            verify_runtime_imports(Path(__file__).resolve().parents[1])
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True)
    service = virtual_config('minerva')
    service['endpoint'] = 'tcp://127.0.0.1:5592'
    for bus, name in zip(service['buses'], ('vcanDirectL', 'vcanDirectR')):
        bus['interface'] = name
    cfg = service['controller']
    cfg.update(lease_ms=150, feedback_timeout_ms=30, interpolation_ms=0,
               direct=dict(command_ms=max(30, 3000//args.hz), velocity_timeout_ms=math.ceil(1500/args.hz),
                           limit_reaction_ms=10))
    cfg['identity']['collision_policy'] = 'disabled-v1'
    config_path = args.output / 'config.json'
    config_path.write_text(json.dumps(service, indent=2)+'\n', encoding='utf-8')
    binary = args.binary or default_binary()
    report = dict(backend=args.backend, hz=args.hz, physical_validation=False,
                  independent_receiver_clock=args.backend == 'vcan', source='synthetic sine, 0.01 rad, 0.5 Hz',
                  source_identity=source, source_commit=source['commit'], source_dirty=bool(source.get('status')),
                  config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  limitation=__doc__, failure=None, fault_checks={}, fault_checks_passed=False)
    records = []
    harness = transport = emulator = process = controller = log = None
    receiver_verified = False
    try:
        if args.backend == 'harness':
            harness = NativeControlHarness(cfg, [0.]*len(cfg['joints']), binary=binary)
            harness.step(0)
            epoch = time.perf_counter()
            def exchange(request):
                now = max(harness.now_ms, int((time.perf_counter()-epoch)*1000))
                if now > harness.now_ms:
                    harness.step(now)
                return harness.step(now, request)['reply']
            snapshot = lambda: harness.last
        else:
            subprocess.run([str(binary.resolve()), str(config_path.resolve()), '--validate'], check=True)
            emulator = Motors(service)  # Checks kernel kind=vcan before any socket is opened.
            log = (args.output/'service.log').open('w', encoding='utf-8')
            process = subprocess.Popen([str(binary.resolve()), str(config_path.resolve()), '--seconds',
                                        str(args.cycles/args.hz+20)], stdout=log, stderr=log)
            transport = GuardedZmqTransport(service['endpoint'], timeout_ms=100)
            exchange, snapshot = transport.exchange, lambda: transport.last
        controller = GuardedControllerClient(exchange, group=cfg['group'], identity=cfg['identity'],
                                             joint_names=tuple(j['name'] for j in cfg['joints']))
        for _ in range(50):
            if process is not None and process.poll() is not None:
                raise RuntimeError('Virtual service exited; inspect service.log')
            try:
                controller.poll()
                if args.backend == 'vcan' and snapshot().get('mode') != 'virtual_can':
                    raise RuntimeError('Endpoint did not identify an explicitly virtual CAN receiver')
                receiver_verified = True
                if all(f['sequence'] > 0 for f in snapshot()['feedback']):
                    break
            except TimeoutError:
                pass
            time.sleep(.02)
        else:
            raise RuntimeError('No complete virtual feedback')
        if controller.status['state'] == 'faulted' and controller.status['reason'] == 'feedback_invalid':
            report['startup_fault'] = controller.status
            controller.acquire()
            controller.reset()
            controller.release()
        controller.acquire()
        controller.arm()
        offered = controller.freshness_offer()
        period, deadline = 1/args.hz, time.perf_counter()
        origin = deadline
        for sequence in range(1, args.cycles+1):
            wait = deadline-time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            started = time.perf_counter()
            scheduled = deadline
            t = started-origin
            q, v = .01*math.sin(math.pi*t), .01*math.pi*math.cos(math.pi*t)
            commands = [dict(position=q, velocity=v, effort=0., kp=j['hold_kp'], kd=j['hold_kd'])
                        for j in cfg['joints']]
            before = controller.exchanges
            rpc_started = time.perf_counter()
            offered = controller.direct_update(commands, offered, valid_for_ms=cfg['direct']['command_ms'])
            completed, result = time.perf_counter(), snapshot()
            deadline, skipped = next_scheduled_slot(scheduled, completed, period)
            status = offered['status']
            records.append(dict(operation='move', accepted=True, cycle_ms=(completed-started)*1000,
                rpc_ms=(completed-rpc_started)*1000, rpc_count=controller.exchanges-before,
                map_ms=(rpc_started-started)*1000,
                scheduled_at=scheduled, started_at=started, completed_at=completed,
                start_lateness_ms=max(0., started-scheduled)*1000, skipped_slots=skipped,
                scheduled_deadline_miss=completed > scheduled+period,
                source_sequence=[sequence], source_age_ms=(completed-started)*1000,
                desired=[q]*len(commands), commanded=[q]*len(commands),
                measured=[f['position'] for f in result['feedback']],
                request_sequence=status['lease']['last_sequence'], receiver_accepted_ms=status['now_ms'],
                receiver_epoch=status['epoch'],
                command_expires_at_ms=status['command_expires_at_ms'],
                receiver_applied=status.get('last_applied_target'), receiver_timing=result.get('timing')))
        # Deliberate producer silence: the receiver must expire its last command
        # independently of authority renewal or a newly submitted target.
        report['producer_silence'] = quiet_observation(
            controller, snapshot, cfg['direct']['command_ms']/1000+.01)
        status = controller.status
        # A deferred service's final update is applied after its acknowledgement.
        # This observation closes that sample without adding another motion cycle.
        records.append(dict(operation='status', accepted=True, receiver_epoch=status['epoch'],
                            receiver_applied=status.get('last_applied_target'),
                            receiver_state=status['state'], receiver_reason=status['reason']))
        report['silence_reason'] = status['reason']
        if status['reason'] != 'command_expired':
            raise RuntimeError(f"Producer silence ended with {status['reason']}")
        if args.backend == 'vcan' and not (report['producer_silence']['quiet_requests'] == 0
                and (report['producer_silence']['receiver_tick_delta'] or 0) > 1):
            raise RuntimeError('No independent receiver ticks observed during producer silence')
        controller.emergency_stop()
        if snapshot().get('hardware_enabled'):
            raise RuntimeError('Emergency stop left synthetic motors enabled')
        report['emergency_stop'] = True
        if args.backend == 'vcan':
            qualify_vcan_faults(controller, cfg, snapshot, emulator, report['fault_checks'])
            report['fault_checks_passed'] = all(case['passed'] for case in report['fault_checks'].values())
    except Exception as exc:
        report['failure'] = str(exc)
        # Capture the rejection before emergency-stop cleanup replaces its cause.
        status = getattr(exc, 'status', None) or (controller.status if controller else None) or {}
        report['failure_status'] = status
        report['failure_snapshot'] = (harness.last if harness is not None else
                                      transport.last if transport is not None else None)
        records.append(dict(operation='failure', accepted=False, failure=str(exc),
                            receiver_epoch=status.get('epoch'),
                            receiver_applied=status.get('last_applied_target'),
                            receiver_state=status.get('state'), receiver_reason=status.get('reason')))
    finally:
        if controller is not None and receiver_verified:
            try:
                controller.emergency_stop()
            except Exception as exc:
                report['cleanup_stop_error'] = str(exc)
        for resource in (harness, transport):
            if resource is not None:
                resource.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if emulator is not None:
            try:
                emulator.close()
            except Exception as exc:
                report['emulator_error'] = str(exc)
                report['failure'] = report['failure'] or str(exc)
        if log is not None:
            log.close()
        report['timing'] = summarize(records, 1/args.hz)
        report['scheduled_deadline_misses'] = report['timing']['scheduled_deadline_misses']
        report['skipped_slots'] = report['timing']['skipped_slots']
        cycle = report['timing']['cycle_ms']
        report['processing_target_ms'] = .8*1000/args.hz
        report['processing_target_met'] = bool(cycle and cycle['p99'] < report['processing_target_ms'])
        report['application_samples_complete'] = report['timing']['receiver_unobserved_targets'] == 0
        report['complete'] = (report['failure'] is None and report['timing']['cycles'] == args.cycles
                              and report['application_samples_complete'])
        report['scheduling_target_met'] = not (report['scheduled_deadline_misses'] or report['skipped_slots'])
        report['vcan_qualification_complete'] = (args.backend == 'vcan' and report['complete']
            and report['fault_checks_passed'] and report['processing_target_met']
            and report['scheduling_target_met'])
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records), encoding='utf-8')
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('backend', 'complete', 'failure', 'processing_target_met',
                                           'fault_checks_passed', 'vcan_qualification_complete', 'timing')}, indent=2))
    passed = (report['vcan_qualification_complete'] if args.backend == 'vcan'
              else report['complete'] and report['processing_target_met'])
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
