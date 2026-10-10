"""Measure direct updates on the native harness or Linux virtual CAN, never physical CAN.

Synthetic 14-joint targets isolate receiver/transport cost. This is not leader USB
latency, physical tracking accuracy, or independent scheduler evidence in harness mode.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from dev import source_identity
from guarded_vcan_bench import Motors, virtual_config
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.control.guarded_transport import GuardedZmqTransport
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.teleop.latency import next_scheduled_slot, summarize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('harness', 'vcan'), default='harness')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hz', type=int, choices=(50, 100, 200), default=100)
    parser.add_argument('--cycles', type=int, default=1000)
    args = parser.parse_args()
    if args.output.exists() or not 10 <= args.cycles <= 45000:
        parser.error('Use a new output directory and 10-45000 cycles')
    if args.backend == 'vcan' and (sys.platform != 'linux' or args.binary is None):
        parser.error('vcan requires Linux and --binary pointing to guarded_controller')
    source = source_identity()
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
                  source_identity=source, source_commit=source['commit'], source_dirty=bool(source['status']),
                  config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                  binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                  limitation=__doc__, failure=None)
    records = []
    harness = transport = emulator = process = controller = log = None
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
        time.sleep(cfg['direct']['command_ms']/1000+.01)
        status = controller.poll()
        # A deferred service's final update is applied after its acknowledgement.
        # This observation closes that sample without adding another motion cycle.
        records.append(dict(operation='status', accepted=True, receiver_epoch=status['epoch'],
                            receiver_applied=status.get('last_applied_target'),
                            receiver_state=status['state'], receiver_reason=status['reason']))
        report['silence_reason'] = status['reason']
        if status['reason'] != 'command_expired':
            raise RuntimeError(f"Producer silence ended with {status['reason']}")
        controller.emergency_stop()
        if snapshot().get('hardware_enabled'):
            raise RuntimeError('Emergency stop left synthetic motors enabled')
        report['emergency_stop'] = True
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
        if controller is not None:
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
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records), encoding='utf-8')
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('backend', 'complete', 'failure', 'processing_target_met', 'timing')}, indent=2))
    return 0 if report['complete'] and report['processing_target_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
