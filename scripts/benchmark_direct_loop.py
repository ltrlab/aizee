"""Bounded full TeleopCore/run_loop benchmark with synthetic input and no physical CAN.

Uses saved Minerva calibration, limits, servo settings and leader references.
The leader waveform and ideal virtual motors are synthetic. Full-loop timing
includes recorder work and UI construction, not physical leader/motor latency.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from multiprocessing import Pipe, get_context
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import threading
import time
import traceback
from types import SimpleNamespace

from benchmark_direct_teleop import verified_source, verify_runtime_imports
from guarded_vcan_bench import Motors
from robot_platform.control.commissioning import motor_models, wire_ranges
from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.control.guarded_transport import GuardedZmqTransport
from robot_platform.helm.session import HelmSession
from robot_platform.helm.teleop import TeleopCore, TeleopSession, run_loop
from robot_platform.simulation.control_harness import NativeControlHarness, default_binary
from robot_platform.teleop.latency import distribution, summarize
from robot_platform.teleop.openrb import LeaderSample


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_candidate(root, manifest=None):
    identity = verified_source(manifest, root=root)
    if manifest is not None:
        verify_runtime_imports(root)
    release = None
    if (root/'release.json').is_file():
        from robot_platform.ops.release import activate
        release = activate(root)
    return identity, release


def require_virtual_mode(result):
    if result.get('mode') != 'virtual_can':
        raise ValueError('Benchmark endpoint did not identify itself as virtual_can; refusing authority')


def make_bundle(root, output, arm, runs=None, profile=None):
    """Load existing inputs unchanged; HelmSession only writes in output/runs."""
    devices = root / 'config/devices'
    seed = root / 'docs/devices/MINERVA_ARM_DIRECTIONS_20260924.json'
    profile = profile or devices / 'minerva-teleop-direct.json'
    ranges_path = devices / 'gello-dual-leader-ranges.json'
    ranges = json.loads(ranges_path.read_text(encoding='utf-8'))
    operator_inputs = {}
    if runs is not None:
        from robot_platform.control.joint_calibration import load_plan
        bindings = load_plan(devices / 'minerva-joint-calibration.json')[-1]
        candidates = [path for path in runs.glob('minerva-production-01-directions*/session.json')
                      if json.loads(path.read_text(encoding='utf-8')).get('bindings') == bindings]
        if len(candidates) != 1:
            raise ValueError('Provided --runs must contain exactly one direction session matching current bindings')
        routes = runs / 'minerva-production-01-teleop/routes.json'
        if not routes.is_file():
            raise ValueError('Provided --runs has no saved direct-profile routes.json')
        for source, relative in ((candidates[0], Path('minerva-production-01-directions/session.json')),
                                 (routes, Path('minerva-production-01-teleop/routes.json'))):
            destination = output / 'runs' / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            operator_inputs[str(source.relative_to(runs))] = sha(source)
    helm = HelmSession(devices / 'minerva-joint-calibration.json', runs=output / 'runs',
                       limits=devices / 'minerva-production-01-joint-limits.json', seed=seed if runs is None else None)
    try:
        if helm.saved_limits_error:
            raise ValueError(helm.saved_limits_error)
        if helm.saved_limits is None:
            raise ValueError('Saved Minerva joint limits are required')
        session = TeleopSession(helm, profile, runs=output / 'runs',
                               leaders=SimpleNamespace(saved=ranges, missing=lambda routes: []))
        bundle = session._bundle()
        bundle['receiver'] = dict(mode='live', arm=arm, motor_models=motor_models(helm.device, root))
        if runs is None:
            bundle['routes_path'] = None
            bundle['route_edits'] = {}
        inputs = [profile, ranges_path, helm.profile_path, helm.limits_path,
                  devices / 'minerva-production-01-servo.json', devices / 'gello-dual-viewer.json']
        if runs is None:
            inputs.append(seed)
        inputs += [root / b['hardware_profile'] for b in helm.device['buses']]
        # load_plan bindings also hash the reference, model and device inputs.
        identity = dict(bindings=helm.bindings, inputs={str(p.relative_to(root)): sha(p) for p in inputs},
                        operator_runs=operator_inputs,
                        directions_source='provided saved session' if runs else 'historical direction seed')
        (output/'bundle.json').write_text(json.dumps(bundle, indent=2)+'\n', encoding='utf-8')
        return bundle, ranges, identity
    finally:
        helm.close()


class ScriptedLeaders:
    inputs = SimpleNamespace(max_age_s=.05, max_skew_s=.02)

    def __init__(self, ranges, *, clock=time.perf_counter):
        self.reference = ranges['reference']['leaders']
        self.clock, self.sequence, self.follow_started = clock, 0, None

    def start_following(self, now):
        self.follow_started = now

    def poll(self):
        self.sequence += 1
        now = self.clock()
        delta = 0. if self.follow_started is None else .01 * math.sin(math.pi * (now-self.follow_started))
        # Small motion on the first six channels; the saved trigger reference
        # remains fixed, so the benchmark does not invent a gripper zero.
        return {name: LeaderSample(
            tuple((rows[f'channel_{i}'] + (delta if i < 7 else 0.)) % (2*math.pi) for i in range(1, 8)),
            (0,)*7, self.sequence, now, now, (0, 0, 1, 1), self.sequence, self.sequence)
            for name, rows in self.reference.items()}

    def close(self):
        pass


class QuantizedLeaders(ScriptedLeaders):
    """Independent 90 Hz sample grid, including immutable repeats at a 100 Hz consumer.

    This is a synthetic source, not a timing model of USB. Positions use the
    XL330's 4096-count encoder grid. Motion starts only after actual TELEOP.
    Fresh samples can retain an unchanged quantized position, independently of
    repeated source sequence numbers. Neither case receives a fabricated stamp.
    """
    source_hz, quantum_rad, pattern_s = 90., 2*math.pi/4096, 5.
    stages = ('held_start', 'positive_ramp', 'held_positive', 'negative_ramp',
              'held_negative', 'return_ramp', 'held_return')

    def __init__(self, ranges, *, clock=time.perf_counter, source_delay_s=.015):
        super().__init__(ranges, clock=clock)
        if not math.isfinite(source_delay_s) or not 0 <= source_delay_s < self.inputs.max_age_s-1/self.source_hz:
            raise ValueError('Synthetic source delay must leave room for one complete sample period')
        self.source_delay_s = source_delay_s
        self.delivery_epoch = clock()
        self.epoch, self.slot, self.cached = self.delivery_epoch-source_delay_s, None, None
        self.stage, self.source_elapsed_s = 'alignment', None

    def waveform(self, elapsed):
        at, amplitude = elapsed % self.pattern_s, 6*self.quantum_rad
        if at < .3:
            return 'held_start', 0.
        if at < 1.1:
            return 'positive_ramp', amplitude*(at-.3)/.8
        if at < 1.6:
            return 'held_positive', amplitude
        if at < 3.2:
            return 'negative_ramp', amplitude*(1.-2.*(at-1.6)/1.6)
        if at < 3.7:
            return 'held_negative', -amplitude
        if at < 4.5:
            return 'return_ramp', -amplitude*(1.-(at-3.7)/.8)
        return 'held_return', 0.

    def poll(self):
        now = self.clock()
        # Equivalent to now-epoch-delay, without cancellation which can make
        # the first slot -1 at large monotonic timestamps.
        slot = math.floor((now-self.delivery_epoch)*self.source_hz)
        if slot < 0 or self.slot is not None and slot < self.slot:
            raise ValueError('Synthetic leader clock moved backwards')
        if slot == self.slot:
            return self.cached
        sampled_at = self.epoch + slot/self.source_hz
        elapsed = None if self.follow_started is None else max(0., sampled_at-self.follow_started)
        self.stage, delta = ('alignment', 0.) if elapsed is None else self.waveform(elapsed)
        self.source_elapsed_s = elapsed
        self.sequence, self.slot = slot+1, slot
        samples = {}
        for name, rows in self.reference.items():
            raw = tuple(round((rows[f'channel_{i}'] + (delta if i < 7 else 0.))/self.quantum_rad)
                        for i in range(1, 8))
            # Keep the trigger at its saved reference, as in the default source.
            positions = tuple((q*self.quantum_rad if i < 6 else rows['channel_7']) % (2*math.pi)
                              for i, q in enumerate(raw))
            board_ms = round(slot/self.source_hz*1000)
            samples[name] = LeaderSample(positions, raw, self.sequence, sampled_at, now,
                                         (0, 0, 1, 1), board_ms, board_ms)
        self.cached = samples
        return samples


def initial_positions(config, reference):
    refs = {m['name']: m['raw_reference_rad'] for m in reference['motors']}
    return [min(j['upper'], max(j['lower'], refs[j['name']])) for j in config['joints']]


class HarnessReceiver:
    """Native harness with the same recorder callback surface as the live transport."""
    def __init__(self, config, reference, binary, directory):
        self.directory, self.transport = directory, SimpleNamespace(on_result=None)
        self.harness = NativeControlHarness(config, initial_positions(config, reference), binary=binary)
        self.harness.step(0)
        self.epoch = time.perf_counter()

    @property
    def last(self):
        return self.harness.last

    def exchange(self, request):
        now = max(self.harness.now_ms, int((time.perf_counter()-self.epoch)*1000))
        if now > self.harness.now_ms:
            self.harness.step(now)
        result = self.harness.step(now, request)
        if self.transport.on_result:
            self.transport.on_result(result)
        return result['reply']

    def close(self):
        self.harness.close()


def virtual_service(config, reference, models):
    """Actual limits/control config, synthetic transport only; no physical pins or enable flags."""
    refs = {m['name']: m for m in reference['motors']}
    used = sorted({m['bus'] for m in refs.values()})
    names = {'canL': 'vcanDirectL', 'canR': 'vcanDirectR'}
    if any(bus not in names for bus in used):
        raise ValueError('Benchmark requires the known Minerva left/right virtual bus mapping')
    return dict(mode='virtual_can', endpoint='tcp://127.0.0.1:5593', period_us=2500,
                maximum_lateness_us=10000, host_id=170, conflicting_services=[],
                buses=[dict(interface=names[bus], usb_serial=None) for bus in used],
                motors=[dict(name=j['name'], bus=used.index(refs[j['name']]['bus']), id=refs[j['name']]['id'],
                             scale=1., zero=0., ranges=wire_ranges(models[j['name']])) for j in config['joints']],
                controller=copy.deepcopy(config))


def encode_signed(value, span):
    return round(max(0., min(1., (value+span)/(2*span)))*65535)


def decode_signed(value, span):
    return value/65535*(2*span)-span


class ConfiguredMotors(Motors):
    """Ideal virtual positions with each saved model's wire encoding, no invented physics."""
    def run(self):
        try:
            ranges = {(m['bus'], m['id']): m['ranges'] for m in self.cfg['motors']}
            while not self.done:
                for bus, sock in enumerate(self.sockets):
                    for _ in range(512):
                        try:
                            raw = sock.recv(16)
                        except BlockingIOError:
                            break
                        arb, _, data = struct.unpack('=IB3x8s', raw)
                        arb &= 0x1fffffff
                        kind, motor_id = arb >> 24, arb & 255
                        key = (bus, motor_id)
                        state = self.state.get(key)
                        if state is None:
                            continue
                        if kind == 3:
                            state['enabled'] = True
                        elif kind == 4:
                            state.update(enabled=False, velocity=0., effort=0.)
                            self.last_disable = time.monotonic()
                        elif kind == 1 and state['enabled']:
                            q, v, _, _ = struct.unpack('>4H', data)
                            r = ranges[key]
                            state.update(position=decode_signed(q, r['position']),
                                         velocity=decode_signed(v, r['velocity']),
                                         effort=decode_signed((arb >> 8) & 65535, r['effort']))
                    else:
                        raise RuntimeError('Emulator receive budget exhausted')
                if not self.freeze:
                    for key, state in self.state.items():
                        bus, motor_id = key
                        r = ranges[key]
                        data = struct.pack('>4H', *(encode_signed(state[k], r[k]) for k in
                                                  ('position', 'velocity', 'effort')), 300)
                        arb = (2 << 24) | (int(state['enabled'])*2 << 22) | (motor_id << 8) | 170
                        self.sockets[bus].send(struct.pack('=IB3x8s', arb | 0x80000000, 8, data))
                time.sleep(.005)
        except BaseException as exc:
            self.error = repr(exc)


def motor_process_main(config, positions, reports, stop):
    """Independent feedback producer: parent GC/UI work must not pause virtual CAN."""
    emulator = None
    try:
        emulator = ConfiguredMotors(config)
        for motor, position in zip(config['motors'], positions):
            emulator.state[(motor['bus'], motor['id'])]['position'] = position
        reports.send(dict(kind='ready'))
        while not stop.wait(.01):
            if emulator.error:
                raise RuntimeError(emulator.error)
    except BaseException as exc:
        reports.send(dict(kind='error', message=str(exc), traceback=traceback.format_exc()))
    finally:
        if emulator:
            try:
                emulator.close()
            except Exception as exc:
                reports.send(dict(kind='error', message=str(exc)))
        reports.close()


class MotorProcess:
    """Bounded lifetime and error reporting for synthetic feedback on verified vcan."""
    def __init__(self, config, positions, *, context=None):
        ctx = context or get_context('spawn')
        self.reports, sender = ctx.Pipe(duplex=False)
        self.stop = ctx.Event()
        self.process = ctx.Process(target=motor_process_main, args=(config, positions, sender, self.stop),
                                   daemon=True, name='direct-virtual-motors')
        self.error, self.ready, self.closed, self.eof = None, False, False, False
        try:
            self.process.start()
            sender.close()
            if not self.reports.poll(10):
                raise TimeoutError('Virtual motor process did not become ready in 10 seconds')
            self.check()
            if not self.ready:
                raise RuntimeError('Virtual motor process exited before ready')
        except BaseException:
            sender.close()
            self.close()
            raise

    def _read_reports(self):
        while not self.eof and self.reports.poll():
            try:
                message = self.reports.recv()
            except EOFError:
                self.eof = True
                break
            if message['kind'] == 'ready':
                self.ready = True
            elif message['kind'] == 'error':
                self.error = self.error or message

    def check(self):
        self._read_reports()
        if self.error:
            raise RuntimeError('Virtual motor process: ' + self.error['message'])
        if self.process.exitcode is not None:
            raise RuntimeError(f'Virtual motor process exited unexpectedly ({self.process.exitcode})')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        if self.process.pid is not None:
            self.process.join(3)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(3)
                if self.process.is_alive():
                    self.process.kill()
                    self.process.join(3)
                self.error = self.error or dict(kind='error', message='Virtual motor process required termination')
            self._read_reports()
        self.reports.close()
        if self.error:
            raise RuntimeError('Virtual motor process: ' + self.error['message'])


class VirtualReceiver:
    def __init__(self, config, reference, models, binary, directory, seconds):
        self.directory, self.process, self.transport, self.emulator, self.log = directory, None, None, None, None
        self.startup_fault = self.close_error = self.first_fault = None
        self._initializing = True
        cfg = virtual_service(config, reference, models)
        path = directory / 'virtual-service.json'
        path.write_text(json.dumps(cfg, indent=2)+'\n', encoding='utf-8')
        subprocess.run([str(binary), str(path), '--validate'], check=True, timeout=10)
        try:
            # The child verifies kernel kind=vcan before opening sockets. It is
            # independent of TeleopCore's GC, recorder and view construction.
            self.emulator = MotorProcess(cfg, initial_positions(config, reference))
            self.log = (directory / 'service.log').open('w', encoding='utf-8')
            self.process = subprocess.Popen([str(binary), str(path), '--seconds', str(seconds)],
                                            stdout=self.log, stderr=self.log)
            self.transport = GuardedZmqTransport(cfg['endpoint'], timeout_ms=100)
            controller = GuardedControllerClient(self.exchange, group=config['group'], identity=config['identity'],
                                                 joint_names=tuple(j['name'] for j in config['joints']))
            deadline = time.perf_counter()+5
            while time.perf_counter() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('Virtual receiver exited; inspect service.log')
                try:
                    controller.poll()
                    require_virtual_mode(self.last)
                    if (len(self.last['feedback']) == len(config['joints'])
                            and all(row['sequence'] > 0 for row in self.last['feedback'])):
                        break
                except TimeoutError:
                    pass
                time.sleep(.02)
            else:
                raise TimeoutError('No complete virtual feedback')
            require_virtual_mode(self.last)
            if controller.status['state'] == 'faulted' and controller.status['reason'] == 'feedback_invalid':
                self.startup_fault = copy.deepcopy(controller.status)
                controller.acquire()
                controller.reset()
                controller.release()
            if controller.status['state'] != 'disarmed':
                raise RuntimeError(f'Virtual receiver is not disarmed: {controller.status}')
            self._initializing = False
        except BaseException:
            self.close()
            raise

    @property
    def last(self):
        return self.transport.last

    def exchange(self, request):
        self.emulator.check()
        try:
            return self.transport.exchange(request)
        finally:
            # Keep the first acquire/arm/update failure before lifecycle cleanup
            # deliberately changes the receiver reason to requested_stop.
            result = self.transport.last or {}
            reply, status = result.get('reply') or {}, result.get('status') or {}
            unexpected = (reply.get('accepted') is False or
                          (request.get('type') not in ('status', 'emergency_stop', 'disable', 'release')
                           and (status.get('state') == 'faulted'
                                or (request.get('type') == 'acquire' and status.get('state') != 'disarmed'))))
            if unexpected and not self._initializing and self.first_fault is None:
                self.first_fault = copy.deepcopy(dict(request=request, result=result))

    def close(self):
        if self.transport:
            self.transport.close()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        if self.emulator:
            try:
                self.emulator.close()
            except Exception as exc:
                self.close_error = str(exc)
        if self.log:
            self.log.close()


class ScriptedCommands:
    """One lifecycle key per measured cycle, with a wall-time bound and fail-closed cleanup."""
    def __init__(self, core, leaders, cycles, stop, maximum_seconds, *, on_follow_ready=None):
        self.core, self.leaders, self.cycles, self.stop_key = core, leaders, cycles, stop
        self.on_follow_ready = on_follow_ready
        self.deadline = core.clock()+maximum_seconds
        self.phase, self.followed, self.pending, self.sent_at = 'idle', 0, None, None
        self.failure, self.failure_snapshot, self.transitions = None, None, []
        self.last_following_target = None
        self.drain_cycles, self.drain_observed, self.drain_deadline = 0, False, None
        self.drain_max_seconds, self.drain_max_cycles = .05, 5

    def observe_cycle(self, entry):
        if self.phase == 'follow' and entry.get('accepted') and entry.get('request_sequence') is not None:
            self.last_following_target = dict(epoch=entry.get('receiver_epoch'),
                                             sequence=entry['request_sequence'],
                                             accepted_at_ms=entry.get('receiver_accepted_ms'))
        if self.phase == 'drain':
            self.drain_cycles += 1
            applied = entry.get('receiver_applied') or {}
            expected = self.last_following_target or {}
            self.drain_observed = self.drain_observed or bool(expected
                and entry.get('receiver_epoch') == expected['epoch']
                and applied.get('sequence') == expected['sequence']
                and applied.get('accepted_at_ms') == expected['accepted_at_ms']
                and applied.get('applied_at_ms', -1) >= expected['accepted_at_ms'])

    def poll(self):
        core = self.core
        if self.pending:
            return True
        if self.sent_at == core.driver_cycle:
            return False
        state = core.session.lifecycle.state
        if not self.failure and (core.fault or core.clock() > self.deadline):
            self.failure = core.fault or 'Full-loop benchmark exceeded its bounded duration'
            self.failure_snapshot = copy.deepcopy(core.harness.last)
            self.phase = 'abort'
        message = None
        if self.phase == 'abort':
            message, self.phase = ('key', 'H'), 'exit'
        elif self.phase == 'idle':
            message, self.phase = ('key', 'I'), 'enable'
        elif self.phase == 'enable' and state == 'IDLE':
            message, self.phase = ('key', 'E'), 'engage'
        elif self.phase == 'engage' and state == 'HOLD':
            message, self.phase = ('key', 'T'), 'align'
        elif self.phase == 'align' and state == 'TELEOP':
            self.phase = 'follow'
            # Drive the source actually opened by TeleopCore, including injected
            # scripted sources. The previous unused default instance hid motion.
            start = getattr(self.leaders, 'start_following', None)
            if start is not None:
                start(core.clock())
            elif hasattr(self.leaders, 'follow_started'):
                self.leaders.follow_started = core.clock()
            if self.on_follow_ready is not None:
                self.on_follow_ready()
        elif self.phase == 'follow' and self.followed >= self.cycles:
            # Observe the last measured target on a later receiver tick before
            # H can deliberately cancel it. Drain updates are clearly labelled
            # and excluded from the requested following measurements.
            self.phase, self.drain_deadline = 'drain', core.clock()+self.drain_max_seconds
        elif self.phase == 'drain':
            if self.drain_observed:
                message, self.phase = ('key', self.stop_key), 'stopping'
            elif core.clock() >= self.drain_deadline or self.drain_cycles >= self.drain_max_cycles:
                self.failure = 'Final measured target application was not observed during the bounded drain'
                self.failure_snapshot = copy.deepcopy(core.harness.last)
                message, self.phase = ('key', 'H'), 'exit'
        elif self.phase == 'stopping' and state == 'DISABLED':
            message, self.phase = ('stop',), 'done'
        elif self.phase == 'exit':
            message, self.phase = ('stop',), 'done'
        if message:
            self.transitions.append(dict(at=core.clock(), state=state, message=list(message)))
            self.pending, self.sent_at = message, core.driver_cycle
        return self.pending is not None

    def recv(self):
        message, self.pending = self.pending, None
        return message

    def close(self):
        pass


class BenchmarkCore(TeleopCore):
    driver_cycle = 0

    def finish_cycle(self, started, scheduled, completed, skipped_slots=0):
        super().finish_cycle(started, scheduled, completed, skipped_slots)
        self.driver_cycle += 1
        self._pending_latency['benchmark_phase'] = self.driver.phase
        self._pending_latency['lifecycle'] = self.session.lifecycle.state
        if getattr(self, 'benchmark_trial_origin', None) is None:
            origin = getattr(self.session.lifecycle, '_trial_origin', None)
            if origin is not None:
                self.benchmark_trial_origin = tuple(origin)
        if isinstance(self.driver.leaders, QuantizedLeaders):
            self._pending_latency['scripted_source_stage'] = self.driver.leaders.stage
            self._pending_latency['scripted_source_elapsed_s'] = self.driver.leaders.source_elapsed_s
            self._pending_latency['command_proposed_at'] = (self.session.last_cycle or {}).get('mapped_at')
        self.driver.observe_cycle(self._pending_latency)
        if self.driver.phase == 'follow' and self.session.lifecycle.state == 'TELEOP':
            self.driver.followed += 1


def summarize_loop(records, period):
    following = [r for r in records if r.get('benchmark_phase') == 'follow']
    timing = summarize(following, period)
    # The last following target is normally observed during the stop cycle.
    observations = [r for r in records if r.get('benchmark_phase') != 'follow']
    timing.update({k: v for k, v in summarize(following + [dict(r, operation='observation') for r in observations], period).items()
                   if k.startswith('receiver_')})
    reasons = Counter(reason for r in following for values in (r.get('limits') or {}).values() for reason in values)
    stamps = [r['started_at'] for r in following if r.get('started_at') is not None]
    elapsed = max(stamps)-min(stamps) if len(stamps) > 1 else 0
    timing['unique_source_rate_hz'] = (timing['unique_source_samples']-1)/elapsed if elapsed > 0 else None
    timing['limit_reasons'] = dict(reasons)
    timing['all_loop_ms'] = distribution(r.get('full_loop_ms') for r in records)
    return timing


def moving_source_coverage(records, joints, *, position_only, trial=None, trial_origin=None):
    """Fail qualification if a successful timing run did not exercise the defect.

    Checks command requests before motor CAN quantization. The ideal emulator
    cannot establish physical tracking smoothness, friction or stopping distance.
    Only joints which actually moved need plateau arrival: a trial profile may
    deliberately latch every joint except one.
    """
    from robot_platform.control.gravity import gravity_effort
    following = [r for r in records if r.get('benchmark_phase') == 'follow'
                 and r.get('lifecycle') == 'TELEOP' and r.get('accepted')]
    count = len(joints)
    complete = bool(following) and all(
        all(isinstance(r.get(key), list) and len(r[key]) == count
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in r[key])
            for key in ('desired', 'commanded', 'commanded_velocity', 'commanded_effort'))
        and len(r.get('source_sequence') or []) == count for r in following)
    result = dict(complete_command_observations=complete, stages={}, moved_joints=[],
                  reversed_joints=[], repeated_samples=0, fresh_unchanged_positions=0,
                  immutable_repeats=True, position_only_required=position_only,
                  zero_output_velocity=False, gravity_only_effort=False,
                  plateau_reached={}, checks={}, complete=False,
                  output_scope='direct_update joint values before motor CAN quantization',
                  goal_tolerance_rad=1e-8)
    if not complete:
        result['checks']['complete_command_observations'] = False
        return result
    stages = Counter(r.get('scripted_source_stage') for r in following)
    result['stages'] = dict(stages)
    origin_valid = not trial or trial_origin is not None and len(trial_origin) == count
    result['trial'] = trial
    result['trial_origin'] = list(trial_origin) if trial_origin is not None else None
    trial_bounds = not trial or origin_valid and all(
        abs(command-origin) <= (trial['max_excursion_rad'] if joint['name'] == trial['motor'] else 0.) + 1e-12
        for record in following for joint, origin, command in zip(joints, trial_origin, record['commanded']))
    trial_speed, reference_speeds = True, []
    for before, after in zip(following, following[1:]):
        times = [row.get('command_proposed_at') for row in (before, after)]
        if not all(isinstance(t, (int, float)) and math.isfinite(t) for t in times) or times[1] <= times[0]:
            trial_speed = False
            continue
        dt = times[1]-times[0]
        increments = [abs(a-b) for a, b in zip(after['commanded'], before['commanded'])]
        reference_speeds.extend(delta/dt for delta in increments)
        if trial and any(delta > trial['max_velocity']*dt + 1e-12 for delta in increments):
            trial_speed = False
    result['maximum_reference_speed_rad_s'] = max(reference_speeds, default=None)
    def goal(record):
        if not trial or not origin_valid:
            return record['desired']
        radius = trial['max_excursion_rad']
        return [max(origin-radius, min(origin+radius, desired)) if joint['name'] == trial['motor'] else origin
                for joint, origin, desired in zip(joints, trial_origin, record['desired'])]
    moved, reversed_joints = [], []
    for i, joint in enumerate(joints):
        values = [r['commanded'][i] for r in following]
        differences = [b-a for a, b in zip(values, values[1:])]
        if max(values)-min(values) > 1e-5:
            moved.append(i)
            result['moved_joints'].append(joint['name'])
        if any(d > 1e-8 for d in differences) and any(d < -1e-8 for d in differences):
            reversed_joints.append(i)
            result['reversed_joints'].append(joint['name'])
    for before, after in zip(following, following[1:]):
        if before['source_sequence'] == after['source_sequence']:
            result['repeated_samples'] += 1
            stamps = [r.get('completed_at') is not None and r.get('source_age_ms') is not None
                      for r in (before, after)]
            same_stamp = all(stamps) and abs(
                (before['completed_at']-before['source_age_ms']/1000)
                -(after['completed_at']-after['source_age_ms']/1000)) <= 1e-8
            result['immutable_repeats'] &= before['desired'] == after['desired'] and same_stamp
        elif before['desired'] == after['desired']:
            result['fresh_unchanged_positions'] += 1
    for stage in ('held_positive', 'held_negative', 'held_return'):
        result['plateau_reached'][stage] = sum(
            r.get('scripted_source_stage') == stage and bool(moved)
            and all(abs(r['commanded'][i]-goal(r)[i]) <= result['goal_tolerance_rad'] for i in moved)
            for r in following)
    result['zero_output_velocity'] = all(v == 0. for r in following for v in r['commanded_velocity'])
    errors = []
    for record in following:
        positions = dict(zip((j['name'] for j in joints), record['commanded']))
        for joint, effort in zip(joints, record['commanded_effort']):
            gravity = gravity_effort(joint.get('gravity') or [], positions)
            limit = .9*joint.get('max_effort', math.inf)
            errors.append(abs(effort-max(-limit, min(limit, gravity))))
    result['maximum_non_gravity_effort_nm'] = max(errors)
    result['gravity_only_effort'] = max(errors) <= 1e-10
    result['checks'] = dict(complete_command_observations=True,
                           trial_origin_observed=bool(origin_valid),
                           trial_command_bounds=bool(trial_bounds),
                           trial_command_speed=not trial or trial_speed,
                           all_stages=all(stages.get(stage, 0) >= 2 for stage in QuantizedLeaders.stages),
                           commanded_movement=bool(moved), commanded_reversal=bool(reversed_joints),
                           repeated_samples=result['repeated_samples'] >= 2,
                           immutable_repeats=result['immutable_repeats'],
                           fresh_unchanged_positions=result['fresh_unchanged_positions'] >= 2,
                           reached_plateaus=all(n >= 2 for n in result['plateau_reached'].values()),
                           position_only_output=(not position_only or result['zero_output_velocity']
                                                 and result['gravity_only_effort']))
    result['complete'] = all(result['checks'].values())
    return result


def main(argv=None, *, open_leaders=None, source_label=None, on_follow_ready=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--backend', choices=('harness', 'vcan'), default='harness')
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--source-manifest', type=Path)
    parser.add_argument('--runs', type=Path, help='Saved operator runs to copy into output; missing/mismatched data fails')
    parser.add_argument('--profile', type=Path, help='Teleop profile within --root; default config/devices/minerva-teleop-direct.json')
    parser.add_argument('--source-mode', choices=('sine', 'quantized90'), default='sine',
                        help='quantized90 exercises holds, ramps, reversals and reached goals over a 5-second pattern')
    parser.add_argument('--arm', choices=('left', 'right', 'both'), default='both')
    parser.add_argument('--stop', choices=('H', 'X'), default='X')
    parser.add_argument('--cycles', type=int, default=3000)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists() or not 10 <= args.cycles <= 30000:
        parser.error('Use a new output directory and 10-30000 cycles')
    if args.backend == 'vcan' and (sys.platform != 'linux' or args.binary is None):
        parser.error('vcan requires Linux and --binary guarded_controller')
    if args.source_mode == 'quantized90' and (open_leaders is not None or args.cycles < 500):
        parser.error('quantized90 needs at least 500 cycles and cannot replace an injected source')
    args.root, args.output = args.root.resolve(), args.output.resolve()
    if args.profile is not None:
        args.profile = (args.root/args.profile).resolve()
        if not args.profile.is_relative_to(args.root):
            parser.error('--profile must be inside --root')
    if (args.root/'release.json').exists() and args.output.is_relative_to(args.root):
        parser.error('Release benchmark output must stay outside its verified candidate tree')
    args.output.mkdir(parents=True)
    report = dict(complete=False, failure=None, backend=args.backend, arm=args.arm, stop=args.stop,
                  requested_cycles=args.cycles, physical_validation=False, synthetic_source=open_leaders is None,
                  source_mode=args.source_mode if open_leaders is None else 'injected',
                  source_delivery_delay_ms=15. if open_leaders is None and args.source_mode == 'quantized90' else None,
                  source_waveform=(None if open_leaders is not None else
                                   '4096-count, 90 Hz immutable samples; 5 s holds/ramps/reversals, +/-6 counts on channels 1..6'
                                   if args.source_mode == 'quantized90' else
                                   'saved leader reference + 0.01 rad sine, 0.5 Hz on channels 1..6'),
                  source_label=source_label or ('scripted leaders' if open_leaders is None else 'injected leader source'),
                  independent_receiver_clock=args.backend == 'vcan', limitation=__doc__,
                  script_sha256=sha(__file__))
    core = receiver = driver = reader = writer = leaders = None
    view_report = dict(messages=0, error=None)
    def consume_views():
        try:
            while True:
                reader.recv()
                view_report['messages'] += 1
        except EOFError:
            pass
        except Exception as exc:
            view_report['error'] = str(exc)
        finally:
            reader.close()
    try:
        source, release = prepare_candidate(args.root, args.source_manifest)
        report['source_identity'] = source
        if release:
            report['release'] = dict(id=release['id'], content_sha256=release['content_sha256'])
        binary = (args.binary or default_binary()).resolve()
        if args.source_manifest and not binary.is_relative_to(args.root):
            raise ValueError('Verified candidate benchmark binary must be inside its root')
        report['binary_sha256'] = sha(binary)
        bundle, ranges, identity = make_bundle(args.root, args.output, args.arm,
                                               args.runs.resolve() if args.runs else None, args.profile)
        if args.source_manifest:
            verify_runtime_imports(args.root)
        report['inputs'] = identity
        def open_source(profile):
            nonlocal leaders
            leaders = (open_leaders(profile) if open_leaders is not None else
                       QuantizedLeaders(ranges) if args.source_mode == 'quantized90' else ScriptedLeaders(ranges))
            if driver is not None:
                driver.leaders = leaders
            return leaders
        def open_receiver(config, device, reference):
            nonlocal receiver
            (args.output/'controller.json').write_text(json.dumps(config, indent=2)+'\n', encoding='utf-8')
            report['config_sha256'] = sha(args.output/'controller.json')
            report['initial_positions'] = initial_positions(config, reference)
            receiver = (HarnessReceiver(config, reference, binary, args.output) if args.backend == 'harness' else
                        VirtualReceiver(config, reference, bundle['receiver']['motor_models'], binary,
                                        args.output, args.cycles/100+65))
            return receiver
        def no_geometry(_):
            raise RuntimeError('Direct benchmark attempted to construct collision geometry')
        core = BenchmarkCore(bundle, open_leaders=open_source, open_receiver=open_receiver,
                             open_backend=no_geometry)
        if core.period_s != .01 or core.collision_mode != 'disabled':
            raise ValueError('This benchmark requires the saved 100 Hz collision-disabled profile')
        report['position_only'] = core.profile['receiver']['direct'].get('position_only', False)
        driver = ScriptedCommands(core, leaders, args.cycles, args.stop, args.cycles*.01+60,
                                  on_follow_ready=on_follow_ready)
        core.driver = driver
        reader, writer = Pipe(duplex=False)
        thread = threading.Thread(target=consume_views, daemon=True, name='benchmark-view-reader')
        thread.start()
        run_loop(driver, writer, core)
        thread.join(3)
        if args.source_manifest:
            verify_runtime_imports(args.root)
        report.update(failure=driver.failure or core.fault, transitions=driver.transitions,
                      drain=dict(cycles=driver.drain_cycles, observed=driver.drain_observed,
                                 target=driver.last_following_target, maximum_seconds=driver.drain_max_seconds,
                                 maximum_cycles=driver.drain_max_cycles, included_in_following_timing=False),
                      failure_snapshot=driver.failure_snapshot, phase=driver.phase,
                      final_state=core.session.lifecycle.state if core.session else None,
                      final_snapshot=receiver.last if receiver else None,
                      first_fault=getattr(receiver, 'first_fault', None),
                      startup_fault=getattr(receiver, 'startup_fault', None),
                      emulator_error=getattr(receiver, 'close_error', None))
        if report['emulator_error']:
            report['failure'] = report['failure'] or report['emulator_error']
    except Exception as exc:
        report['failure'] = str(exc)
        report['traceback'] = traceback.format_exc()
        if core:
            core.close()
    finally:
        trace = args.output/'direct-latency.jsonl'
        records = [json.loads(line) for line in trace.read_text(encoding='utf-8').splitlines()] if trace.exists() else []
        report['timing'] = summarize_loop(records, .01)
        report['views'] = view_report
        timing = report['timing']
        report['processing_target_ms'] = 8.
        report['processing_target_met'] = bool(timing['full_loop_ms'] and timing['full_loop_ms']['p99'] < 8.)
        report['scheduling_target_met'] = (timing['scheduled_cycles'] == args.cycles
                                           and timing['scheduled_deadline_misses'] == 0
                                           and timing['skipped_slots'] == 0)
        report['application_samples_complete'] = timing['receiver_unobserved_targets'] == 0
        if args.source_mode == 'quantized90':
            joints = core.session.lifecycle.joints if core is not None and core.session is not None else []
            report['moving_source_coverage'] = moving_source_coverage(
                records, joints, position_only=report.get('position_only', False),
                trial=core.profile.get('trial') if core is not None else None,
                trial_origin=getattr(core, 'benchmark_trial_origin', None))
            if not report['moving_source_coverage']['complete']:
                report['failure'] = report['failure'] or 'Moving-source qualification coverage failed'
        report['complete'] = (report['failure'] is None and report.get('phase') == 'done'
                              and report.get('final_state') == 'DISABLED' and timing['cycles'] == args.cycles
                              and timing['accepted'] == args.cycles and timing['maximum_rpc_count'] == 1
                              and report['application_samples_complete'] and view_report['error'] is None
                              and view_report['messages'] >= 2
                              and report.get('final_snapshot', {}).get('hardware_enabled') is False)
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in
                     ('complete', 'failure', 'processing_target_met', 'scheduling_target_met', 'timing')}, indent=2))
    return 0 if report['complete'] and report['processing_target_met'] and report['scheduling_target_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
