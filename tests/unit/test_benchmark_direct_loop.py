"""The full-loop benchmark fails closed and distinguishes source/timing evidence."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def bench(monkeypatch):
    scripts = Path(__file__).resolve().parents[2] / 'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('benchmark_direct_loop', scripts / 'benchmark_direct_loop.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_virtual_service_retains_real_bounds_and_only_maps_known_virtual_buses(bench):
    config = dict(joints=[dict(name='right_arm_j1', lower=-1., upper=2.)], direct=dict(command_ms=30))
    reference = dict(motors=[dict(name='right_arm_j1', bus='canR', id=4, raw_reference_rad=.1)])
    service = bench.virtual_service(config, reference, {'right_arm_j1': 'ROBSTRIDE03'})
    assert service['mode'] == 'virtual_can'
    assert service['buses'] == [dict(interface='vcanDirectR', usb_serial=None)]
    assert service['controller'] == config and service['controller'] is not config
    assert service['motors'][0]['ranges']['effort'] == 60.
    assert service['endpoint'] == 'tcp://127.0.0.1:5593'
    reference['motors'][0]['bus'] = 'canX'
    with pytest.raises(ValueError, match='virtual bus mapping'):
        bench.virtual_service(config, reference, {'right_arm_j1': 'ROBSTRIDE03'})


def test_emulator_encodes_per_model_spans(bench):
    for span in (14., 17., 60., 120.):
        for value in (-span, -.1, 0., .1, span):
            encoded = bench.encode_signed(value, span)
            assert bench.decode_signed(encoded, span) == pytest.approx(value, abs=2*span/65535)


def test_scripted_command_failure_does_not_need_sequence_to_advance(bench):
    now = [1.]
    core = SimpleNamespace(clock=lambda: now[0], sequence=0, driver_cycle=0, fault=None,
                           session=SimpleNamespace(lifecycle=SimpleNamespace(state='DISABLED')),
                           harness=SimpleNamespace(last={'status': {'reason': 'command_expired'}}))
    driver = bench.ScriptedCommands(core, SimpleNamespace(follow_started=None), 20, 'X', 10)
    assert driver.poll() and driver.recv() == ('key', 'I')
    assert not driver.poll()
    core.driver_cycle += 1
    core.fault = 'command_expired'
    assert driver.poll() and driver.recv() == ('key', 'H')
    assert driver.failure == 'command_expired'
    core.driver_cycle += 1
    assert driver.poll() and driver.recv() == ('stop',)
    assert driver.phase == 'done'


def test_full_loop_summary_counts_lateness_and_repeated_sources_and_stop_observation(bench):
    records = [dict(benchmark_phase='follow', operation='move', accepted=True, cycle_ms=2., full_loop_ms=4.,
                    started_at=t, source_sequence=[seq], scheduled_deadline_miss=late, rpc_count=1,
                    limits={'q': ['acceleration']}, receiver_epoch=1, request_sequence=i,
                    receiver_accepted_ms=i*10)
               for i, (t, seq, late) in enumerate(((1., 8, False), (1.01, 8, True), (1.02, 9, False)), 1)]
    records.append(dict(benchmark_phase='stopping', receiver_epoch=1,
                        receiver_applied=dict(sequence=3, accepted_at_ms=30, applied_at_ms=32)))
    result = bench.summarize_loop(records, .01)
    assert result['full_loop_ms']['p99'] == 4.
    assert result['scheduled_deadline_misses'] == 1
    assert result['unique_source_samples'] == 2
    assert result['unique_source_rate_hz'] == pytest.approx(50.)
    assert result['receiver_apply_ms']['p99'] == 2.
    assert result['limit_reasons'] == {'acceleration': 3}


def test_candidate_verification_precedes_release_activation(bench, monkeypatch, tmp_path):
    import robot_platform.ops.release as release
    (tmp_path/'release.json').write_text('{}')
    manifest = tmp_path.parent/'candidate-source.json'
    calls = []
    def verify(source, *, root):
        calls.append(('verify', source, root))
        return {'content_sha256': 'source'}
    monkeypatch.setattr(bench, 'verified_source', verify)
    monkeypatch.setattr(bench, 'verify_runtime_imports', lambda root: calls.append(('imports', root)))
    monkeypatch.setattr(release, 'activate', lambda root: calls.append(('activate', root)) or {'id': 'release'})
    identity, activated = bench.prepare_candidate(tmp_path, manifest)
    assert identity['content_sha256'] == 'source' and activated['id'] == 'release'
    assert calls == [('verify', manifest, tmp_path), ('imports', tmp_path), ('activate', tmp_path)]
    calls.clear()
    def corrupt(*args, **kwargs):
        raise ValueError('Source manifest hash mismatch')
    monkeypatch.setattr(bench, 'verified_source', corrupt)
    with pytest.raises(ValueError, match='hash mismatch'):
        bench.prepare_candidate(tmp_path, manifest)
    assert calls == []


def test_vcan_endpoint_cannot_acquire_or_reset_when_mode_is_physical(bench, monkeypatch, tmp_path):
    config = dict(group='test', identity={}, joints=[dict(name='left_arm_j1', lower=-1., upper=1.)])
    reference = dict(motors=[dict(name='left_arm_j1', bus='canL', id=4, raw_reference_rad=0.)])
    process = SimpleNamespace(code=None)
    process.poll = lambda: process.code
    process.terminate = lambda: setattr(process, 'code', 0)
    process.wait = lambda **kwargs: 0
    transport = SimpleNamespace(last={'mode': 'teleop_direct', 'feedback': [{'sequence': 1}]},
                                close=lambda: None, exchange=lambda _: None)
    controller = SimpleNamespace(status={'state': 'faulted', 'reason': 'feedback_invalid'}, poll=lambda: None,
                                 acquire=lambda: pytest.fail('must not acquire a physical endpoint'),
                                 reset=lambda: pytest.fail('must not reset a physical endpoint'))
    monkeypatch.setattr(bench, 'MotorProcess', lambda *args: SimpleNamespace(check=lambda: None, close=lambda: None))
    monkeypatch.setattr(bench.subprocess, 'run', lambda *args, **kwargs: None)
    monkeypatch.setattr(bench.subprocess, 'Popen', lambda *args, **kwargs: process)
    monkeypatch.setattr(bench, 'GuardedZmqTransport', lambda *args, **kwargs: transport)
    monkeypatch.setattr(bench, 'GuardedControllerClient', lambda *args, **kwargs: controller)
    with pytest.raises(ValueError, match='refusing authority'):
        bench.VirtualReceiver(config, reference, {'left_arm_j1': 'ROBSTRIDE03'},
                              tmp_path/'guarded_controller', tmp_path, 5)
    assert process.poll() == 0


def test_virtual_receiver_preserves_first_fault_before_stop_cleanup(bench):
    receiver = bench.VirtualReceiver.__new__(bench.VirtualReceiver)
    receiver.first_fault, receiver._initializing = None, False
    receiver.emulator = SimpleNamespace(check=lambda: None)
    result = dict(reply={'accepted': True}, status={'state': 'faulted', 'reason': 'feedback_invalid'})
    receiver.transport = SimpleNamespace(last=result, exchange=lambda request: result['reply'])
    receiver.exchange({'type': 'acquire'})
    result['status']['reason'] = 'requested_stop'
    receiver.exchange({'type': 'emergency_stop'})
    assert receiver.first_fault['request'] == {'type': 'acquire'}
    assert receiver.first_fault['result']['status']['reason'] == 'feedback_invalid'


def test_virtual_motor_process_is_spawned_and_child_errors_are_propagated(bench, monkeypatch):
    messages, calls = [{'kind': 'ready'}], []
    reports = SimpleNamespace(poll=lambda *args: bool(messages), recv=lambda: messages.pop(0),
                              close=lambda: calls.append('reports_closed'))
    sender = SimpleNamespace(close=lambda: calls.append('sender_closed'))
    stopped = [False]
    event = SimpleNamespace(set=lambda: stopped.__setitem__(0, True))
    process = SimpleNamespace(pid=123, exitcode=None)
    process.start = lambda: calls.append('started')
    def join(timeout):
        calls.append(('joined', timeout))
        if stopped[0]:
            process.exitcode = 0
    process.join, process.is_alive = join, lambda: process.exitcode is None
    def make_process(**kwargs):
        assert kwargs['target'] is bench.motor_process_main
        assert kwargs['daemon'] is True
        return process
    context = SimpleNamespace(Pipe=lambda **kwargs: (reports, sender), Event=lambda: event, Process=make_process)
    def get_context(method):
        assert method == 'spawn'
        return context
    monkeypatch.setattr(bench, 'get_context', get_context)
    emulator = bench.MotorProcess({}, [])
    assert emulator.ready and calls[:2] == ['started', 'sender_closed']
    messages.append({'kind': 'error', 'message': 'emulator socket failed'})
    with pytest.raises(RuntimeError, match='emulator socket failed'):
        emulator.check()
    with pytest.raises(RuntimeError, match='emulator socket failed'):
        emulator.close()
    assert stopped[0] and ('joined', 3) in calls and calls[-1] == 'reports_closed'
    emulator.close()  # Idempotent cleanup must not leave a child behind.


def test_final_target_is_observed_in_separate_drain_cycle_before_h(bench):
    now = [1.]
    core = SimpleNamespace(clock=lambda: now[0], sequence=30, driver_cycle=30, fault=None,
                           session=SimpleNamespace(lifecycle=SimpleNamespace(state='TELEOP')),
                           harness=SimpleNamespace(last={}))
    driver = bench.ScriptedCommands(core, SimpleNamespace(follow_started=0.), 20, 'H', 10)
    driver.phase, driver.followed = 'follow', 20
    driver.observe_cycle(dict(accepted=True, request_sequence=25, receiver_accepted_ms=100, receiver_epoch='boot'))
    assert not driver.poll() and driver.phase == 'drain'
    assert driver.followed == 20
    # This extra direct update observes the prior measured target, and does not
    # become part of the requested20-cycle timing/application denominator.
    driver.observe_cycle(dict(receiver_epoch='boot',
                              receiver_applied=dict(sequence=25, accepted_at_ms=100, applied_at_ms=103)))
    core.driver_cycle += 1
    now[0] += .01
    assert driver.poll() and driver.recv() == ('key', 'H')
    assert driver.phase == 'stopping' and driver.drain_cycles == 1 and driver.drain_observed
    assert driver.followed == 20


def test_drain_fails_if_later_replies_do_not_observe_the_exact_final_target(bench):
    now = [1.]
    core = SimpleNamespace(clock=lambda: now[0], sequence=30, driver_cycle=30, fault=None,
                           session=SimpleNamespace(lifecycle=SimpleNamespace(state='TELEOP')),
                           harness=SimpleNamespace(last={'status': {'state': 'executing'}}))
    driver = bench.ScriptedCommands(core, SimpleNamespace(follow_started=0.), 20, 'X', 10)
    driver.phase, driver.followed = 'follow', 20
    driver.observe_cycle(dict(accepted=True, request_sequence=25, receiver_accepted_ms=100, receiver_epoch='boot'))
    assert not driver.poll()
    for _ in range(5):
        driver.observe_cycle(dict(receiver_epoch='other-boot',
                                  receiver_applied=dict(sequence=25, accepted_at_ms=100, applied_at_ms=103)))
    now[0] += .05
    core.driver_cycle += 1
    assert driver.poll() and driver.recv() == ('key', 'H')
    assert driver.failure and 'not observed' in driver.failure
    assert driver.phase == 'exit' and not driver.drain_observed
