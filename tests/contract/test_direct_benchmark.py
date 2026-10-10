"""Exercise benchmark fault assertions on scripted native ticks, not Linux CAN."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import robot_platform

from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.simulation.control_harness import NativeControlHarness


@pytest.fixture
def benchmark(monkeypatch):
    project = Path(robot_platform.__file__).resolve().parents[2]
    monkeypatch.syspath_prepend(str(project/'scripts'))
    path = Path(__file__).resolve().parents[2]/'scripts/benchmark_direct_teleop.py'
    spec = importlib.util.spec_from_file_location('direct_benchmark_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def receiver(benchmark):
    config = benchmark.virtual_config()['controller']
    config.update(lease_ms=150, feedback_timeout_ms=30, interpolation_ms=0,
                  direct=dict(command_ms=30, velocity_timeout_ms=15, limit_reaction_ms=10))
    config['identity']['collision_policy'] = 'disabled-v1'
    emulator = SimpleNamespace(freeze=False, state={j['name']: {'enabled': False} for j in config['joints']})
    clock = dict(now=0, ticks=0)
    with NativeControlHarness(config, [0.]*len(config['joints'])) as harness:
        def step(request=None):
            clock['now'] += 2
            clock['ticks'] += 1
            result = harness.step(clock['now'], request, faults={'frozen': emulator.freeze})
            # Fixture-only facade: production qualification gets these from the Linux service.
            result.update(mode='virtual_can', timing={'ticks': clock['ticks']})
            for motor in emulator.state.values():
                motor['enabled'] = result['hardware_enabled']
            return result

        def wait(seconds):
            until = clock['now']+math.ceil(seconds*1000)
            while clock['now'] < until:
                step()

        step()
        controller = GuardedControllerClient(lambda request: step(request)['reply'],
            group=config['group'], identity=config['identity'],
            joint_names=tuple(j['name'] for j in config['joints']))
        yield controller, config, lambda: harness.last, emulator, wait, lambda: clock['now']/1000


def test_vcan_fault_suite_assertions_against_native_receiver(benchmark, receiver):
    controller, config, snapshot, emulator, wait, clock = receiver
    checks = {}
    benchmark.qualify_vcan_faults(controller, config, snapshot, emulator, checks, wait=wait, clock=clock)
    assert set(checks) == {'command_replay', 'consumed_offer', 'expired_source_validity',
                           'source_interval_independent', 'requested_stop', 'stale_feedback', 'emergency_stop'}
    assert all(case['passed'] for case in checks.values())
    assert checks['command_replay']['authority_and_target_unchanged']
    assert checks['expired_source_validity']['rejection'] == 'offer_expired'
    assert checks['stale_feedback']['after']['hardware_enabled'] is False
    assert checks['requested_stop']['after']['hardware_enabled'] is True
    assert checks['stale_feedback']['quiet_requests'] == 0
    assert checks['stale_feedback']['receiver_tick_delta'] > 1


def test_fault_injection_refuses_a_physical_receiver(benchmark, receiver):
    controller, config, snapshot, emulator, wait, clock = receiver
    snapshot()['mode'] = 'teleop_direct'
    with pytest.raises(ValueError, match='virtual CAN'):
        benchmark.qualify_vcan_faults(controller, config, snapshot, emulator, {}, wait=wait, clock=clock)
    assert controller.exchanges == 0


def manifest_tree(tmp_path, benchmark):
    paths = [*benchmark.MANIFEST_REQUIRED, 'config/devices/fixture.json']
    files = {}
    digest = hashlib.sha256()
    for name in sorted(paths):
        path = tmp_path/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(name, encoding='utf-8')
        files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        digest.update(name.encode())
        digest.update(bytes.fromhex(files[name]))
    document = dict(source_identity=dict(commit='a'*40, branch='fixture', status='',
                    scope='verified_candidate_bundle', content_sha256=digest.hexdigest()), files=files)
    path = tmp_path/'source-manifest.json'
    path.write_text(json.dumps(document), encoding='utf-8')
    return path, document


def test_release_source_identity_is_verified_without_git(tmp_path, benchmark):
    path, document = manifest_tree(tmp_path, benchmark)
    identity = benchmark.verified_source(path, root=tmp_path)
    assert identity['content_sha256'] == document['source_identity']['content_sha256']
    assert identity['scope'] == 'verified_candidate_bundle'
    assert identity['manifest_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize('change', ['hash', 'unlisted_config', 'unlisted_python', 'escape', 'digest', 'scope',
                                    'required_deleted'])
def test_manifest_rejects_changed_omitted_or_escaping_source(tmp_path, benchmark, change):
    path, document = manifest_tree(tmp_path, benchmark)
    if change == 'hash':
        (tmp_path/benchmark.MANIFEST_REQUIRED[0]).write_text('changed', encoding='utf-8')
    elif change == 'unlisted_config':
        (tmp_path/'config/unlisted.json').write_text('{}', encoding='utf-8')
    elif change == 'unlisted_python':
        (tmp_path/'src/unlisted.py').write_text('changed', encoding='utf-8')
    elif change == 'escape':
        document['files']['../escape'] = '0'*64
    elif change == 'digest':
        document['source_identity']['content_sha256'] = '0'*64
    elif change == 'required_deleted':
        document['files'][benchmark.MANIFEST_REQUIRED[0]] = None
    else:
        document['source_identity']['scope'] = 'unverified'
    path.write_text(json.dumps(document), encoding='utf-8')
    with pytest.raises(ValueError):
        benchmark.verified_source(path, root=tmp_path)


def test_local_source_identity_retains_git_behavior(monkeypatch, benchmark):
    identity = dict(commit='local', content_sha256='fixture')
    monkeypatch.setattr(benchmark, 'source_identity', lambda: identity)
    assert benchmark.verified_source() == identity


@pytest.mark.parametrize('module,relative', [
    ('robot_platform.outside_fixture', 'build/outside.py'),
    ('robot_platform.outside_fixture', 'scripts/outside.py'),
    ('guarded_vcan_bench', 'src/guarded_vcan_bench.py'),
    ('benchmark_direct_loop', 'build/benchmark_direct_loop.py'),
    ('__main__', 'build/benchmark_direct_teleop.py'),
])
def test_candidate_cannot_import_another_checkout(monkeypatch, benchmark, module, relative):
    project = Path(robot_platform.__file__).resolve().parents[2]
    benchmark.verify_runtime_imports(project)
    # Deliberately remain inside the checkout: project/build is not verified
    # runtime source, even when a gate puts its pytest basetemp there.
    monkeypatch.setitem(sys.modules, module, SimpleNamespace(__file__=str(project/relative)))
    with pytest.raises(ValueError, match='outside the verified candidate'):
        benchmark.verify_runtime_imports(project)


def test_candidate_rejects_package_module_without_source(monkeypatch, benchmark):
    project = Path(robot_platform.__file__).resolve().parents[2]
    monkeypatch.setitem(sys.modules, 'robot_platform.outside_fixture', SimpleNamespace())
    with pytest.raises(ValueError, match='outside the verified candidate'):
        benchmark.verify_runtime_imports(project)
