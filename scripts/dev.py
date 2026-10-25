"""Reproducible, hardware-free development gate. Run from any directory.

`check` is the gate. `quick` is the prototype lane: only the gate's tests that touch what changed since this
checkout's last passing check, in seconds rather than minutes. It refuses a motion-contract change, and it does not
stand in for `check` before a branch is handed off.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import uuid
import venv
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / '.venv-dev' / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
CORE = [
    'tests/unit/test_approach.py', 'tests/unit/test_joint_box_bound.py',
    'tests/unit/test_runtime_collision.py', 'tests/contract/test_guarded_controller.py',
    'tests/contract/test_collision_envelope.py', 'tests/contract/test_motion_envelope.py',
    'tests/contract/test_guarded_lifecycle_session.py', 'tests/contract/test_rehearsal.py',
    'tests/unit/test_mujoco_plant.py', 'tests/unit/test_physics_audit.py',
    'tests/unit/test_response_fit.py', 'tests/unit/test_sim2real.py',
    'tests/unit/test_mjcf_export.py',
    'tests/contract/test_mujoco_receiver.py',
    'tests/contract/test_helm_sim.py', 'tests/contract/test_helm_capture_failure.py',
    'tests/contract/test_two_arm_teleop.py',
    'tests/contract/test_direct_teleop.py',
    'tests/contract/test_manual_teleop.py', 'tests/unit/test_manual_tuning.py',
    'tests/unit/test_manual_teleop_lifecycle.py', 'tests/unit/test_manual_tuning_api.py',
    'tests/unit/test_manual_tuning_rejections.py',
    'tests/unit/test_teleop_ui_budget.py', 'tests/unit/test_helm_recording.py',
    'tests/unit/test_teleop_recording.py', 'tests/unit/test_helm_pedals.py',
    'tests/contract/test_helm_pedals.py',
    'tests/unit/test_direct_teleop.py', 'tests/unit/test_direct_latency.py', 'tests/unit/test_leader_bridge.py',
    'tests/unit/test_qualify_direct_motion.py', 'tests/unit/test_position_only_trial.py',
    'tests/unit/test_teleop_lifecycle.py', 'tests/unit/test_guarded_config.py',
    'tests/unit/test_helm_teleop.py', 'tests/unit/test_live_receiver.py',
    'tests/contract/test_direct_benchmark.py', 'tests/unit/test_benchmark_direct_loop.py',
    'tests/unit/test_direct_readiness.py', 'tests/unit/test_ssh_worktree.py',
    'tests/unit/test_minerva_check.py', 'tests/unit/test_helm_check.py',
    'tests/unit/test_ops_release.py', 'tests/unit/test_joint_calibration.py',
    'tests/contract/test_helm_settings.py', 'tests/unit/test_helm_settings_api.py',
    'tests/unit/test_helm_telemetry.py', 'tests/unit/test_helm_app.py', 'tests/unit/test_helm_live_view.py', 'tests/unit/test_qualify_manual_tuning.py', 'tests/unit/test_plain_data.py',
    'tests/unit/test_joint_capture_preflight.py', 'tests/unit/test_leader_link.py', 'tests/unit/test_helm_edge.py',
    'tests/unit/test_cpu_partition.py', 'tests/unit/test_cpu_partition_install.py',
    'tests/unit/test_openrb_leader_setup.py', 'tests/unit/test_gripper_zero.py',
    'tests/unit/test_velocity_feedforward.py', 'tests/unit/test_browser_scripts.py',
    'tests/unit/test_helm_task_workflow.py', 'tests/unit/test_helm_task_capture.py',
    'tests/unit/test_helm_episodes.py', 'tests/unit/test_grip_feedback.py', 'tests/unit/test_ups.py',
    'tests/unit/test_helm_ups.py', 'tests/unit/test_lerobot_export.py',
    'tests/unit/test_episode_source.py', 'tests/unit/test_helm_live_replay.py', 'tests/contract/test_live_replay.py',
    'tests/unit/test_policy_source.py', 'tests/unit/test_policy_worker.py', 'tests/contract/test_policy_mode.py',
    'tests/unit/test_lerobot_policy.py', 'tests/unit/test_helm_policy_mode.py',
    'tests/unit/test_gello_bridge.py', 'tests/unit/test_helm_on_robot.py', 'tests/unit/test_helm_service.py',
    'tests/unit/test_task_capture.py', 'tests/unit/test_task_scene.py',
    'tests/unit/test_capture_process.py', 'tests/contract/test_task_receiver_capture.py',
    'tests/unit/test_eval_stats.py', 'tests/unit/test_eval_report.py',
    'tests/contract/test_eval_reference_policies.py',
    'tests/unit/test_ci_windows_gl.py', 'tests/unit/test_dev_gate.py', 'tests/unit/test_gate_time_study.py',
    'tests/unit/test_platform_packaging.py', 'tests/unit/test_neck_feetech.py', 'tests/unit/test_neck_head.py',
    'tests/contract/test_helm_tune_head.py', 'tests/contract/test_head_teleop.py',
    'tests/contract/test_teleop_head.py',
]
# The motion contract (AGENTS.md: approach, collision envelope, guarded lifecycle and receiver checks, the receiver
# and collision crates) and the locks: a change there runs the whole check, never the quick lane.
MOTION_CONTRACT = ('rust/', 'requirements/', 'pyproject.toml', 'src/robot_platform/control/approach.py',
                   'src/robot_platform/control/collision_envelope.py', 'src/robot_platform/control/collision_guard.py',
                   'src/robot_platform/control/guarded', 'src/robot_platform/control/live_receiver.py',
                   'src/robot_platform/control/freshness.py', 'src/robot_platform/teleop/guarded_lifecycle.py',
                   'src/robot_platform/teleop/lifecycle.py', 'src/robot_platform/simulation/native_collision.py')
PAGES = ('src/robot_platform/helm/static/', 'src/robot_platform/viewer/static/', 'tests/ui/')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture(*args):
    return subprocess.check_output(args, cwd=ROOT, text=True, encoding='utf-8').strip()


def source_identity():
    # Includes tracked and untracked source, excluding ignored build outputs.
    paths = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others',
                                     '--exclude-standard'], cwd=ROOT).decode().split('\0')
    digest = hashlib.sha256()
    for name in sorted(set(filter(None, paths))):
        path = ROOT / name
        digest.update(name.encode())
        digest.update(bytes.fromhex(sha(path)) if path.is_file() else b'<deleted>')
    return {'commit': capture('git', 'rev-parse', 'HEAD'),
            'branch': capture('git', 'branch', '--show-current'),
            'status': capture('git', 'status', '--porcelain'),
            'content_sha256': digest.hexdigest()}


def rust_environment():
    env = os.environ.copy()
    tool_root = Path(env.get('ROBOT_RUST_TOOLCHAIN_ROOT', ROOT / 'rust/target/jetson-toolchain'))
    if tool_root.is_dir():
        env['CARGO_HOME'] = str(tool_root / 'cargo-home')
        env['RUSTUP_HOME'] = str(tool_root / 'rustup-home')
        env['PATH'] = str(tool_root / 'cargo-home/bin') + os.pathsep + env['PATH']
    if not shutil.which('cargo', path=env['PATH']):
        raise RuntimeError('Install Rust 1.98.1, or set ROBOT_RUST_TOOLCHAIN_ROOT to an existing toolchain root')
    env['CARGO_TARGET_DIR'] = str(ROOT / 'rust/target')
    env.setdefault('CARGO_BUILD_JOBS', '2')
    env['ROBOT_CONTROL_HARNESS'] = str(ROOT / 'rust/target/debug' / (
        'control_harness.exe' if os.name == 'nt' else 'control_harness'))
    lib = 'robot_collision.dll' if os.name == 'nt' else (
        'librobot_collision.dylib' if sys.platform == 'darwin' else 'librobot_collision.so')
    env['ROBOT_COLLISION_LIBRARY'] = str(ROOT / 'rust/target/release' / lib)
    return env


def setup():
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError('The development lock targets CPython 3.12. Run setup with Python 3.12.')
    subprocess.run(['git', 'lfs', 'pull'], cwd=ROOT, check=True)
    if not PYTHON.exists():
        venv.EnvBuilder(with_pip=True).create(PYTHON.parent.parent)
    if not capture(str(PYTHON), '--version').startswith('Python 3.12.'):
        raise RuntimeError('Existing .venv-dev uses another Python version; recreate it with Python 3.12')
    subprocess.run([str(PYTHON), '-m', 'pip', 'install', '--require-hashes',
                    '-r', str(ROOT / 'requirements/dev.lock')], cwd=ROOT, check=True)
    subprocess.run([str(PYTHON), '-m', 'pip', 'install', '--no-deps', '--no-build-isolation',
                    '-e', '.'], cwd=ROOT, check=True)


def release_artifacts(env):
    """Fail early on absent/stale/wrong-architecture mandatory release inputs."""
    result = subprocess.run([str(PYTHON), '-c',
        'import hashlib, json; from robot_platform.ops.release import collect, check_binaries; '
        'files = collect(); check_binaries(files); '
        'print(json.dumps({key: {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} '
        'for key, path in files.items() if key.startswith(("bin/", "lib/"))}))'],
        cwd=ROOT, env=env, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('Release prerequisites failed. Run python scripts/build_release.py --bootstrap '
                           '(or the existing provisioned Windows cross-build).\n' + result.stderr.strip())
    return json.loads(result.stdout)


def required_test_results(path, report=None):
    cases = ET.parse(path).findall('.//testcase')
    skipped = [case.get('name') for case in cases if case.find('skipped') is not None]
    results = {'collected': len(cases), 'skipped': skipped}
    if report is not None:
        report['tests'] = results
    if not cases or skipped:
        raise RuntimeError(f'Required gate must execute every test; skipped: {skipped}')
    return results


def new_run(folder, gate):
    """A run's directory, its report and the command runner that logs into it."""
    run_dir = ROOT / 'build' / folder / (
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8])
    run_dir.mkdir(parents=True)
    report = {'schema_version': 1, 'gate': gate, 'passed': False, 'commands': []}

    def run(args, env):
        args = list(map(str, args))
        args[0] = shutil.which(args[0], path=env['PATH']) or args[0]
        print('+ ' + ' '.join(args), flush=True)
        with (run_dir / 'output.log').open('a', encoding='utf-8') as log:
            log.write('+ ' + ' '.join(args) + '\n')
            log.flush()
            result = subprocess.run(args, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        report['commands'].append({'args': args, 'returncode': result.returncode})
        if result.returncode:
            raise RuntimeError(f'Command failed ({result.returncode}); see {run_dir / "output.log"}')
    return run_dir, report, run


def gate_environment(run_dir):
    env = rust_environment()
    temp = run_dir / 'temp'
    temp.mkdir()
    env.update(TEMP=str(temp), TMP=str(temp), TMPDIR=str(temp), PYTHONNOUSERSITE='1')
    # Remove external Python path overrides; an editable install must belong here.
    env.pop('PYTHONPATH', None)
    return env


def check(offline=False):
    run_dir, report, run = new_run('dev-check', 'offline-control')
    try:
        report['source_before'] = source_identity()
        report['lock_sha256'] = sha(ROOT / 'requirements/dev.lock')
        if not PYTHON.is_file():
            raise RuntimeError('Run python scripts/dev.py setup first')
        env = gate_environment(run_dir)
        run([PYTHON, '-c', 'from pathlib import Path; import robot_platform; '
             'assert Path(robot_platform.__file__).resolve().parent == '
             'Path("src/robot_platform").resolve(), "Wrong editable install"'], env)
        run([PYTHON, '-c', 'from robot_platform.models import UrdfModel, ModelProfile; '
             'UrdfModel(ModelProfile.load("urdf/minerva/model.json"))'], env)
        report['python'] = capture(str(PYTHON), '--version')
        if not report['python'].startswith('Python 3.12.'):
            raise RuntimeError('The development gate requires Python 3.12')
        report['packages'] = json.loads(capture(str(PYTHON), '-m', 'pip', 'list', '--format=json'))
        report['release_binaries'] = release_artifacts(env)
        rustc = shutil.which('rustc', path=env['PATH'])
        if not rustc:
            raise RuntimeError('Rust compiler missing from the selected toolchain')
        report['rustc'] = subprocess.check_output([rustc, '--version'], env=env, text=True).strip()
        if not report['rustc'].startswith('rustc 1.98.1 '):
            raise RuntimeError('Use the baseline Rust compiler 1.98.1; install with rustup and select it')
        flags = ['--locked'] + (['--offline'] if offline else [])
        run(['cargo', 'test', '-p', 'robot_control', '-p', 'robot_collision', *flags], env)
        run(['cargo', 'build', '-p', 'robot_control', '--bin', 'control_harness', *flags], env)
        run(['cargo', 'build', '-p', 'robot_collision', '--release', *flags], env)
        report['binaries'] = {key: {'path': env[key], 'sha256': sha(env[key])}
                              for key in ('ROBOT_CONTROL_HARNESS', 'ROBOT_COLLISION_LIBRARY')}
        # A broken ABI must fail before tests can skip missing native prerequisites.
        run([PYTHON, '-c', 'import ctypes, os; '
             'lib=ctypes.CDLL(os.environ["ROBOT_COLLISION_LIBRARY"]); '
             'assert lib.rpc_abi_version() == 3; import fcl, numpy, scipy, pytest, mujoco, hypothesis'], env)
        run([PYTHON, '-m', 'pytest', '-q', '--strict-markers', '-o', 'xfail_strict=true',
             '--basetemp', run_dir / 'pytest', '--junitxml', run_dir / 'tests.xml', *CORE], env)
        required_test_results(run_dir / 'tests.xml', report)
        report['source_after'] = source_identity()
        if report['source_before'] != report['source_after']:
            raise RuntimeError('Source changed during verification; rerun in an isolated worktree')
        report['passed'] = True
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        (run_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'Report: {run_dir / "report.json"}', flush=True)
    return 0 if report['passed'] else 1


def last_passed_commit():
    """The source commit of this checkout's latest passing check, or None."""
    for path in sorted((ROOT / 'build/dev-check').glob('*/report.json'), reverse=True):
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            continue
        if report.get('gate') == 'offline-control' and report.get('passed'):
            return report['source_before']['commit']
    return None


def changed_files(base):
    """Every path that differs from `base`: committed since, staged, unstaged and untracked."""
    tracked = capture('git', 'diff', '--name-only', base).splitlines()
    untracked = capture('git', 'ls-files', '--others', '--exclude-standard').splitlines()
    return sorted(set(filter(None, tracked + untracked)))


def module_name(path):
    """src/robot_platform/a/b.py -> robot_platform.a.b; a package's __init__.py -> the package."""
    parts = Path(path).with_suffix('').parts[1:]
    return '.'.join(parts[:-1] if parts[-1] == '__init__' else parts)


def select_tests(changed, named=()):
    """What the quick lane runs: the named test files, or else the gate's tests that name a changed module (or a
    changed config or script file); with either, every changed test and, for a changed page file, the page checks."""
    selected = set(named) | {path for path in changed if path.startswith('tests/') and path.endswith('.py')
                             and Path(path).name.startswith('test_') and (ROOT / path).is_file()}
    if not named:
        modules = {module_name(path) for path in changed
                   if path.startswith('src/robot_platform/') and path.endswith('.py')}
        patterns = [re.compile(r'\b' + re.escape(name) + r'\b') for name in modules]
        patterns += [re.compile(r'from ' + re.escape(parent) + r' import [^\n]*\b' + re.escape(leaf) + r'\b')
                     for parent, _, leaf in (name.rpartition('.') for name in modules)]
        files = [Path(path).name for path in changed if path.startswith(('config/', 'scripts/'))]
        for test in CORE:
            text = (ROOT / test).read_text(encoding='utf-8')
            if any(pattern.search(text) for pattern in patterns) or any(name in text for name in files):
                selected.add(test)
    if any(path.startswith(PAGES) for path in changed):
        selected.add('tests/unit/test_browser_scripts.py')
    return sorted(selected)


def quick(base=None, named=()):
    """The prototype lane: select_tests for what changed since `base` (default: the commit of this checkout's last
    passing check), run with the gate's rules (zero skips, no source edits during the run)."""
    run_dir, report, run = new_run('dev-quick', 'quick')
    try:
        report['source_before'] = source_identity()
        if not PYTHON.is_file():
            raise RuntimeError('Run python scripts/dev.py setup first')
        base = base or last_passed_commit()
        if base is None:
            raise RuntimeError('No passing check in this checkout to compare with: pass --base <commit>')
        if subprocess.run(['git', 'merge-base', '--is-ancestor', base, 'HEAD'], cwd=ROOT).returncode:
            raise RuntimeError(f'{base} is not in this branch: pass --base <commit>')
        changed = changed_files(base)
        report.update(base=base, changed=changed)
        contract = [path for path in changed if path.startswith(MOTION_CONTRACT)]
        if contract:
            raise RuntimeError('The motion contract or a lock changed; run python scripts/dev.py check:\n  '
                               + '\n  '.join(contract))
        tests = select_tests(changed, named)
        report['selected'] = tests
        print(f'{len(changed)} file(s) changed since {base[:10]}; {len(tests)} test file(s) touch them', flush=True)
        if tests:
            env = gate_environment(run_dir)
            run([PYTHON, '-m', 'pytest', '-q', '--strict-markers', '-o', 'xfail_strict=true',
                 '--basetemp', run_dir / 'pytest', '--junitxml', run_dir / 'tests.xml', *tests], env)
            required_test_results(run_dir / 'tests.xml', report)
        else:
            print('No test of the gate touches this change.', flush=True)
        report['source_after'] = source_identity()
        if report['source_before'] != report['source_after']:
            raise RuntimeError('Source changed during verification; rerun in an isolated worktree')
        report['passed'] = True
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        (run_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'Report: {run_dir / "report.json"}', flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['setup', 'check', 'quick'])
    parser.add_argument('--offline', action='store_true', help='Use cached Cargo dependencies only')
    parser.add_argument('--base', help='quick: compare with this commit instead of the last passing check')
    parser.add_argument('tests', nargs='*', help='quick: run these test files instead of the ones it finds')
    args = parser.parse_args()
    if args.command == 'setup':
        setup()
        return 0
    if args.command == 'quick':
        return quick(args.base, args.tests)
    return check(args.offline)


if __name__ == '__main__':
    raise SystemExit(main())
