"""Build the ARM64 release prerequisites locally on Windows/Linux x86_64.

Use --bootstrap once to install hash-locked build tools and the Rust target.
No robot connection, service start, or activation occurs here.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv

import dev

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'build/cross-tools/venv'
TOOL_PYTHON = TOOLS / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
TARGET = 'aarch64-unknown-linux-gnu'


def build_environment():
    env = dev.rust_environment()
    env['PATH'] = str(TOOL_PYTHON.parent) + os.pathsep + env['PATH']
    # A caller's shared caches/target settings must not share native build output.
    env['CARGO_TARGET_DIR'] = str(ROOT / 'rust/target')
    env['CARGO_ZIGBUILD_CACHE_DIR'] = str(ROOT / 'rust/target/zigbuild-cache')
    env['ZIG_GLOBAL_CACHE_DIR'] = str(ROOT / 'rust/target/zig-cache')
    env.pop('PYTHONPATH', None)
    env['PYTHONNOUSERSITE'] = '1'
    return env


def build(*, bootstrap=False, offline=False, jobs=2):
    if bootstrap and offline:
        raise ValueError('--bootstrap downloads prerequisites; it cannot be combined with --offline')
    run_dir = ROOT / 'build/cross-build'
    run_dir.mkdir(parents=True, exist_ok=True)
    report = {'schema_version': 1, 'passed': False, 'target': TARGET + '.2.35',
              'started_at': datetime.now(timezone.utc).isoformat(), 'jobs': jobs, 'commands': []}
    log_path = run_dir / 'output.log'
    log_path.write_text('', encoding='utf-8')

    def run(args, env):
        args = list(map(str, args))
        args[0] = shutil.which(args[0], path=env['PATH']) or args[0]
        print('+ ' + ' '.join(args), flush=True)
        with log_path.open('a', encoding='utf-8') as log:
            log.write('+ ' + ' '.join(args) + '\n')
            log.flush()
            result = subprocess.run(args, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        report['commands'].append({'args': args, 'returncode': result.returncode})
        if result.returncode:
            raise RuntimeError(f'Cross-build command failed ({result.returncode}); see {log_path}')

    def version(args, env, expected):
        args[0] = shutil.which(args[0], path=env['PATH']) or args[0]
        actual = subprocess.check_output(args, cwd=ROOT, env=env, text=True).strip()
        if actual != expected and not actual.startswith(expected + ' '):
            raise RuntimeError(f'Expected {expected}, found {actual}')
        return actual

    try:
        report['source_before'] = dev.source_identity()
        report['lock_sha256'] = dev.sha(ROOT / 'requirements/cross-build.lock')
        env = build_environment()
        report['rustc'] = version(['rustc', '--version'], env, 'rustc 1.98.1')
        if bootstrap:
            if not TOOL_PYTHON.is_file():
                venv.EnvBuilder(with_pip=True).create(TOOLS)
            run([TOOL_PYTHON, '-m', 'pip', 'install', '--require-hashes', '--only-binary=:all:',
                 '-r', ROOT / 'requirements/cross-build.lock'], env)
            run(['rustup', 'target', 'add', TARGET], env)
        if not TOOL_PYTHON.is_file():
            raise RuntimeError('Run python scripts/build_release.py --bootstrap first')
        zig = subprocess.check_output(
            [str(TOOL_PYTHON), '-c', 'from pathlib import Path; import ziglang, os; '
             'print(Path(ziglang.__file__).parent / ("zig.exe" if os.name == "nt" else "zig"))'],
            cwd=ROOT, env=env, text=True).strip()
        env['CARGO_ZIGBUILD_ZIG_PATH'] = zig
        report['zig'] = version([zig, 'version'], env, '0.14.1')
        zigbuild = shutil.which('cargo-zigbuild', path=env['PATH'])
        if not zigbuild:
            raise RuntimeError('Missing cargo-zigbuild; rerun with --bootstrap')
        report['cargo_zigbuild'] = version([zigbuild, '--version'], env, 'cargo-zigbuild 0.23.3')
        flags = ['--offline'] if offline else []
        run(['cargo', 'zigbuild', '--release', '--locked', '--jobs', str(jobs), '--target', TARGET + '.2.35',
             '-p', 'robot_control', '-p', 'robot_collision', '--features',
             'robot_control/linux-service', *flags], env)
        # Same mandatory release checks as the development gate; never execute ARM64 here.
        report['binaries'] = dev.release_artifacts(env)
        report['source_after'] = dev.source_identity()
        if report['source_before'] != report['source_after']:
            raise RuntimeError('Source changed during cross-build; rerun on frozen source')
        report['passed'] = True
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        report['error'] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        path = run_dir / 'report.json'
        path.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print(f'Report: {path}', flush=True)
    return 0 if report['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bootstrap', action='store_true')
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--jobs', type=int, default=2, help='Bound compiler concurrency (default: 2)')
    args = parser.parse_args()
    if args.bootstrap and args.offline:
        parser.error('--bootstrap and --offline are mutually exclusive')
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    return build(bootstrap=args.bootstrap, offline=args.offline, jobs=args.jobs)


if __name__ == '__main__':
    raise SystemExit(main())
