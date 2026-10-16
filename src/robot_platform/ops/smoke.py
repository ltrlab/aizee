"""Smoke-test an activated release on the robot: imports, calibration, collision, receiver.

Opens no device and touches no CAN interface. Prints one JSON object.
Usage (on the robot): python3 -m robot_platform.ops.smoke <release root>
"""
import json
from importlib import import_module
from pathlib import Path
import sys
import time


def check_requirements(manifest):
    """Check the verified release's declared modules before model/native startup."""
    names = manifest.get('requires', {}).get('modules', [])
    if not isinstance(names, list) or any(not isinstance(name, str) or not name
                                         for name in names):
        raise RuntimeError('Release requires.modules must be a list of module names')
    modules = {}
    for name in names:
        try:
            module = import_module(name)
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError(f'Release requires Python module {name}: {exc}', name=exc.name) from exc
        except Exception as exc:
            raise RuntimeError(f'Release requires Python module {name}: {exc}') from exc
        modules[name] = getattr(module, '__version__', 'available')
    return modules


def run(root):
    started = time.perf_counter()
    from .release import CALIBRATION, activate
    manifest = activate(root)
    modules = check_requirements(manifest)
    root = Path(root)
    import msgpack
    import numpy
    import scipy
    import zmq
    from robot_platform.control.joint_calibration import load_plan
    from robot_platform.simulation.control_harness import NativeControlHarness
    from robot_platform.simulation.native_collision import NativeCollisionBackend
    from robot_platform.simulation.runtime_model import RuntimeCollisionModel
    report = {'release': manifest['id'], 'commit': manifest['source']['commit'],
              'python': sys.version.split()[0], 'numpy': numpy.__version__, 'scipy': scipy.__version__,
              'msgpack': '.'.join(map(str, msgpack.version)), 'pyzmq': zmq.__version__,
              'required_modules': modules}
    _plan, _paths, _device, _reference, model, pose, _bindings = load_plan(root / CALIBRATION)
    runtime = RuntimeCollisionModel(model.profile)
    if not runtime.kinematics_only:
        raise RuntimeError('The collision model did not load from the verified release')
    report['load_ms'] = round((time.perf_counter() - started) * 1000)
    backend = NativeCollisionBackend(runtime, geometry_mode='keepout')
    try:
        state = {name: pose[name] for name in runtime.scene.independent_joints}
        backend.proximity(state, threshold=.05, scope='inter_branch')
        samples = []
        for _ in range(20):
            begin = time.perf_counter()
            backend.proximity(state, threshold=.05, scope='inter_branch')
            samples.append((time.perf_counter() - begin) * 1000)
        report['keepout_query_ms'] = {'p50': round(sorted(samples)[10], 2), 'max': round(max(samples), 2)}
    finally:
        backend.close()
    config = {'group': 'smoke', 'identity': {'model': 'smoke', 'calibration': 'smoke', 'collision_policy': 'smoke'},
              'lease_ms': 100, 'feedback_timeout_ms': 20, 'settled_velocity': .001,
              'joints': [{'name': 'axis', 'unit': 'radians', 'lower': -1., 'upper': 1., 'max_velocity': 1.,
                          'max_effort': 1., 'max_kp': 1., 'max_kd': 1., 'stop_acceleration': 1., 'hold_kp': 1.,
                          'hold_kd': 1., 'max_tracking_error': .1, 'max_temperature': 70., 'gripper': True}]}
    with NativeControlHarness(config, [0.]) as harness:
        harness.step(0)
        report['receiver_harness'] = harness.step(10, {'type': 'status'})['status']['state']
    report['ok'] = True
    return report


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        report = run(argv[0])
    except Exception as exc:
        report = {'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
    print(json.dumps(report))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
