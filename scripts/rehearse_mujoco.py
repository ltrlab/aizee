"""Record a synthetic MuJoCo fixture driven by the actual offline Rust receiver."""
import argparse
import csv
import json
from pathlib import Path

from robot_platform.control.guarded import GuardedControllerClient
from robot_platform.simulation.arm_plant import PlantReceiver
from robot_platform.simulation.mujoco_plant import JointBinding, MujocoPlant

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cfg = {'group': 'synthetic',
           'identity': {'model': 'synthetic-slider', 'calibration': 'synthetic', 'collision_policy': 'fixture'},
           'lease_ms': 100, 'feedback_timeout_ms': 20, 'settled_velocity': .001,
           'joints': [{'name': 'motor', 'unit': 'radians', 'lower': -3., 'upper': 3.,
                       'max_velocity': 2., 'max_effort': 5., 'max_kp': 20., 'max_kd': 5.,
                       'stop_acceleration': 2., 'hold_kp': 10., 'hold_kd': 1.,
                       'max_tracking_error': .5, 'max_temperature': 70.}]}
    plant = MujocoPlant(ROOT / 'tests/fixtures/physics/slider.xml', [JointBinding('motor', 'slide', 5.)],
                       position_step=0., seed=1)
    now = [100.]
    receiver = PlantReceiver(cfg, plant, clock=lambda: now[0], directory=args.output / 'receiver')
    client = GuardedControllerClient(receiver.exchange, group=cfg['group'], identity=cfg['identity'],
                                     joint_names=('motor',))
    report = {'fixture': 'synthetic', 'model': plant.metadata(), 'completed': False, 'config': cfg}
    try:
        client.acquire()
        client.arm()
        with (args.output / 'trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'position_rad', 'velocity_rad_s', 'effort_nm', 'state', 'reason'])
            for index in range(100):
                client.renew()  # Ownership and target freshness are independent receiver contracts.
                client.move([{'position': .05, 'velocity': 0., 'effort': 0., 'kp': 10., 'kd': 1.}])
                now[0] = 100. + (index + 1) * .02
                receiver.advance()
                status = receiver.last['status']
                writer.writerow([plant.t, plant.q[0], plant.v[0], plant.torque[0], status['state'], status['reason']])
                if status['state'] not in {'executing', 'holding'}:
                    raise RuntimeError(f'Receiver stopped: {status["reason"]}')
        client.stop()
        # The receiver waits for measured settling, not just the planned stop duration.
        for _ in range(2000):
            now[0] += .005
            receiver.advance()
            if receiver.last['status']['state'] in {'stopped', 'faulted'}:
                break
        report['final_state'] = receiver.last['status']['state']
        report['final_reason'] = receiver.last['status']['reason']
        if report['final_state'] != 'stopped':
            raise RuntimeError('Synthetic receiver did not complete its stop')
        report['completed'] = True
    finally:
        receiver.close()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(args.output / 'report.json')


if __name__ == '__main__':
    main()
