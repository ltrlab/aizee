"""Minerva Check in Helm: run it in the background, show each result as it lands.

The check reads the motors itself (stopped-only) and may read the leaders, so
the server refuses to start it while the motor link or a teleop session runs,
and refuses those while it runs. The last saved report is shown at startup.
"""
from copy import deepcopy
import threading

from robot_platform.ops.check import FAIL, latest_report, outdated, record_live, run_check


class CheckWorkspace:
    def __init__(self, *, profile, teleop, runs, limits=None, runner=run_check, address=None):
        self.options = dict(profile=profile, teleop=teleop, runs=runs, limits=limits)
        self.runner = runner
        self.address = address  # () -> the robot address Helm is set to use; None: the runner's (the tether).
        self.lock = threading.Lock()
        self.report = latest_report(runs)
        self.error = None
        self._thread = None

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        with self.lock:
            if self.running:
                raise ValueError('Minerva Check is already running')
            self.report, self.error = None, None
            options = dict(self.options, **({'address': self.address()} if self.address else {}))
            self._thread = threading.Thread(target=self._run, args=(options,), daemon=True,
                                            name='helm-minerva-check')
            self._thread.start()

    def _publish(self, report):
        with self.lock:
            self.report = report

    def _run(self, options):
        try:
            self.runner(**options, publish=self._publish)
        except Exception as exc:  # The runner records failed checks itself; this is a crash in it.
            with self.lock:
                self.error = f'Minerva Check stopped: {exc}'

    def live_ready(self, arm):
        """Raise unless the latest report's stopped checks all passed: the live sweep follows them."""
        with self.lock:
            report = self.report
        if self.running:
            raise ValueError('Minerva Check is running; wait for it to finish')
        if report is None or report.get('state') != 'finished':
            raise ValueError('Run Minerva Check first: the live joint sweep follows the stopped checks')
        if outdated(report):
            raise ValueError('The last Minerva Check was saved before it had the live joint sweep: run '
                             'Minerva Check again first')
        failed = [c['title'] for stage in report['stages'] if stage['id'] != 'sweep' for c in stage['checks']
                  if c['status'] == FAIL]
        if failed:
            raise ValueError('The stopped checks did not pass (' + ', '.join(failed) + '): fix them and run '
                             'Minerva Check again before the live sweep')

    def record_live(self, arm, results, *, fault=None):
        with self.lock:
            if self.report is not None and self.report.get('state') == 'finished':
                self.report = record_live(self.report, arm, results, fault=fault)

    def snapshot(self):
        with self.lock:
            report, error = deepcopy(self.report), self.error
        for stage in (report or {}).get('stages', []):
            for entry in stage['checks']:
                entry.pop('data', None)  # Evidence only; the page shows the details.
        return {'running': self.running, 'error': error, 'report': report}
