"""Helm's and the viewer's browser scripts parse as the ES modules their pages load them as.

`node --check file.js` passed a helm.js the browser refused (an unescaped apostrophe in a string, 2026-10-15):
it parses the file as CommonJS first. Checked as a module, as the page loads it, the error shows.
"""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = sorted([*ROOT.glob('src/robot_platform/helm/static/*.js'), *ROOT.glob('src/robot_platform/viewer/static/*.js')])


def test_every_page_script_is_checked():
    assert {p.name for p in SCRIPTS} >= {'helm.js', 'settings.js', 'viewer.js'}


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda path: path.name)
def test_a_browser_script_parses_as_a_module(script):
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser scripts'
    result = subprocess.run([node, '--input-type=module', '--check'], input=script.read_bytes(),
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-800:]


def test_the_check_catches_a_quote_the_browser_rejects(tmp_path):
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser scripts'
    broken = b"export const note = 'about its command's speed';\n"
    result = subprocess.run([node, '--input-type=module', '--check'], input=broken, capture_output=True, timeout=60)
    assert result.returncode != 0


def test_task_viewer_unloads_duplicate_rendering_and_restores_after_stop():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser lifecycle'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_task_viewer.mjs')],
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_capture_failure_locks_following_but_keeps_disable_shutdown_and_stop():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser lifecycle'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_capture_failure.mjs')],
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_live_camera_packets_bound_headers_pixels_and_freshness_metadata():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the camera packet boundary'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_live_camera_packets.mjs')],
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_pedal_offers_and_stopped_input_test_keep_motion_and_freshness_guards():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the pedal input boundary'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_pedals.mjs')],
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_replay_plays_by_recorded_time_and_scrubs_within_the_episode():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the replay clock'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_replay.mjs')],
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_force_bars_run_green_to_red_and_say_when_the_triggers_push():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the force feedback view'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_grip.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_calibration_card_says_what_blocks_a_still_and_what_the_solve_found():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the calibration card'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_calibration.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_ups_pill_and_banner_say_what_the_battery_is_doing():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the UPS view'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_ups.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_policy_mode_choices_and_card_text():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the policy mode page'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_policy.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_the_head_card_says_what_the_head_follows_and_where_the_headset_page_is():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the head card'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_head.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headset_screens_say_what_the_head_does_and_its_camera_streams_the_newest_frame():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset view'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_view.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_network_pill_and_settings_say_how_minerva_is_connected():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the network view'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_network.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]
