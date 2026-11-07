"""Helm's and the viewer's browser scripts parse as the ES modules their pages load them as.

`node --check file.js` passed a helm.js the browser refused (an unescaped apostrophe in a string, 2026-10-15):
it parses the file as CommonJS first. Checked as a module, as the page loads it, the error shows.
"""

from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = sorted(
    [*ROOT.glob('src/robot_platform/helm/static/*.js'), *ROOT.glob('src/robot_platform/viewer/static/*.js')]
)


def test_every_page_script_is_checked():
    assert {p.name for p in SCRIPTS} >= {'helm.js', 'settings.js', 'viewer.js'}


@pytest.mark.parametrize('script', SCRIPTS, ids=lambda path: path.name)
def test_a_browser_script_parses_as_a_module(script):
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser scripts'
    result = subprocess.run(
        [node, '--input-type=module', '--check'], input=script.read_bytes(), capture_output=True, timeout=60
    )
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
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_task_viewer.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_capture_failure_locks_following_but_keeps_disable_shutdown_and_stop():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the browser lifecycle'
    result = subprocess.run(
        [node, str(ROOT / 'tests/ui/test_helm_capture_failure.mjs')], capture_output=True, timeout=60
    )
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_live_camera_packets_bound_headers_pixels_and_freshness_metadata():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the camera packet boundary'
    result = subprocess.run(
        [node, str(ROOT / 'tests/ui/test_helm_live_camera_packets.mjs')], capture_output=True, timeout=60
    )
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-1200:]


def test_pedal_offers_and_stopped_input_test_keep_motion_and_freshness_guards():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the pedal input boundary'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_pedals.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_replay_plays_by_recorded_time_and_scrubs_within_the_episode():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the replay clock'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_replay.mjs')], capture_output=True, timeout=60)
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


def test_the_calibration_guide_says_the_next_step_and_what_a_solve_means():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the calibration guide'
    result = subprocess.run(
        [node, str(ROOT / 'tests/ui/test_helm_calibration_guide.mjs')], capture_output=True, timeout=60
    )
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


def test_teleop_modes_choose_the_cards_and_a_calibrate_pause_takes_one_still():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the teleop modes'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_teleop_mode.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


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


def test_the_headset_picture_follows_the_head_or_stays_and_the_ring_and_alert_sit_in_view():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset scene'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_follow.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headsets_ring_starts_steps_and_stops_a_session_with_the_controllers_buttons():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset ring'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_ring.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headset_shows_the_head_cameras_depth_in_3d_from_the_head_depth_stream():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset 3D picture'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_depth.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headset_fills_the_depths_holes_and_a_background_layer_behind_near_objects():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset depth fill'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_fill.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headsets_3d_stays_where_the_head_camera_saw_it_in_the_room():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset world-locked 3D'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_world.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headset_shows_minervas_model_and_the_wrist_cameras_on_its_wrists():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset model and wrist cameras'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_model.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_settings_shows_what_the_headset_shows_and_streams_only_while_on_screen():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset section'
    result = subprocess.run(
        [node, str(ROOT / 'tests/ui/test_helm_headset_mirror.mjs')], capture_output=True, timeout=60
    )
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_retro_look_outlines_the_model_and_redraws_it_only_when_it_changes():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the 3D view outline'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_viewer_outline.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_head_setup_section_follows_its_reader_reference_and_sweep():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the head setup section'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_head_setup.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_network_pill_and_settings_say_how_minerva_is_connected():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the network view'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_network.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_headset_draws_minervas_splat_map_placed_in_its_world():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the headset splat map'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_xr_splat.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_map_card_captures_with_the_live_head_and_lists_maps():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the map card'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_maps.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_twin_card_says_what_the_3d_views_draw_and_when_to_apply():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the twin card'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_twin.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]


def test_the_joint_zero_card_says_what_waits_for_the_power_cycle_and_what_to_sweep_again():
    node = shutil.which('node')
    assert node, 'Node.js is required to check the joint zero card'
    result = subprocess.run([node, str(ROOT / 'tests/ui/test_helm_joint_zero.mjs')], capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr.decode(errors='replace')[-2000:]
