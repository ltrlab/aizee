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
