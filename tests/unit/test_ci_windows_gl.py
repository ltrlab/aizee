"""Checksum failure is fatal; the generated hook cannot affect ordinary Python."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

_spec = importlib.util.spec_from_file_location("ci_windows_gl", Path(__file__).resolve().parents[2] / "scripts/ci_windows_gl.py")
ci = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ci)


def test_corrupt_download_is_rejected_before_it_can_be_extracted(tmp_path):
    archive = tmp_path / "mesa.7z"
    archive.write_bytes(b"not the approved graphics dependency")
    with pytest.raises(RuntimeError, match="SHA256 mismatch"):
        ci.verify(archive, ci.ARCHIVE_SHA256)


@pytest.mark.parametrize("corrupt", [False, True])
def test_requested_broken_graphics_fails_python_startup_instead_of_falling_back(tmp_path, corrupt):
    site = tmp_path / "site"
    site.mkdir()
    (site / "aizee_ci_mesa.py").write_text(ci.bootstrap_source())
    (site / "aizee_ci_mesa.pth").write_text("import aizee_ci_mesa\n")
    code = "import os,site,sys; os.environ['ROBOT_CI_MESA_DIR']=sys.argv[2]; site.addsitedir(sys.argv[1]); print('UNSAFE_FALLBACK')"
    directory = tmp_path / "dependency"
    if corrupt:
        directory.mkdir()
        (directory / "opengl32.dll").write_bytes(b"changed DLL")
    result = subprocess.run([sys.executable, "-S", "-c", code, str(site), str(directory)],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert "Required CI Mesa startup failed" in result.stderr
    if corrupt:
        assert "checksum mismatch" in result.stderr
    assert "UNSAFE_FALLBACK" not in result.stdout


def test_hook_is_inert_without_explicit_ci_opt_in(monkeypatch):
    monkeypatch.delenv(ci.ENV_NAME, raising=False)
    namespace = {}
    exec(ci.bootstrap_source(), namespace)
    assert "_mesa" not in namespace

