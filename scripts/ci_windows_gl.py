#!/usr/bin/env python3
"""Opt-in, checkout-local Mesa prerequisite for hosted Windows camera tests.

Never installs a system graphics driver. A venv-only startup hook is inert unless
ROBOT_CI_MESA_DIR is explicitly set, including in fresh renderer subprocesses.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
VERSION = "26.2.4"
URL = f"https://github.com/pal1000/mesa-dist-win/releases/download/{VERSION}/mesa3d-{VERSION}-release-msvc.7z"
ARCHIVE_SHA256 = "351fc8c8b695878ffb3eaa044b3ead08672a48b1a045e3c3e3975811df0f6695"
DLL_SHA256 = {
    "opengl32.dll": "75b295fc80df0eaa8f1264c28663f6932c8b72d36513eb092556608858b7f5e0",
    "libgallium_wgl.dll": "53e49adf87590d5b0cab8553b1e98084812040cc1fca8c487695b17b7af3702e",
}
ENV_NAME = "ROBOT_CI_MESA_DIR"


def digest(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def verify(path: Path, expected: str) -> str:
    actual = digest(path)
    if actual != expected:
        raise RuntimeError(f"SHA256 mismatch for {path.name}: expected {expected}, got {actual}")
    return actual


def bootstrap_source() -> str:
    # SystemExit is intentional: site.py prints and ignores ordinary exceptions
    # from .pth files. A requested but broken software backend must abort startup.
    return f'''"""Generated, opt-in CI Mesa bootstrap; no effect without {ENV_NAME}."""
import os
if os.environ.get({ENV_NAME!r}):
    try:
        import ctypes
        import hashlib
        from pathlib import Path
        directory = Path(os.environ[{ENV_NAME!r}]).resolve(strict=True)
        for name, expected in {DLL_SHA256!r}.items():
            with (directory / name).open("rb") as stream:
                actual = hashlib.file_digest(stream, "sha256").hexdigest()
            if actual != expected:
                raise RuntimeError("CI Mesa DLL checksum mismatch: " + name)
        if os.environ.get("GALLIUM_DRIVER") != "llvmpipe":
            raise RuntimeError("CI Mesa requires the llvmpipe software driver")
        _directory_handle = os.add_dll_directory(str(directory))
        _mesa = ctypes.WinDLL(str(directory / "opengl32.dll"))
    except Exception as error:
        raise SystemExit("Required CI Mesa startup failed: " + str(error)) from error
'''


def child_json(command: list[str], *, environment: dict | None = None) -> dict:
    child = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=60)
    if child.returncode:
        raise RuntimeError(f"Required renderer child failed ({child.returncode}):\n{child.stdout}\n{child.stderr}")
    return json.loads(child.stdout)


def render_smoke() -> dict:
    """Run only in a fresh, opted-in venv interpreter after its startup hook."""
    import ctypes
    import glfw
    import mujoco
    import numpy as np

    if "aizee_ci_mesa" not in sys.modules:
        raise RuntimeError("The fresh renderer child did not load its CI Mesa startup hook")
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><light pos="0 0 3"/>'
        '<geom type="sphere" size="0.2" rgba="1 0 0 1"/></worldbody></mujoco>'
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, height=120, width=160) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
        # Query the actual GLFW context. PyOpenGL may independently resolve the
        # system opengl32 DLL and does not identify this renderer's loaded driver.
        address = glfw.get_proc_address("glGetString")
        if not address:
            raise RuntimeError("Renderer context has no glGetString")
        get_string = ctypes.WINFUNCTYPE(ctypes.c_char_p, ctypes.c_uint)(address)
        strings = {name: (get_string(enum) or b"").decode("ascii") for name, enum in
                   (("vendor", 0x1F00), ("renderer", 0x1F01), ("version", 0x1F02))}
        if strings["vendor"] != "Mesa" or "llvmpipe" not in strings["renderer"]:
            raise RuntimeError(f"Software Mesa was not selected: {strings}")
        if f"Mesa {VERSION}" not in strings["version"]:
            raise RuntimeError(f"Unexpected Mesa version: {strings}")
        if pixels.shape != (120, 160, 3) or pixels.dtype != np.uint8 or pixels.std() <= 10:
            raise RuntimeError("The real MuJoCo render produced an invalid or blank RGB image")
        return dict(**strings, pid=os.getpid(), parent_pid=os.getppid(),
                    executable=sys.executable, mujoco=mujoco.__version__,
                    shape=list(pixels.shape), image_std=float(pixels.std()),
                    image_sha256=hashlib.sha256(pixels.tobytes()).hexdigest())


def configure(archive: Path | None) -> dict:
    if sys.platform != "win32" or sys.maxsize <= 2**32:
        raise RuntimeError("This prerequisite supports only Windows x86_64 CI")
    output = ROOT / "build/ci-mesa"
    output.mkdir(parents=True, exist_ok=True)
    if archive is None:
        archive = output / f"mesa3d-{VERSION}-release-msvc.7z"
        if not archive.exists():
            temporary = archive.with_suffix(".download")
            urllib.request.urlretrieve(URL, temporary)
            verify(temporary, ARCHIVE_SHA256)
            temporary.replace(archive)
    verify(archive, ARCHIVE_SHA256)  # Never extract or load an unverified archive.
    extractor = shutil.which("7z") or str(Path(os.environ.get("ProgramFiles", "C:/Program Files")) / "7-Zip/7z.exe")
    if not Path(extractor).is_file():
        raise RuntimeError("7-Zip is required to extract the pinned Windows CI Mesa archive")
    extracted = output / VERSION
    subprocess.run([extractor, "x", str(archive.resolve()), f"-o{extracted}",
                    "x64/opengl32.dll", "x64/libgallium_wgl.dll", "-y"], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    directory = (extracted / "x64").resolve(strict=True)
    dlls = {name: verify(directory / name, expected) for name, expected in DLL_SHA256.items()}
    python = ROOT / ".venv-dev/Scripts/python.exe"
    site = Path(subprocess.check_output([str(python), "-c",
                "import sysconfig; print(sysconfig.get_path('purelib'))"], text=True).strip())
    if not site.resolve().is_relative_to((ROOT / ".venv-dev").resolve()):
        raise RuntimeError("Refusing to install the CI hook outside this checkout's development venv")
    bootstrap = site / "aizee_ci_mesa.py"
    bootstrap.write_text(bootstrap_source(), encoding="utf-8")
    (site / "aizee_ci_mesa.pth").write_text("import aizee_ci_mesa\n", encoding="utf-8")
    environment = dict(os.environ, **{ENV_NAME: str(directory), "GALLIUM_DRIVER": "llvmpipe",
                                     "LIBGL_ALWAYS_SOFTWARE": "true"})
    # An intermediate Python child launches another fresh interpreter, matching
    # the renderer's inherited environment rather than an in-process preload.
    proof = child_json([str(python), str(Path(__file__).resolve()), "--smoke-parent"], environment=environment)
    report = dict(passed=True, version=VERSION, url=URL, archive_sha256=ARCHIVE_SHA256,
                  dll_sha256=dlls, bootstrap_sha256=digest(bootstrap), directory=str(directory),
                  extractor=extractor, smoke=proof)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if os.environ.get("GITHUB_ENV"):
        with Path(os.environ["GITHUB_ENV"]).open("a", encoding="utf-8") as stream:
            for name in (ENV_NAME, "GALLIUM_DRIVER", "LIBGL_ALWAYS_SOFTWARE"):
                stream.write(f"{name}={environment[name]}\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, help="Use an already downloaded archive; checksum still mandatory")
    parser.add_argument("--smoke-parent", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--smoke-child", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.smoke_child:
        result = render_smoke()
    elif args.smoke_parent:
        result = dict(spawn_parent_pid=os.getpid(), child=child_json(
            [sys.executable, str(Path(__file__).resolve()), "--smoke-child"]))
    else:
        result = configure(args.archive)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
