"""Exercise the privileged recipe using temporary files and fake commands only."""

import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.fixture
def maintenance(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[2] / "scripts/maintenance/gs_usb/maintain_minerva.py"
    spec = importlib.util.spec_from_file_location("can_driver_maintenance_under_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    linux = tmp_path / "linux"

    def fake_path(value):
        text = str(value)
        return linux / text.lstrip("/") if text.startswith(("/etc/", "/proc/", "/sys/")) else Path(value)

    module.STAGE = tmp_path / "stage"
    module.STAGE.mkdir()
    module.ORIGINAL = tmp_path / "modules/kernel/gs_usb.ko"
    module.TARGET = tmp_path / "modules/updates/aizee/gs_usb.ko"
    module.CANDIDATE = module.STAGE / "candidate-full/gs_usb.ko"
    for path, data in ((module.ORIGINAL, b"stock module"), (module.CANDIDATE, b"reviewed candidate")):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    fake_path("/sys/bus/usb/devices").mkdir(parents=True)
    module.ORIGINAL_SHA = hashlib.sha256(module.ORIGINAL.read_bytes()).hexdigest()
    module.CANDIDATE_SHA = hashlib.sha256(module.CANDIDATE.read_bytes()).hexdigest()
    for name, data in (
        ("/etc/machine-id", module.MACHINE.encode() + b"\n"),
        ("/proc/device-tree/serial-number", module.SERIAL.encode() + b"\0"),
        ("/proc/sys/kernel/random/boot_id", b"offline-test-boot\n"),
    ):
        path = fake_path(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    monkeypatch.setattr(module, "Path", fake_path)
    monkeypatch.setattr(module.os, "geteuid", lambda: 0, raising=False)
    calls = []

    def command(*args):
        calls.append(args)
        if args == ("uname", "-r"):
            return module.KERNEL
        if args[:3] == ("modinfo", "-F", "vermagic"):
            return module.KERNEL + " SMP preempt mod_unload modversions aarch64"
        if args[:2] == ("systemctl", "show"):
            return "0" if "--property=MainPID" in args else "disabled"
        if args == ("ss", "-H", "-ltn", "sport", "=", ":5590"):
            return ""
        if args == ("depmod", "-a", module.KERNEL):
            return ""
        if args == ("modinfo", "-n", "gs_usb"):
            return str(module.TARGET if module.TARGET.exists() else module.ORIGINAL)
        raise AssertionError(f"Unexpected command in offline test: {args}")

    monkeypatch.setattr(module, "command", command)
    module.test_calls = calls
    module.test_path = fake_path
    module.test_root = tmp_path
    return module


def run_action(module, monkeypatch, action):
    monkeypatch.setattr(sys, "argv", ["maintain_minerva.py", action])
    module.main()


def file_snapshot(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in root.rglob("*") if p.is_file()}


@pytest.mark.parametrize(
    ("fault", "message"),
    [
        ("machine", "Wrong machine identity"),
        ("serial", "Wrong Jetson module"),
        ("kernel", "Kernel changed"),
        ("original", "Original module changed"),
        ("candidate", "Candidate hash mismatch"),
    ],
)
def test_install_rejects_wrong_identity_or_module_before_mutation(maintenance, monkeypatch, fault, message):
    m = maintenance
    if fault == "machine":
        m.test_path("/etc/machine-id").write_text("different-machine")
    elif fault == "serial":
        m.test_path("/proc/device-tree/serial-number").write_bytes(b"different-module\0")
    elif fault == "kernel":
        original_command = m.command
        monkeypatch.setattr(m, "command", lambda *args: "other-kernel" if args == ("uname", "-r") else original_command(*args))
    elif fault == "original":
        m.ORIGINAL.write_bytes(b"unexpected stock module")
    else:
        m.CANDIDATE.write_bytes(b"unexpected candidate")
    before = file_snapshot(m.test_root)
    with pytest.raises(RuntimeError, match=message):
        run_action(m, monkeypatch, "install")
    assert file_snapshot(m.test_root) == before
    assert not any(args[0] == "depmod" for args in m.test_calls)


def test_atomic_copy_rejects_changed_source_without_touching_destination(maintenance):
    m = maintenance
    m.TARGET.parent.mkdir(parents=True)
    m.TARGET.write_bytes(b"existing destination")
    m.CANDIDATE.write_bytes(b"changed after preflight")
    before = file_snapshot(m.test_root)
    with pytest.raises(RuntimeError, match="Source hash changed"):
        m.atomic_copy(m.CANDIDATE, m.TARGET, m.CANDIDATE_SHA)
    assert file_snapshot(m.test_root) == before


def test_atomic_copy_verification_failure_preserves_existing_destination(maintenance, monkeypatch):
    m = maintenance
    m.TARGET.parent.mkdir(parents=True)
    m.TARGET.write_bytes(b"existing destination")
    original_digest = m.digest
    monkeypatch.setattr(m, "digest", lambda path: "corrupt-copy" if path.name.startswith(".gs_usb-") else original_digest(path))
    with pytest.raises(RuntimeError, match="Copy verification failed"):
        m.atomic_copy(m.CANDIDATE, m.TARGET, m.CANDIDATE_SHA)
    assert m.TARGET.read_bytes() == b"existing destination"
    assert list(m.TARGET.parent.glob(".gs_usb-*")) == []


@pytest.mark.parametrize("prior_override", [False, True])
@pytest.mark.parametrize("failure", ["depmod", "selection"])
def test_install_failure_restores_prior_disk_selection(maintenance, monkeypatch, prior_override, failure):
    m = maintenance
    if prior_override:
        m.TARGET.parent.mkdir(parents=True)
        m.TARGET.write_bytes(m.CANDIDATE.read_bytes())
    original_command = m.command
    depmods = 0

    def command(*args):
        nonlocal depmods
        if args[0] == "depmod":
            depmods += 1
            if failure == "depmod" and depmods == 1:
                raise subprocess.CalledProcessError(1, args)
        if args == ("modinfo", "-n", "gs_usb") and failure == "selection" and depmods == 1:
            return str(m.ORIGINAL)
        return original_command(*args)

    monkeypatch.setattr(m, "command", command)
    with pytest.raises((RuntimeError, subprocess.CalledProcessError)):
        run_action(m, monkeypatch, "install")
    assert depmods == 2
    assert m.TARGET.exists() == prior_override
    if prior_override:
        assert m.digest(m.TARGET) == m.CANDIDATE_SHA
    assert m.digest(m.ORIGINAL) == m.ORIGINAL_SHA
    assert m.digest(m.STAGE / "gs_usb.original.ko") == m.ORIGINAL_SHA
    assert not (m.STAGE / "installation-records").exists()
    assert not any(args[0] in ("modprobe", "rmmod", "insmod", "reboot") for args in m.test_calls)


def test_rollback_protects_unknown_override_before_mutation(maintenance, monkeypatch):
    m = maintenance
    m.TARGET.parent.mkdir(parents=True)
    m.TARGET.write_bytes(b"another administrator's override")
    before = file_snapshot(m.test_root)
    with pytest.raises(RuntimeError, match="different override"):
        run_action(m, monkeypatch, "rollback")
    assert file_snapshot(m.test_root) == before
    assert not any(args[0] == "depmod" for args in m.test_calls)


@pytest.mark.parametrize("candidate_state", ["missing", "corrupt"])
@pytest.mark.parametrize("fail_depmod", [False, True])
def test_rollback_does_not_depend_on_staged_candidate(maintenance, monkeypatch, candidate_state, fail_depmod):
    m = maintenance
    installed = m.CANDIDATE.read_bytes()
    m.TARGET.parent.mkdir(parents=True)
    m.TARGET.write_bytes(installed)
    if candidate_state == "missing":
        m.CANDIDATE.unlink()
    else:
        m.CANDIDATE.write_bytes(b"damaged staging artifact")
    original_command = m.command
    depmods = 0

    def command(*args):
        nonlocal depmods
        if args[0] == "depmod":
            depmods += 1
            if fail_depmod and depmods == 1:
                raise subprocess.CalledProcessError(1, args)
        return original_command(*args)

    monkeypatch.setattr(m, "command", command)
    if fail_depmod:
        with pytest.raises(subprocess.CalledProcessError):
            run_action(m, monkeypatch, "rollback")
        assert depmods == 2
        assert m.TARGET.read_bytes() == installed
    else:
        run_action(m, monkeypatch, "rollback")
        assert depmods == 1
        assert not m.TARGET.exists()
    assert m.digest(m.ORIGINAL) == m.ORIGINAL_SHA
    assert not any(args[:3] == ("modinfo", "-F", "vermagic") for args in m.test_calls)


@pytest.mark.parametrize("usb_id", [("1d50", "606f"), ("0483", "df11")])
def test_unbound_or_dfu_adapter_blocks_install(maintenance, monkeypatch, usb_id):
    m = maintenance
    usb = m.test_path("/sys/bus/usb/devices") / "1-2.4"
    usb.mkdir()
    (usb / "idVendor").write_text(usb_id[0])
    (usb / "idProduct").write_text(usb_id[1])
    before = file_snapshot(m.test_root)
    with pytest.raises(RuntimeError, match="USB-CAN/DFU device still connected"):
        run_action(m, monkeypatch, "install")
    assert file_snapshot(m.test_root) == before
    assert not any(args[0] == "depmod" for args in m.test_calls)
