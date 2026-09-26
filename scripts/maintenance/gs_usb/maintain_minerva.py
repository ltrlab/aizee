#!/usr/bin/env python3
"""Pinned Minerva driver installation/rollback; never hot-unloads or reboots.

This is a device-specific maintenance recipe, not part of robot control.
Run `check` unprivileged before the operator runs `install` with sudo.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

KERNEL = '5.15.185-tegra'
MACHINE = '661607618a944c90a32de217d0c92476'
SERIAL = '1421623029971'
ORIGINAL_SHA = '8ff2f6fadea5b591a20976fa7114b2823e4c10aa048a979b347f9356acc0105e'
CANDIDATE_SHA = 'e3153247afcc9c2468c353015fc01f5821fc6bef430c1e6d2d79eae210269323'
STAGE = Path('/home/ltr/aizee/device-maintenance/gs-usb-backport-20260925')
ORIGINAL = Path('/lib/modules') / KERNEL / 'kernel/drivers/net/can/usb/gs_usb.ko'
TARGET = Path('/lib/modules') / KERNEL / 'updates/aizee/gs_usb.ko'
CANDIDATE = STAGE / 'candidate-full/gs_usb.ko'
SERVICES = ('aizee-minerva-left', 'aizee-minerva-right', 'aizee-motor-control-rover')


def command(*args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT, timeout=30).strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def preflight(require_disconnected=True, require_candidate=True):
    require(Path('/etc/machine-id').read_text().strip() == MACHINE, 'Wrong machine identity')
    require(Path('/proc/device-tree/serial-number').read_bytes().rstrip(b'\0\n').decode() == SERIAL, 'Wrong Jetson module')
    require(command('uname', '-r') == KERNEL, 'Kernel changed: rebuild/review required')
    require(digest(ORIGINAL) == ORIGINAL_SHA, 'Original module changed: do not overwrite')
    if require_candidate:
        require(digest(CANDIDATE) == CANDIDATE_SHA, 'Candidate hash mismatch')
        require(command('modinfo', '-F', 'vermagic', str(CANDIDATE)).split()[0] == KERNEL, 'Candidate kernel mismatch')
    for service in SERVICES:
        require(command('systemctl', 'show', service, '--property=MainPID', '--value') == '0', service + ' is running')
        require(command('systemctl', 'show', service, '--property=UnitFileState', '--value') in ('disabled', 'masked'), service + ' could start on reboot')
    require(not command('ss', '-H', '-ltn', 'sport', '=', ':5590'), 'Guarded receiver is running')
    if require_disconnected:
        directory = Path('/sys/bus/usb/drivers/gs_usb')
        attached = [p.name for p in directory.iterdir() if ':' in p.name and p.is_symlink()] if directory.exists() else []
        require(not attached, 'Unplug adapter USB before maintenance: ' + ', '.join(attached))
        # BOOT mode or an unbound USB interface must not bypass this check.
        for usb in Path('/sys/bus/usb/devices').iterdir():
            vendor, product = usb / 'idVendor', usb / 'idProduct'
            if vendor.is_file() and product.is_file():
                identity = (vendor.read_text().strip(), product.read_text().strip())
                require(identity not in (('1d50', '606f'), ('0483', 'df11')),
                        'USB-CAN/DFU device still connected at ' + usb.name)
    for bus in ('canL', 'canR'):
        flags = Path('/sys/class/net') / bus / 'flags'
        require(not flags.exists() or not int(flags.read_text(), 16) & 1, bus + ' is up')
    require(not TARGET.is_symlink(), 'Refusing a symlink at the override destination')
    if TARGET.exists():
        require(digest(TARGET) == CANDIDATE_SHA, 'A different override already exists')


def atomic_copy(source, target, expected_sha):
    atomic_write(source.read_bytes(), target, expected_sha)


def atomic_write(data, target, expected_sha):
    require(hashlib.sha256(data).hexdigest() == expected_sha, 'Source hash changed before copy')
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.gs_usb-', delete=False) as output:
            temporary = Path(output.name)
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o644)
        require(digest(temporary) == expected_sha, 'Copy verification failed')
        os.replace(temporary, target)
        temporary = None
    finally:
        if temporary is not None:
            temporary.unlink()


def receipt(action, details):
    directory = STAGE / 'installation-records'
    directory.mkdir(exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    record = {'at_utc': stamp, 'action': action, 'asset_id': 'minerva-production-01',
              'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
              'original_sha256': ORIGINAL_SHA, 'candidate_sha256': CANDIDATE_SHA,
              'hot_module_reload_performed': False, 'reboot_performed': False, **details}
    (directory / (stamp + '-' + action + '.json')).write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'install', 'rollback', 'trace-on', 'trace-off'))
    action = parser.parse_args().action
    tracing = action.startswith('trace-')
    preflight(require_disconnected=not tracing, require_candidate=action in ('check', 'install'))
    if action == 'check':
        print('Preflight passed. No files, modules, services, or motor settings changed.')
        return
    require(os.geteuid() == 0, 'Run this specific maintenance action with sudo; do not send a password in chat')
    if tracing:
        parameter = Path('/sys/module/gs_usb/parameters/diag_frames')
        require(TARGET.exists() and digest(TARGET) == CANDIDATE_SHA and parameter.exists(), 'Patched driver must be installed and loaded first')
        value = '16384' if action == 'trace-on' else '0'
        parameter.write_text(value)
        require(parameter.read_text().strip() == value, 'Trace parameter verification failed')
        receipt(action, {'diag_frames': int(value)})
        return
    backup = STAGE / 'gs_usb.original.ko'
    if backup.exists():
        require(digest(backup) == ORIGINAL_SHA, 'Existing original backup does not match')
    else:
        atomic_copy(ORIGINAL, backup, ORIGINAL_SHA)
    previous_override = TARGET.exists()
    previous_data = TARGET.read_bytes() if previous_override else None
    if previous_override:
        require(hashlib.sha256(previous_data).hexdigest() == CANDIDATE_SHA, 'Override changed before maintenance')
    try:
        if action == 'install':
            atomic_copy(CANDIDATE, TARGET, CANDIDATE_SHA)
        elif TARGET.exists():
            TARGET.unlink()  # Remove only this hash-verified override.
        command('depmod', '-a', KERNEL)
        selected = Path(command('modinfo', '-n', 'gs_usb')).resolve()
        expected = TARGET if action == 'install' else ORIGINAL
        require(selected == expected.resolve(), 'Module lookup does not select the expected file')
        require(digest(selected) == (CANDIDATE_SHA if action == 'install' else ORIGINAL_SHA), 'Selected module hash mismatch')
    except Exception:
        # Restore the prior on-disk selection; never change the loaded module.
        if previous_override:
            atomic_write(previous_data, TARGET, CANDIDATE_SHA)
        elif TARGET.exists() and digest(TARGET) == CANDIDATE_SHA:
            TARGET.unlink()
        command('depmod', '-a', KERNEL)
        raise
    receipt(action, {'selected_module': str(selected), 'reboot_required': True})
    print('Disk selection verified. Keep motor power and adapter USB disconnected, then reboot the Jetson.')
    print('The currently loaded driver has not been unloaded or replaced.')


if __name__ == '__main__':
    main()
