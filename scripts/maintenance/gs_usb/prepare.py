#!/usr/bin/env python3
"""Reproduce the reviewed Linux gs_usb source; never build/install/load it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='New output directory')
    parser.add_argument('--source', type=Path, help='Optional previously downloaded baseline source')
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parent
    manifest = json.loads((bundle / 'source-manifest.json').read_text())
    patch = shutil.which('patch')
    if not patch:
        parser.error('Linux patch utility is required; run this recipe on the Jetson/build host')
    if args.output.exists():
        parser.error('Output already exists; choose a new directory to preserve prior evidence')
    if args.source:
        data = args.source.read_bytes()
    else:
        with urllib.request.urlopen(manifest['baseline_url'], timeout=30) as response:
            data = response.read()
    expected = manifest['files']['baseline/gs_usb.c']
    if hashlib.sha256(data).hexdigest() != expected:
        parser.error('Baseline source hash mismatch')
    patches = sorted(bundle.glob('000[1-5]-*.patch'))
    if len(patches) != 5:
        parser.error('Expected exactly five reviewed patches')
    for path in patches:
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['files'][path.name]:
            parser.error('Patch hash mismatch: ' + path.name)
    args.output.mkdir(parents=True)
    (args.output / 'gs_usb.c').write_bytes(data)
    for path in patches:
        subprocess.run([patch, '--batch', '--forward', '--fuzz=0', '-p1', '-i', str(path.resolve())],
                       cwd=args.output, check=True, timeout=10)
    actual = hashlib.sha256((args.output / 'gs_usb.c').read_bytes()).hexdigest()
    if actual != manifest['files']['candidate-full/gs_usb.c']:
        parser.error('Patched source differs from reviewed candidate')
    (args.output / 'Makefile').write_text('obj-m += gs_usb.o\n')
    print(json.dumps({'source_sha256': actual, 'output': str(args.output.resolve()),
                      'built': False, 'installed': False}))


if __name__ == '__main__':
    main()
