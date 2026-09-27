#!/bin/bash
# Deploy Minerva's guarded arm runtime to the Jetson as a verified release.
#
# Built on this PC: the cross-compiled ARM64 receiver, harness and collision
# library; the Python platform; the device profiles; the model's core files. The
# build checks the collision model against the CAD meshes here and ships their
# digests, not the meshes. On the Jetson it becomes ~/minerva/releases/<id>,
# verified file by file, smoke-tested, then made ~/minerva/current atomically (a
# failed smoke test rolls back). Only the last three releases are kept.
#
# User space only: no sudo, no services, no compiling on the Jetson. The one-time
# system setup (CAN serial pinning, CAN helpers, sudoers) is
# scripts/setup_minerva_system.sh, which needs sudo; run that one yourself.
#
# Usage: scripts/deploy_minerva_arms.sh [--build] [--keep N] [--address IP]
#   --build    cross-compile the ARM64 binaries first (scripts/build_jetson.ps1)
#   --address  robot address (default: the USB Ethernet tether, 10.42.0.1)
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_DIR"
PYTHON="$REPO_DIR/.venv-platform/Scripts/python.exe"
[[ -x "$PYTHON" ]] || PYTHON="$REPO_DIR/.venv-platform/bin/python"
[[ -x "$PYTHON" ]] || PYTHON=python3

if [[ "${1:-}" == "--build" ]]; then
    shift
    BUILD_SCRIPT="$REPO_DIR/scripts/build_jetson.ps1"
    # PowerShell cannot open Git Bash paths (/c/Users/...); hand it a Windows path.
    command -v cygpath >/dev/null 2>&1 && BUILD_SCRIPT="$(cygpath -w "$BUILD_SCRIPT")"
    powershell -NoProfile -ExecutionPolicy Bypass -Command \
        "& '$BUILD_SCRIPT' -Offline -Package robot_control, robot_collision"
fi
exec "$PYTHON" -m robot_platform deploy "$@"
