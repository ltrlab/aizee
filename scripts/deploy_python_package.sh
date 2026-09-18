#!/usr/bin/env bash
# Install the shared Python package and wire compatibility shim on an existing
# Jetson checkout. This helper does not restart services. Bootstrap installs the
# same package; camera-only updates call this helper before replacing a node.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_DIR/scripts/deploy_common.sh"
TARGET="${1:-$AIZEE_TARGET}"
TARBALL="$(mktemp "${TMPDIR:-/tmp}/aizee_python_deploy.XXXXXX.tar.gz")"
trap 'rm -f "$TARBALL"' EXIT

tar czf "$TARBALL" -C "$REPO_DIR" \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='*.egg-info' \
    pyproject.toml README.md src python/common
scp -i "$SSH_KEY" "$TARBALL" "$TARGET:/tmp/aizee_python_deploy.tar.gz"
ssh -i "$SSH_KEY" "$TARGET" \
    "mkdir -p ~/aizee && cd ~/aizee && tar xzf /tmp/aizee_python_deploy.tar.gz && \
     rm /tmp/aizee_python_deploy.tar.gz && \
     python3 -m pip install --user 'setuptools>=68' wheel && \
     python3 -m pip install --user --no-deps --no-build-isolation -e ."
