#!/bin/bash
# Install a staged scene-camera update on the Jetson: the publisher (camera_node.py) and its systemd unit.
#
# Run it on the Jetson, as root, on a directory staged from the dev machine:
#     sudo bash install_scene_cam.sh /home/ltr/minerva/camera-update-<stamp>
# The directory holds camera_node.py and aizee-scene-cam.service. The files they replace are kept in its before/
# subdirectory; to roll back, run this script on before/. The camera config and the udev rule are left as they are.
# Unlike deploy_scene_cam.sh, nothing here reads or sends a password.

set -euo pipefail

STAGE="${1:?usage: sudo bash install_scene_cam.sh <staged directory>}"
NODE=/home/ltr/aizee/python/nodes/camera_node.py
UNIT=/etc/systemd/system/aizee-scene-cam.service

if [[ $EUID -ne 0 ]]; then
    echo "Run it with sudo: it installs a systemd unit." >&2
    exit 1
fi
for f in camera_node.py aizee-scene-cam.service; do
    [[ -f "$STAGE/$f" ]] || { echo "Missing $STAGE/$f" >&2; exit 1; }
done
python3 -m py_compile "$STAGE/camera_node.py"

mkdir -p "$STAGE/before"
[[ -f "$STAGE/before/camera_node.py" ]] || cp -p "$NODE" "$STAGE/before/camera_node.py"
[[ -f "$STAGE/before/aizee-scene-cam.service" ]] || cp -p "$UNIT" "$STAGE/before/aizee-scene-cam.service"
chown -R ltr:ltr "$STAGE/before"

install -m 0755 -o ltr -g ltr "$STAGE/camera_node.py" "$NODE"
install -m 0644 -o root -g root "$STAGE/aizee-scene-cam.service" "$UNIT"
systemctl daemon-reload
systemctl reset-failed aizee-scene-cam.service || true
systemctl restart aizee-scene-cam.service
sleep 8
systemctl --no-pager --lines=12 status aizee-scene-cam.service || true
