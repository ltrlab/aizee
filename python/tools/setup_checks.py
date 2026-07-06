#!/usr/bin/env python3
"""AIZEE setup validation checks — the engine behind the /setup wizard.

Runs a battery of pass/fail/warn checks that validate a Jetson bring-up end to
end: python deps, rust build, systemd units, CAN bus, udev device symlinks,
live ZMQ telemetry (motors / UPS / cameras), and network reach paths.

Each check returns:
    {"id", "group", "title", "status": "pass"|"warn"|"fail"|"skip",
     "detail": str, "hint": str}

"warn" = degraded but not blocking (e.g. optional hardware not plugged in).
"skip" = prerequisite missing, check could not run.

Used two ways:
  * imported by heartbeat_server.py (GET /api/checks, the /setup page)
  * standalone over SSH:  python3 python/tools/setup_checks.py   (exit code 1
    if any check fails; --json for machine-readable output)

Stdlib-only by design; pyyaml / pyzmq are probed lazily and their absence is
itself a reported check result rather than an ImportError.
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import time
from pathlib import Path

# Repo root: this file lives at <root>/python/tools/setup_checks.py
ROOT = Path(__file__).resolve().parents[2]

HARDWARE_YAML = ROOT / "config" / "hardware_jetson_rover.yaml"
MOTOR_BIN = ROOT / "rust" / "target" / "release" / "motor_control"

# Boot services must be enabled+active on a healthy robot. Device-bound
# services are started by udev, so "inactive" only warns when the device
# symlink is also missing.
BOOT_SERVICES = [
    "aizee-motor-control-rover",
    "aizee-heartbeat",
    "aizee-ups-monitor",
    "aizee-estop-bridge",
]
DEVICE_SERVICES = {
    "aizee-gripper-cam": "/dev/aizee_gripper_cam",
    "aizee-scene-cam": "/dev/aizee_scene_cam",
    "aizee-display": "/dev/tufty_display",
}

# Python modules every node needs -> fail; nice-to-have -> warn.
PY_REQUIRED = ["zmq", "msgpack", "numpy", "yaml", "serial", "PIL", "cv2"]
PY_OPTIONAL = ["pyrealsense2", "smbus"]

DEV_SYMLINKS = {
    "/dev/aizee_gripper_cam": ("gripper camera (ELP UVC)", "warn"),
    "/dev/aizee_scene_cam": ("scene camera (RealSense)", "warn"),
    "/dev/tufty_display": ("Tufty2040 display", "warn"),
    "/dev/estop-receiver": ("e-stop receiver (ESP32)", "warn"),
    "/dev/i2c-7": ("UPS INA219 i2c bus", "warn"),
}


def _run(args, timeout=8):
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return r.stdout
    except Exception:
        return ""


def _check(id_, group, title, status, detail="", hint=""):
    return {"id": id_, "group": group, "title": title,
            "status": status, "detail": detail, "hint": hint}


# ----------------------------------------------------------------- 1. system
def check_system():
    out = []
    model = ""
    try:
        model = Path("/proc/device-tree/model").read_text().strip("\x00 \n")
    except Exception:
        pass
    out.append(_check("jetson_model", "system", "Jetson platform",
                      "pass" if "Orin" in model or "Jetson" in model else "warn",
                      model or "not a Jetson (or /proc/device-tree missing)",
                      "This is fine on a dev machine; on-device it should read 'Jetson Orin Nano'."))

    try:
        import shutil as _sh
        free_gb = _sh.disk_usage("/").free / 1e9
        out.append(_check("disk_free", "system", "Free disk space",
                          "pass" if free_gb > 5 else ("warn" if free_gb > 2 else "fail"),
                          f"{free_gb:.1f} GB free on /",
                          "Recordings + cargo build need headroom; clear logs/ or old episodes."))
    except Exception as e:
        out.append(_check("disk_free", "system", "Free disk space", "skip", str(e)))

    for mod in PY_REQUIRED:
        try:
            __import__(mod)
            out.append(_check(f"py_{mod}", "system", f"python3: {mod}", "pass"))
        except Exception as e:
            out.append(_check(f"py_{mod}", "system", f"python3: {mod}", "fail",
                              str(e)[:120],
                              "Run: python3 -m pip install --user -r requirements_jetson.txt "
                              "(cv2/smbus come from apt: python3-opencv python3-smbus)"))
    for mod in PY_OPTIONAL:
        try:
            __import__(mod)
            out.append(_check(f"py_{mod}", "system", f"python3: {mod} (optional)", "pass"))
        except Exception:
            out.append(_check(f"py_{mod}", "system", f"python3: {mod} (optional)", "warn",
                              "not importable",
                              "pyrealsense2: only needed for the scene cam — see "
                              "scripts/build_librealsense_rsusb.sh. smbus: apt install python3-smbus."))
    return out


# ------------------------------------------------------------------ 2. build
def check_build():
    out = []
    if MOTOR_BIN.exists():
        age_h = (time.time() - MOTOR_BIN.stat().st_mtime) / 3600
        out.append(_check("motor_bin", "build", "motor_control binary",
                          "pass", f"built {age_h:.0f}h ago ({MOTOR_BIN})"))
    else:
        out.append(_check("motor_bin", "build", "motor_control binary", "fail",
                          f"missing: {MOTOR_BIN}",
                          "cd ~/aizee/rust/motor_control && cargo build --release "
                          "(or re-run scripts/setup_jetson.sh)"))
    cargo = _run(["bash", "-lc", "command -v cargo"]).strip()
    out.append(_check("cargo", "build", "Rust toolchain", "pass" if cargo else "warn",
                      cargo or "cargo not on PATH",
                      "Needed to rebuild after config/code changes: "
                      "curl https://sh.rustup.rs -sSf | sh -s -- -y"))
    if HARDWARE_YAML.exists():
        out.append(_check("hw_yaml", "build", "hardware_jetson_rover.yaml", "pass",
                          str(HARDWARE_YAML)))
    else:
        out.append(_check("hw_yaml", "build", "hardware_jetson_rover.yaml", "fail",
                          "config missing — motor_control cannot start",
                          "Re-sync the repo (scripts/bootstrap_jetson.sh from the dev machine)."))
    return out


# --------------------------------------------------------------- 3. services
def check_services():
    out = []
    for svc in BOOT_SERVICES:
        installed = Path(f"/etc/systemd/system/{svc}.service").exists()
        if not installed:
            out.append(_check(f"svc_{svc}", "services", svc, "fail",
                              "unit file not installed",
                              "Re-run scripts/setup_jetson.sh (installs + enables all units)."))
            continue
        active = _run(["systemctl", "is-active", f"{svc}.service"]).strip()
        enabled = _run(["systemctl", "is-enabled", f"{svc}.service"]).strip()
        if active == "active" and enabled == "enabled":
            out.append(_check(f"svc_{svc}", "services", svc, "pass", "active, enabled"))
        elif active == "active":
            out.append(_check(f"svc_{svc}", "services", svc, "warn",
                              f"active but {enabled or 'not enabled'} — won't start on boot",
                              f"sudo systemctl enable {svc}"))
        else:
            hint = f"sudo systemctl start {svc}; sudo journalctl -u {svc} -n 50"
            if svc == "aizee-estop-bridge":
                hint = "Plug in the e-stop receiver (creates /dev/estop-receiver), then: " + hint
            out.append(_check(f"svc_{svc}", "services", svc, "fail",
                              f"state: {active or 'unknown'}", hint))
    for svc, dev in DEVICE_SERVICES.items():
        installed = Path(f"/etc/systemd/system/{svc}.service").exists()
        if not installed:
            out.append(_check(f"svc_{svc}", "services", f"{svc} (device-bound)", "fail",
                              "unit file not installed", "Re-run scripts/setup_jetson.sh."))
            continue
        active = _run(["systemctl", "is-active", f"{svc}.service"]).strip()
        if active == "active":
            out.append(_check(f"svc_{svc}", "services", f"{svc} (device-bound)", "pass",
                              "active"))
        elif not Path(dev).exists():
            out.append(_check(f"svc_{svc}", "services", f"{svc} (device-bound)", "warn",
                              f"idle — {dev} not present",
                              "Starts automatically when the device is plugged in (udev)."))
        else:
            out.append(_check(f"svc_{svc}", "services", f"{svc} (device-bound)", "fail",
                              f"{dev} present but service is {active or 'unknown'}",
                              f"sudo journalctl -u {svc} -n 50"))
    return out


# --------------------------------------------------------------------- 4. CAN
def check_can():
    out = []
    helper = Path("/usr/local/bin/aizee-reset-usb-can")
    out.append(_check("can_helper", "can", "CAN reset helper",
                      "pass" if helper.exists() else "fail",
                      str(helper) if helper.exists() else "not installed",
                      "Re-run scripts/setup_jetson.sh (installs helper + sudoers)."))
    sudoers = Path("/etc/sudoers.d/aizee-can")
    out.append(_check("can_sudoers", "can", "CAN sudoers rule",
                      "pass" if sudoers.exists() else "fail",
                      "" if sudoers.exists() else "missing — ExecStartPre will hang on a password prompt",
                      "Re-run scripts/setup_jetson.sh."))

    link = _run(["ip", "-details", "link", "show", "can1"])
    if not link:
        # A gs_usb adapter that enumerated as can0 gets renamed by the helper;
        # report what exists to make the fix obvious.
        other = re.findall(r"\d+: (can\d+)", _run(["ip", "link", "show"]))
        out.append(_check("can_up", "can", "can1 interface", "fail",
                          f"can1 not found (present: {', '.join(other) or 'no CAN interfaces'})",
                          "Check the USB-CAN adapter is plugged in and powered (it has a physical "
                          "switch), then: sudo /usr/local/bin/aizee-reset-usb-can can1"))
        return out
    up = "state UP" in link or ",UP" in link.split("\n", 1)[0]
    m = re.search(r"bitrate (\d+)", link)
    bitrate = int(m.group(1)) if m else None
    if up and bitrate == 1000000:
        out.append(_check("can_up", "can", "can1 interface", "pass", "UP @ 1 Mbps"))
    elif up:
        out.append(_check("can_up", "can", "can1 interface", "warn",
                          f"UP but bitrate {bitrate or 'unknown'} (motors need 1000000)",
                          "sudo /usr/local/bin/aizee-reset-usb-can can1"))
    else:
        out.append(_check("can_up", "can", "can1 interface", "fail", "DOWN",
                          "sudo /usr/local/bin/aizee-reset-usb-can can1"))
    return out


# ----------------------------------------------------------------- 5. devices
def check_devices():
    out = []
    for dev, (label, missing_level) in DEV_SYMLINKS.items():
        if Path(dev).exists():
            out.append(_check(f"dev_{Path(dev).name}", "devices", label, "pass", dev))
        else:
            out.append(_check(f"dev_{Path(dev).name}", "devices", label, missing_level,
                              f"{dev} missing",
                              "Optional hardware — plug it in; udev creates the symlink and "
                              "starts its service. If plugged in, check config/udev rules were "
                              "installed (re-run scripts/setup_jetson.sh)."))
    return out


# --------------------------------------------------------------- 6. telemetry
def _expected_motors():
    """Motor names from hardware_jetson_rover.yaml (wheels + swivel + arm)."""
    try:
        import yaml
        cfg = yaml.safe_load(HARDWARE_YAML.read_text())
        motors = cfg.get("motors", {}) or {}
        names = []
        for group in ("wheels", "swivel", "arm"):
            entries = motors.get(group) or []
            if isinstance(entries, dict):
                entries = [entries]
            for e in entries:
                if isinstance(e, dict) and e.get("name"):
                    names.append(e["name"])
        return names
    except Exception:
        return []


def check_telemetry(snapshot=None):
    """`snapshot` is heartbeat_server.TelemetryPoller.snapshot() when embedded;
    standalone we do a short one-shot ZMQ subscribe instead."""
    out = []
    if snapshot is None:
        snapshot = _oneshot_snapshot()
    if not snapshot or not snapshot.get("available"):
        reason = (snapshot or {}).get("reason", "no ZMQ snapshot")
        out.append(_check("telem", "telemetry", "Robot telemetry", "fail", reason,
                          "Is aizee-motor-control-rover running? pyzmq/msgpack installed?"))
        return out

    ms = snapshot.get("motors", {}) or {}
    if ms.get("stale"):
        out.append(_check("telem_motors", "telemetry", "Motor telemetry", "fail",
                          "stale / no messages on :5556",
                          "sudo journalctl -u aizee-motor-control-rover -n 50 — usually a CAN "
                          "problem (adapter power, wheels configured but not on the bus)."))
    else:
        reported = {m["name"]: m for m in ms.get("list", [])}
        expected = _expected_motors()
        missing = [n for n in expected if n not in reported]
        faulted = [n for n, m in reported.items() if m.get("error") or m.get("state") == "error"]
        if not missing and not faulted:
            out.append(_check("telem_motors", "telemetry", "Motor telemetry", "pass",
                              f"{len(reported)}/{len(expected) or '?'} motors reporting, none faulted"))
        elif missing:
            out.append(_check("telem_motors", "telemetry", "Motor telemetry", "fail",
                              f"missing from telemetry: {', '.join(missing)}",
                              "Motor not on the CAN bus or wrong CAN id. If the wheels are "
                              "physically absent, set motors.wheels: [] in "
                              "config/hardware_jetson_rover.yaml (keep the key!) and restart."))
        else:
            out.append(_check("telem_motors", "telemetry", "Motor telemetry", "fail",
                              f"faulted: {', '.join(faulted)}",
                              "Check motor power (30V pack) and per-motor error in the dashboard."))
        estop = snapshot.get("estop")
        out.append(_check("telem_estop", "telemetry", "E-stop state",
                          "pass" if estop is False else ("fail" if estop else "warn"),
                          "CLEAR" if estop is False else ("ENGAGED" if estop else "unknown"),
                          "Release the physical e-stop (twist) if engaged."))
        bv = ms.get("battery_voltage")
        if bv is None:
            out.append(_check("telem_batt", "telemetry", "Motor pack voltage", "warn",
                              "no VBUS reading yet"))
        else:
            out.append(_check("telem_batt", "telemetry", "Motor pack voltage",
                              "pass" if bv >= 21.0 else "warn", f"{bv} V (6S pack)",
                              "Charge the pack below ~21 V."))

    ups = snapshot.get("ups", {}) or {}
    out.append(_check("telem_ups", "telemetry", "Logic UPS telemetry",
                      "warn" if ups.get("stale") else "pass",
                      "stale" if ups.get("stale") else f"{ups.get('voltage')} V, {ups.get('percentage')}%",
                      "aizee-ups-monitor down or INA219 not on i2c bus 7 (check /dev/i2c-7)."))

    for cam in snapshot.get("cameras", []) or []:
        name = cam.get("name", "?")
        if cam.get("online"):
            out.append(_check(f"telem_cam_{name}", "telemetry", f"{name} camera stream", "pass",
                              f"{cam.get('fps')} fps @ {cam.get('width')}x{cam.get('height')}"))
        else:
            lvl = "warn" if name == "scene" else "fail"
            out.append(_check(f"telem_cam_{name}", "telemetry", f"{name} camera stream", lvl,
                              "no frames",
                              f"Camera unplugged, or aizee-{name if name != 'gripper' else 'gripper'}-cam "
                              "service down. Scene cam is optional (rover mode)."))
    return out


def _oneshot_snapshot(wait_s=2.5):
    """Standalone-mode substitute for TelemetryPoller: subscribe briefly."""
    try:
        import zmq
        import msgpack
    except Exception:
        return {"available": False, "reason": "pyzmq/msgpack not installed"}
    ctx = zmq.Context.instance()
    eps = {"motor": "tcp://localhost:5556", "ups": "tcp://localhost:5562",
           "gripper": "tcp://localhost:5563", "scene": "tcp://localhost:5564"}
    socks, latest = {}, {}
    for name, ep in eps.items():
        s = ctx.socket(zmq.SUB)
        s.setsockopt_string(zmq.SUBSCRIBE, "")
        s.setsockopt(zmq.RCVTIMEO, 100)
        s.connect(ep)
        socks[name] = s
    deadline = time.time() + wait_s
    while time.time() < deadline:
        for name, s in socks.items():
            try:
                parts = s.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                continue
            except Exception:
                continue
            try:
                latest[name] = msgpack.unpackb(parts[0], raw=False)
            except Exception:
                pass
        time.sleep(0.05)
    for s in socks.values():
        s.close(linger=0)

    motor = latest.get("motor")
    cams = []
    for name in ("gripper", "scene"):
        hdr = latest.get(name) or {}
        color = hdr.get("color") or {}
        cams.append({"name": name, "online": name in latest, "fps": None,
                     "width": color.get("width"), "height": color.get("height")})
    ups_msg = (latest.get("ups") or {}).get("ups", {}) or {}
    return {
        "available": True,
        "estop": bool(motor.get("emergency_stop", False)) if motor else None,
        "motors": {
            "stale": motor is None,
            "battery_voltage": motor.get("battery_voltage") if motor else None,
            "list": [
                {"name": n, **{k: (v or {}).get(k) for k in ("state", "error")}}
                for n, v in ((motor or {}).get("motors", {}) or {}).items()
            ],
        },
        "ups": {"stale": "ups" not in latest,
                "voltage": ups_msg.get("voltage"),
                "percentage": ups_msg.get("percentage")},
        "cameras": cams,
    }


# ----------------------------------------------------------------- 7. network
def check_network():
    out = []
    addrs = _run(["ip", "-o", "-4", "addr", "show"])
    have_ap = "192.168.50.1" in addrs
    have_usb = "10.42.0.1" in addrs
    nm = _run(["nmcli", "-t", "-f", "NAME,DEVICE,STATE", "con", "show", "--active"])
    out.append(_check("net_ap", "network", "WiFi AP (192.168.50.1)",
                      "pass" if have_ap else "warn",
                      next((l for l in nm.splitlines() if "aizee" in l.lower()), "") or
                      ("active" if have_ap else "not active"),
                      "Re-run scripts/setup_jetson.sh --ap-pass <psk>, or: "
                      "sudo nmcli con up aizee-ap"))
    out.append(_check("net_usb", "network", "USB-C shared link (10.42.0.1)",
                      "pass" if have_usb else "warn",
                      "active" if have_usb else "not active",
                      "Optional. scripts/setup_jetson.sh --usb-eth <iface> creates it; "
                      "JetPack's built-in device-mode (192.168.55.1) also works."))
    return out


# -------------------------------------------------------------------- runner
def run_all(snapshot=None):
    checks = []
    for fn in (check_system, check_build, check_services, check_can, check_devices):
        try:
            checks.extend(fn())
        except Exception as e:  # a crashed checker is itself a finding
            checks.append(_check(fn.__name__, "internal", fn.__name__, "fail", repr(e)))
    try:
        checks.extend(check_telemetry(snapshot))
    except Exception as e:
        checks.append(_check("check_telemetry", "internal", "check_telemetry", "fail", repr(e)))
    try:
        checks.extend(check_network())
    except Exception as e:
        checks.append(_check("check_network", "internal", "check_network", "fail", repr(e)))
    counts = {s: sum(1 for c in checks if c["status"] == s)
              for s in ("pass", "warn", "fail", "skip")}
    return {"generated": time.strftime("%Y-%m-%d %H:%M:%S"),
            "counts": counts, "checks": checks}


def main():
    import argparse
    ap = argparse.ArgumentParser(description="AIZEE setup validation")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()
    result = run_all()
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        icons = {"pass": "✓", "warn": "!", "fail": "✗", "skip": "-"}
        group = None
        for c in result["checks"]:
            if c["group"] != group:
                group = c["group"]
                print(f"\n[{group}]")
            print(f"  {icons[c['status']]} {c['title']}: {c['detail'] or c['status']}")
            if c["status"] in ("fail", "warn") and c["hint"]:
                print(f"      -> {c['hint']}")
        n = result["counts"]
        print(f"\n{n['pass']} pass, {n['warn']} warn, {n['fail']} fail, {n['skip']} skip")
    raise SystemExit(1 if result["counts"]["fail"] else 0)


if __name__ == "__main__":
    main()
