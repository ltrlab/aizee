#!/usr/bin/env python3
"""AIZEE Heartbeat - a status dashboard for the Jetson.

Serves a single HTML page plus a small JSON API showing:
  * robot telemetry at a glance: E-stop, per-motor state/pos/vel/torque/temp,
    motor-pack + logic-UPS battery, and gripper/scene camera health
  * systemd status of all aizee-* services (+ a few core units)
  * recent journald logs per service
  * host metrics (CPU, memory, disk, thermal zones, uptime, load)
  * network interfaces + WiFi AP info

Host/service/log metrics use only the Python 3 standard library (read from
/proc and /sys, shell out to systemctl / journalctl with list-args, no shell).
Robot telemetry is read-only over ZMQ (pyzmq + msgpack — the same libs the
ups/display nodes already use under /usr/bin/python3); if those imports fail
the telemetry section degrades gracefully and the rest of the page still works.

Run:  python3 heartbeat_server.py [--host 0.0.0.0] [--port 8088]
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

# Units always shown even if not matched by the aizee-* glob.
EXTRA_UNITS = ["NetworkManager.service", "docker.service"]

# systemd properties fetched per unit (machine-readable, no root needed).
PROPS = [
    "Id", "Description", "LoadState", "ActiveState", "SubState",
    "UnitFileState", "ActiveEnterTimestampMonotonic", "MemoryCurrent",
    "MainPID", "NRestarts",
]

UNIT_RE = re.compile(r"^[A-Za-z0-9@._\-]+\.service$")  # injection guard for log requests


def run(args, timeout=8):
    """Run a command (list args, no shell) and return stdout, '' on failure."""
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return r.stdout
    except Exception:
        return ""


# ----------------------------------------------------------------------------- services
def discover_units():
    out = run(["systemctl", "list-unit-files", "--type=service",
               "--no-legend", "aizee-*.service"])
    units = [ln.split()[0] for ln in out.splitlines() if ln.strip()]
    for u in EXTRA_UNITS:
        if u not in units:
            units.append(u)
    # de-dupe, keep aizee-* sorted first then extras
    aizee = sorted(u for u in units if u.startswith("aizee-"))
    extra = [u for u in units if not u.startswith("aizee-")]
    return aizee + extra


def uptime_seconds():
    try:
        with open("/proc/uptime") as f:
            return float(f.read().split()[0])
    except Exception:
        return 0.0


def service_statuses(units):
    if not units:
        return []
    out = run(["systemctl", "show", "--property=" + ",".join(PROPS)] + units)
    blocks = [b for b in out.split("\n\n") if b.strip()]
    up = uptime_seconds()
    result = []
    for block in blocks:
        d = {}
        for line in block.splitlines():
            if "=" in line:
                k, _, v = line.partition("=")
                d[k] = v
        if not d.get("Id"):
            continue
        mono = d.get("ActiveEnterTimestampMonotonic", "0")
        try:
            since = up - (int(mono) / 1_000_000) if mono and mono != "0" else None
        except ValueError:
            since = None
        mem = d.get("MemoryCurrent", "")
        try:
            mem_bytes = int(mem)
        except ValueError:
            mem_bytes = None
        result.append({
            "name": d.get("Id"),
            "description": d.get("Description", ""),
            "load": d.get("LoadState", ""),
            "active": d.get("ActiveState", ""),       # active / inactive / failed / activating
            "sub": d.get("SubState", ""),             # running / dead / exited / failed
            "enabled": d.get("UnitFileState", ""),    # enabled / disabled / static
            "uptime_s": round(since) if since and since > 0 else None,
            "mem_bytes": mem_bytes,
            "pid": d.get("MainPID", "0"),
            "restarts": d.get("NRestarts", "0"),
        })
    return result


# ----------------------------------------------------------------------------- host metrics
def cpu_percent(interval=0.2):
    def snap():
        with open("/proc/stat") as f:
            parts = f.readline().split()[1:]
        vals = list(map(int, parts))
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return sum(vals), idle
    try:
        t1, i1 = snap()
        time.sleep(interval)
        t2, i2 = snap()
        dt, di = t2 - t1, i2 - i1
        return round(100 * (1 - di / dt), 1) if dt > 0 else 0.0
    except Exception:
        return None


def mem_info():
    info = {}
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                k, _, v = line.partition(":")
                info[k] = int(v.strip().split()[0]) * 1024  # kB -> bytes
        total = info.get("MemTotal", 0)
        avail = info.get("MemAvailable", 0)
        return {"total": total, "used": total - avail,
                "percent": round(100 * (total - avail) / total, 1) if total else 0}
    except Exception:
        return None


def thermals():
    zones = []
    base = "/sys/class/thermal"
    try:
        for name in sorted(os.listdir(base)):
            if not name.startswith("thermal_zone"):
                continue
            try:
                with open(f"{base}/{name}/type") as f:
                    ztype = f.read().strip()
                with open(f"{base}/{name}/temp") as f:
                    temp = int(f.read().strip()) / 1000.0
                if temp > 0:
                    zones.append({"name": ztype, "temp_c": round(temp, 1)})
            except Exception:
                continue
    except Exception:
        pass
    return zones


def disk_info():
    try:
        u = shutil.disk_usage("/")
        return {"total": u.total, "used": u.used,
                "percent": round(100 * u.used / u.total, 1)}
    except Exception:
        return None


def networks():
    out = run(["ip", "-o", "-4", "addr", "show"])
    ifaces = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == "inet":
            name, addr = parts[1], parts[3]
            if name != "lo":
                ifaces.append({"iface": name, "addr": addr})
    ap = None
    nm = run(["nmcli", "-t", "-f", "GENERAL.CONNECTION,GENERAL.STATE",
              "device", "show", "wlP1p1s0"])
    if nm:
        conn = ""
        for line in nm.splitlines():
            if line.startswith("GENERAL.CONNECTION:"):
                conn = line.split(":", 1)[1]
        stations = run(["iw", "dev", "wlP1p1s0", "station", "dump"]).count("Station ")
        ap = {"connection": conn, "clients": stations}
    return {"interfaces": ifaces, "ap": ap}


def host_metrics():
    with open("/proc/loadavg") as f:
        load = f.read().split()[:3]
    return {
        "hostname": os.uname().nodename,
        "uptime_s": round(uptime_seconds()),
        "cpu_percent": cpu_percent(),
        "cpu_count": os.cpu_count(),
        "loadavg": load,
        "mem": mem_info(),
        "disk": disk_info(),
        "thermals": thermals(),
        "network": networks(),
        "now": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


# ----------------------------------------------------------------------------- logs
def service_logs(unit, lines=80):
    if not UNIT_RE.match(unit):
        return "invalid unit name"
    lines = max(1, min(int(lines), 1000))
    out = run(["journalctl", "-u", unit, "-n", str(lines),
               "--no-pager", "-o", "short-iso"], timeout=12)
    return out or "(no log entries)"


# ----------------------------------------------------------------------------- robot telemetry (ZMQ, read-only)
# pyzmq + msgpack are present under the same /usr/bin/python3 that runs the
# ups/display nodes.  Import lazily so a host missing them still serves the
# host/service/log dashboard — telemetry just reports "unavailable".
try:
    import zmq
    import msgpack
    _ZMQ_OK = True
except Exception:  # pragma: no cover - depends on host env
    _ZMQ_OK = False

# 6S LiPo motor pack: ~3.3 V/cell empty .. 4.2 V/cell full (matches display_node).
MOTOR_FULL_V = 25.2
MOTOR_MIN_V = 19.8

# Preferred motor display order — Minerva bimanual (7-DoF per arm, ids 4..10).
# Each arm is its own motor_control instance; names disambiguate left/right.
MOTOR_ORDER = [
    "left_arm_j1", "left_arm_j2", "left_arm_j3", "left_arm_j4",
    "left_arm_j5", "left_arm_j6", "left_gripper",
    "right_arm_j1", "right_arm_j2", "right_arm_j3", "right_arm_j4",
    "right_arm_j5", "right_arm_j6", "right_gripper",
]

# A stream older than this (seconds) is treated as stale / offline.
MOTOR_STALE_S = 3.0
UPS_STALE_S = 15.0
CAM_STALE_S = 3.0


def _rnd(v, n):
    """round() that tolerates None / non-numeric and returns None on failure."""
    try:
        return round(float(v), n) if v is not None else None
    except (TypeError, ValueError):
        return None


class TelemetryPoller:
    """Background thread that subscribes to the robot's ZMQ telemetry streams.

    Holds the latest motor / UPS message and per-camera frame stats so the
    /api/status handler can render them.  All sockets connect to localhost; if
    a publisher isn't up the SUB socket simply stays quiet.  Strictly read-only
    — it never sends a command, so it's safe to run alongside live teleop.
    """

    def __init__(self, motor_eps, ups_ep, cam_eps):
        # motor_eps: list of (arm_label, endpoint) — one motor_control per arm.
        self.motor_eps = motor_eps
        self.ups_ep = ups_ep
        self.cam_eps = cam_eps  # list of (name, endpoint)
        self._lock = threading.Lock()
        self._motor = {label: None for label, _ in motor_eps}
        self._motor_t = {label: 0.0 for label, _ in motor_eps}
        self._ups = None
        self._ups_t = 0.0
        self._cams = {
            name: {"header": None, "t": 0.0, "arrivals": deque(maxlen=90)}
            for name, _ in cam_eps
        }

    def start(self):
        if not _ZMQ_OK:
            return
        threading.Thread(target=self._run, name="telem-poll", daemon=True).start()

    # -- socket loop --------------------------------------------------------
    def _run(self):
        ctx = zmq.Context.instance()

        def _sub(ep, hwm):
            s = ctx.socket(zmq.SUB)
            s.setsockopt(zmq.RCVHWM, hwm)
            s.setsockopt_string(zmq.SUBSCRIBE, "")
            s.connect(ep)
            return s

        motor_socks = {_sub(ep, 10): label for label, ep in self.motor_eps}
        ups = _sub(self.ups_ep, 10)
        cam_socks = {_sub(ep, 4): name for name, ep in self.cam_eps}

        poller = zmq.Poller()
        for s in [*motor_socks, ups, *cam_socks]:
            poller.register(s, zmq.POLLIN)

        while True:
            try:
                socks = dict(poller.poll(timeout=250))
            except Exception:
                time.sleep(0.5)
                continue
            now = time.monotonic()
            for s, label in motor_socks.items():
                if socks.get(s) == zmq.POLLIN:
                    latest = self._drain_msg(s)
                    if latest is not None:
                        with self._lock:
                            self._motor[label] = latest
                            self._motor_t[label] = time.time()
            if socks.get(ups) == zmq.POLLIN:
                latest = self._drain_msg(ups)
                if latest is not None:
                    with self._lock:
                        self._ups, self._ups_t = latest, time.time()
            for s, name in cam_socks.items():
                if socks.get(s) == zmq.POLLIN:
                    hdr = self._drain_cam(s)
                    if hdr is not None:
                        with self._lock:
                            c = self._cams[name]
                            c["header"], c["t"] = hdr, time.time()
                            c["arrivals"].append(now)

    @staticmethod
    def _drain_msg(sock):
        """Drain a SUB socket to the newest msgpack message ('latest wins')."""
        latest = None
        while True:
            try:
                latest = msgpack.unpackb(sock.recv(zmq.NOBLOCK), raw=False)
            except zmq.Again:
                break
            except Exception:
                break
        return latest

    @staticmethod
    def _drain_cam(sock):
        """Drain a camera SUB socket; decode only the msgpack header (frame[0]),
        discarding the JPEG/depth payload frames we don't need."""
        hdr = None
        while True:
            try:
                parts = sock.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception:
                break
            if parts:
                try:
                    hdr = msgpack.unpackb(parts[0], raw=False)
                except Exception:
                    pass
        return hdr

    # -- snapshot for the API ----------------------------------------------
    def snapshot(self):
        if not _ZMQ_OK:
            return {"available": False, "reason": "pyzmq/msgpack not installed"}

        now_wall, now_mono = time.time(), time.monotonic()
        with self._lock:
            motor = dict(self._motor)
            motor_t = dict(self._motor_t)
            ups, ups_t = self._ups, self._ups_t
            cams_raw = {
                n: {"header": c["header"], "t": c["t"],
                    "arrivals": list(c["arrivals"])}
                for n, c in self._cams.items()
            }

        # --- motors (one motor_control instance per arm; merge + per-arm status) ---
        def _key(name):
            idx = MOTOR_ORDER.index(name) if name in MOTOR_ORDER else len(MOTOR_ORDER)
            return (idx, name)

        motors_out, arms_out = [], []
        estops, batt_vs, all_stale = [], [], True
        for label, _ep in self.motor_eps:
            msg, t = motor.get(label), motor_t.get(label, 0.0)
            age = (now_wall - t) if t else None
            stale = age is None or age > MOTOR_STALE_S
            running = present = 0
            if msg and not stale:
                all_stale = False
                estops.append(bool(msg.get("emergency_stop", False)))
                bv = _rnd(msg.get("battery_voltage"), 2)
                if bv is not None:
                    batt_vs.append(bv)
                mdict = msg.get("motors", {}) or {}
                for name in sorted(mdict.keys(), key=_key):
                    md = mdict[name] or {}
                    present += 1
                    if md.get("state") == "running":
                        running += 1
                    motors_out.append({
                        "arm": label,
                        "name": name,
                        "state": md.get("state", ""),
                        "mode": md.get("mode", ""),
                        "position": _rnd(md.get("position"), 3),
                        "velocity": _rnd(md.get("velocity"), 3),
                        "torque": _rnd(md.get("torque"), 3),
                        "temperature": _rnd(md.get("temperature"), 1),
                        "error": md.get("error"),
                    })
            arms_out.append({"arm": label, "stale": stale, "age_s": _rnd(age, 1),
                             "present": present, "running": running})
        estop = None if not estops else any(estops)
        batt_v = batt_vs[0] if batt_vs else None
        m_stale = all_stale
        _ages = [a["age_s"] for a in arms_out if a["age_s"] is not None]
        m_age = min(_ages) if _ages else None
        batt_pct = None
        if batt_v is not None:
            batt_pct = int(max(0, min(100,
                (batt_v - MOTOR_MIN_V) / (MOTOR_FULL_V - MOTOR_MIN_V) * 100)))

        # --- UPS ---
        u_age = (now_wall - ups_t) if ups_t else None
        u_stale = u_age is None or u_age > UPS_STALE_S
        ups_out = {"stale": u_stale,
                   "age_s": _rnd(u_age, 1),
                   "voltage": None, "current": None,
                   "power": None, "percentage": None}
        if ups and not u_stale:
            ud = ups.get("ups", {}) or {}
            ups_out["voltage"] = _rnd(ud.get("voltage"), 2)
            ups_out["current"] = _rnd(ud.get("current"), 2)
            ups_out["power"] = _rnd(ud.get("power"), 2)
            ups_out["percentage"] = _rnd(ud.get("percentage"), 0)

        # --- cameras ---
        cams_out = []
        for name, _ep in self.cam_eps:
            c = cams_raw.get(name, {})
            age = (now_wall - c["t"]) if c.get("t") else None
            online = age is not None and age <= CAM_STALE_S
            fps = None
            recent = [a for a in c.get("arrivals", []) if now_mono - a <= 3.0]
            if online and len(recent) >= 2:
                span = recent[-1] - recent[0]
                if span > 0:
                    fps = round((len(recent) - 1) / span, 1)
            hdr = c.get("header") or {}
            color = hdr.get("color") or {}
            cams_out.append({
                "name": name,
                "online": online,
                "age_s": _rnd(age, 1),
                "fps": fps,
                "width": color.get("width"),
                "height": color.get("height"),
                "frame_number": hdr.get("frame_number"),
            })

        return {
            "available": True,
            "estop": estop,
            "motors": {
                "stale": m_stale,
                "age_s": _rnd(m_age, 1),
                "battery_voltage": batt_v,
                "battery_percent": batt_pct,
                "arms": arms_out,
                "list": motors_out,
            },
            "ups": ups_out,
            "cameras": cams_out,
        }


# ----------------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    poller: "TelemetryPoller | None" = None  # set in main()

    def log_message(self, *a):  # silence default stderr access log
        pass

    def _send(self, code, body, ctype):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        try:
            if u.path == "/" or u.path == "/index.html":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif u.path == "/api/status":
                telem = (self.poller.snapshot() if self.poller is not None
                         else {"available": False, "reason": "telemetry disabled"})
                payload = {"host": host_metrics(),
                           "services": service_statuses(discover_units()),
                           "telemetry": telem}
                self._send(200, json.dumps(payload), "application/json")
            elif u.path == "/api/logs":
                q = parse_qs(u.query)
                unit = q.get("unit", [""])[0]
                lines = q.get("lines", ["80"])[0]
                self._send(200, json.dumps({"unit": unit,
                           "log": service_logs(unit, lines)}), "application/json")
            elif u.path == "/setup":
                self._send(200, SETUP_PAGE, "text/html; charset=utf-8")
            elif u.path == "/api/checks":
                # setup_checks.py sits next to this file; the script dir is on
                # sys.path when run as `python3 .../heartbeat_server.py`.
                import setup_checks
                snap = self.poller.snapshot() if self.poller is not None else None
                self._send(200, json.dumps(setup_checks.run_all(snap)),
                           "application/json")
            elif u.path == "/healthz":
                self._send(200, "ok", "text/plain")
            else:
                self._send(404, "not found", "text/plain")
        except BrokenPipeError:
            pass
        except Exception as e:
            self._send(500, json.dumps({"error": str(e)}), "application/json")


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Minerva Heartbeat</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --border:#30363d; --txt:#c9d1d9;
          --muted:#8b949e; --green:#3fb950; --red:#f85149; --yellow:#d29922;
          --grey:#6e7681; --blue:#58a6ff; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--txt);
         font:14px/1.5 -apple-system,Segoe UI,Roboto,Helvetica,Arial,monospace; }
  header { display:flex; align-items:center; gap:16px; flex-wrap:wrap;
           padding:14px 20px; border-bottom:1px solid var(--border); background:var(--panel); }
  header h1 { font-size:18px; margin:0; color:#fff; }
  header .host { color:var(--blue); font-weight:600; }
  header .spacer { flex:1; }
  .pill { background:#21262d; border:1px solid var(--border); border-radius:20px;
          padding:3px 10px; font-size:12px; color:var(--muted); }
  label.toggle { font-size:12px; color:var(--muted); cursor:pointer; user-select:none; }
  main { padding:20px; max-width:1400px; margin:0 auto; }
  .metrics { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
             gap:12px; margin-bottom:22px; }
  .metric { background:var(--panel); border:1px solid var(--border); border-radius:8px; padding:12px 14px; }
  .metric .k { font-size:11px; text-transform:uppercase; letter-spacing:.5px; color:var(--muted); }
  .metric .v { font-size:22px; font-weight:600; color:#fff; margin-top:4px; }
  .metric .sub { font-size:11px; color:var(--muted); margin-top:2px; }
  .bar { height:5px; background:#21262d; border-radius:3px; margin-top:8px; overflow:hidden; }
  .bar > i { display:block; height:100%; background:var(--green); }
  h2 { font-size:13px; text-transform:uppercase; letter-spacing:.5px; color:var(--muted);
       margin:0 0 10px; }
  table { width:100%; border-collapse:collapse; background:var(--panel);
          border:1px solid var(--border); border-radius:8px; overflow:hidden; }
  th,td { text-align:left; padding:9px 12px; border-bottom:1px solid var(--border); font-size:13px; }
  th { color:var(--muted); font-weight:600; font-size:11px; text-transform:uppercase; }
  tr:last-child td { border-bottom:none; }
  tr.svc { cursor:pointer; }
  tr.svc:hover { background:#1c2330; }
  tr.sel { background:#1f2937; }
  .badge { display:inline-block; padding:2px 9px; border-radius:12px; font-size:11px;
           font-weight:600; }
  .b-active { background:rgba(63,185,80,.15); color:var(--green); }
  .b-failed { background:rgba(248,81,73,.15); color:var(--red); }
  .b-inactive { background:rgba(110,118,129,.18); color:var(--grey); }
  .b-activating { background:rgba(210,153,34,.15); color:var(--yellow); }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:7px; vertical-align:middle; }
  .mono { font-family:ui-monospace,SFMono-Regular,Consolas,monospace; }
  .muted { color:var(--muted); }
  #logwrap { margin-top:24px; }
  #logbar { display:flex; align-items:center; gap:12px; margin-bottom:8px; }
  #logbar select, #logbar button { background:#21262d; color:var(--txt);
       border:1px solid var(--border); border-radius:6px; padding:5px 10px; font-size:13px; }
  pre#log { background:#010409; border:1px solid var(--border); border-radius:8px;
       padding:14px; max-height:460px; overflow:auto; font-size:12px; line-height:1.5;
       white-space:pre-wrap; word-break:break-word; margin:0; }
  .net { color:var(--muted); font-size:12px; }
  .err { color:var(--red); }
  /* --- robot telemetry --- */
  .hero { display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr));
          gap:12px; margin-bottom:24px; }
  .tile { position:relative; overflow:hidden; background:var(--panel);
          border:1px solid var(--border); border-radius:10px; padding:15px 16px 14px 18px; }
  .tile .accent { position:absolute; left:0; top:0; bottom:0; width:4px; background:var(--grey); }
  .tile.ok .accent { background:var(--green); } .tile.ok { border-color:rgba(63,185,80,.45); }
  .tile.warn .accent { background:var(--yellow); } .tile.warn { border-color:rgba(210,153,34,.5); }
  .tile.bad .accent { background:var(--red); } .tile.bad { border-color:rgba(248,81,73,.55); }
  .tile .tk { font-size:11px; text-transform:uppercase; letter-spacing:.5px; color:var(--muted); }
  .tile .tv { font-size:27px; font-weight:700; color:#fff; margin-top:6px; line-height:1.1; }
  .tile .ts { font-size:12px; color:var(--muted); margin-top:5px; }
  .cards { display:grid; grid-template-columns:repeat(auto-fit,minmax(230px,1fr));
           gap:12px; margin-bottom:22px; }
  .card { background:var(--panel); border:1px solid var(--border); border-radius:8px; padding:14px; }
  .card h3 { margin:0 0 10px; font-size:13px; color:#fff; font-weight:600;
             display:flex; align-items:center; gap:8px; }
  .bigpct { font-size:30px; font-weight:700; margin-bottom:2px; }
  .kv { display:flex; justify-content:space-between; gap:10px; font-size:12px; padding:3px 0; }
  .kv .vk { color:var(--muted); } .kv .vv { color:var(--txt); font-variant-numeric:tabular-nums; }
  .stale-tag { font-size:10px; font-weight:600; color:var(--yellow);
               border:1px solid rgba(210,153,34,.45); border-radius:10px; padding:1px 7px; }
  td.num { text-align:right; font-variant-numeric:tabular-nums; }
  th.num { text-align:right; }
  .empty { color:var(--muted); padding:14px; background:var(--panel);
           border:1px solid var(--border); border-radius:8px; font-size:13px; }
</style>
</head>
<body>
<header>
  <h1>Minerva <span style="color:var(--green)">&#9829;</span> Heartbeat</h1>
  <span class="host" id="host">&hellip;</span>
  <span class="pill" id="uptime"></span>
  <span class="pill" id="clock"></span>
  <span class="spacer"></span>
  <a href="/setup" style="color:var(--blue);font-size:12px;text-decoration:none">Setup wizard &rarr;</a>
  <span class="pill" id="apinfo"></span>
  <label class="toggle"><input type="checkbox" id="auto" checked> auto-refresh 3s</label>
  <button onclick="refresh()" style="background:#21262d;color:var(--txt);border:1px solid var(--border);border-radius:6px;padding:5px 12px;cursor:pointer">Refresh</button>
</header>
<main>
  <div class="hero" id="hero"></div>
  <h2>Motors</h2>
  <div id="motorwrap"></div>
  <h2 style="margin-top:22px">Batteries</h2>
  <div class="cards" id="batteries"></div>
  <h2>Cameras</h2>
  <div class="cards" id="cameras"></div>
  <h2>Host</h2>
  <div class="metrics" id="metrics"></div>
  <h2>Services</h2>
  <table>
    <thead><tr><th>Service</th><th>State</th><th>Enabled</th><th>Uptime</th><th>Mem</th><th>PID</th><th>Restarts</th></tr></thead>
    <tbody id="svcs"></tbody>
  </table>
  <div id="logwrap">
    <div id="logbar">
      <h2 style="margin:0">Logs</h2>
      <span class="muted" id="logunit">&mdash; select a service &mdash;</span>
      <span class="spacer" style="flex:1"></span>
      <select id="lines">
        <option>50</option><option selected>100</option><option>250</option><option>500</option>
      </select>
      <button onclick="loadLog()">Reload</button>
      <label class="toggle"><input type="checkbox" id="logfollow"> follow</label>
    </div>
    <pre id="log" class="muted">Select a service above to view its journald logs.</pre>
  </div>
</main>
<script>
let selected = null;
const fmtBytes = b => { if(b==null) return "&mdash;"; const u=["B","KB","MB","GB","TB"]; let i=0;
  while(b>=1024 && i<u.length-1){b/=1024;i++;} return b.toFixed(b<10&&i>0?1:0)+u[i]; };
const fmtDur = s => { if(s==null) return "&mdash;"; s=Math.floor(s);
  const d=Math.floor(s/86400); s%=86400; const h=Math.floor(s/3600); s%=3600;
  const m=Math.floor(s/60); if(d) return d+"d "+h+"h"; if(h) return h+"h "+m+"m";
  if(m) return m+"m"; return s%60+"s"; };

function badge(active, sub){
  let cls="b-inactive", txt=active;
  if(active==="active"){cls="b-active"; txt=sub==="running"?"running":sub;}
  else if(active==="failed"){cls="b-failed"; txt="failed";}
  else if(active==="activating"||active==="deactivating"){cls="b-activating";}
  else {cls="b-inactive"; txt=sub||active;}
  return '<span class="badge '+cls+'">'+txt+'</span>';
}
function metricCard(k,v,sub,pct,color){
  let bar = pct!=null ? '<div class="bar"><i style="width:'+Math.min(pct,100)+'%;background:'+
      (pct>90?'var(--red)':pct>70?'var(--yellow)':'var(--green)')+'"></i></div>':'';
  return '<div class="metric"><div class="k">'+k+'</div><div class="v">'+v+'</div>'+
      '<div class="sub">'+(sub||"")+'</div>'+bar+'</div>';
}

// ---- robot telemetry rendering ----
const fmtNum = v => (v==null||v==="")?"&mdash;":(typeof v==="number"?v.toFixed(3):v);
const kv = (k,v) => '<div class="kv"><span class="vk">'+k+'</span><span class="vv">'+v+'</span></div>';
const camTitle = n => n.replace(/_/g," ").replace(/\b\w/g,c=>c.toUpperCase())+" Cam";
function tile(k,v,sub,cls){
  return '<div class="tile '+(cls||"")+'"><span class="accent"></span>'+
    '<div class="tk">'+k+'</div><div class="tv">'+v+'</div>'+
    '<div class="ts">'+(sub||"")+'</div></div>';
}
function battClass(p){ return p==null?"":(p<=20?"bad":p<=40?"warn":"ok"); }
function battColor(p){ return p==null?"var(--grey)":(p<=20?"var(--red)":p<=40?"var(--yellow)":"var(--green)"); }
function tempColor(t){ return t==null?"var(--txt)":(t>=75?"var(--red)":t>=60?"var(--yellow)":"var(--txt)"); }
function motorBadge(state){
  let cls="b-inactive";
  if(state==="running") cls="b-active";
  else if(state==="error") cls="b-failed";
  else if(state==="enabling"||state==="enabled") cls="b-activating";
  return '<span class="badge '+cls+'">'+(state||"&mdash;")+'</span>';
}
function battCard(title, pct, rows, stale){
  const head='<h3>'+title+(stale?' <span class="stale-tag">stale</span>':'')+'</h3>';
  const col=battColor(stale?null:pct);
  const big='<div class="bigpct" style="color:'+col+'">'+(pct==null?"&mdash;":pct+"%")+'</div>';
  const bar=pct==null?"":'<div class="bar" style="margin:8px 0 10px"><i style="width:'+
      Math.min(pct,100)+'%;background:'+col+'"></i></div>';
  return '<div class="card">'+head+big+bar+rows.map(r=>kv(r[0],r[1])).join("")+'</div>';
}
function renderTelemetry(t){
  const hero=document.getElementById("hero"), motorwrap=document.getElementById("motorwrap"),
        batt=document.getElementById("batteries"), cams=document.getElementById("cameras");
  if(!t || !t.available){
    hero.innerHTML=tile("Robot Telemetry","offline", (t&&t.reason)||"no ZMQ feed","bad");
    motorwrap.innerHTML=""; batt.innerHTML=""; cams.innerHTML="";
    return;
  }
  const ms=t.motors||{}, list=ms.list||[], u=t.ups||{}, cl=t.cameras||[];
  const running=list.filter(m=>m.state==="running").length;
  const faulted=list.filter(m=>m.state==="error"||m.error).length;
  const mp=ms.battery_percent, upct=u.percentage;
  const online=cl.filter(c=>c.online).length;

  // hero tiles — the at-a-glance row
  const tiles=[];
  if(t.estop===true) tiles.push(tile("E-Stop","ENGAGED","emergency stop active","bad"));
  else if(t.estop===false) tiles.push(tile("E-Stop","CLEAR","motors armed","ok"));
  else tiles.push(tile("E-Stop","&mdash;","motor telemetry stale","warn"));
  const arms=ms.arms||[];
  const armSub=arms.map(a=>a.arm.charAt(0).toUpperCase()+" "+(a.stale?"stale":a.running+"/"+a.present)).join(" · ");
  tiles.push(tile("Motors", ms.stale?"stale":(running+"/"+list.length),
      armSub||(faulted?faulted+" faulted":"none"),
      ms.stale?"warn":(faulted?"bad":(running?"ok":""))));
  tiles.push(tile("Motor Battery", mp==null?"&mdash;":mp+"%",
      ms.battery_voltage==null?"motor bus":(ms.battery_voltage+" V"),
      ms.stale?"warn":battClass(mp)));
  tiles.push(tile("UPS Battery", (u.stale||upct==null)?"&mdash;":Math.round(upct)+"%",
      u.voltage==null?"logic UPS":(u.voltage+" V"),
      u.stale?"warn":battClass(upct)));
  tiles.push(tile("Cameras", online+"/"+cl.length,
      cl.map(c=>c.name+(c.online?"":" ✕")).join(" · ")||"none",
      cl.length&&online===cl.length?"ok":(online?"warn":"bad")));
  hero.innerHTML=tiles.join("");

  // motor detail table, grouped by arm (one motor_control instance each)
  if(list.length || arms.length){
    let rows="";
    (arms.length?arms:[{arm:null,stale:ms.stale}]).forEach(a=>{
      const mine=list.filter(m=>a.arm==null||m.arm===a.arm);
      if(a.arm!=null){
        const st=a.stale
          ? '<span class="stale-tag">stale / offline</span>'
          : '<span class="muted">'+a.running+"/"+a.present+" running"+(a.age_s!=null?" &middot; "+a.age_s+"s":"")+'</span>';
        rows+='<tr><td colspan="8" style="background:#1c2330;font-weight:600;letter-spacing:.5px">'+
          a.arm.toUpperCase()+' ARM &nbsp; '+st+'</td></tr>';
      }
      rows+=mine.map(m=>'<tr>'+
        '<td><span class="mono">'+m.name+'</span></td>'+
        '<td>'+motorBadge(m.state)+'</td>'+
        '<td class="num">'+fmtNum(m.position)+'</td>'+
        '<td class="num">'+fmtNum(m.velocity)+'</td>'+
        '<td class="num">'+fmtNum(m.torque)+'</td>'+
        '<td class="num" style="color:'+tempColor(m.temperature)+'">'+
          (m.temperature==null?"&mdash;":m.temperature+"&deg;")+'</td>'+
        '<td class="muted">'+(m.mode||"&mdash;")+'</td>'+
        '<td class="'+(m.error?"err":"muted")+'">'+(m.error||"&mdash;")+'</td></tr>').join("");
      if(a.arm!=null && !mine.length){
        rows+='<tr><td colspan="8" class="muted">'+
          (a.stale?"no telemetry — is aizee-minerva-"+a.arm+" running and its bus up?":"no motors reporting")+
          '</td></tr>';
      }
    });
    motorwrap.innerHTML='<table><thead><tr><th>Motor</th><th>State</th>'+
      '<th class="num">Pos (rad)</th><th class="num">Vel</th><th class="num">Torque</th>'+
      '<th class="num">Temp</th><th>Mode</th><th>Error</th></tr></thead><tbody>'+rows+'</tbody></table>';
  } else {
    motorwrap.innerHTML='<div class="empty">'+
      (ms.stale?"Both arms stale or offline.":"No motors reporting.")+'</div>';
  }

  // battery cards
  batt.innerHTML=
    battCard("Motor Pack", mp,
      [["Voltage", ms.battery_voltage==null?"&mdash;":ms.battery_voltage+" V"]], ms.stale)+
    battCard("Logic UPS", upct==null?null:Math.round(upct),
      [["Voltage", u.voltage==null?"&mdash;":u.voltage+" V"],
       ["Current", u.current==null?"&mdash;":u.current+" A"],
       ["Power",   u.power==null?"&mdash;":u.power+" W"]], u.stale);

  // camera cards
  cams.innerHTML = cl.length ? cl.map(c=>{
    const dot='<span class="dot" style="background:'+(c.online?"var(--green)":"var(--red)")+'"></span>';
    const res=(c.width&&c.height)?(c.width+"&times;"+c.height):"&mdash;";
    return '<div class="card"><h3>'+dot+camTitle(c.name)+'</h3>'+
      kv("Status", c.online?"online":"offline")+
      kv("FPS", c.fps==null?"&mdash;":c.fps)+
      kv("Resolution", res)+
      kv("Frame #", c.frame_number==null?"&mdash;":c.frame_number)+
      kv("Last frame", c.age_s==null?"never":c.age_s+"s ago")+
      '</div>';
  }).join("") : '<div class="empty">No cameras configured.</div>';
}

async function refresh(){
  let d;
  try { d = await (await fetch("/api/status")).json(); }
  catch(e){ document.getElementById("host").innerHTML='<span class="err">unreachable</span>'; return; }
  renderTelemetry(d.telemetry);
  const h=d.host;
  document.getElementById("host").textContent=h.hostname;
  document.getElementById("uptime").textContent="up "+fmtDur(h.uptime_s);
  document.getElementById("clock").textContent=h.now;
  if(h.network && h.network.ap){
    document.getElementById("apinfo").textContent="AP: "+(h.network.ap.connection||"-")+
        " ("+h.network.ap.clients+" client"+(h.network.ap.clients==1?"":"s")+")";
  }
  // metrics
  const m=[];
  m.push(metricCard("CPU", (h.cpu_percent==null?"&mdash;":h.cpu_percent+"%"),
      h.cpu_count+" cores &middot; load "+(h.loadavg?h.loadavg.join(" "):""), h.cpu_percent));
  if(h.mem) m.push(metricCard("Memory", h.mem.percent+"%",
      fmtBytes(h.mem.used)+" / "+fmtBytes(h.mem.total), h.mem.percent));
  if(h.disk) m.push(metricCard("Disk /", h.disk.percent+"%",
      fmtBytes(h.disk.used)+" / "+fmtBytes(h.disk.total), h.disk.percent));
  (h.thermals||[]).forEach(z=>m.push(metricCard(z.name, z.temp_c+"&deg;C","",
      Math.min(z.temp_c,100))));
  if(h.network){ const nets=h.network.interfaces.map(i=>i.iface+" "+i.addr).join("<br>");
      m.push(metricCard("Network", '<span style="font-size:13px" class="mono">'+
        (nets||"&mdash;")+'</span>', "")); }
  document.getElementById("metrics").innerHTML=m.join("");
  // services
  const rows=d.services.map(s=>{
    const sel = s.name===selected ? " sel":"";
    return '<tr class="svc'+sel+'" data-u="'+s.name+'">'+
      '<td><span class="mono">'+s.name.replace(".service","")+'</span><div class="muted" style="font-size:11px">'+
        (s.description||"")+'</div></td>'+
      '<td>'+badge(s.active,s.sub)+'</td>'+
      '<td class="muted">'+s.enabled+'</td>'+
      '<td>'+fmtDur(s.uptime_s)+'</td>'+
      '<td>'+fmtBytes(s.mem_bytes)+'</td>'+
      '<td class="muted">'+(s.pid!=="0"?s.pid:"&mdash;")+'</td>'+
      '<td class="'+(s.restarts!=="0"?"err":"muted")+'">'+s.restarts+'</td></tr>';
  });
  document.getElementById("svcs").innerHTML=rows.join("");
  document.querySelectorAll("tr.svc").forEach(tr=>tr.onclick=()=>{
    selected=tr.dataset.u; document.getElementById("logunit").textContent=selected;
    document.querySelectorAll("tr.svc").forEach(x=>x.classList.remove("sel"));
    tr.classList.add("sel"); loadLog();
  });
  if(document.getElementById("logfollow").checked && selected) loadLog();
}

async function loadLog(){
  if(!selected) return;
  const n=document.getElementById("lines").value;
  const log=document.getElementById("log");
  try {
    const d=await (await fetch("/api/logs?unit="+encodeURIComponent(selected)+"&lines="+n)).json();
    log.classList.remove("muted"); log.textContent=d.log;
    if(document.getElementById("logfollow").checked) log.scrollTop=log.scrollHeight;
  } catch(e){ log.textContent="failed to load log: "+e; }
}

let timer=null;
function loop(){ if(document.getElementById("auto").checked) refresh(); }
document.getElementById("auto").addEventListener("change",()=>{});
refresh();
setInterval(loop,3000);
</script>
</body>
</html>"""


SETUP_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Minerva Setup Wizard</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --border:#30363d; --txt:#c9d1d9;
          --muted:#8b949e; --green:#3fb950; --red:#f85149; --yellow:#d29922;
          --grey:#6e7681; --blue:#58a6ff; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--txt);
         font:14px/1.55 -apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif; }
  header { display:flex; align-items:center; gap:16px; padding:14px 20px;
           border-bottom:1px solid var(--border); background:var(--panel); }
  header h1 { font-size:18px; margin:0; color:#fff; }
  header a { color:var(--blue); font-size:12px; text-decoration:none; margin-left:auto; }
  main { padding:20px; max-width:940px; margin:0 auto; }
  .step { background:var(--panel); border:1px solid var(--border); border-radius:10px;
          margin-bottom:14px; overflow:hidden; }
  .step > summary { padding:13px 16px; cursor:pointer; font-weight:600; color:#fff;
          display:flex; align-items:center; gap:10px; list-style:none; }
  .step > summary::-webkit-details-marker { display:none; }
  .step .n { display:inline-flex; align-items:center; justify-content:center;
          width:24px; height:24px; border-radius:50%; background:#21262d;
          border:1px solid var(--border); font-size:12px; color:var(--blue); flex:none; }
  .step .body { padding:2px 18px 16px 50px; color:var(--txt); font-size:13.5px; }
  .step .body p { margin:8px 0; }
  code, pre { font-family:ui-monospace,SFMono-Regular,Consolas,monospace; font-size:12.5px; }
  code { background:#21262d; border:1px solid var(--border); border-radius:4px; padding:1px 6px; }
  pre { background:#010409; border:1px solid var(--border); border-radius:8px;
        padding:12px 14px; overflow-x:auto; margin:8px 0; }
  .muted { color:var(--muted); }
  .tag { font-size:10px; font-weight:700; border-radius:10px; padding:1px 8px;
         text-transform:uppercase; letter-spacing:.5px; }
  .tag.host { background:rgba(88,166,255,.15); color:var(--blue); }
  .tag.robot { background:rgba(63,185,80,.15); color:var(--green); }
  .tag.hands { background:rgba(210,153,34,.15); color:var(--yellow); }
  /* --- checks --- */
  #checkbar { display:flex; align-items:center; gap:12px; margin:6px 0 12px; }
  #checkbar button { background:#238636; color:#fff; border:0; border-radius:6px;
        padding:8px 18px; font-size:14px; font-weight:600; cursor:pointer; }
  #checkbar button:disabled { opacity:.6; cursor:wait; }
  #summary { font-size:13px; color:var(--muted); }
  .grp { margin:14px 0 4px; font-size:12px; text-transform:uppercase;
         letter-spacing:.6px; color:var(--muted); font-weight:600; }
  .chk { display:flex; gap:10px; padding:8px 10px; border-bottom:1px solid var(--border);
         align-items:baseline; }
  .chk:last-child { border-bottom:none; }
  .chk .ic { width:18px; flex:none; text-align:center; font-weight:700; }
  .chk.pass .ic { color:var(--green); } .chk.fail .ic { color:var(--red); }
  .chk.warn .ic { color:var(--yellow); } .chk.skip .ic { color:var(--grey); }
  .chk .t { min-width:230px; font-weight:600; color:#fff; }
  .chk .d { color:var(--muted); }
  .chk .hint { display:block; color:var(--yellow); font-size:12px; margin-top:2px; }
  .chkwrap { background:var(--panel); border:1px solid var(--border); border-radius:10px;
             padding:4px 8px; }
  .bignum { font-weight:700; }
  .ok-banner { border:1px solid rgba(63,185,80,.5); background:rgba(63,185,80,.08);
       color:var(--green); border-radius:10px; padding:12px 16px; font-weight:600;
       margin-bottom:14px; display:none; }
</style>
</head>
<body>
<header>
  <h1>Minerva Setup Wizard</h1>
  <a href="/">&larr; Heartbeat dashboard</a>
</header>
<main>
  <p class="muted">Bring up a fresh Jetson Orin Nano and validate the whole robot, step by
  step. Tags: <span class="tag hands">hands</span> physical action,
  <span class="tag host">dev&nbsp;pc</span> run on your computer,
  <span class="tag robot">jetson</span> runs here on the robot.</p>

  <details class="step" open><summary><span class="n">1</span> Flash JetPack <span class="tag hands">hands</span></summary>
  <div class="body">
    <p>Flash JetPack 6.x onto the Orin Nano (SD image or SDK Manager). During first-boot
    <b>oem-config</b>, create user <code>ltr</code> and connect it to your WiFi or plug in
    ethernet — the bootstrap needs internet on the device for apt/rustup/pip.</p>
    <p class="muted">USB-C to your PC also gives a fallback link at <code>192.168.55.1</code>
    (JetPack device-mode).</p>
  </div></details>

  <details class="step" open><summary><span class="n">2</span> Bootstrap from the dev machine <span class="tag host">dev&nbsp;pc</span></summary>
  <div class="body">
    <p>From the repo root in git-bash — installs your SSH key, syncs the repo, then runs
    the on-device setup (apt, Rust, python deps, udev, systemd, CAN helper, cargo build,
    WiFi AP):</p>
    <pre>./scripts/bootstrap_jetson.sh ltr@192.168.55.1 -- --ap-pass '&lt;wifi-ap-password&gt;' --hostname aizee-jetson</pre>
    <p>Re-running is safe — the setup script is idempotent. To refresh the arm stack later
    without the full setup: <code>./scripts/deploy_minerva_arms.sh</code>.</p>
  </div></details>

  <details class="step"><summary><span class="n">3</span> Wire up the hardware <span class="tag hands">hands</span></summary>
  <div class="body">
    <p><b>CAN (two buses):</b> each arm has its own USB-CAN adapter (physical power switch
    on) — LEFT arm on <code>can1</code>, RIGHT arm on <code>can2</code> (the internal
    mttcan owns the unused <code>can0</code>). Motor ids 4..10 (shoulder&rarr;gripper) on
    each bus, motor pack on. Every motor in
    <code>config/hardware_minerva_{left,right}.yaml</code> must be present or that arm's
    motor_control wedges during init — start the arm services only with the bus powered.</p>
    <p><b>USB:</b> wrist cams (ELP, optional), head RealSense (optional). The two OpenRB
    GELLO leaders stay on the dev PC.</p>
    <p><b>UPS:</b> INA219 on i2c bus 7 (0x41).</p>
    <p class="muted">Start the arms (bus powered):
    <code>sudo systemctl start aizee-minerva-left aizee-minerva-right</code></p>
  </div></details>

  <details class="step" open><summary><span class="n">4</span> Validate <span class="tag robot">jetson</span></summary>
  <div class="body">
    <div class="ok-banner" id="okbanner">All required checks pass — the robot is good to go.</div>
    <div id="checkbar">
      <button id="runbtn" onclick="runChecks()">Run all checks</button>
      <span id="summary"></span>
    </div>
    <div id="checks" class="muted">Checks haven&rsquo;t run yet.</div>
    <p class="muted" style="margin-top:10px">Same checks over SSH:
    <code>python3 python/tools/setup_checks.py</code> (exit code 1 on failure).</p>
  </div></details>

  <details class="step"><summary><span class="n">5</span> Teleop smoke test <span class="tag host">dev&nbsp;pc</span></summary>
  <div class="body">
    <p>With everything green above, plug the two OpenRB GELLO leaders into the dev PC and
    run the Minerva collector (or the terminal teleop) against the robot:</p>
    <pre>python python/scripts/collect_minerva.py --host 10.42.0.1</pre>
    <p>Register the leaders with <code>Z</code> (leader-zero) or <code>M</code> (mirror),
    <code>E</code> to track, <code>K</code> for RobStride mechanical zero (disable first),
    <code>R</code> to record, <code>Q</code> to quit. Details in
    <code>docs/MINERVA_BRINGUP.md</code>.</p>
  </div></details>
</main>
<script>
const icons = {pass:"✓", warn:"!", fail:"✗", skip:"–"};
async function runChecks(){
  const btn=document.getElementById("runbtn"), box=document.getElementById("checks"),
        sum=document.getElementById("summary");
  btn.disabled=true; sum.textContent="running… (a few seconds)";
  try{
    const d=await (await fetch("/api/checks")).json();
    const groups={};
    d.checks.forEach(c=>{(groups[c.group]=groups[c.group]||[]).push(c);});
    let html="";
    for(const g in groups){
      html+='<div class="grp">'+g+'</div><div class="chkwrap">';
      html+=groups[g].map(c=>'<div class="chk '+c.status+'"><span class="ic">'+icons[c.status]+
        '</span><span class="t">'+c.title+'</span><span class="d">'+(c.detail||c.status)+
        ((c.status==="fail"||c.status==="warn")&&c.hint?'<span class="hint">&rarr; '+c.hint+'</span>':"")+
        '</span></div>').join("");
      html+='</div>';
    }
    box.classList.remove("muted"); box.innerHTML=html;
    const n=d.counts;
    sum.innerHTML='<span class="bignum" style="color:var(--green)">'+n.pass+' pass</span> · '+
      '<span class="bignum" style="color:var(--yellow)">'+n.warn+' warn</span> · '+
      '<span class="bignum" style="color:var(--red)">'+n.fail+' fail</span> · '+d.generated;
    document.getElementById("okbanner").style.display = n.fail===0 ? "block":"none";
  } catch(e){ sum.textContent="check run failed: "+e; }
  btn.disabled=false;
}
runChecks();
</script>
</body>
</html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--left-telem", default="tcp://localhost:5556",
                    help="ZMQ motor telemetry — LEFT arm (aizee-minerva-left)")
    ap.add_argument("--right-telem", default="tcp://localhost:5576",
                    help="ZMQ motor telemetry — RIGHT arm (aizee-minerva-right)")
    ap.add_argument("--ups", default="tcp://localhost:5562",
                    help="ZMQ UPS telemetry endpoint")
    ap.add_argument("--left-wrist-cam", default="tcp://localhost:5563",
                    help="ZMQ left-wrist camera endpoint")
    ap.add_argument("--right-wrist-cam", default="tcp://localhost:5565",
                    help="ZMQ right-wrist camera endpoint")
    ap.add_argument("--head-cam", default="tcp://localhost:5564",
                    help="ZMQ head (RealSense) camera endpoint")
    ap.add_argument("--no-telemetry", action="store_true",
                    help="disable the ZMQ telemetry subscribers")
    args = ap.parse_args()

    if not args.no_telemetry:
        poller = TelemetryPoller(
            motor_eps=[("left", args.left_telem), ("right", args.right_telem)],
            ups_ep=args.ups,
            cam_eps=[("left_wrist", args.left_wrist_cam),
                     ("right_wrist", args.right_wrist_cam),
                     ("head", args.head_cam)],
        )
        poller.start()
        Handler.poller = poller
        if not _ZMQ_OK:
            print("AIZEE Heartbeat: pyzmq/msgpack unavailable — telemetry disabled",
                  flush=True)

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"AIZEE Heartbeat on http://{args.host}:{args.port}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
