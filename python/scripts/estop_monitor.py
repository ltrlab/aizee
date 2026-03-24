#!/usr/bin/env python3
"""E-Stop monitor GUI — shows live e-stop state from the receiver ESP32.

Can read from a local serial port or from the Jetson over SSH.

Usage:
    python estop_monitor.py --port COM10                          # local receiver
    python estop_monitor.py --jetson 192.168.0.27                 # receiver on Jetson via SSH
    python estop_monitor.py --port COM10 --jetson 192.168.0.27    # both
"""

import argparse
import json
import subprocess
import threading
import time
import tkinter as tk
from collections import deque

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

SSH_KEY = "/p/Workspace/ssh-keys/aizee_rover_id"


class EStopMonitor:
    def __init__(self, port: str = None, baud: int = 115200, jetson: str = None):
        self.port = port
        self.baud = baud
        self.jetson = jetson
        self.ser = None
        self.running = True

        # Local serial state
        self.local_estop = None
        self.local_seq = None
        self.local_nc = None
        self.local_rx_time = None
        self.local_count = 0

        # Jetson serial state (via SSH)
        self.jetson_estop = None
        self.jetson_seq = None
        self.jetson_nc = None
        self.jetson_rx_time = None
        self.jetson_count = 0
        self.jetson_connected = False

        self.log = deque(maxlen=200)

        self._build_ui()
        if self.port:
            threading.Thread(target=self._local_serial_loop, daemon=True).start()
        if self.jetson:
            threading.Thread(target=self._jetson_ssh_loop, daemon=True).start()
        self._tick()

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("E-Stop Monitor")
        self.root.configure(bg="#1e1e1e")
        self.root.resizable(True, True)
        self.root.minsize(520, 420)

        # Big status indicator
        self.status_frame = tk.Frame(self.root, height=120, bg="#333")
        self.status_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        self.status_frame.pack_propagate(False)

        self.status_label = tk.Label(
            self.status_frame, text="WAITING...", font=("Consolas", 36, "bold"),
            fg="white", bg="#333"
        )
        self.status_label.pack(expand=True)

        # Two-column layout
        columns = tk.Frame(self.root, bg="#1e1e1e")
        columns.pack(fill=tk.X, padx=10, pady=5)

        # Left: local serial
        left = tk.LabelFrame(columns, text=" Local (Serial) ", font=("Consolas", 10),
                              fg="#888", bg="#1e1e1e", labelanchor="nw")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self.local_labels = {}
        for row, key in enumerate(["Source", "E-Stop", "Messages", "Seq", "Pin", "Last RX", "Age"]):
            tk.Label(left, text=key, font=("Consolas", 10),
                     fg="#666", bg="#1e1e1e", anchor="w", width=10).grid(row=row, column=0, sticky="w", padx=4)
            lbl = tk.Label(left, text="—", font=("Consolas", 10, "bold"),
                           fg="white", bg="#1e1e1e", anchor="w")
            lbl.grid(row=row, column=1, sticky="w", padx=(4, 4))
            self.local_labels[key] = lbl

        if self.port:
            self.local_labels["Source"].config(text=self.port)
        else:
            self.local_labels["Source"].config(text="disabled", fg="#555")

        # Right: jetson serial via SSH
        right = tk.LabelFrame(columns, text=" Jetson (SSH) ", font=("Consolas", 10),
                               fg="#888", bg="#1e1e1e", labelanchor="nw")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        self.jetson_labels = {}
        for row, key in enumerate(["Source", "E-Stop", "Messages", "Seq", "Pin", "Last RX", "Age"]):
            tk.Label(right, text=key, font=("Consolas", 10),
                     fg="#666", bg="#1e1e1e", anchor="w", width=10).grid(row=row, column=0, sticky="w", padx=4)
            lbl = tk.Label(right, text="—" if self.jetson else "disabled",
                           font=("Consolas", 10, "bold"),
                           fg="white" if self.jetson else "#555", bg="#1e1e1e", anchor="w")
            lbl.grid(row=row, column=1, sticky="w", padx=(4, 4))
            self.jetson_labels[key] = lbl

        if self.jetson:
            self.jetson_labels["Source"].config(text=self.jetson)

        # Log area
        log_label = tk.Label(self.root, text="Log", font=("Consolas", 9),
                             fg="#666", bg="#1e1e1e", anchor="w")
        log_label.pack(fill=tk.X, padx=10, pady=(10, 0))

        log_frame = tk.Frame(self.root, bg="#1e1e1e")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        self.log_text = tk.Text(
            log_frame, font=("Consolas", 9), bg="#111", fg="#aaa",
            insertbackground="white", wrap=tk.WORD, state=tk.DISABLED,
            height=10
        )
        scrollbar = tk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _parse_line(self, line, prefix):
        """Parse a serial line and return (estop, seq, nc) or None."""
        if line.startswith("#"):
            parts = line[2:].split()
            seq = nc = None
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    if k == "nc":
                        nc = int(v)
                    elif k == "seq":
                        seq = int(v)
            return {"type": "diag", "seq": seq, "nc": nc}
        else:
            try:
                data = json.loads(line)
                if "estop" in data:
                    return {"type": "estop", "estop": data["estop"]}
            except json.JSONDecodeError:
                pass
        return None

    def _local_serial_loop(self):
        while self.running:
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                    self.log.append(f"[LOCAL] Opened {self.port}")
                except serial.SerialException:
                    time.sleep(3)
                    continue

            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                self.log.append(f"[LOCAL] {line}")
                parsed = self._parse_line(line, "LOCAL")
                if parsed:
                    self.local_rx_time = time.time()
                    self.local_count += 1
                    if parsed["type"] == "estop":
                        self.local_estop = parsed["estop"]
                    elif parsed["type"] == "diag":
                        if parsed["seq"] is not None:
                            self.local_seq = parsed["seq"]
                        if parsed["nc"] is not None:
                            self.local_nc = parsed["nc"]

            except serial.SerialException:
                self.log.append("[LOCAL] Disconnected")
                self.ser = None
                time.sleep(2)

    def _jetson_ssh_cmd(self, remote_cmd):
        """Run a command on the Jetson over SSH."""
        subprocess.run(
            ["ssh", "-i", SSH_KEY, "-o", "ConnectTimeout=5",
             f"ltr@{self.jetson}", remote_cmd],
            capture_output=True, timeout=10
        )

    def _jetson_ssh_loop(self):
        """SSH into the Jetson and cat the receiver's serial port.
        Stops the bridge service while monitoring (it holds the port),
        restarts it on exit."""
        self.log.append(f"[JETSON] Stopping bridge to access serial port...")
        try:
            self._jetson_ssh_cmd("sudo systemctl stop aizee-estop-bridge 2>/dev/null")
        except Exception:
            pass
        self.log.append(f"[JETSON] Connecting to {self.jetson}...")

        while self.running:
            cmd = [
                "ssh", "-i", SSH_KEY,
                "-o", "ConnectTimeout=5",
                "-o", "ServerAliveInterval=5",
                "-o", "ServerAliveCountMax=2",
                f"ltr@{self.jetson}",
                "stty -F /dev/estop-receiver 115200 raw -echo 2>/dev/null; cat /dev/estop-receiver"
            ]
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self.jetson_connected = True
                self.log.append("[JETSON] Connected, reading /dev/estop-receiver")

                buf = b""
                while self.running:
                    chunk = proc.stdout.read(1)
                    if not chunk:
                        break
                    if chunk == b"\n":
                        line = buf.decode(errors="replace").strip()
                        buf = b""
                        if not line:
                            continue

                        self.log.append(f"[JETSON] {line}")
                        parsed = self._parse_line(line, "JETSON")
                        if parsed:
                            self.jetson_rx_time = time.time()
                            self.jetson_count += 1
                            if parsed["type"] == "estop":
                                self.jetson_estop = parsed["estop"]
                            elif parsed["type"] == "diag":
                                if parsed["seq"] is not None:
                                    self.jetson_seq = parsed["seq"]
                                if parsed["nc"] is not None:
                                    self.jetson_nc = parsed["nc"]
                    else:
                        buf += chunk

                proc.kill()
            except Exception as e:
                self.log.append(f"[JETSON] {e}")

            self.jetson_connected = False
            self.log.append("[JETSON] Disconnected, retrying...")
            time.sleep(3)

    def _format_estop(self, estop):
        if estop is None:
            return "—", "white"
        elif estop:
            return "ACTIVE", "#ff5555"
        else:
            return "clear", "#4CAF50"

    def _tick(self):
        if not self.running:
            return

        # Main status — pick the most relevant source
        estop = self.local_estop if self.local_estop is not None else self.jetson_estop
        rx_time = self.local_rx_time or self.jetson_rx_time

        if estop is None:
            color, text = "#333", "WAITING..."
        elif estop:
            color, text = "#cc0000", "E-STOP"
        else:
            color, text = "#006600", "SAFE"

        if rx_time:
            age = time.time() - rx_time
            if age > 2.0 and estop is not None:
                color, text = "#665500", text + " (STALE)"

        self.status_frame.configure(bg=color)
        self.status_label.configure(bg=color, text=text)

        # Local serial panel
        etxt, ecol = self._format_estop(self.local_estop)
        self.local_labels["E-Stop"].config(text=etxt, fg=ecol)
        self.local_labels["Messages"].config(text=str(self.local_count) if self.local_count else "—")
        self.local_labels["Seq"].config(text=str(self.local_seq) if self.local_seq is not None else "—")
        self.local_labels["Pin"].config(text=str(self.local_nc) if self.local_nc is not None else "—")
        if self.local_rx_time:
            age = time.time() - self.local_rx_time
            self.local_labels["Last RX"].config(
                text=time.strftime("%H:%M:%S", time.localtime(self.local_rx_time)))
            self.local_labels["Age"].config(text=f"{age:.1f}s")

        # Jetson panel
        if self.jetson:
            etxt, ecol = self._format_estop(self.jetson_estop)
            self.jetson_labels["E-Stop"].config(text=etxt, fg=ecol)
            self.jetson_labels["Messages"].config(text=str(self.jetson_count) if self.jetson_count else "—")
            self.jetson_labels["Seq"].config(text=str(self.jetson_seq) if self.jetson_seq is not None else "—")
            self.jetson_labels["Pin"].config(text=str(self.jetson_nc) if self.jetson_nc is not None else "—")
            if self.jetson_rx_time:
                jage = time.time() - self.jetson_rx_time
                self.jetson_labels["Last RX"].config(
                    text=time.strftime("%H:%M:%S", time.localtime(self.jetson_rx_time)))
                self.jetson_labels["Age"].config(text=f"{jage:.1f}s")

        # Log
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(self.log))
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

        self.root.after(100, self._tick)

    def _on_close(self):
        self.running = False
        if self.ser:
            self.ser.close()
        if self.jetson:
            try:
                self._jetson_ssh_cmd("sudo systemctl restart aizee-estop-bridge 2>/dev/null")
            except Exception:
                pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="E-Stop monitor GUI")
    parser.add_argument("--port", default=None, help="Local serial port of receiver ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--jetson", default=None,
                        help="Jetson IP — reads receiver serial over SSH")
    args = parser.parse_args()

    if not args.port and not args.jetson:
        parser.error("Provide at least one of --port or --jetson")

    monitor = EStopMonitor(port=args.port, baud=args.baud, jetson=args.jetson)
    monitor.run()


if __name__ == "__main__":
    main()
