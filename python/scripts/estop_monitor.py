#!/usr/bin/env python3
"""E-Stop monitor GUI — connects to the receiver ESP32 over serial
and shows live e-stop state, diagnostics, and message history.

Usage:
    python estop_monitor.py [--port COM10]
"""

import argparse
import json
import threading
import time
import tkinter as tk
from collections import deque

import serial


class EStopMonitor:
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = True

        # State
        self.estop = None  # None = no data yet
        self.last_seq = None
        self.last_nc = None
        self.last_no = None
        self.last_rx_time = None
        self.msg_count = 0
        self.log = deque(maxlen=200)

        self._build_ui()
        self._start_serial()

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("E-Stop Monitor")
        self.root.configure(bg="#1e1e1e")
        self.root.resizable(True, True)
        self.root.minsize(400, 350)

        # Big status indicator
        self.status_frame = tk.Frame(self.root, height=120, bg="#333")
        self.status_frame.pack(fill=tk.X, padx=10, pady=(10, 5))
        self.status_frame.pack_propagate(False)

        self.status_label = tk.Label(
            self.status_frame, text="WAITING...", font=("Consolas", 36, "bold"),
            fg="white", bg="#333"
        )
        self.status_label.pack(expand=True)

        # Info panel
        info_frame = tk.Frame(self.root, bg="#1e1e1e")
        info_frame.pack(fill=tk.X, padx=10, pady=5)

        self.info_labels = {}
        for row, key in enumerate(["Port", "Messages", "Seq", "NC pin", "NO pin", "Last RX", "Age"]):
            tk.Label(info_frame, text=key, font=("Consolas", 11),
                     fg="#888", bg="#1e1e1e", anchor="w", width=12).grid(row=row, column=0, sticky="w")
            lbl = tk.Label(info_frame, text="—", font=("Consolas", 11, "bold"),
                           fg="white", bg="#1e1e1e", anchor="w")
            lbl.grid(row=row, column=1, sticky="w", padx=(10, 0))
            self.info_labels[key] = lbl

        self.info_labels["Port"].config(text=self.port)

        # Log area
        log_label = tk.Label(self.root, text="Serial log", font=("Consolas", 9),
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

    def _start_serial(self):
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.serial_thread.start()
        self._tick()

    def _serial_loop(self):
        while self.running:
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                    self.log.append(f"[CONN] Opened {self.port}")
                except serial.SerialException as e:
                    self.log.append(f"[ERR] {e}")
                    time.sleep(2)
                    continue

            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                self.log.append(line)

                if line.startswith("#"):
                    # Diagnostic line: # nc=X no=X seq=X age=X
                    parts = line[2:].split()
                    for p in parts:
                        if "=" in p:
                            k, v = p.split("=", 1)
                            if k == "nc":
                                self.last_nc = int(v)
                            elif k == "no":
                                self.last_no = int(v)
                            elif k == "seq":
                                self.last_seq = int(v)
                    self.last_rx_time = time.time()
                    self.msg_count += 1
                else:
                    try:
                        data = json.loads(line)
                        if "estop" in data:
                            self.estop = data["estop"]
                            self.last_rx_time = time.time()
                            self.msg_count += 1
                    except json.JSONDecodeError:
                        pass

            except serial.SerialException:
                self.log.append("[ERR] Serial disconnected")
                self.ser = None
                time.sleep(1)

    def _tick(self):
        if not self.running:
            return

        # Update status indicator
        if self.estop is None:
            color, text = "#333", "WAITING..."
        elif self.estop:
            color, text = "#cc0000", "E-STOP"
        else:
            color, text = "#006600", "SAFE"

        # Check staleness
        age_str = "—"
        if self.last_rx_time:
            age = time.time() - self.last_rx_time
            age_str = f"{age:.1f}s"
            if age > 2.0 and self.estop is not None:
                color, text = "#665500", text + " (STALE)"

        self.status_frame.configure(bg=color)
        self.status_label.configure(bg=color, text=text)

        # Update info labels
        self.info_labels["Messages"].config(text=str(self.msg_count))
        self.info_labels["Seq"].config(text=str(self.last_seq) if self.last_seq is not None else "—")
        self.info_labels["NC pin"].config(text=str(self.last_nc) if self.last_nc is not None else "—")
        self.info_labels["NO pin"].config(text=str(self.last_no) if self.last_no is not None else "—")
        self.info_labels["Last RX"].config(
            text=time.strftime("%H:%M:%S", time.localtime(self.last_rx_time)) if self.last_rx_time else "—"
        )
        self.info_labels["Age"].config(text=age_str)

        # Update log
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
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="E-Stop monitor GUI")
    parser.add_argument("--port", default="COM10", help="Serial port of receiver ESP32")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    monitor = EStopMonitor(args.port, args.baud)
    monitor.run()


if __name__ == "__main__":
    main()
