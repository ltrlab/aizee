#!/usr/bin/env python3
"""E-Stop monitor GUI — shows live e-stop state via motor controller telemetry.

The e-stop bridge on the Jetson forwards serial e-stop events to
motor_control, which includes emergency_stop in every telemetry message.
This monitor subscribes to that ZMQ telemetry stream.

Usage:
    python estop_monitor.py                        # default 192.168.0.27
    python estop_monitor.py --telem tcp://192.168.0.27:5556
"""

import argparse
import json
import threading
import time
import tkinter as tk
from collections import deque

import zmq

DEFAULT_TELEM = "tcp://192.168.0.27:5556"


class EStopMonitor:
    def __init__(self, telem_ep: str):
        self.telem_ep = telem_ep
        self.running = True

        self.estop = None
        self.rx_time = None
        self.count = 0
        self.connected = False

        self.log = deque(maxlen=200)

        self._build_ui()
        threading.Thread(target=self._zmq_loop, daemon=True).start()
        self._tick()

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title("E-Stop Monitor")
        self.root.configure(bg="#1e1e1e")
        self.root.resizable(True, True)
        self.root.minsize(400, 340)

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
        info = tk.LabelFrame(self.root, text=" Telemetry ", font=("Consolas", 10),
                             fg="#888", bg="#1e1e1e", labelanchor="nw")
        info.pack(fill=tk.X, padx=10, pady=5)

        self.info_labels = {}
        for row, key in enumerate(["Source", "E-Stop", "Messages", "Last RX", "Age"]):
            tk.Label(info, text=key, font=("Consolas", 10),
                     fg="#666", bg="#1e1e1e", anchor="w", width=10).grid(
                row=row, column=0, sticky="w", padx=4)
            lbl = tk.Label(info, text="—", font=("Consolas", 10, "bold"),
                           fg="white", bg="#1e1e1e", anchor="w")
            lbl.grid(row=row, column=1, sticky="w", padx=(4, 4))
            self.info_labels[key] = lbl

        self.info_labels["Source"].config(text=self.telem_ep)

        # Log area
        tk.Label(self.root, text="Log", font=("Consolas", 9),
                 fg="#666", bg="#1e1e1e", anchor="w").pack(
            fill=tk.X, padx=10, pady=(10, 0))

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

    def _zmq_loop(self):
        self.log.append(f"Subscribing to {self.telem_ep}")

        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, "")
        sock.connect(self.telem_ep)

        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)

        try:
            while self.running:
                events = dict(poller.poll(timeout=500))
                if sock not in events:
                    if self.connected:
                        self.log.append("No telemetry (motor_control running?)")
                        self.connected = False
                    continue

                try:
                    msg = json.loads(sock.recv_string(zmq.NOBLOCK))
                except Exception:
                    continue

                self.connected = True
                self.rx_time = time.time()
                self.count += 1
                estop = msg.get("emergency_stop")
                if estop is not None:
                    if estop != self.estop:
                        self.log.append(f"emergency_stop={estop}")
                    self.estop = estop
        finally:
            sock.close()
            ctx.term()

    def _tick(self):
        if not self.running:
            return

        # Main status
        if self.estop is None:
            color, text = "#333", "WAITING..."
        elif self.estop:
            color, text = "#cc0000", "E-STOP"
        else:
            color, text = "#006600", "SAFE"

        if self.rx_time:
            age = time.time() - self.rx_time
            if age > 2.0 and self.estop is not None:
                color, text = "#665500", text + " (STALE)"

        self.status_frame.configure(bg=color)
        self.status_label.configure(bg=color, text=text)

        # Info panel
        if self.estop is None:
            etxt, ecol = "—", "white"
        elif self.estop:
            etxt, ecol = "ACTIVE", "#ff5555"
        else:
            etxt, ecol = "clear", "#4CAF50"

        self.info_labels["E-Stop"].config(text=etxt, fg=ecol)
        self.info_labels["Messages"].config(
            text=str(self.count) if self.count else "—")
        if self.rx_time:
            self.info_labels["Last RX"].config(
                text=time.strftime("%H:%M:%S", time.localtime(self.rx_time)))
            self.info_labels["Age"].config(
                text=f"{time.time() - self.rx_time:.1f}s")

        # Log
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(self.log))
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

        self.root.after(100, self._tick)

    def _on_close(self):
        self.running = False
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="E-Stop monitor GUI")
    parser.add_argument("--telem", default=DEFAULT_TELEM,
                        help=f"ZMQ telemetry endpoint (default: {DEFAULT_TELEM})")
    args = parser.parse_args()

    monitor = EStopMonitor(telem_ep=args.telem)
    monitor.run()


if __name__ == "__main__":
    main()
