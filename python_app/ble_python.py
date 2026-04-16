"""
ECNG 4504 - Maze Robot BLE Controller
=====================================
Communicates with the maze robot over HM-10 BLE.

Commands sent to robot:
  T  -> Begin training phase
  S  -> Begin solving phase
  M  -> Manual / emergency stop

Requires:  pip install bleak
Python   :  3.8+
"""

from __future__ import annotations

import threading
import asyncio
import time
import re
import subprocess
from datetime import datetime

try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext, messagebox
except ImportError:
    tk = None
    ttk = scrolledtext = messagebox = None

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

# Robot Commands
# Newline termination keeps compatibility with both single-byte readers and
# line-based parsers on the robot side.
CMD_TRAIN = b"T\n"
CMD_SOLVE = b"S\n"
CMD_RESET = b"M\n"

# HM-10 default BLE GATT service/characteristic.
HM10_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
HM10_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Path Visualizer
class PathVisualizer(tk.Canvas if tk is not None else object):
    """Draws the robot's L/R/S/B path as a 2D grid on a canvas."""

    CELL   = 30   # pixels per maze cell
    MARGIN = 40

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg="#1a1a2e", **kwargs)
        self.path = ""
        self._draw_empty()

    def _draw_empty(self):
        self.delete("all")
        self.create_text(
            self.winfo_reqwidth() // 2 or 200,
            self.winfo_reqheight() // 2 or 150,
            text="Path will appear here\nafter training completes",
            fill="#555577", font=("Consolas", 11), justify="center"
        )

    def update_path(self, path_str: str):
        """Re-render whenever a new path string arrives."""
        self.path = path_str
        self.delete("all")
        if not path_str:
            self._draw_empty()
            return
        self._render(path_str)

    def _render(self, path_str: str):
        C = self.CELL
        M = self.MARGIN

        # Simulate walking the path to collect visited (x,y) coordinates
        x, y   = 0, 0
        dx, dy = 0, -1   # Facing north initially
        visited = [(x, y)]

        direction_map = {
            (0, -1): {"L": (-1, 0), "R": (1, 0), "S": (0, -1), "B": (0, 1)},
            (0,  1): {"L": (1, 0),  "R": (-1, 0),"S": (0, 1),  "B": (0, -1)},
            (-1, 0): {"L": (0, 1),  "R": (0, -1),"S": (-1, 0), "B": (1, 0)},
            (1,  0): {"L": (0, -1), "R": (0, 1), "S": (1, 0),  "B": (-1, 0)},
        }

        for move in path_str:
            if move not in ("L", "R", "S", "B"):
                continue
            new_dir  = direction_map[(dx, dy)][move]
            dx, dy   = new_dir
            x       += dx
            y       += dy
            visited.append((x, y))

        if not visited:
            return

        xs = [p[0] for p in visited]
        ys = [p[1] for p in visited]
        min_x, min_y = min(xs), min(ys)

        # Offset so grid starts at margin
        cells = [(px - min_x, py - min_y) for px, py in visited]
        total_w = (max(xs) - min_x + 1) * C + 2 * M
        total_h = (max(ys) - min_y + 1) * C + 2 * M
        self.config(width=max(total_w, 300), height=max(total_h, 250))

        # Draw grid dots
        grid_w = max(xs) - min_x + 1
        grid_h = max(ys) - min_y + 1
        for gx in range(grid_w):
            for gy in range(grid_h):
                cx = M + gx * C + C // 2
                cy = M + gy * C + C // 2
                self.create_oval(cx-2, cy-2, cx+2, cy+2, fill="#333355", outline="")

        # Draw path segments
        for i in range(len(cells) - 1):
            ax, ay = cells[i]
            bx, by = cells[i + 1]
            x1 = M + ax * C + C // 2
            y1 = M + ay * C + C // 2
            x2 = M + bx * C + C // 2
            y2 = M + by * C + C // 2
            color = "#00d4ff" if i < len(cells) - 2 else "#ff6b6b"
            self.create_line(x1, y1, x2, y2, fill=color, width=3, capstyle="round")

        # Draw start marker
        sx, sy = cells[0]
        self.create_oval(
            M + sx * C + C // 2 - 8, M + sy * C + C // 2 - 8,
            M + sx * C + C // 2 + 8, M + sy * C + C // 2 + 8,
            fill="#2ecc71", outline="white", width=2
        )
        self.create_text(M + sx * C + C // 2, M + sy * C + C // 2,
                         text="S", fill="white", font=("Consolas", 9, "bold"))

        # Draw end marker
        ex, ey = cells[-1]
        self.create_oval(
            M + ex * C + C // 2 - 8, M + ey * C + C // 2 - 8,
            M + ex * C + C // 2 + 8, M + ey * C + C // 2 + 8,
            fill="#e74c3c", outline="white", width=2
        )
        self.create_text(M + ex * C + C // 2, M + ey * C + C // 2,
                         text="E", fill="white", font=("Consolas", 9, "bold"))

        # Path label
        self.create_text(
            M, M - 15, anchor="w",
            text=f"Path ({len(path_str)} moves): {path_str}",
            fill="#aaaacc", font=("Consolas", 9)
        )


# ─────────────────── Main Application ───────────────────
class MazeControllerApp:
    BLE_NAME_HINTS = ("hmsoft", "hm-10", "hm10", "bt05")

    def __init__(self, root: tk.Tk):
        self.root      = root
        self.root.title("ECNG 4504 - Maze Robot BLE Controller")
        self.root.configure(bg="#0f0f23")
        self.root.resizable(True, True)

        self.connected     = False
        self.robot_state   = "IDLE"      # IDLE / TRAINING / SOLVING / DONE
        self.scan_results  = {}          # label -> {"address": str, "name": str}
        self.rx_buffer     = ""          # partial-line buffer for BLE chunks
        self.final_path    = ""
        self.ble_client    = None
        self.ble_write_char = None
        self.ble_notify_char = None
        self.ble_loop      = None
        self.ble_thread    = None
        self.ble_busy      = False
        self.last_sensor_bits = None
        self.last_sensor_log_ts = 0.0

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # UI BUILD
    def _build_ui(self):
        root = self.root

        # Header
        hdr = tk.Frame(root, bg="#16213e", pady=10)
        hdr.pack(fill="x")
        tk.Label(hdr, text="ECNG 4504 Maze Robot", bg="#16213e",
                 fg="#00d4ff", font=("Consolas", 16, "bold")).pack()
        tk.Label(hdr, text="HM-10 BLE Controller", bg="#16213e",
                 fg="#8888aa", font=("Consolas", 10)).pack()

        # Main paned layout
        paned = tk.PanedWindow(root, orient="horizontal", bg="#0f0f23",
                               sashwidth=4, sashrelief="flat")
        paned.pack(fill="both", expand=True, padx=8, pady=8)

        left  = tk.Frame(paned, bg="#0f0f23")
        right = tk.Frame(paned, bg="#0f0f23")
        paned.add(left,  minsize=280)
        paned.add(right, minsize=380)

        self._build_left(left)
        self._build_right(right)
        self._build_status_bar()

    def _build_left(self, parent):
        # Link Scan
        scan_frame = tk.LabelFrame(parent, text=" BLE Device Scan ", bg="#0f0f23",
                                   fg="#00d4ff", font=("Consolas", 10, "bold"),
                                   bd=1, relief="solid")
        scan_frame.pack(fill="x", padx=5, pady=5)

        self.device_var = tk.StringVar()
        self.device_box = ttk.Combobox(scan_frame, textvariable=self.device_var,
                                       state="readonly", font=("Consolas", 9))
        self.device_box.pack(fill="x", padx=8, pady=(6, 4))

        btn_row = tk.Frame(scan_frame, bg="#0f0f23")
        btn_row.pack(fill="x", padx=8, pady=(0, 8))
        self.btn_scan    = self._btn(btn_row, "Scan BLE", self._on_scan, "#1a6b8a")
        self.btn_connect = self._btn(btn_row, "Connect", self._on_connect, "#1a6b3a",
                                     state="disabled")
        self.btn_scan.pack(side="left", expand=True, fill="x", padx=(0, 3))
        self.btn_connect.pack(side="left", expand=True, fill="x")

        # Commands
        cmd_frame = tk.LabelFrame(parent, text=" Commands ", bg="#0f0f23",
                                  fg="#00d4ff", font=("Consolas", 10, "bold"),
                                  bd=1, relief="solid")
        cmd_frame.pack(fill="x", padx=5, pady=5)

        self.btn_train = self._btn(cmd_frame, "START TRAINING",
                                   self._on_train, "#8a4a00", state="disabled",
                                   height=2)
        self.btn_train.pack(fill="x", padx=8, pady=(8, 4))

        self.btn_solve = self._btn(cmd_frame, "START SOLVING",
                                   self._on_solve, "#005a8a", state="disabled",
                                   height=2)
        self.btn_solve.pack(fill="x", padx=8, pady=(0, 4))

        self.btn_reset = self._btn(cmd_frame, "RESET",
                                   self._on_reset, "#5a0000", state="disabled")
        self.btn_reset.pack(fill="x", padx=8, pady=(0, 8))

        # Robot State indicator
        state_frame = tk.LabelFrame(parent, text=" Robot Status ", bg="#0f0f23",
                                    fg="#00d4ff", font=("Consolas", 10, "bold"),
                                    bd=1, relief="solid")
        state_frame.pack(fill="x", padx=5, pady=5)

        self.lbl_state = tk.Label(state_frame, text="DISCONNECTED",
                                  bg="#0f0f23", fg="#666688",
                                  font=("Consolas", 12, "bold"))
        self.lbl_state.pack(pady=8)

        self.progress = ttk.Progressbar(state_frame, mode="indeterminate", length=200)
        self.progress.pack(padx=8, pady=(0, 8))

        # Sensor strip
        sensor_outer = tk.LabelFrame(parent, text=" Sensors [FL L C R FR] ",
                                     bg="#0f0f23", fg="#00d4ff",
                                     font=("Consolas", 10, "bold"), bd=1, relief="solid")
        sensor_outer.pack(fill="x", padx=5, pady=5)

        self.sensor_leds = []
        led_row = tk.Frame(sensor_outer, bg="#0f0f23")
        led_row.pack(pady=8)
        for name in ("FL", "L", "C", "R", "FR"):
            col = tk.Frame(led_row, bg="#0f0f23")
            col.pack(side="left", padx=6)
            led = tk.Canvas(col, width=24, height=24, bg="#0f0f23",
                            highlightthickness=0)
            led.pack()
            led.create_oval(2, 2, 22, 22, fill="#222244", outline="#444466",
                            tags="dot")
            tk.Label(col, text=name, bg="#0f0f23", fg="#666688",
                     font=("Consolas", 8)).pack()
            self.sensor_leds.append(led)

    def _build_right(self, parent):
        # Log
        log_frame = tk.LabelFrame(parent, text=" Link Log ", bg="#0f0f23",
                                  fg="#00d4ff", font=("Consolas", 10, "bold"),
                                  bd=1, relief="solid")
        log_frame.pack(fill="both", expand=True, padx=5, pady=5)

        self.log = scrolledtext.ScrolledText(
            log_frame, bg="#050510", fg="#ccccee",
            font=("Consolas", 9), state="disabled",
            insertbackground="#00d4ff", relief="flat", bd=0
        )
        self.log.pack(fill="both", expand=True, padx=4, pady=4)

        # color tags
        self.log.tag_config("info",    foreground="#00d4ff")
        self.log.tag_config("success", foreground="#2ecc71")
        self.log.tag_config("warn",    foreground="#f39c12")
        self.log.tag_config("error",   foreground="#e74c3c")
        self.log.tag_config("path",    foreground="#9b59b6")
        self.log.tag_config("dim",     foreground="#555577")
        self.log.tag_config("sensor",  foreground="#1abc9c")

        clr_btn = self._btn(log_frame, "Clear Log", self._clear_log, "#222244",
                            height=1)
        clr_btn.pack(fill="x", padx=4, pady=(0, 4))

        # Path Visualizer
        viz_frame = tk.LabelFrame(parent, text=" Path Visualization ",
                                  bg="#0f0f23", fg="#00d4ff",
                                  font=("Consolas", 10, "bold"), bd=1, relief="solid")
        viz_frame.pack(fill="both", expand=True, padx=5, pady=5)

        scroll_x = tk.Scrollbar(viz_frame, orient="horizontal")
        scroll_y = tk.Scrollbar(viz_frame, orient="vertical")
        self.visualizer = PathVisualizer(viz_frame, width=380, height=220,
                                         xscrollcommand=scroll_x.set,
                                         yscrollcommand=scroll_y.set)
        scroll_x.config(command=self.visualizer.xview)
        scroll_y.config(command=self.visualizer.yview)
        scroll_y.pack(side="right", fill="y")
        scroll_x.pack(side="bottom", fill="x")
        self.visualizer.pack(fill="both", expand=True, padx=4, pady=4)

    def _build_status_bar(self):
        bar = tk.Frame(self.root, bg="#16213e", pady=3)
        bar.pack(fill="x", side="bottom")
        self.lbl_status_bar = tk.Label(bar, text="Ready", bg="#16213e",
                                       fg="#555577", font=("Consolas", 9),
                                       anchor="w")
        self.lbl_status_bar.pack(side="left", padx=10)
        self.lbl_time = tk.Label(bar, text="", bg="#16213e",
                                 fg="#555577", font=("Consolas", 9))
        self.lbl_time.pack(side="right", padx=10)
        self._tick_clock()

    def _tick_clock(self):
        self.lbl_time.config(text=datetime.now().strftime("%H:%M:%S"))
        self.root.after(1000, self._tick_clock)

    # Widget helpers
    def _btn(self, parent, text, cmd, bg, state="normal", height=1):
        return tk.Button(parent, text=text, command=cmd,
                         bg=bg, fg="white", activebackground=bg,
                         activeforeground="white", relief="flat",
                         font=("Consolas", 10, "bold"),
                         cursor="hand2", state=state, height=height,
                         bd=0, padx=6, pady=4)

    # Logging
    def _log(self, text: str, tag: str = ""):
        def _do():
            self.log.config(state="normal")
            ts = datetime.now().strftime("%H:%M:%S")
            self.log.insert("end", f"[{ts}] {text}\n", tag)
            self.log.see("end")
            self.log.config(state="disabled")
        self.root.after(0, _do)

    def _clear_log(self):
        self.log.config(state="normal")
        self.log.delete("1.0", "end")
        self.log.config(state="disabled")

    def _status(self, text: str):
        self.root.after(0, lambda: self.lbl_status_bar.config(text=text))

    # BLE Scan
    def _on_scan(self):
        self.btn_scan.config(state="disabled", text="Scanning...")
        self._status("Scanning for HM-10 BLE devices...")
        self._log("Scanning for HM-10 BLE devices...", "info")
        threading.Thread(target=self._scan_devices, daemon=True).start()

    def _scan_devices(self):
        try:
            self.scan_results = {}
            found = []
            preferred_ble = ""

            ble_devices = []
            try:
                ble_devices = self._scan_ble_devices()
            except Exception as e:
                self._log(f"BLE scan skipped: {e}", "warn")

            for device in ble_devices:
                name = device.name or "Unknown BLE Device"
                label = f"BLE | {name} | {device.address}"
                self.scan_results[label] = {"address": device.address, "name": name}
                found.append(label)
                matched = any(hint in name.lower() for hint in self.BLE_NAME_HINTS)
                if matched and not preferred_ble:
                    preferred_ble = label
                tag = "success" if matched else "dim"
                self._log(f"  Found: {label}", tag)

            preferred = preferred_ble or (found[0] if found else "")

            self.root.after(0, lambda: self._apply_scan_results(found, preferred))
            self._status(f"Found {len(found)} BLE device(s)")
            self._log(f"Scan complete. {len(found)} BLE device(s) found.", "info")
        except Exception as e:
            self._log(f"BLE scan error: {e}", "error")
            self.root.after(0, lambda: self.btn_scan.config(state="normal", text="Scan BLE"))

    def _apply_scan_results(self, found, preferred):
        self.device_box["values"] = found
        self.device_var.set(preferred if preferred else "")
        self.btn_connect.config(state="normal" if found else "disabled")
        self.btn_scan.config(state="normal", text="Scan BLE")

    def _ensure_ble_loop(self):
        if self.ble_loop is not None:
            return

        def _runner():
            loop = asyncio.new_event_loop()
            self.ble_loop = loop
            asyncio.set_event_loop(loop)
            loop.run_forever()

        self.ble_thread = threading.Thread(target=_runner, daemon=True)
        self.ble_thread.start()
        while self.ble_loop is None:
            time.sleep(0.01)

    def _recover_ble_adapter(self):
        commands = (
            ["rfkill", "unblock", "bluetooth"],
            ["bluetoothctl", "power", "on"],
        )
        for command in commands:
            try:
                subprocess.run(
                    command,
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                )
            except Exception:
                pass
        time.sleep(1.0)

    def _scan_ble_devices(self):
        if BleakScanner is None:
            return []
        self._ensure_ble_loop()
        try:
            future = asyncio.run_coroutine_threadsafe(
                BleakScanner.discover(timeout=5.0),
                self.ble_loop,
            )
            return future.result(timeout=10)
        except Exception as e:
            if "No powered Bluetooth adapters found" not in str(e):
                raise
            self._log("BLE adapter is off. Attempting to unblock and power it on...", "warn")
            self._recover_ble_adapter()
            future = asyncio.run_coroutine_threadsafe(
                BleakScanner.discover(timeout=5.0),
                self.ble_loop,
            )
            return future.result(timeout=10)

    # BLE Connect
    def _on_connect(self):
        if self.ble_busy:
            self._log("BLE operation in progress. Please wait...", "warn")
            return

        if self.connected:
            self.ble_busy = True
            self.btn_connect.config(state="disabled", text="Disconnecting...")
            threading.Thread(target=self._disconnect, daemon=True).start()
            return

        selection = self.device_var.get()
        if not selection:
            messagebox.showwarning("No Device", "Please scan and select a BLE device first.")
            return

        device = self.scan_results.get(selection)
        if not device:
            return

        self.ble_busy = True
        self.btn_scan.config(state="disabled")
        self.btn_connect.config(state="disabled", text="Connecting...")
        self._log(f"Connecting to {selection} over BLE...", "info")
        threading.Thread(target=self._connect_ble, args=(device["address"],), daemon=True).start()

    def _post_connect_failed(self, message: str):
        self._log(message, "error")

        def _ui():
            self.ble_busy = False
            self.btn_scan.config(state="normal", text="Scan BLE")
            self.btn_connect.config(state="normal", text="Connect")
            self._status("Disconnected")

        self.root.after(0, _ui)

    def _connect_ble(self, address: str):
        if BleakClient is None:
            self._post_connect_failed("BLE support is not installed. Install 'bleak' first.")
            return

        self._ensure_ble_loop()
        last_error = None
        for attempt in range(2):
            future = asyncio.run_coroutine_threadsafe(
                self._ble_connect_task(address),
                self.ble_loop,
            )
            try:
                future.result(timeout=20)
                return
            except Exception as e:
                last_error = e
                future.cancel()
                if attempt == 0:
                    self._log(f"BLE connect attempt 1 failed: {e}. Retrying...", "warn")
                    time.sleep(0.8)

        self._post_connect_failed(f"BLE connection failed: {last_error}")

    async def _ble_connect_task(self, address: str):
        client = BleakClient(address, disconnected_callback=self._on_ble_disconnected)
        await client.connect()

        services = await client.get_services()

        write_char = None
        notify_char = None

        hm10_char = None
        get_char = getattr(services, "get_characteristic", None)
        if callable(get_char):
            hm10_char = get_char(HM10_CHAR_UUID)
        if hm10_char is None:
            for service in services:
                for char in service.characteristics:
                    if str(char.uuid).lower() == HM10_CHAR_UUID:
                        hm10_char = char
                        break
                if hm10_char is not None:
                    break

        all_chars = []
        for service in services:
            all_chars.extend(service.characteristics)

        candidates = []
        if hm10_char is not None:
            candidates.append(hm10_char)
        candidates.extend([c for c in all_chars if c is not hm10_char])

        for char in candidates:
            props = {p.lower() for p in getattr(char, "properties", [])}
            if write_char is None and ("write-without-response" in props or "write" in props):
                write_char = char
            if notify_char is None and "notify" in props:
                notify_char = char
            if write_char is not None and notify_char is not None:
                break

        if write_char is None:
            await client.disconnect()
            raise RuntimeError("No writable BLE characteristic found on the device")

        if notify_char is not None:
            await client.start_notify(notify_char, self._handle_ble_notification)
        else:
            self._log("Connected, but notify characteristic was not found.", "warn")

        self.ble_client = client
        self.ble_write_char = write_char
        self.ble_notify_char = notify_char
        self.rx_buffer = ""
        self.connected = True
        self.root.after(0, self._post_ble_connected)

    def _post_ble_connected(self):
        self.ble_busy = False
        self.btn_scan.config(state="normal", text="Scan BLE")
        self.btn_connect.config(state="normal", text="Disconnect")
        self.btn_train.config(state="normal")
        self.btn_reset.config(state="normal")
        self._set_state_label("CONNECTED", "#2ecc71")
        self._log("HM-10 BLE connected!", "success")
        self._status("Connected")

    def _handle_ble_notification(self, _char, data: bytearray):
        self._on_ble_data(bytes(data))

    def _on_ble_disconnected(self, _client):
        self.ble_client = None
        self.ble_write_char = None
        self.ble_notify_char = None
        self.connected = False
        if not self.ble_busy:
            self.root.after(0, lambda: self._post_disconnect("HM-10 BLE disconnected"))

    def _disconnect(self):
        if self.ble_client is not None and self.ble_loop is not None:
            future = asyncio.run_coroutine_threadsafe(self._ble_disconnect_task(), self.ble_loop)
            try:
                future.result(timeout=10)
            except Exception as e:
                self._log(f"BLE disconnect warning: {e}", "warn")
        self._post_disconnect("HM-10 BLE disconnected")

    async def _ble_disconnect_task(self):
        client = self.ble_client
        notify_char = self.ble_notify_char
        self.ble_client = None
        self.ble_write_char = None
        self.ble_notify_char = None
        if client:
            try:
                if notify_char is not None:
                    await client.stop_notify(notify_char)
            except Exception:
                pass
            await client.disconnect()

    def _post_disconnect(self, message: str):
        self.connected = False
        self.ble_client = None
        self.ble_write_char = None
        self.ble_notify_char = None

        def _ui():
            self.ble_busy = False
            self.robot_state = "IDLE"
            self.btn_scan.config(state="normal", text="Scan BLE")
            self.btn_connect.config(state="normal", text="Connect")
            self.btn_train.config(state="disabled")
            self.btn_solve.config(state="disabled")
            self.btn_reset.config(state="disabled")
            self._set_state_label("DISCONNECTED", "#666688")
            self._log(message, "warn")
            self._status("Disconnected")
            self.progress.stop()

        self.root.after(0, _ui)

    # BLE Receive
    def _on_ble_data(self, data: bytes):
        """Called whenever BLE bytes arrive from the robot."""
        text = data.decode("utf-8", errors="replace")
        self.rx_buffer += text

        # Process complete lines
        while "\n" in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split("\n", 1)
            line = line.strip()
            if line:
                self.root.after(0, lambda l=line: self._handle_line(l))

    def _handle_line(self, line: str):
        """Parse each incoming line and update UI accordingly."""
        upper = line.upper()

        # Compact BLE firmware status:
        #   SENS:10001|STATE:TRAINING|PATH:LLR|IDX:2
        if self._handle_compact_status_line(line):
            return

        # Compact BLE firmware acknowledgements / errors
        if upper.startswith("OK:TRAINING_STARTED"):
            self._set_robot_state("TRAINING")
            self.progress.start(12)
            self._log(line, "success")
            return

        if upper.startswith("OK:TRAINING_COMPLETE"):
            self._update_path_from_line(line)
            self._set_state_label("TRAINING DONE", "#f39c12")
            self.btn_solve.config(state="normal")
            self.progress.stop()
            self._log(line, "success")
            return

        if upper.startswith("OK:SOLVING"):
            self._update_path_from_line(line)
            self._set_robot_state("SOLVING")
            self.progress.start(12)
            self._log(line, "success")
            return

        if upper.startswith("OK:MAZE_SOLVED"):
            self._set_robot_state("DONE")
            self.progress.stop()
            self._log(line, "success")
            return

        if upper.startswith("ERR:TIMEOUT"):
            self.progress.stop()
            self._log(line, "error")
            return

        if upper.startswith("ERR:"):
            self.progress.stop()
            self._log(line, "error")
            return

        # Sensor status
        m = re.search(r"Sensors:\s*([01] [01] [01] [01] [01])", line)
        if m:
            bits = m.group(1).split()
            self._update_sensors(bits)
            bits_key = "".join(bits)
            now = time.monotonic()
            if bits_key != self.last_sensor_bits or (now - self.last_sensor_log_ts) >= 0.4:
                self._log(line, "sensor")
                self.last_sensor_bits = bits_key
                self.last_sensor_log_ts = now
            return

        # Path extraction
        if self._update_path_from_line(line):
            self._log(f"Final Path: {self.final_path}", "path")
            self._log("Path visualized", "success")

        # State detection
        if "TRAINING COMPLETE" in upper:
            self._set_state_label("TRAINING DONE", "#f39c12")
            self.btn_solve.config(state="normal")
            self.progress.stop()
            self._log(line, "success")
            return

        if "STARTING SOLVING" in upper or "STATE: SOLVING" in upper:
            self._set_robot_state("SOLVING")
            self._log(line, "info")
            return

        if "MAZE SOLVED" in upper:
            self._set_robot_state("DONE")
            self.progress.stop()
            messagebox.showinfo("Maze Solved", "Robot has solved the maze.\nCheck the path visualization.")
            self._log(line, "success")
            return

        if "TIMEOUT" in upper:
            self._log(line, "error")
            self.progress.stop()
            return

        if "STATE: TRAINING" in upper:
            self._set_robot_state("TRAINING")
            self._log(line, "info")
            return

        # Icons / severity
        if any(x in upper for x in ("SUCCESS", "DONE", "CONNECTED", "SOLVED")):
            self._log(line, "success")
        elif any(x in upper for x in ("WARNING", "ERROR", "TIMEOUT", "FAILED")):
            self._log(line, "warn")
        elif any(x in upper for x in ("STATE", "SCAN", "SEARCH")):
            self._log(line, "info")
        else:
            self._log(line)

    def _handle_compact_status_line(self, line: str) -> bool:
        if not line.startswith("SENS:"):
            return False

        parts = {}
        for item in line.split("|"):
            if ":" not in item:
                continue
            key, value = item.split(":", 1)
            parts[key.strip().upper()] = value.strip()

        sens = parts.get("SENS", "")
        bits = [bit for bit in sens if bit in ("0", "1")]
        if len(bits) >= 5:
            self._update_sensors(bits[:5])
            bits_key = "".join(bits[:5])
            now = time.monotonic()
            if bits_key != self.last_sensor_bits or (now - self.last_sensor_log_ts) >= 0.4:
                self._log(f"SENS:{bits_key}", "sensor")
                self.last_sensor_bits = bits_key
                self.last_sensor_log_ts = now

        state = parts.get("STATE", "").upper()
        if state == "TRAINING":
            self._set_robot_state("TRAINING")
        elif state == "SOLVING":
            self._set_robot_state("SOLVING")
        elif state in ("MANUAL", "IDLE"):
            self._set_robot_state("IDLE")
            self.progress.stop()
        elif state == "DONE":
            self._set_robot_state("DONE")
            self.progress.stop()

        path = re.sub(r"[^LRSB]", "", parts.get("PATH", "").upper())
        if path:
            self.final_path = path
            self.visualizer.update_path(path)

        if state == "DONE" and path:
            self.btn_solve.config(state="normal")

        return True

    def _update_path_from_line(self, line: str) -> bool:
        m = re.search(r"(?:OPTIMIZED PATH|PATH):\s*([LRSB]+)", line, re.IGNORECASE)
        if not m:
            return False
        self.final_path = m.group(1).upper()
        self.visualizer.update_path(self.final_path)
        return True

    # Commands
    def _on_train(self):
        self._log("Sending T to robot...", "info")
        self._send(CMD_TRAIN)
        self._set_robot_state("TRAINING")
        self.progress.start(12)

    def _on_solve(self):
        self._log("Sending S to robot...", "info")
        self._send(CMD_SOLVE)
        self._set_robot_state("SOLVING")
        self.progress.start(12)

    def _on_reset(self):
        if messagebox.askyesno("Stop Robot", "Send MANUAL/STOP command to robot?"):
            self._log("Sending M to robot...", "warn")
            self._send(CMD_RESET)
            self._set_robot_state("IDLE")
            self.progress.stop()
            self.visualizer.update_path("")
            self.final_path = ""
            self.btn_solve.config(state="disabled")

    def _send(self, data: bytes):
        if not self.connected:
            self._log("Not connected!", "error")
            return

        if self.ble_client is None or self.ble_write_char is None or self.ble_loop is None:
            self._log("BLE link is not ready.", "error")
            return

        future = asyncio.run_coroutine_threadsafe(
            self._ble_write_task(data),
            self.ble_loop,
        )
        def _on_done(done_future):
            try:
                done_future.result()
            except Exception as e:
                self._log(f"BLE send error: {e}", "error")
        future.add_done_callback(_on_done)

    async def _ble_write_task(self, data: bytes):
        properties = {prop.lower() for prop in self.ble_write_char.properties}
        response_modes = []

        if "write-without-response" in properties:
            response_modes.append(False)
        if "write" in properties:
            response_modes.append(True)
        if not response_modes:
            response_modes.append(False)

        last_error = None
        for response in response_modes:
            try:
                await self.ble_client.write_gatt_char(self.ble_write_char, data, response=response)
                return
            except Exception as exc:
                last_error = exc

        if last_error is not None:
            raise last_error

    # UI Helpers
    def _set_robot_state(self, state: str):
        self.robot_state = state
        colors = {
            "IDLE":         ("#888899", "IDLE"),
            "CONNECTED":    ("#2ecc71", "CONNECTED"),
            "TRAINING":     ("#f39c12", "TRAINING..."),
            "TRAINING DONE":("#e67e22", "TRAINING DONE"),
            "SOLVING":      ("#3498db", "SOLVING..."),
            "DONE":         ("#2ecc71", "SOLVED"),
        }
        color, text = colors.get(state, ("#888899", state))
        self._set_state_label(text, color)
        self._status(f"Robot state: {state}")

    def _set_state_label(self, text: str, color: str):
        self.root.after(0, lambda: self.lbl_state.config(text=text, fg=color))

    def _update_sensors(self, bits):
        """bits = list of 5 strings, '1' = on line (black), '0' = off."""
        for i, led in enumerate(self.sensor_leds):
            on = (i < len(bits) and bits[i] == "1")
            led.itemconfig("dot", fill="#00ff88" if on else "#222244",
                           outline="#00cc66" if on else "#444466")

    # Shutdown
    def _on_close(self):
        if self.connected:
            try:
                self._disconnect()
            except Exception:
                pass
        if self.ble_loop is not None:
            self.ble_loop.call_soon_threadsafe(self.ble_loop.stop)
        time.sleep(0.2)
        self.root.destroy()


# Entry Point
if __name__ == "__main__":
    if tk is None:
        print("ERROR: Tkinter is not available in this Python installation.")
        print("Install/use a Python build with Tk support, then run this script again.")
        raise SystemExit(1)

    if BleakClient is None or BleakScanner is None:
        print("ERROR: BLE support is not installed.")
        print("Install it with: pip install bleak")
        raise SystemExit(1)

    root = tk.Tk()
    root.geometry("900x680")

    # Dark title bar on Windows
    try:
        root.tk.call("tk", "scaling", 1.3)
    except Exception:
        pass

    app = MazeControllerApp(root)
    root.mainloop()