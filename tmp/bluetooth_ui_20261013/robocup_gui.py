#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M10P简洁上位机：Tk负责界面，串口工作线程负责启停/采集，报告单独生成。"""
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from types import SimpleNamespace

from bluetooth_session import BluetoothSession

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "log")


class App:
    def __init__(self, root):
        self.root = root
        root.title("RoboCup · M10P 蓝牙上位机")
        root.geometry("920x650")
        root.minsize(800, 600)
        self.events = queue.Queue()
        self.session = None
        self.connected = self.capturing = self.reporting = self.exiting = False
        self.phase = "idle"
        self.last_report = None
        os.makedirs(LOG_DIR, exist_ok=True)
        self._build_ui()
        self.refresh_ports()
        self._buttons()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<F5>", lambda event: self.start_detection())
        root.after(80, self._tick)
        self.log("一键检测：协商成功后自动启动电机并采集，结束后自动生成报告。")

    def _build_ui(self):
        style = ttk.Style(self.root)
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 16, "bold"))
        style.configure("Action.TButton", padding=(12, 9))
        style.configure("Muted.TLabel", foreground="#666666")
        page = ttk.Frame(self.root, padding=16)
        page.pack(fill="both", expand=True)
        ttk.Label(page, text="M10P 蓝牙调试", style="Title.TLabel").pack(anchor="w")
        row = ttk.Frame(page, padding=(0, 12, 0, 10))
        row.pack(fill="x")
        ttk.Label(row, text="串口").pack(side="left")
        self.port = ttk.Combobox(row, width=30, state="readonly")
        self.port.pack(side="left", padx=(6, 4))
        self.refresh_button = ttk.Button(row, text="刷新", width=6, command=self.refresh_ports)
        self.refresh_button.pack(side="left")
        ttk.Label(row, text="波特率").pack(side="left", padx=(12, 4))
        self.baud = ttk.Combobox(row, width=8, state="readonly", values=("115200", "9600", "57600", "230400"))
        self.baud.set("115200")
        self.baud.pack(side="left")
        self.connect_button = ttk.Button(row, text="连接", width=8, command=self.toggle_connection)
        self.connect_button.pack(side="left", padx=8)
        self.connection_text = tk.StringVar(value="未连接")
        ttk.Label(row, textvariable=self.connection_text).pack(side="left")

        motor = ttk.LabelFrame(page, text="电机控制", padding=12)
        motor.pack(fill="x", pady=(0, 12))
        row = ttk.Frame(motor)
        row.pack(fill="x")
        ttk.Label(row, text="启动速度").pack(side="left")
        self.speed = ttk.Spinbox(row, from_=1, to=18, width=5)
        self.speed.set("17")
        self.speed.pack(side="left", padx=(6, 12))
        self.motor_start = ttk.Button(row, text="启动电机", style="Action.TButton", command=self.start_motor)
        self.motor_start.pack(side="left", padx=(0, 8))
        self.motor_stop = tk.Button(row, text="关闭电机", command=self.stop_motor,
                                   bg="#b42318", fg="white", activebackground="#912018",
                                   activeforeground="white", relief="flat", padx=22, pady=9,
                                   font=("Microsoft YaHei UI", 10, "bold"))
        self.motor_stop.pack(side="left")
        self.motor_text = tk.StringVar(value="电机状态未知")
        ttk.Label(motor, textvariable=self.motor_text).pack(anchor="w", pady=(10, 0))
        ttk.Label(motor, text="启动设置直道速度上限；弯道速度由车端控制。采集和导出时仍可关闭电机。",
                  style="Muted.TLabel").pack(anchor="w", pady=(5, 0))

        capture = ttk.LabelFrame(page, text="遥测检测", padding=12)
        capture.pack(fill="x")
        row = ttk.Frame(capture)
        row.pack(fill="x")
        ttk.Label(row, text="采集时长（秒）").pack(side="left")
        self.seconds = ttk.Spinbox(row, from_=1, to=600, increment=5, width=6)
        self.seconds.set("30")
        self.seconds.pack(side="left", padx=(6, 12))
        self.detect_button = ttk.Button(row, text="一键检测并启动电机 · F5", style="Action.TButton", command=self.start_detection)
        self.detect_button.pack(side="left", padx=(0, 8))
        self.capture_stop = ttk.Button(row, text="停止采集", style="Action.TButton", command=self.stop_capture)
        self.capture_stop.pack(side="left")
        self.capture_text = tk.StringVar(value="等待检测")
        self.progress_text = tk.StringVar(value="")
        ttk.Label(capture, textvariable=self.capture_text).pack(anchor="w", pady=(10, 0))
        ttk.Label(capture, textvariable=self.progress_text, style="Muted.TLabel").pack(anchor="w", pady=(4, 0))
        ttk.Label(capture, text="停止采集只结束记录，随后自动导出点云和报告；需要停电机请点上方红色按钮。",
                  style="Muted.TLabel").pack(anchor="w", pady=(5, 0))
        files = ttk.Frame(page, padding=(0, 10))
        files.pack(fill="x")
        ttk.Button(files, text="打开报告", command=self.open_report).pack(side="left", padx=(0, 8))
        ttk.Button(files, text="打开数据目录", command=lambda: self.open_path(LOG_DIR)).pack(side="left", padx=(0, 8))
        self.report_button = ttk.Button(files, text="处理已有 BIN", command=self.process_file)
        self.report_button.pack(side="left")
        logs = ttk.LabelFrame(page, text="操作记录", padding=6)
        logs.pack(fill="both", expand=True)
        self.log_text = tk.Text(logs, height=7, wrap="word", state="disabled", font=("Microsoft YaHei UI", 9))
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll = ttk.Scrollbar(logs, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=scroll.set)

    def log(self, message):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", time.strftime("[%H:%M:%S] ") + str(message).strip() + "\n")
        if int(self.log_text.index("end-1c").split(".")[0]) > 500:
            self.log_text.delete("1.0", "101.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    @staticmethod
    def open_path(path):
        try:
            os.startfile(path)
        except OSError as exc:
            messagebox.showerror("打开失败", str(exc))

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            selected = self.port.get()
            values = [f"{p.device} | {p.description}" for p in list_ports.comports()]
            self.port["values"] = values
            self.port.set(selected if selected in values else next((p for p in values if "blue" in p.lower()), values[0] if values else ""))
            if not values:
                self.log("未发现串口，请先配对 HC-05 并选择传出 COM 口。")
        except ImportError:
            self.log("缺少 pyserial，请使用“启动上位机.bat”启动。")

    def _buttons(self):
        linked = self.connected and not self.exiting
        busy = self.capturing or self.reporting
        for button in (self.motor_start, self.motor_stop):
            button.configure(state="normal" if linked else "disabled")
        self.detect_button.configure(state="normal" if linked and not busy else "disabled")
        can_stop = self.capturing and self.phase != "dump" and self.session and not self.session.capture_stop.is_set()
        self.capture_stop.configure(state="normal" if can_stop and not self.exiting else "disabled")
        self.report_button.configure(state="disabled" if busy or self.exiting else "normal")
        self.connect_button.configure(text="断开" if self.connected else "连接",
            state="disabled" if self.capturing or self.exiting or (self.session and not self.connected) else "normal")
        self.refresh_button.configure(state="disabled" if self.session else "normal")
        for box in (self.port, self.baud):
            box.configure(state="disabled" if self.session else "readonly")

    def toggle_connection(self):
        if self.session:
            self.connected = False
            self.connection_text.set("正在断开")
            self.session.closing.set()
        else:
            port = self.port.get().split("|")[0].strip()
            if not port:
                messagebox.showwarning("请选择串口", "刷新后选择 HC-05 传出 COM 口。")
                return
            self.connection_text.set("正在连接")
            self.session = BluetoothSession(port, int(self.baud.get()), self.events)
            self.session.start()
        self._buttons()

    def _start_speed(self):
        try:
            speed = int(self.speed.get())
            if not 1 <= speed <= 18:
                raise ValueError
            return speed
        except ValueError:
            messagebox.showwarning("速度输入错误", "启动速度应为1～18的整数，当前默认17。")
            return None

    def start_motor(self):
        if self.connected:
            speed = self._start_speed()
            if speed is not None:
                self.session.request_motor(speed)
                self.log(f"请求启动电机，速度上限 {speed}；等待车端确认。")

    def stop_motor(self):
        # 关闭不读速度输入框：输入有误也不妨碍关闭；手动关闭取消待执行的自动启动。
        if self.connected:
            self.session.request_motor(0)
            self.log("请求关闭电机，等待车端确认。")

    def start_detection(self):
        if not self.connected or self.capturing or self.reporting:
            return
        speed = self._start_speed()
        if speed is None:
            return
        try:
            seconds = float(self.seconds.get())
            if not 0 < seconds <= 600:
                raise ValueError
        except ValueError:
            messagebox.showwarning("时长输入错误", "采集时长应大于0且不超过600秒。")
            return
        name = time.strftime("v2_%m%d_%H%M%S") + f"_{time.monotonic_ns()}.bin"
        args = SimpleNamespace(port=self.session.port, baud=self.session.baud, sec=seconds,
                               rate=1, out=os.path.join(LOG_DIR, name), stop_file=None)
        self.capturing = True
        self.phase = "negotiating"
        self.last_report = None
        self.capture_text.set("正在协商；成功后自动启动电机")
        self.progress_text.set("")
        self.session.request_capture(args, speed)
        self.log(f"一键检测 {seconds:g} 秒，启动速度 {speed}。协商失败时不自动启动。")
        self._buttons()

    def stop_capture(self):
        if self.capturing and self.session:
            self.session.capture_stop.set()
            self.capture_text.set("已请求结束采集，正在保存和导出")
            self.log("已请求停止采集；电机启停独立控制，数据处理会自动完成。")
            self._buttons()

    def open_report(self):
        self.open_path(self.last_report if self.last_report and os.path.isfile(self.last_report) else LOG_DIR)

    def process_file(self):
        if self.capturing or self.reporting:
            return
        path = filedialog.askopenfilename(initialdir=LOG_DIR, filetypes=[("遥测原始数据", "*.bin")])
        if path:
            self._report(path)

    def _report(self, path):
        self.reporting = True
        self.capture_text.set("正在生成报告；电机按钮仍可使用")
        self._buttons()
        def work():
            report = os.path.splitext(path)[0] + "_report.md"
            try:
                result = subprocess.run([sys.executable, "-B", os.path.join(HERE, "robocup_telemetry.py"), "report", path],
                    capture_output=True, text=True, encoding="utf-8", errors="replace",
                    env=dict(os.environ, PYTHONIOENCODING="utf-8"),
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                self.events.put(("log", result.stdout + result.stderr))
                self.events.put(("reported", report if result.returncode == 0 and os.path.isfile(report) else None))
            except Exception as exc:
                self.events.put(("log", f"报告生成失败：{exc}"))
                self.events.put(("reported", None))
        threading.Thread(target=work, name="telemetry-report", daemon=True).start()

    def _event(self, kind, value):
        if kind == "log":
            if value:
                self.log(value)
        elif kind == "connected":
            self.connected = True
            self.connection_text.set(f"已连接 {value}")
            self.log(f"已连接 {value}；连接本身不改变电机目标。")
        elif kind == "disconnected":
            self.session = None
            self.connected = self.capturing = False
            self.connection_text.set("未连接")
            self.log("串口已关闭。断开蓝牙不代表电机已停止。")
        elif kind == "motor":
            self.motor_text.set(value)
        elif kind == "phase":
            self.phase = value
            text = {"negotiating": "正在协商；成功后自动启动电机", "capture": "正在采集，可随时关闭电机或结束记录",
                    "dump": "正在导出冻结点云，电机按钮仍可使用"}.get(value)
            if text:
                self.capture_text.set(text)
        elif kind == "progress":
            speed = "—" if value["speed"] is None else f'{value["speed"]:.1f}'
            self.progress_text.set(f'已收 {value["frames"]} 帧 · {value["bytes"]:,} 字节 · 采样车速 {speed}')
        elif kind == "captured":
            self.capturing = False
            if os.path.isfile(value["path"] + ".meta.json"):
                self._report(value["path"])
            else:
                self.capture_text.set("未生成采集文件，请查看操作记录")
        elif kind == "reported":
            self.reporting = False
            self.last_report = value
            self.capture_text.set("数据处理完成" if value else "报告处理失败，原始数据保留")
            if value:
                self.log("报告已生成，可直接再次检测或单独控制电机。")
        self._buttons()

    def _tick(self):
        for _ in range(100):
            try:
                self._event(*self.events.get_nowait())
            except queue.Empty:
                break
        if self.exiting and self.session is None and not self.reporting:
            self.root.destroy()
            return
        self.root.after(80, self._tick)

    def close(self):
        self.exiting = True
        if self.session:
            self.session.capture_stop.set()
            self.session.closing.set()
        self.capture_text.set("正在保存已收数据并关闭连接")
        self._buttons()


def main():
    root = tk.Tk()
    try:
        ttk.Style(root).theme_use("vista")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
