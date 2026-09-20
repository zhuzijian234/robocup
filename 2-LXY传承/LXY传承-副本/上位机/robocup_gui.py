#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robocup_gui.py — 诊断遥测上位机 图形界面(双击 启动上位机.bat 即可,不用手打指令)

界面分区:
    ① 连接条:串口下拉(可刷新) + 波特率 + 连接/断开
    ② 操作条:采集时长 + 开始采集 / 停止 / ★一键诊断(全流程) + 生成报告 / 打开报告 / 打开文件夹
    ③ 实时计数:帧数 / 丢帧率 / 当前模式 / err / pwm / 速度
    ④ 实时波形:err · servo_pwm · pid_select · Speed 四条滚动曲线
    ⑤ 日志:连接状态、蓝牙 ASCII 回复(参数变更)、判读结论摘要

判读逻辑在 robocup_telemetry.py(与 CLI 共用),本文件只做界面与串口轮询(无线程,
用 root.after 定时器,避免线程安全问题)。
"""

import os
import sys
import time
import threading
import queue
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robocup_telemetry as core   # noqa: E402

core.setup_cjk_font()   # 必须在建 Figure 之前

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "log")
LIVE_WIN = 300            # 实时波形显示最近 N 帧(≈23 秒)
POLL_MS = 100             # 串口轮询周期


class App:
    def __init__(self, root):
        self.root = root
        root.title("RoboCup 诊断遥测上位机 — LXY传承-副本")
        root.geometry("1180x780")

        self.v2_worker = None
        self.v2_stop = None
        self.worker_events = queue.Queue()
        self.ser = None
        self.parser = core.StreamParser()
        self.raw = bytearray()
        self.vofa_buf = bytearray()   # 老遥测(VOFA+ 8通道)实时显示用, 与 raw 记录无关
        self.capturing = False
        self.recording = False
        self.t_stop = 0.0
        self.t_start = 0.0
        self.last_report = None
        self.last_bin = None
        os.makedirs(LOG_DIR, exist_ok=True)

        self._build_ui()
        self.refresh_ports()
        self.root.after(POLL_MS, self._tick)
        self.log("V2固件默认115200，先确认模块匹配；【一键诊断】自动协商会话和配置。旧固件仍可9600普通采集。")

    # ==================== 界面 ====================
    def _build_ui(self):
        # ① 连接条
        top = ttk.Frame(self.root, padding=(8, 6))
        top.pack(fill="x")
        ttk.Label(top, text="串口").pack(side="left")
        self.cb_port = ttk.Combobox(top, width=26, state="readonly")
        self.cb_port.pack(side="left", padx=(4, 2))
        ttk.Button(top, text="刷新", width=6, command=self.refresh_ports).pack(side="left")
        ttk.Label(top, text="波特率").pack(side="left", padx=(12, 0))
        self.cb_baud = ttk.Combobox(top, width=8, state="readonly",
                                    values=["115200", "9600", "230400", "57600", "38400"])
        self.cb_baud.current(0)
        self.cb_baud.pack(side="left", padx=4)
        self.btn_conn = ttk.Button(top, text="连接", width=8, command=self.toggle_conn)
        self.btn_conn.pack(side="left", padx=6)
        self.lb_conn = ttk.Label(top, text="● 未连接", foreground="gray")
        self.lb_conn.pack(side="left", padx=6)

        # ② 操作条
        bar = ttk.Frame(self.root, padding=(8, 0))
        bar.pack(fill="x")
        ttk.Label(bar, text="采集时长(秒)").pack(side="left")
        self.sp_sec = ttk.Spinbox(bar, from_=5, to=600, increment=5, width=6)
        self.sp_sec.set(30)
        self.sp_sec.pack(side="left", padx=4)

        self.btn_start = ttk.Button(bar, text="开始采集(F5)", width=14, command=self.start_capture)
        self.btn_start.pack(side="left", padx=(10, 3))
        self.btn_stop = ttk.Button(bar, text="停止", width=8, state="disabled", command=self.stop_capture)
        self.btn_stop.pack(side="left", padx=3)
        self.btn_diag = ttk.Button(bar, text="★ 一键诊断(全流程)", width=20, command=self.one_click)
        self.btn_diag.pack(side="left", padx=(12, 3))
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(bar, text="生成报告", width=10, command=self.make_report).pack(side="left", padx=3)
        ttk.Button(bar, text="打开报告", width=10, command=self.open_report).pack(side="left", padx=3)
        ttk.Button(bar, text="打开文件夹", width=11, command=lambda: self.open_path(LOG_DIR)).pack(side="left", padx=3)
        ttk.Button(bar, text="模拟数据看效果", width=15, command=self.demo).pack(side="left", padx=(12, 3))

        # ③ 实时计数
        info = ttk.Frame(self.root, padding=(10, 6))
        info.pack(fill="x")
        self.lb_count = ttk.Label(info, text="帧数 0", font=("Consolas", 11, "bold"))
        self.lb_count.pack(side="left")
        self.lb_stat = ttk.Label(info, text="", font=("Consolas", 11))
        self.lb_stat.pack(side="left", padx=18)
        self.lb_time = ttk.Label(info, text="", font=("Consolas", 11))
        self.lb_time.pack(side="right")
        self.lb_radar = ttk.Label(info, text="", font=("Consolas", 11), foreground="#0a6")
        self.lb_radar.pack(side="right", padx=18)

        # ④ 波形
        pf = ttk.Frame(self.root, padding=(8, 0))
        pf.pack(fill="both", expand=True)
        self.fig = Figure(figsize=(11.5, 4.6), dpi=100)
        self.ax = self.fig.subplots(4, 1, sharex=True)
        for a, t in zip(self.ax, ["err", "servo_pwm", "pid_select", "speed"]):
            a.set_ylabel(t, fontsize=9)
            a.grid(alpha=0.3, lw=0.5)
        self.ax[2].set_yticks(range(10))
        self.ax[3].set_xlabel("帧序号(最近 %d 帧)" % LIVE_WIN, fontsize=9)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=pf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._last_draw = 0.0

        # ⑤ 日志
        lf = ttk.LabelFrame(self.root, text="日志", padding=4)
        lf.pack(fill="both", padx=8, pady=6)
        self.txt = tk.Text(lf, height=7, wrap="none", font=("Consolas", 9))
        self.txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt["yscrollcommand"] = sb.set
        self.root.bind("<F5>", lambda e: self.start_capture())

    # ==================== 工具 ====================
    def log(self, msg):
        self.txt.insert("end", time.strftime("[%H:%M:%S] ") + msg + "\n")
        self.txt.see("end")

    def open_path(self, p):
        if not os.path.exists(p):
            messagebox.showwarning("提示", f"还没生成:{p}")
            return
        os.startfile(p)

    def log_dir(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        return LOG_DIR

    # ==================== 串口 ====================
    def refresh_ports(self):
        try:
            from serial.tools import list_ports
        except ImportError:
            self.cb_port["values"] = []
            self.log("✗ 缺少 pyserial。请用【启动上位机.bat】启动(它会自动安装),"
                     "或手动执行: python -m pip install pyserial")
            return
        ports = list(list_ports.comports())
        vals = [f"{p.device} | {p.description}" for p in ports]
        self.cb_port["values"] = vals
        if vals:
            self.cb_port.current(0)
            for i, p in enumerate(ports):
                if "blue" in (p.description or "").lower():
                    self.cb_port.current(i)
                    break
            self.log(f"发现 {len(vals)} 个串口")
        else:
            self.log("未发现串口。请先在 Windows 设置里配对 HC-05,配对成功后会出现一个传出 COM 口。")

    def _port(self):
        v = self.cb_port.get()
        return v.split("|")[0].strip() if v else ""

    def toggle_conn(self):
        if self.ser:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self._port()
        if not port:
            messagebox.showwarning("提示", "先选串口(点【刷新】)")
            return
        try:
            import serial
        except ImportError:
            messagebox.showerror("缺少 pyserial", "请用【启动上位机.bat】启动,或执行:\n"
                                                "python -m pip install pyserial")
            return
        try:
            self.ser = serial.Serial(port, int(self.cb_baud.get()), timeout=0)
        except Exception as e:
            self.ser = None
            messagebox.showerror("打开失败", f"{port} 打开失败:\n{e}\n\n"
                                            f"常见原因:已被 Vofa+/串口助手占用;或被别的程序占着。")
            return
        self.parser = core.StreamParser()
        self.raw = bytearray()
        self.vofa_buf = bytearray()
        self.btn_conn["text"] = "断开"
        self.lb_conn["text"] = f"● 已连接 {port}"
        self.lb_conn["foreground"] = "green"
        self.log(f"已打开 {port} @ {self.cb_baud.get()}")

    def disconnect(self):
        if self.capturing:
            self.stop_capture()
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn_conn["text"] = "连接"
        self.lb_conn["text"] = "● 未连接"
        self.lb_conn["foreground"] = "gray"
        self.log("已断开")

    def send(self, cmd):
        if not self.ser:
            return False
        try:
            self.ser.write(cmd.encode("ascii"))
            self.log(f"→ {cmd.strip()}")
            return True
        except Exception as e:
            self.log(f"✗ 发送失败:{e}")
            return False

    # ==================== 采集 ====================
    def start_capture(self, send_tele=False):
        if send_tele:
            return self.start_v2_capture()
        if not self.ser:
            messagebox.showwarning("提示", "先点【连接】")
            return
        if self.capturing:
            return
        self.parser = core.StreamParser()
        self.raw = bytearray()
        self.rx_events = []
        self.vofa_buf = bytearray()
        self.diag_pending = send_tele
        if send_tele and not self.send("tele 2\n"):
            return
        self.capturing = True
        self.t_start = time.monotonic()
        self.t_stop = self.t_start + float(self.sp_sec.get())
        self.btn_start["state"] = "disabled"
        self.btn_diag["state"] = "disabled"
        self.btn_stop["state"] = "normal"
        self.log(f"开始采集 {self.sp_sec.get()} 秒 …(记录到 {LOG_DIR})")

    def stop_capture(self, quiet=False):
        if getattr(self,"v2_worker",None) is not None:
            with open(self.v2_stop,"w") as f:f.write("stop")
            self.log("已请求停止，等待原始文件和元数据写入完成")
            return None
        self.capturing = False
        self.diag_pending = False
        self.btn_start["state"] = "normal"
        self.btn_diag["state"] = "normal"
        self.btn_stop["state"] = "disabled"
        n = self.parser.total_frames
        dur = max(0.0, time.monotonic() - self.t_start)
        if n == 0 and not self.raw:
            self.log("⚠ 本次没有收到任何数据 — 检查串口是否选对、蓝牙是否已连接")
            return None
        name = time.strftime("run_%m%d_%H%M%S") + f"_{time.monotonic_ns()}"
        binp = os.path.join(self.log_dir(), name + ".bin")
        core.save_capture(binp, self.raw, self.rx_events, dur)
        self.parser.capture_duration = dur
        self.last_bin = binp
        if n == 0:
            self.log(f"停止采集:未解析到57字节诊断帧,但原始数据已存 {len(self.raw)} 字节"
                     f"(时长 {dur:.1f}s) → {os.path.basename(binp)}")
            self.log(f"  若是老遥测(VOFA+ 8通道), 解码看雷达转速:")
            self.log(f"  python robocup_telemetry.py vofa \"{binp}\" --sec {dur:.1f}")
        else:
            self.log(f"停止采集:{n} 帧,{len(self.raw)} 字节,时长 {dur:.1f}s "
                     f"→ {os.path.basename(binp)}")
        return binp

    def start_v2_capture(self):
        if self.capturing or self.v2_worker is not None:return
        if not self.ser:
            messagebox.showwarning("提示","先连接串口")
            return
        port=self.ser.port;baud=int(self.cb_baud.get());seconds=float(self.sp_sec.get())
        self.disconnect()  # subprocess is the sole serial owner during this run
        stamp=time.strftime("v2_%m%d_%H%M%S")+f"_{time.monotonic_ns()}"
        binp=os.path.join(self.log_dir(),stamp+".bin")
        self.last_report=None;self.last_bin=None
        self.v2_stop=binp+".stop"
        self.capturing=True
        self.btn_start["state"]="disabled";self.btn_diag["state"]="disabled";self.btn_stop["state"]="normal"
        self.btn_conn["state"]="disabled"
        def work():
            flags=getattr(subprocess,"CREATE_NO_WINDOW",0)
            command=[sys.executable,"-B",os.path.join(HERE,"robocup_telemetry.py"),"capture-v2",
                     "--port",port,"--baud",str(baud),"--sec",str(seconds),"--out",binp,"--stop-file",self.v2_stop]
            try:
                result=subprocess.run(command,capture_output=True,text=True,encoding="utf-8",errors="replace",
                                      env=dict(os.environ,PYTHONIOENCODING="utf-8"),creationflags=flags)
                self.worker_events.put(("log",result.stdout+result.stderr))
                if os.path.exists(binp+".meta.json"):
                    report=subprocess.run([sys.executable,"-B",os.path.join(HERE,"robocup_telemetry.py"),"report",binp],
                         capture_output=True,text=True,encoding="utf-8",errors="replace",creationflags=flags,
                         env=dict(os.environ,PYTHONIOENCODING="utf-8"))
                    self.worker_events.put(("log",report.stdout+report.stderr))
                self.worker_events.put(("done",binp))
            except Exception as exc:
                self.worker_events.put(("log",str(exc)));self.worker_events.put(("done",None))
        self.v2_worker=threading.Thread(target=work,daemon=True);self.v2_worker.start()
        self.log("V2后台独占串口：协商→采集→保存→报告；完成前可点停止。过程中不显示实时曲线。")

    def one_click(self):
        """★ 全流程:切诊断流 → 采集 → 停止 → 出报告 → 打开"""
        if not self.ser:
            messagebox.showwarning("提示", "先点【连接】")
            return
        self.log("=== 一键诊断开始 ===")
        self.start_capture(send_tele=True)

    def _finish_auto(self):
        binp = self.stop_capture()
        if binp and self.parser.total_frames:      # 老遥测流出不了57字节帧报告
            self.make_report(binp)
            self.open_report()

    # ==================== 报告 ====================
    def make_report(self, binp=None):
        if self.capturing:
            self.log("请结束采集后生成报告；V2一键诊断会自动生成。")
            return
        binp = binp or self.last_bin
        if not binp:
            b = filedialog.askopenfilename(initialdir=self.log_dir(),
                                           filetypes=[("遥测数据", "*.bin"), ("全部", "*.*")])
            if not b:
                return
            binp = b
        try:
            p = core.parse_file(binp)
            res = core.analyze(p)
            base = os.path.splitext(binp)[0]
            core.write_csv(p, base + ".csv")
            core.write_report(res, base + "_report.md", os.path.basename(binp))
            try:
                core.write_plot(p, res, base + ".png")
            except Exception as e:
                self.log(f"(图生成跳过:{e})")
            self.last_report = base + "_report.md"
            self.log(f"报告已生成:{os.path.basename(self.last_report)}")
            self._summarize(res)
        except Exception as e:
            self.log(f"✗ 报告生成失败:{e}")

    def _summarize(self, res):
        if res["n_frames"] == 0:
            self.log("判读:无有效帧")
            return
        if res.get("v2"):
            self.log(f"V2判读：{res['n_frames']}条CONTROL，未知配置{res['unknown_config']}条")
            return
        L = res["link"]
        self.log(f"判读:接收帧率 {L['fps']:.1f}/s（nan=无时间记录）,丢帧 {L['drop_rate']:.2f}%,"
                 f"饱和 {res['err_sat']['overall_pct']:.1f}%,"
                 f"方向异常 {res['dir_anomaly']['runs_ge3']} 段,"
                 f"强制打满 {res['carry']['frames']} 帧,"
                 f"PWM 猛甩(>250) {res['pwm_jumps']['gt250']} 帧,"
                 f"出弯反踢 {res['exit_kick']['kicks']} 次")
        for s in res["segments"][:3]:
            self.log(f"  可疑片段:帧 {s['start']}~{s['end']-1} "
                     f"({s['t0']:.1f}s,评分{s['score']:.0f},|err|峰 {s['max_abs_err']:.0f})")

    def open_report(self):
        if self.last_report and os.path.exists(self.last_report):
            self.open_path(self.last_report)
        else:
            self.open_path(LOG_DIR)

    def demo(self):
        """无车也能看效果:生成模拟数据 → 出报告 → 打开"""
        binp = os.path.join(self.log_dir(), "demo.bin")
        core.make_demo_bin(binp, seconds=60.0)
        self.log("已生成模拟数据 demo.bin(含注入的典型病症)")
        self.make_report(binp)

    # ==================== 定时轮询 ====================
    def _tick(self):
        while not self.worker_events.empty():
            kind,value=self.worker_events.get_nowait()
            if kind=="log":self.log(value)
            else:
                self.v2_worker=None;self.capturing=False
                self.btn_start["state"]="normal";self.btn_diag["state"]="normal";self.btn_stop["state"]="disabled";self.btn_conn["state"]="normal"
                if value:
                    self.last_bin=value;report=os.path.splitext(value)[0]+"_report.md"
                    if os.path.exists(report):self.last_report=report
                self.log("V2任务结束，串口已关闭；下一次采集请重新连接。")
        if self.v2_worker is not None:
            self.lb_time["text"]="V2后台采集/生成报告中"
            self.root.after(POLL_MS,self._tick)
            return
        try:
            if self.ser:
                n = 0
                try:
                    n = self.ser.in_waiting
                except Exception:
                    n = 0
                if n:
                    chunk = self.ser.read(min(n, 8192))
                    if chunk:
                        if self.capturing:
                            elapsed = time.monotonic() - self.t_start
                            self.rx_events.append({"offset": len(self.raw), "length": len(chunk), "t": elapsed})
                            self.parser.feed(chunk, elapsed)
                        self.vofa_buf += chunk
                        if len(self.vofa_buf) > 16384:
                            del self.vofa_buf[:-8192]
                        if self.capturing:
                            self.raw += chunk
        except Exception as e:
            self.log(f"✗ 串口异常,已断开:{e}")
            self.disconnect()

        if self.capturing and self.diag_pending:
            if self.parser.total_frames:
                self.diag_pending = False
                self.log("已收到有效V1诊断帧，诊断流确认")
            elif time.monotonic() - self.t_start >= 3:
                self.log("诊断切换未确认：无有效V1帧。现役固件不支持tele 2；保留原始回复，请用普通采集。")
                self.stop_capture()

        # 自动停止
        if self.capturing and time.monotonic() >= self.t_stop:
            self._finish_auto()

        self._update_labels()
        if time.monotonic() - self._last_draw > 0.2:
            self._update_plot()
            self._last_draw = time.monotonic()
        self.root.after(POLL_MS, self._tick)

    def _update_labels(self):
        p = self.parser
        self.lb_count["text"] = f"帧数 {p.total_frames}"
        if p.total_frames:
            total = p.total_frames + p.dropped
            r = p.records[-1]
            mode = int(r["pid_select"]) if core.math.isfinite(r["pid_select"]) else -1
            self.lb_stat["text"] = (f"丢帧 {100.0*p.dropped/total:.1f}%  坏帧 {p.bad_frames}   "
                                    f"模式 {mode}({core.MODE_NAME.get(mode,'?')})   "
                                    f"err {r['err']:+.0f}   pwm {r['servo_pwm']:.0f}   "
                                    f"速度 {r['Speed_now']:.1f}/{r['Speed_mubiao']:.0f}")
        if self.capturing:
            left = max(0.0, self.t_stop - time.monotonic())
            self.lb_time["text"] = f"采集中… 剩余 {left:.0f} s"
        elif p.total_frames:
            self.lb_time["text"] = (f"采集时长 {p.capture_duration:.1f} s" if p.capture_duration is not None else "采集中")

        # 老遥测(VOFA+ 8通道)实时读数: 第8通道 = 雷达转速(度/秒)
        # 要求至少3帧且间距正常, 免得把诊断流(57字节帧)里的巧合字节当成帧
        txt = ""
        if len(self.vofa_buf) >= core.VOFA_LEN * 3:
            rows, _bad = core.parse_vofa_bytes(bytes(self.vofa_buf))
            if len(rows) >= 3:
                dps = rows[-1][7]
                txt = f"雷达 {dps:.0f}°/s = {dps/360:.2f} Hz"
        self.lb_radar["text"] = txt

    def _update_plot(self):
        recs = self.parser.records[-LIVE_WIN:]
        n = len(recs)
        x = list(range(n))
        for i, key in enumerate(["err", "servo_pwm", "pid_select", "Speed_now"]):
            y = [r[key] for r in recs]
            a = self.ax[i]
            if a.lines:
                a.lines[0].set_data(x, y)
            else:
                a.plot(x, y, lw=1.0)
            if n:
                a.set_xlim(0, max(LIVE_WIN, n))
                finite = [v for v in y if core.math.isfinite(v)]
                lo, hi = (min(finite), max(finite)) if finite else (0, 1)
                pad = (hi - lo) * 0.15 + 1.0
                a.set_ylim(lo - pad, hi + pad)
        self.canvas.draw_idle()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
