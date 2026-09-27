"""M10P serial health check. Python 3.10+, pyserial; optional Tk point cloud."""
from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import struct
import time


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad = 0
        self.discarded = 0
        self.lengths = Counter()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 4:
            pos = self.buffer.find(b"\xa5\x5a")
            if pos < 0:
                keep = int(self.buffer[-1] == 0xA5)
                n = len(self.buffer) - keep
                self.discarded += n
                del self.buffer[:n]
                break
            self.discarded += pos
            del self.buffer[:pos]
            if len(self.buffer) < 4:
                break
            size = int.from_bytes(self.buffer[2:4], "big")
            # Hardware has 158/160/162-byte frames (69/70/71 slots).
            # Keep a bounded buffer while honoring the on-wire length field.
            if size < 22 or size > 512 or size % 2:
                self.bad += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                break
            raw = bytes(self.buffer[:size])
            angle, ticks = struct.unpack(">HH", raw[4:8])
            if raw[-2:] != b"\xfa\xfb" or angle > 36000 or ticks == 0:
                self.bad += 1
                del self.buffer[0]
                continue
            del self.buffer[:size]
            self.lengths[size] += 1
            words = struct.unpack(f">{(size - 20) // 2}H", raw[8:-12])
            values = [w for w in words if w != 0xFFFF]
            points = [((angle / 100 + 15 * i / len(values)) % 360,
                       w & 0x7FFF, bool(w & 0x8000))
                      for i, w in enumerate(values)]
            frames.append((angle / 100 % 360, 2500000 / ticks, points,
                           len(words) - len(values)))
        return frames


class Stats:
    def __init__(self):
        self.frames = self.points = self.valid = self.invalid = self.zero = 0
        self.reflective = self.jumps = self.wraps = 0
        self.rpms = []
        self.sectors = set()
        self.previous = None
        self.minimum = None
        self.maximum = None
        self.last_frame_time = None
        self.max_gap = 0.0

    def add(self, frame, now):
        angle, rpm, points, invalid = frame
        if self.last_frame_time is not None:
            self.max_gap = max(self.max_gap, now - self.last_frame_time)
        self.last_frame_time = now
        self.frames += 1
        self.rpms.append(rpm)
        self.sectors.add(int(angle // 15))
        self.invalid += invalid
        self.points += len(points)
        if self.previous is not None:
            if abs((angle - self.previous) % 360 - 15) > 1:
                self.jumps += 1
            if self.previous > 300 and angle < 60:
                self.wraps += 1
        self.previous = angle
        for _, distance, reflective in points:
            self.reflective += reflective
            if distance == 0:
                self.zero += 1
                continue
            self.valid += 1
            self.minimum = distance if self.minimum is None else min(self.minimum, distance)
            self.maximum = distance if self.maximum is None else max(self.maximum, distance)

    def summary(self, parser, elapsed, byte_count, error=None, interrupted=False):
        rpm = statistics.mean(self.rpms) if self.rpms else 0
        hz = rpm / 60
        fps = self.frames / max(elapsed, 0.001)
        issues = []
        if error:
            issues.append(error)
        if interrupted:
            issues.append("测试被提前结束，结果不完整")
        if elapsed < 5:
            issues.append("采集不足 5 秒，建议测试至少 15 秒")
        if not self.frames:
            issues.append("未收到合法帧：检查供电、串口、接线及 512000 波特率")
        else:
            if not 10 <= hz <= 14:
                issues.append("平均转速偏离默认 12 Hz（检查范围 10–14 Hz）")
            if len(self.sectors) < 24 or self.wraps < 2:
                issues.append("未观察到充分的 360 度扫描")
            if self.jumps / max(self.frames - 1, 1) > 0.01:
                issues.append("相邻帧角度不连续超过 1%，可能丢帧")
            if parser.bad:
                issues.append("发现结构异常帧，检查通信稳定性")
            if self.max_gap > 0.5:
                issues.append("合法数据接收间隔超过 0.5 秒")
            if not 0.85 <= fps / max(hz * 24, 0.001) <= 1.15:
                issues.append("收帧速率与转速不匹配，可能丢帧或采集卡顿")
            if self.valid / max(self.points, 1) < 0.1:
                issues.append("非零测距点不足 10%，请在周围放置可测物体复测")
        return dict(status="FAIL" if error or not self.frames else "WARN" if issues else "PASS",
                    issues=issues, duration_s=round(elapsed, 3), bytes=byte_count,
                    frames=self.frames, frames_per_s=round(fps, 2),
                    points_per_s=round(self.points / max(elapsed, 0.001), 2),
                    rpm_mean=round(rpm, 2), scan_hz=round(hz, 3),
                    rpm_min=round(min(self.rpms), 2) if self.rpms else None,
                    rpm_max=round(max(self.rpms), 2) if self.rpms else None,
                    sectors_seen=len(self.sectors), rotations_observed=self.wraps,
                    angle_jumps=self.jumps, bad_frames=parser.bad,
                    discarded_bytes=parser.discarded, buffered_bytes=len(parser.buffer),
                    frame_lengths=dict(parser.lengths), nonzero_points=self.valid,
                    invalid_ffff=self.invalid, zero_points=self.zero,
                    high_reflection_points=self.reflective,
                    distance_min_mm=self.minimum, distance_max_mm=self.maximum,
                    max_receive_gap_s=round(self.max_gap, 3))


class Plot:
    def __init__(self, range_m):
        import tkinter as tk
        self.root = tk.Tk()
        self.root.title("M10P 实时点云 | 关闭窗口结束测试")
        self.canvas = tk.Canvas(self.root, width=720, height=760, bg="#101820")
        self.canvas.pack()
        self.open = True
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.range_m = range_m

    def close(self):
        self.open = False
        self.root.destroy()

    def update(self, recent, label):
        c = self.canvas
        c.delete("all")
        for i in range(1, 5):
            r = i * 80
            c.create_oval(360-r, 370-r, 360+r, 370+r, outline="#344955")
            c.create_text(365, 370-r, text=f"{self.range_m*i/4:g}m", fill="#93a8b3", anchor="w")
        c.create_line(40, 370, 680, 370, fill="#344955")
        c.create_line(360, 50, 360, 690, fill="#344955")
        c.create_text(360, 25, text="0° 前方（出线端对面） / 顺时针增大", fill="white")
        c.create_text(360, 720, text=label, fill="white")
        for _, points in recent:
            for angle, distance, reflective in points:
                if not 0 < distance <= self.range_m * 1000:
                    continue
                a = math.radians(angle)
                r = distance / (self.range_m * 1000) * 320
                x, y = 360 + r * math.sin(a), 370 - r * math.cos(a)
                c.create_oval(x-1, y-1, x+1, y+1, fill="#ffca58" if reflective else "#49e5c2", outline="")
        self.root.update()


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是大于 0 的有限数值")
    return number


def main():
    ap = argparse.ArgumentParser(description="M10P 串口自检；默认采集 15 秒并保存 JSON 报告")
    ap.add_argument("--list-ports", action="store_true", help="仅列出串口")
    ap.add_argument("--port", help="例如 COM11；只有一个串口时自动选择")
    ap.add_argument("--seconds", type=positive, default=15)
    ap.add_argument("--plot", action="store_true", help="显示实时点云")
    ap.add_argument("--range", type=positive, default=5, dest="range_m", help="点云显示半径，米")
    ap.add_argument("--output", type=Path, help="报告路径；默认保存至脚本目录 reports")
    ap.add_argument("--raw", type=Path, help="可选：保存串口原始二进制数据")
    args = ap.parse_args()
    import serial
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if args.list_ports:
        for p in ports:
            print(f"{p.device}: {p.description} [{p.hwid}]")
        if not ports:
            print("未检测到串口")
        return 0
    if not args.port:
        if len(ports) != 1:
            ap.error("请用 --list-ports 查看设备，再用 --port COMxx 指定串口")
        args.port = ports[0].device
    output = args.output or Path(__file__).parent / "reports" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    parser, stats = Parser(), Stats()
    recent = deque(maxlen=24)
    byte_count = 0
    error = None
    interrupted = False
    plot = None
    raw_file = None
    started = time.monotonic()
    try:
        if args.plot:
            plot = Plot(args.range_m)
        if args.raw:
            args.raw.parent.mkdir(parents=True, exist_ok=True)
            raw_file = args.raw.open("wb")
        with serial.Serial(args.port, 512000, timeout=0.05, bytesize=8,
                           parity="N", stopbits=1, xonxoff=False,
                           rtscts=False, dsrdtr=False) as port:
            # Windows default receive buffer can overflow while a plot is drawn.
            if hasattr(port, "set_buffer_size"):
                port.set_buffer_size(rx_size=65536, tx_size=4096)
            port.reset_input_buffer()
            print(f"采集 {args.port} / 512000 8N1，持续 {args.seconds:g} 秒；Ctrl+C 结束")
            started = time.monotonic()
            next_print, next_plot = started + 1, started
            while time.monotonic() - started < args.seconds:
                data = port.read(min(max(port.in_waiting, 1), 65536))
                now = time.monotonic()
                byte_count += len(data)
                if raw_file:
                    raw_file.write(data)
                for frame in parser.feed(data):
                    stats.add(frame, now)
                    recent.append((now, frame[2]))
                while recent and now - recent[0][0] > 0.25:
                    recent.popleft()
                label = f"{now-started:.1f}s | frames={stats.frames} | rpm={stats.rpms[-1] if stats.rpms else 0:.1f} | bad={parser.bad}"
                if now >= next_print:
                    print(label, flush=True)
                    next_print = now + 1
                if plot and now >= next_plot:
                    plot.update(recent, label)
                    next_plot = now + 0.1
                    if not plot.open:
                        interrupted = True
                        break
    except KeyboardInterrupt:
        interrupted = True
    except (serial.SerialException, OSError) as exc:
        error = f"串口/文件错误：{exc}"
    finally:
        if raw_file:
            raw_file.close()
        if plot and plot.open:
            plot.close()
    elapsed = time.monotonic() - started
    if stats.last_frame_time is not None:
        stats.max_gap = max(stats.max_gap, time.monotonic() - stats.last_frame_time)
    result = stats.summary(parser, elapsed, byte_count, error, interrupted)
    result.update(port=args.port, baudrate=512000, timestamp=datetime.now().astimezone().isoformat(),
                  note="基本通信与扫描自检；不等同于距离精度标定。协议无校验和，帧结构正确不保证每个字节正确。")
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"报告：{output.resolve()}")
    return {"PASS": 0, "WARN": 1, "FAIL": 2}[result["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
