#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robocup_telemetry.py — 诊断遥测核心库(解析 / 判读 / 报告),CLI + GUI 共用

帧格式(定长 57B,旧版诊断格式；现役固件仅发送36B JustFloat):
    [0]=0xAA [1]=0x55 [2]=ver(0x01) [3]=seq
    [4..55]  26 × int16 小端(定标见 FIELD_TABLE)
    [56]     校验和 = byte[4..55] 累加 & 0xFF

用法(CLI):
    python robocup_telemetry.py list
    python robocup_telemetry.py capture --port COM5 --sec 30 --name lap1
    python robocup_telemetry.py report log/lap1.bin
    python robocup_telemetry.py demo            # 生成模拟数据并出报告(无车也能验证工具)

图形界面: robocup_gui.py(双击 启动上位机.bat)

设计依据: 《诊断遥测方案.md》 《诊断遥测实施方案.md》
"""

import os
import sys
import csv
import time
import struct
import argparse
import telemetry_v2 as v2
import json
import math
import hashlib
from pathlib import Path

import numpy as np

# ============================ 协议常量 ============================

VER = 0x01
FRAME_LEN = 57
HEADER = b"\xAA\x55"
PAYLOAD_OFF = 4
FIELD_N = 26

# (字段名, 定标指数) —— 顺序 = 帧内顺序 = CSV 列顺序, 必须与 ble_diag.c 字段表一致
FIELD_TABLE = [
    ("err",               1),   #  0  实际值 = 存值 / 10^scale
    ("servo_pwm",         0),   #  1
    ("pid_select",        0),   #  2
    ("Midline.k",         3),   #  3
    ("Midline.b",         0),   #  4
    ("Forward.k",         3),   #  5
    ("Forward.b",         0),   #  6
    ("Forward_2.k",       3),   #  7
    ("Forward_3.k",       3),   #  8
    ("RIGHT_duandian",    0),   #  9
    ("LEFT_duandian",     0),   # 10
    ("paodao_distance",   0),   # 11
    ("valid_couter",      0),   # 12
    ("RIGHT_cnt",         0),   # 13
    ("LEFT_cnt",          0),   # 14
    ("Forward_cnt",       0),   # 15
    ("Forward_cnt_2",     0),   # 16
    ("Forward_cnt_3",     0),   # 17
    ("Speed_now",         1),   # 18
    ("Speed_mubiao",      0),   # 19
    ("danbian_flag",      0),   # 20
    ("state_left_cnt",    0),   # 21  直道强制打满右残留
    ("state_right_cnt",   0),   # 22  直道强制打满左残留
    ("state_left_cnt_2",  0),   # 23
    ("state_right_cnt_2", 0),   # 24
    ("zhongxian_chuizhi", 0),   # 25
]
FIELD_NAMES = [n for n, _ in FIELD_TABLE]
SCALES = np.array([10.0 ** s for _, s in FIELD_TABLE], dtype=np.float64)

MODE_NAME = {
    0: "直道", 1: "小右", 2: "小左", 3: "大右", 4: "大左",
    5: "垂线", 6: "均值A", 7: "均值B", 8: "中右", 9: "中左",
}
CORNER_MODES_RIGHT = (3, 8)      # 期望 err <= 0 (右转)
CORNER_MODES_LEFT = (4, 9)       # 期望 err >= 0 (左转)
TURN_MODES = (1, 2, 3, 4, 8, 9)
STRAIGHT_MODES = (0, 5)
ERR_SAT = 490.0                  # |err| 达到此值算饱和(全局限幅 ±500)
JITTER_MM = 150.0                # 断点相邻帧跳变阈值

# ---- 帧周期推导(时间轴用) ----
# LD14P 按6Hz标称工况假设4000点/秒，其他转速下恒定性未实测; 每包 47 B 装 12 点 → 包率 333.33 包/秒
#   → 字节率 = 4000/12 × 47 = 15,666.7 B/s
# 车端 DMA 是"缓冲满 1798 B 才拷贝重启"(HARDWARE/DMA/DMA.c:78-86), 不是按整圈停
#   → 帧周期 = 1798 / 15666.7 = 114.8 ms = 8.71 帧/秒
# 注意: 若车端改了 DMA_USART2_RX_BUF_LEN, 还需核实字节率；换雷达必须同时更新协议和速率假设
DMA_BUF_LEN = 1798
LD_BYTES_PER_S = 4000.0 / 12 * 47          # ≈15666.7 B/s
FRAME_DT = DMA_BUF_LEN / LD_BYTES_PER_S    # ≈0.1148 s


def checksum(payload: bytes) -> int:
    return sum(payload) & 0xFF


PHYSICAL_LIMITS = {"err": 600, "Midline.k": 30, "Forward.k": 30,
                   "Forward_2.k": 30, "Forward_3.k": 30,
                   "Midline.b": 30000, "Forward.b": 30000}


def encode_field(name, value):
    """返回 (int16存值, valid, clipped)，binary32运算、半整数远离零。"""
    scale = dict(FIELD_TABLE)[name]
    with np.errstate(over="ignore", invalid="ignore"):
        value = float(np.float32(value))
    if not math.isfinite(value):
        return 0, False, False
    if name == "pid_select" and (value != int(value) or not 0 <= value <= 9):
        return 0, False, False
    if name == "danbian_flag" and value not in (0, 1):
        return 0, False, False
    if name not in PHYSICAL_LIMITS and name != "Speed_now" and value < 0:
        return 0, False, False
    limit = PHYSICAL_LIMITS.get(name)
    clipped = limit is not None and abs(value) > limit
    if limit is not None:
        value = max(-limit, min(limit, value))
    with np.errstate(over="ignore"):
        scaled = float(np.float32(np.float32(value) * np.float32(10 ** scale)))
    if scaled > 32767:
        return 32767, True, True
    if scaled < -32768:
        return -32768, True, True
    iv = math.floor(scaled + 0.5) if scaled >= 0 else math.ceil(scaled - 0.5)
    return iv, True, clipped


def setup_cjk_font():
    """让 matplotlib 能显示中文(必须在创建 Figure 之前调用)"""
    import matplotlib
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False


# ============================ 解析器(增量, 可喂实时流) ============================

class StreamParser:
    """增量解析: 反复 feed() 字节块, 记录累积在 .records / .ascii_lines 等"""

    def __init__(self):
        self.v2 = v2.Decoder()
        self.meta = {}
        self.buf = bytearray()
        self.records = []      # list[dict]
        self.ascii_lines = []  # 蓝牙 ASCII 回复(参数变更等)
        self.bad_frames = 0    # 校验失败帧数
        self.dropped = 0       # 按 seq 推算的丢帧数
        self._ascii = bytearray()
        self._prev_seq = None
        self._n = 0
        self.received_bytes = 0
        self.capture_duration = None
        self._host_t = None

    # ---- 内部 ----
    def _note_ascii(self, raw: bytes):
        if not raw:
            return
        self._ascii.extend(raw)
        while True:
            tail=self._ascii.find(b"\x00\x00\x80\x7f")
            newline=self._ascii.find(b"\n")
            if tail>=0 and (newline<0 or tail<newline):
                del self._ascii[:tail+4]
                continue
            if newline<0:break
            line, _, rest = self._ascii.partition(b"\n")
            self._ascii = bytearray(rest)
            try:
                s = line.strip(b"\r").decode("ascii").strip()
            except UnicodeDecodeError:
                continue
            if s.startswith(("INFO ", "OK ", "ERR ", "radar ")) or any(s.startswith(n+"=") for n in v2.PARAMETERS):
                self.ascii_lines.append({"t": self._host_t if self._host_t is not None else self._n * FRAME_DT, "text": s})
        if len(self._ascii)>256:self._ascii.clear()

    def _add_frame(self, frame: bytes):
        vals = np.frombuffer(frame[PAYLOAD_OFF:PAYLOAD_OFF + FIELD_N * 2],
                             dtype="<i2").astype(np.float64) / SCALES
        seq = frame[3]
        if self._prev_seq is not None:
            gap = (seq - self._prev_seq - 1) % 256
            if 0 < gap < 128:
                self.dropped += gap
        self._prev_seq = seq
        rec = {"seq": seq, "t": self._host_t if self._host_t is not None else self._n * FRAME_DT}
        rec.update({name: float(vals[i]) for i, name in enumerate(FIELD_NAMES)})
        self.records.append(rec)
        self._n += 1

    # ---- 对外 ----
    def feed(self, chunk: bytes, host_t=None):
        self._host_t = host_t
        self.received_bytes += len(chunk)
        self.buf.extend(chunk)
        while True:
            idx = self.buf.find(HEADER)
            if idx < 0:
                keep = 1 if (len(self.buf) and self.buf[-1] == 0xAA) else 0
                self._note_ascii(bytes(self.buf[:len(self.buf) - keep]))
                del self.buf[:len(self.buf) - keep]
                return
            if idx:
                self._note_ascii(bytes(self.buf[:idx]))
                del self.buf[:idx]
            if len(self.buf)<3:return
            if self.buf[2]==2:
                if len(self.buf)<6:return
                n=int.from_bytes(self.buf[4:6],"little")
                if not 16<=n<=256:
                    self.bad_frames+=1;del self.buf[0];continue
                if len(self.buf)<n:return
                try:
                    rec=self.v2.accept(bytes(self.buf[:n]),FIELD_NAMES,SCALES,self._host_t)
                except (ValueError, UnicodeError, struct.error):
                    self.bad_frames+=1;del self.buf[0];continue
                del self.buf[:n]
                if rec is not None:self.records.append(rec)
                continue
            if self.buf[2]!=1:
                self.bad_frames+=1;del self.buf[0];continue
            if len(self.buf) < FRAME_LEN:
                return
            frame = bytes(self.buf[:FRAME_LEN])
            if frame[2] == VER and checksum(frame[PAYLOAD_OFF:FRAME_LEN - 1]) == frame[FRAME_LEN - 1]:
                self._add_frame(frame)
                del self.buf[:FRAME_LEN]
            else:
                self.bad_frames += 1
                self._note_ascii(self.buf[:1])
                del self.buf[:1]

    @property
    def total_frames(self):
        return len(self.records)

    def cols(self, name):
        return np.array([r[name] for r in self.records], dtype=np.float64)

    def matrix(self):
        """返回 (N, 26) 矩阵, 列顺序 = FIELD_NAMES"""
        if not self.records:
            return np.zeros((0, FIELD_N))
        return np.array([[r[n] for n in FIELD_NAMES] for r in self.records],
                        dtype=np.float64)


def save_capture(path, raw, events, duration, metadata=None):
    """保存原始流及接收块时间；时间为主机单调钟相对秒，不是控制周期。"""
    raw = bytes(raw)
    with open(path, "wb") as f:
        f.write(raw)
    with open(path + ".rx.jsonl", "w", encoding="utf-8") as f:
        for event in events:
            f.write(json.dumps(event) + "\n")
    with open(path + ".meta.json", "w", encoding="utf-8") as f:
        json.dump({"version": 1, "clock": "host_monotonic", "duration_s": duration,
                   "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest(), **(metadata or {})}, f)


def parse_file(path: str) -> StreamParser:
    path = os.fspath(path)
    p = StreamParser()
    with open(path, "rb") as f:
        raw = f.read()
    if os.path.exists(path + ".meta.json"):
        with open(path + ".meta.json", encoding="utf-8") as f:
            meta = json.load(f)
        duration = meta["duration_s"]
        if (meta.get("version") != 1 or meta.get("clock") != "host_monotonic"
                or not math.isfinite(duration) or duration <= 0
                or meta["bytes"] != len(raw)
                or meta["sha256"] != hashlib.sha256(raw).hexdigest()):
            raise ValueError("采集元数据与原始文件不匹配或时长无效")
        offset, previous = 0, 0.0
        with open(path + ".rx.jsonl", encoding="utf-8") as f:
            for line in f:
                e = json.loads(line)
                n, t = e["length"], e["t"]
                if (not isinstance(n, int) or n <= 0 or e["offset"] != offset
                        or offset + n > len(raw) or not math.isfinite(t)
                        or not previous <= t <= duration):
                    raise ValueError("接收块时间或偏移无效")
                p.feed(raw[offset:offset+n], t)
                offset += n
                previous = t
        if offset != len(raw):
            raise ValueError("接收块记录不完整")
        p.capture_duration = duration
        p.meta = meta
    else:
        p.feed(raw)
    return p


# =============== VOFA+ JustFloat 流解析(老遥测, 8通道) ===============
# 固件 HARDWARE/hc-05/ble_tune.c 的 BLE_Tune_Telemetry() 每雷达帧发一次:
#   float32 小端 × 8 通道 + 帧尾 00 00 80 7F, 共 36 字节
# 通道顺序: err / servo_pwm / kp / kd / pid_select / Speed_now / Speed_mubiao / 雷达转速(度/秒)
# 用途: 读第 8 通道核对雷达实际转速(6Hz=2160, 8Hz=2880), 验证 PWM 控速是否生效。
# 局限: 这个流不含时间戳, 帧周期只能用"采集时长 ÷ 帧数"估算 → 需要 --sec。

VOFA_CH = ["err", "servo_pwm", "kp", "kd", "pid_select",
           "Speed_now", "Speed_mubiao", "radar_dps"]
VOFA_LEN = len(VOFA_CH) * 4 + 4          # 36
VOFA_TAIL = struct.pack("<f", float("inf"))


def parse_vofa_bytes(data: bytes, nch: int = 8):
    """从原始字节流里抽出 JustFloat 帧。

    返回 (rows, n_bad): rows = [[ch0..ch7], ...], n_bad = 疑似错帧数。
    帧尾标记 00 00 80 7F 作为锚点; 帧尾间距不足 nch*4 字节的判为错帧(丢字节/半帧)。
    """
    rows, n_bad, pos, last_end = [], 0, 0, 0
    while True:
        idx = data.find(VOFA_TAIL, pos)
        if idx < 0:
            break
        st = idx - nch * 4
        if st >= 0 and st >= last_end:
            rows.append(list(struct.unpack_from("<%df" % nch, data, st)))
            last_end = idx + 4
        else:
            n_bad += 1                        # 帧尾挨得太近: 前面少了字节
        pos = idx + 4
    return rows, n_bad


def parse_vofa_file(path: str, nch: int = 8):
    with open(path, "rb") as f:
        return parse_vofa_bytes(f.read(), nch)


def cmd_vofa(a):
    rows, n_bad = parse_vofa_file(a.bin, a.nch)
    if not rows:
        print(f"{a.bin}: 没找到任何 JustFloat 帧(共 {os.path.getsize(a.bin)} 字节)。")
        print("  检查: 车端蓝牙是否连上(未连接不发遥测); 波特率是否为 9600; 是否采到了数据。")
        return 1
    print(f"{a.bin} → {len(rows)} 帧 JustFloat({a.nch}通道, 每帧{a.nch*4+4}字节)"
          + (f", {n_bad} 处疑似错帧" if n_bad else ""))
    duration = a.sec
    if os.path.exists(a.bin + ".meta.json"):
        duration = parse_file(a.bin).capture_duration
    if duration:
        fp = duration / len(rows)
        print(f"  采集时长 {duration:.3f}s ÷ {len(rows)} 帧 → 帧周期 ≈ {fp*1000:.1f} ms"
              f"({1/fp:.1f} 帧/秒)")
    else:
        print("  (加 --sec 采集秒数 可算出帧周期)")
    print(f"  {'通道':<12}{'最小':>10}{'最大':>10}{'平均':>10}{'末值':>10}")
    for i, name in enumerate(VOFA_CH[:a.nch]):
        col = [r[i] for r in rows if math.isfinite(r[i])]
        if not col:
            print(f"  {name:<12} 无有效测量")
            continue
        print(f"  {name:<12}{min(col):>10.2f}{max(col):>10.2f}"
              f"{sum(col)/len(col):>10.2f}{col[-1]:>10.2f}")
    if a.nch >= 8:
        d = [r[7] for r in rows]
        avg = sum(d) / len(d)
        print(f"  → 雷达转速均值 {avg:.0f} 度/秒 = {avg/360:.2f} Hz"
              f"(6Hz=2160, 8Hz=2880; 度/秒与Hz差360倍)")
    return 0


# ============================ 判读 ============================

def _runs(mask: np.ndarray):
    """把 bool 数组切成连续 True 的区间 [(start, end_exclusive), ...]"""
    if len(mask) == 0:
        return []
    d = np.diff(mask.astype(np.int8))
    starts = list(np.where(d == 1)[0] + 1) + ([0] if mask[0] else [])
    ends = list(np.where(d == -1)[0] + 1) + ([len(mask)] if mask[-1] else [])
    starts.sort()
    ends.sort()
    return list(zip(starts, ends))


def analyze(p: StreamParser) -> dict:
    """对解析结果做 10 项判读, 返回结果字典(供报告/图形使用)"""
    if p.v2.messages:
        return v2.analyze(p,FIELD_NAMES)
    M = p.matrix()
    N = len(M)
    res = {"n_frames": N, "bad_frames": p.bad_frames, "dropped": p.dropped,
           "ascii_events": p.ascii_lines, "duration_s": p.capture_duration if p.capture_duration is not None else N * FRAME_DT,
           "measured": p.capture_duration is not None, "times": [r["t"] for r in p.records]}
    if N == 0:
        return res

    idx = {n: i for i, n in enumerate(FIELD_NAMES)}
    err = M[:, idx["err"]]
    pwm = M[:, idx["servo_pwm"]]
    mode = M[:, idx["pid_select"]].astype(int)
    fk = M[:, idx["Forward.k"]]
    dj_r = M[:, idx["RIGHT_duandian"]]
    dj_l = M[:, idx["LEFT_duandian"]]
    sl = M[:, idx["state_left_cnt"]]
    sr = M[:, idx["state_right_cnt"]]
    spd = M[:, idx["Speed_now"]]

    # ---- 1. 链路 ----
    total = N + p.dropped
    res["link"] = {
        "fps": N / res["duration_s"] if res["measured"] and res["duration_s"] else float("nan"),
        "drop_rate": 100.0 * p.dropped / total if total else 0,
        "bad_rate": 100.0 * p.bad_frames / (N + p.bad_frames) if (N + p.bad_frames) else 0,
        "bytes_per_s": p.received_bytes / res["duration_s"] if res["measured"] and res["duration_s"] else float("nan"),
    }

    # ---- 2. 模式分布 ----
    dist = {}
    for m in np.unique(mode):
        c = int(np.sum(mode == m))
        dist[int(m)] = {"count": c, "pct": 100.0 * c / N}
    switches = int(np.sum(mode[1:] != mode[:-1]))
    res["modes"] = {"dist": dist, "switches": switches,
                    "switch_rate": switches / res["duration_s"] if res["measured"] and res["duration_s"] else float("nan")}

    # ---- 3. err 饱和(分模式) ----
    sat = {}
    for m in np.unique(mode):
        sel = mode == m
        sat[int(m)] = {"pct": 100.0 * float(np.mean(np.abs(err[sel]) >= ERR_SAT)),
                       "count": int(np.sum(np.abs(err[sel]) >= ERR_SAT)),
                       "n": int(np.sum(sel))}
    res["err_sat"] = {"by_mode": sat, "overall_pct": 100.0 * float(np.mean(np.abs(err) >= ERR_SAT))}

    # ---- 4. 弯道 k 分布(模式 3/4/8/9) ----
    corner = np.isin(mode, (3, 4, 8, 9))
    ak = np.abs(fk[corner])
    if len(ak):
        res["corner_k"] = {
            "n": int(len(ak)),
            "bands": {"<0.35 (3/4档)": int(np.sum(ak < 0.35)),
                      "0.35~0.7 (8/9档)": int(np.sum((ak >= 0.35) & (ak < 0.7))),
                      ">=0.7 (C类)": int(np.sum(ak >= 0.7))},
            "median": float(np.median(ak)), "max": float(np.max(ak)),
        }
    else:
        res["corner_k"] = {"n": 0}

    # ---- 5. 方向异常 ----
    dir_bad = np.zeros(N, dtype=bool)
    dir_bad |= np.isin(mode, CORNER_MODES_RIGHT) & (err > 0)
    dir_bad |= np.isin(mode, CORNER_MODES_LEFT) & (err < 0)
    runs = [(s, e) for s, e in _runs(dir_bad) if e - s >= 3]
    res["dir_anomaly"] = {
        "frames": int(np.sum(dir_bad)), "pct": 100.0 * float(np.mean(dir_bad)),
        "runs_ge3": len(runs), "runs": runs[:20],
        "note": "模式3/8 期望 err<=0, 模式4/9 期望 err>=0",
    }

    # ---- 6. 出弯反踢(转弯→直道那一帧的 Δ) ----
    prev_mode = np.concatenate([[mode[0]], mode[:-1]])
    d_pwm = np.concatenate([[0.0], np.diff(pwm)])
    d_err = np.concatenate([[0.0], np.diff(err)])
    exits = np.isin(prev_mode, TURN_MODES) & np.isin(mode, STRAIGHT_MODES)
    kick = exits & (np.abs(d_pwm) > 100)
    res["exit_kick"] = {
        "exits": int(np.sum(exits)), "kicks": int(np.sum(kick)),
        "worst_d_pwm": float(np.max(np.abs(d_pwm[exits]))) if np.any(exits) else 0.0,
        "worst_d_err": float(np.max(np.abs(d_err[exits]))) if np.any(exits) else 0.0,
        "frames": np.where(kick)[0].tolist()[:20],
    }

    # ---- 6b. 全场最大 PWM 跳变(不限模式切换: 抓"舵机突然甩一下") ----
    abs_d = np.abs(d_pwm)
    jumps = []
    for k in np.argsort(abs_d)[::-1][:5]:
        if abs_d[k] < 100 or k == 0:
            break
        jumps.append({"frame": int(k), "t": p.records[k]["t"], "d_pwm": float(d_pwm[k]),
                      "mode_prev": int(prev_mode[k]), "mode": int(mode[k]),
                      "err_prev": float(err[k - 1]), "err": float(err[k])})
    res["pwm_jumps"] = {"gt100": int(np.sum(abs_d > 100)), "gt250": int(np.sum(abs_d > 250)),
                        "top": jumps}

    # ---- 7. 强制打满残留 ----
    carry = (sl > 0) | (sr > 0)
    res["carry"] = {
        "frames": int(np.sum(carry)), "pct": 100.0 * float(np.mean(carry)),
        "runs": [(s, e) for s, e in _runs(carry)][:20],
        "zero_mode_in_carry": int(np.sum(carry & (mode == 0))),
        "note": "残留生效时不调用 Midline_PD: err 保持上帧旧值, pid_select=0",
    }

    # ---- 8. 断点抖动 ----
    res["jitter"] = {}
    for nm, arr in (("RIGHT", dj_r), ("LEFT", dj_l)):
        if len(arr) > 1:
            d = np.abs(np.diff(arr))
            res["jitter"][nm] = {"count": int(np.sum((d > JITTER_MM))),
                                 "max": float(np.max(d)),
                                 "frames": (np.where(d > JITTER_MM)[0] + 1).tolist()[:20]}

    # ---- 9. 速度环 ----
    tgt = M[:, idx["Speed_mubiao"]]
    se = spd - tgt
    # 5 帧滑动平均后再数过零: 否则噪声会让每帧都翻号, 测不出真振荡
    se_s = np.convolve(se, np.ones(5) / 5.0, mode="same") if len(se) >= 5 else se
    res["speed"] = {
        "target": float(np.mean(tgt)), "mean": float(np.mean(spd)), "std": float(np.std(spd)),
        "mean_err": float(np.mean(se)), "max": float(np.max(spd)), "min": float(np.min(spd)),
        "zero_cross": int(np.sum(np.diff(np.signbit(se_s).astype(np.int8)) != 0)),
    }

    # ---- 10. 可疑片段评分 -> Top5 ----
    score = np.zeros(N)
    score += 3.0 * (np.abs(err) >= ERR_SAT)
    score += 5.0 * dir_bad
    score += 2.0 * carry
    if len(dj_r) > 1:
        jr = np.concatenate([[0], np.abs(np.diff(dj_r))])
        jl = np.concatenate([[0], np.abs(np.diff(dj_l))])
        score += 2.0 * ((jr > JITTER_MM) | (jl > JITTER_MM))
    score += 1.0 * (np.abs(d_pwm) > 250)
    win = 25  # 25个收到的样本，不保证固定时长
    if N > win:
        ker = np.ones(win)
        rolling = np.convolve(score, ker, mode="valid")
        top = []
        used = np.zeros(N, dtype=bool)
        for start in np.argsort(rolling)[::-1]:
            if rolling[start] <= 0 or len(top) >= 5:
                break
            if used[start:start + win].any():
                continue
            used[start:start + win] = True
            sl_ = slice(start, start + win)
            seg_modes = mode[sl_]
            top.append({
                "start": int(start), "end": int(start + win),
                "t0": p.records[start]["t"], "t1": p.records[start + win - 1]["t"],
                "score": float(rolling[start]),
                "modes": {MODE_NAME.get(int(m), str(m)): int(np.sum(seg_modes == m))
                          for m in np.unique(seg_modes)},
                "max_abs_err": float(np.max(np.abs(err[sl_]))),
                "sat_frames": int(np.sum(np.abs(err[sl_]) >= ERR_SAT)),
                "dir_frames": int(np.sum(dir_bad[sl_])),
                "carry_frames": int(np.sum(carry[sl_])),
                "max_jitter": float(max(np.max(np.abs(np.diff(dj_r[sl_]))) if len(dj_r[sl_]) > 1 else 0,
                                        np.max(np.abs(np.diff(dj_l[sl_]))) if len(dj_l[sl_]) > 1 else 0)),
                "max_abs_k": float(np.max(np.abs(fk[sl_]))),
                "duandian": {"R": dj_r[sl_].tolist(), "L": dj_l[sl_].tolist()},
            })
        res["segments"] = top
    else:
        res["segments"] = []

    return res


# ============================ 输出: CSV / 报告 / 图 ============================

def write_csv(p: StreamParser, path: str):
    if p.v2.messages:
        v2.write_extensions(p,path)
        with open(path,"w",newline="",encoding="utf-8-sig") as f:
            writer=csv.DictWriter(f,fieldnames=["t"]+FIELD_NAMES+v2.EXTRA,extrasaction="ignore")
            writer.writeheader();writer.writerows(r for r in p.records if r.get("version")==2)
        with open(os.path.splitext(path)[0]+".events.jsonl","w",encoding="utf-8") as f:
            for m in p.v2.messages:
                if m["type"]!=1:f.write(json.dumps(m,ensure_ascii=False)+"\n")
        return
    prev_mode, prev_pwm, prev_err = None, None, None
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["t", "seq"] + FIELD_NAMES + ["mode_prev", "d_pwm", "d_err"])
        for r in p.records:
            m = int(r["pid_select"])
            w.writerow(
                [f"{r['t']:.3f}", r["seq"]]
                + [f"{r[n]:.6g}" for n in FIELD_NAMES]
                + ["" if prev_mode is None else prev_mode,
                   "" if prev_pwm is None else f"{r['servo_pwm'] - prev_pwm:.1f}",
                   "" if prev_err is None else f"{r['err'] - prev_err:.1f}"])
            prev_mode, prev_pwm, prev_err = m, r["servo_pwm"], r["err"]


def _fmt_modes(res):
    lines = ["| 模式 | 含义 | 帧数 | 占比 |", "|---|---|---|---|"]
    for m, d in sorted(res["modes"]["dist"].items()):
        lines.append(f"| {m} | {MODE_NAME.get(m, '?')} | {d['count']} | {d['pct']:.1f}% |")
    return "\n".join(lines)


def _fmt_sat(res):
    lines = ["| 模式 | 饱和帧数/总帧数 | 饱和占比 |", "|---|---|---|"]
    for m, d in sorted(res["err_sat"]["by_mode"].items()):
        lines.append(f"| {m} ({MODE_NAME.get(m, '?')}) | {d['count']}/{d['n']} | {d['pct']:.1f}% |")
    return "\n".join(lines)


def write_report(res: dict, path: str, bin_name: str = ""):
    if res.get("v2"):
        return v2.report(res,path,bin_name)
    L = []
    A = L.append
    A(f"# 遥测判读报告 — {bin_name or 'run'}")
    A("")
    A(f"生成时间:{time.strftime('%Y-%m-%d %H:%M:%S')}  ")
    A(f"数据来源:`{bin_name}`  ")
    A(f"帧数:**{res['n_frames']}**,时长约 **{res['duration_s']:.1f} s** "
      + ("(主机接收时间；不是设备控制周期)" if res["measured"] else f"(估计：按 {FRAME_DT*1000:.0f} ms/帧；缺测无法还原)"))
    A("")
    if res["n_frames"] == 0:
        A("## ⚠️ 没有解析到任何有效帧")
        A("")
        A("- 检查:协议/波特率是否匹配。现役固件为9600、36B JustFloat，不支持tele 2；请使用vofa子命令")
        A(f"- 本次共丢弃 {res['bad_frames']} 个疑似帧头、收集到 {len(res['ascii_events'])} 行 ASCII")
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(L))
        return

    L1 = res["link"]
    A("---")
    A("## 1. 链路质量")
    A("")
    A(f"- 主机接收帧率：{L1['fps']:.2f} 帧/秒" if res["measured"] else "- 无接收时间记录：不提供实测帧率或吞吐")
    A(f"- **丢帧率:{L1['drop_rate']:.2f}%**({res['dropped']} 帧,按 seq 跳变推算)")
    A(f"- 校验失败候选:{res['bad_frames']} 处({L1['bad_rate']:.2f}%)")
    if res["measured"]:
        A(f"- 主机接收吞吐：{L1['bytes_per_s']:.0f} B/s（含ASCII及全部收到的字节）")
    if res["ascii_events"]:
        A(f"- 期间收到 {len(res['ascii_events'])} 行蓝牙 ASCII 回复(参数变更/`get`),已剔除,不影响解析")
    A("")
    if L1["drop_rate"] > 5:
        A("> ⚠️ **丢帧率 >5%**:请核对协议和波特率、供电/干扰/距离；仅支持rate命令的固件才能用rate降载")
        A("")

    if res["dropped"] or res["bad_frames"]:
        A("> 存在缺测或损坏：相邻收到样本不一定是相邻控制帧；切换、过零、差分和片段评分仅作线索。")
    A("## 2. 模式分布")
    A("")
    A(_fmt_modes(res))
    A("")
    A(f"- 已收到样本中的模式切换 **{res['modes']['switches']} 次**")
    if res["measured"]:
        A(f"- 接收窗口内可观测切换率：{res['modes']['switch_rate']:.2f} 次/秒（缺测可能漏事件）")
    A("")

    A("## 3. err 饱和(限幅 ±500)")
    A("")
    A(f"- 全局饱和占比:**{res['err_sat']['overall_pct']:.1f}%**")
    A("")
    A(_fmt_sat(res))
    A("")

    A("## 4. 弯道 k 分布(模式 3/4/8/9)")
    A("")
    ck = res["corner_k"]
    if ck["n"]:
        A(f"- 样本 {ck['n']} 帧,|k| 中位数 {ck['median']:.3f},最大 {ck['max']:.3f}")
        A("")
        A("| |k| 区间 | 帧数 | 说明 |")
        A("|---|---|---|")
        for k, v in ck["bands"].items():
            note = ""
            if "0.35" in k and "<" in k:
                note = "模式 3/4 触发域(按新 err 构造会打满)"
            elif "0.35~0.7" in k:
                note = "模式 8/9 触发域(应有比例:err 250~500)"
            A(f"| {k} | {v} | {note} |")
    else:
        A("- 无弯道模式帧")
    A("")

    A("## 5. 方向异常(模式与 err 符号矛盾)")
    A("")
    da = res["dir_anomaly"]
    A(f"- {da['note']}")
    A(f"- 异常帧 **{da['frames']}**({da['pct']:.1f}%),其中连续 ≥3 帧的片段 **{da['runs_ge3']}** 处")
    if da["runs"]:
        A("")
        A("| 起始帧 | 结束帧 | 起始时刻(s) |")
        A("|---|---|---|")
        for s, e in da["runs"][:10]:
            A(f"| {s} | {e-1} | {res['times'][s]:.1f} |")
    A("")

    A("## 6. 出弯反踢(转弯→直道)")
    A("")
    ek = res["exit_kick"]
    A(f"- 切回直道 **{ek['exits']}** 次,其中 ΔPWM>100 的 **{ek['kicks']}** 次")
    A(f"- 最大 ΔPWM **{ek['worst_d_pwm']:.0f}**、最大 Δerr **{ek['worst_d_err']:.0f}**")
    if ek["kicks"]:
        A(f"- 涉及帧:{ek['frames']}")
    A("")
    pj = res["pwm_jumps"]
    A(f"**全场 PWM 跳变**:ΔPWM>100 共 {pj['gt100']} 帧,>250 共 {pj['gt250']} 帧")
    A("")
    if pj["top"]:
        A("| 帧 | 时刻(s) | ΔPWM | 模式变化 | err 变化 |")
        A("|---|---|---|---|---|")
        for j in pj["top"]:
            A(f"| {j['frame']} | {j['t']:.1f} | {j['d_pwm']:+.0f} | "
              f"{j['mode_prev']}({MODE_NAME.get(j['mode_prev'],'?')}) → {j['mode']}({MODE_NAME.get(j['mode'],'?')}) | "
              f"{j['err_prev']:+.0f} → {j['err']:+.0f} |")
        A("")
        A("> 单帧 ΔPWM>250 就是\"舵机猛地甩一下\",看这一行的模式/err 变化即可定位是哪条分支造成的。")
    A("")

    A("## 7. 强制打满残留(直道硬打方向的嫌疑源)")
    A("")
    ca = res["carry"]
    A(f"- 残留生效帧 **{ca['frames']}**({ca['pct']:.1f}%)")
    A(f"- 其中 pid_select=0 的帧 **{ca['zero_mode_in_carry']}**")
    A(f"- {ca['note']}")
    if ca["runs"]:
        A("")
        A("| 起始帧 | 结束帧 | 时长(s) |")
        A("|---|---|---|")
        for s, e in ca["runs"][:10]:
            A(f"| {s} | {e-1} | {res['times'][e-1]-res['times'][s]:.2f} |")
    A("")

    A("## 8. 断点抖动(相邻帧跳变 >150mm)")
    A("")
    for nm, d in res["jitter"].items():
        A(f"- {nm}:{d['count']} 次,最大跳变 {d['max']:.0f} mm")
    A("")

    A("## 9. 速度环")
    A("")
    sp = res["speed"]
    A(f"- 目标均值 {sp['target']:.1f}（误差按每帧目标计算）,实际均值 {sp['mean']:.2f}(std {sp['std']:.2f},"
      f"范围 {sp['min']:.1f}~{sp['max']:.1f})")
    A(f"- 平均误差 {sp['mean_err']:+.2f}")
    A(f"- 过零次数 **{sp['zero_cross']}**(对 5 帧滑动平均后的误差计数;数值大 = 在目标附近来回振荡)")
    if res["measured"] and not (res["dropped"] or res["bad_frames"]) and sp["zero_cross"] / max(res["duration_s"], 1e-6) > 1.5:
        A("- > ⚠️ 过零频繁,速度环有振荡:先降 `sp_kp` 或加 `sp_ki`,再看机械是否卡滞")
    A("")

    A("## 10. 最可疑的片段(优先看这些)")
    A("")
    if not res["segments"]:
        A("- 未发现明显异常片段")
    for i, s in enumerate(res["segments"], 1):
        A(f"### 片段 {i}:帧 {s['start']}~{s['end']-1}({s['t0']:.1f}~{s['t1']:.1f} s,评分 {s['score']:.0f})")
        A("")
        A(f"- 模式构成:{', '.join(f'{k}×{v}' for k, v in s['modes'].items())}")
        A(f"- |err| 峰值 {s['max_abs_err']:.0f},饱和 {s['sat_frames']} 帧,"
          f"方向异常 {s['dir_frames']} 帧,强制打满 {s['carry_frames']} 帧")
        A(f"- 断点最大跳变 {s['max_jitter']:.0f} mm,|Forward.k| 峰值 {s['max_abs_k']:.3f}")
        A("")
    A("---")
    A("")
    A("> 把本报告 + 同名 CSV 交给 Claude 分析。CSV 为逐帧原始值,可精确定位到帧号。")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))


def write_plot(p: StreamParser, res: dict, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    setup_cjk_font()

    if res["n_frames"] == 0:
        return
    if res.get("v2"):
        rows=res["rows"]
        if not rows:return
        fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
        for segment in sorted({r["segment"] for r in rows}):
            group=[r for r in rows if r["segment"]==segment]
            t=[r["host_t"] if r["host_t"] is not None else r["t"] for r in group]
            for ax,key in zip(axes,["err","servo_pwm","Speed_now"]):
                ax.plot(t,[r[key] for r in group],".",markersize=2);ax.set_ylabel(key)
        axes[-1].set_xlabel("主机接收时间(s)，仅画采样点，不连接缺失区间")
        fig.tight_layout();fig.savefig(path,dpi=110);plt.close(fig);return
    M = p.matrix()
    i = {n: k for k, n in enumerate(FIELD_NAMES)}
    t = p.cols("t")
    fig, ax = plt.subplots(5, 1, figsize=(13, 11), sharex=True)

    ax[0].plot(t, M[:, i["err"]], lw=0.9, color="tab:red")
    ax[0].axhline(ERR_SAT, ls="--", lw=0.7, color="gray")
    ax[0].axhline(-ERR_SAT, ls="--", lw=0.7, color="gray")
    ax[0].set_ylabel("err")

    ax[1].plot(t, M[:, i["servo_pwm"]], lw=0.9, color="tab:blue")
    ax[1].set_ylabel("servo_pwm")

    ax[2].step(t, M[:, i["pid_select"]], lw=1.0, color="tab:green", where="post")
    ax[2].set_ylabel("pid_select")
    ax[2].set_yticks(range(10))

    ax[3].plot(t, M[:, i["RIGHT_duandian"]], lw=0.9, label="RIGHT")
    ax[3].plot(t, M[:, i["LEFT_duandian"]], lw=0.9, label="LEFT")
    ax[3].legend(loc="upper right", fontsize=8)
    ax[3].set_ylabel("duandian(mm)")

    ax[4].plot(t, M[:, i["Speed_now"]], lw=0.9, label="now")
    ax[4].plot(t, M[:, i["Speed_mubiao"]], ls="--", lw=0.7, color="gray", label="target")
    ax[4].legend(loc="upper right", fontsize=8)
    ax[4].set_ylabel("speed")
    ax[4].set_xlabel("t (s, 主机接收时间)" if res["measured"] else "t (s, 标称周期估计)")
    ax[0].set_title(os.path.basename(path))
    plt.tight_layout()
    plt.savefig(path, dpi=110)
    plt.close(fig)


# ============================ 模拟数据(无车也能验证工具) ============================

def make_demo_bin(path: str, seconds: float = 60.0, inject_bugs: bool = True):
    """合成一段"像真实日志"的数据(含注入的典型病症), 用于:
       ①验证解析/判读链路  ②无车时预览报告长什么样
       每 20 秒一个循环: 直道 → 小右1 → 大右3 → 中右8 → 直道(残留) → 小左2 → 大左4 → 中左9 → 直道"""
    rng = np.random.default_rng(7)
    n = int(seconds / FRAME_DT)
    out = bytearray()
    seq = 0
    spd = 0.0                     # 速度环自回归状态(不是纯噪声)
    t_ascii = 5.0                 # 此处插入一行蓝牙 ASCII 回复

    for k in range(n):
        t = k * FRAME_DT
        ph = t % 20.0

        # ---- 缺省: 直道 ----
        mode = 0
        err = float(rng.normal(0, 12))
        fk = float(rng.normal(0, 0.05))
        sl = sr = 0
        danbian = 0.0
        dj_r = 850 + rng.normal(0, 25)     # 远端无断点
        dj_l = 850 + rng.normal(0, 25)

        if 6 <= ph < 8:                    # 小右(贴右墙): 模式 1
            mode = 1
            fk = -(0.05 + 0.06 * (ph - 6))
            err = -(70 + 45 * (ph - 6)) + rng.normal(0, 8)
        elif 8 <= ph < 11:                 # 大右: 模式 3, |k|<0.35
            mode = 3
            fk = -(0.15 + 0.05 * (ph - 8))
            dj_r = 300 + rng.normal(0, 15)
            err = -500.0 if inject_bugs else -min(175.0 / abs(fk), 500.0)   # 注入: 老代码钉死 ±500
        elif 11 <= ph < 12.5:              # 中右: 模式 8, |k|≥0.35(应有比例 250~500)
            mode = 8
            fk = -0.5
            dj_r = 270 + rng.normal(0, 12)
            err = -min(175.0 / abs(fk), 500.0)
            danbian = 1.0
        elif 12.5 <= ph < 14 and inject_bugs:
            mode = 0                       # 注入: 出弯后"强制打满残留"未清(直道硬打右)
            sl, sr = (2, 0) if ph < 13.2 else (1, 0)
            err = -420.0                   # err 陈旧, 与 mode=0 矛盾
        elif 14 <= ph < 15.5:              # 小左: 模式 2
            mode = 2
            fk = 0.05 + 0.06 * (ph - 14)
            err = (70 + 45 * (ph - 14)) + rng.normal(0, 8)
            if inject_bugs and 14.6 < ph < 15.0:   # 注入: 模式与 err 符号矛盾(方向错)
                mode, err = 3, +300.0
        elif 15.5 <= ph < 17.5:            # 大左: 模式 4
            mode = 4
            fk = 0.15 + 0.05 * (ph - 15.5)
            dj_l = 320 + rng.normal(0, 15)
            err = min(175.0 / abs(fk), 500.0)
        elif 17.5 <= ph < 19:              # 中左: 模式 9
            mode = 9
            fk = 0.5
            dj_l = 270 + rng.normal(0, 12)
            err = min(175.0 / abs(fk), 500.0)
            danbian = 1.0

        if mode == 0 and k % 97 == 3:      # 偶发瞬时误检断点(断点抖动来源)
            dj_r = 240

        spd = 0.85 * spd + rng.normal(0, 0.25)
        pwm = 1445.0 + 4.0 * err

        vals = {
            "err": err, "servo_pwm": max(1170.0, min(1720.0, pwm)), "pid_select": mode,
            "Midline.k": fk * 0.5, "Midline.b": 100.0, "Forward.k": fk, "Forward.b": 80.0,
            "Forward_2.k": fk * 0.9, "Forward_3.k": fk * 0.8,
            "RIGHT_duandian": dj_r, "LEFT_duandian": dj_l,
            "paodao_distance": 700.0, "valid_couter": float(int(rng.normal(450, 8))),
            "RIGHT_cnt": float(int(rng.normal(120, 10))), "LEFT_cnt": float(int(rng.normal(118, 10))),
            "Forward_cnt": 60.0 if mode in (3, 4, 8, 9) else 0.0,
            "Forward_cnt_2": 12.0 if mode in (3, 8) else 0.0,
            "Forward_cnt_3": 14.0 if mode in (4, 9) else 0.0,
            "Speed_now": 8.0 + spd, "Speed_mubiao": 8.0,
            "danbian_flag": danbian,
            "state_left_cnt": float(sl), "state_right_cnt": float(sr),
            "state_left_cnt_2": 0.0, "state_right_cnt_2": 0.0,
            "zhongxian_chuizhi": 50.0 if mode == 0 else 0.0,
        }
        payload = bytearray()
        for name, _ in FIELD_TABLE:
            iv, valid, clipped = encode_field(name, vals[name])
            if not valid:
                raise ValueError("V1没有有效位，不能表达无效模拟量：" + name)
            payload += struct.pack("<h", iv)
        frame = bytearray([0xAA, 0x55, VER, seq]) + payload
        frame.append(checksum(frame[PAYLOAD_OFF:]))
        out += frame
        seq = (seq + 1) % 256

        # 注入: 丢 2 帧(seq 跳变) + 一行 ASCII 参数变更
        if k == int(30 / FRAME_DT):
            seq = (seq + 2) % 256
        if abs(t - t_ascii) < FRAME_DT / 2:
            out += b"OK kp=450\r\n"
        # 注入: 一处坏帧(校验错)
        if k == int(45 / FRAME_DT):
            bad = bytearray(frame)
            bad[10] ^= 0xFF
            out += bad

    with open(path, "wb") as f:
        f.write(bytes(out))


# ============================ CLI ============================

def _auto_pick_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    ports = list(list_ports.comports())
    if not ports:
        return None
    for p in ports:                      # 优先选蓝牙口
        if "blue" in (p.description or "").lower() or "bt" in (p.description or "").lower():
            return p.device
    return ports[0].device


def cmd_list(_a):
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        print("未发现任何串口。请先配对 HC-05(Windows 设置 → 蓝牙 → 添加设备),配对后会多出一个传出 COM 口。")
        return
    print(f"{'端口':<10}{'描述'}")
    for p in ports:
        print(f"{p.device:<10}{p.description}")


def cmd_capture(a):
    import serial
    port = a.port or _auto_pick_port()
    if not port:
        print("未指定 --port 且未自动找到串口,先用 list 查看")
        return 1
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) if os.path.dirname(a.out) else ".", exist_ok=True)

    ser = serial.Serial(port, a.baud, timeout=0.05)
    print(f"已打开 {port} @ {a.baud}")
    parser = StreamParser()
    raw, events = bytearray(), []
    t0 = time.monotonic()
    last = t0
    try:
        if a.diag:
            ser.write(b"tele 2\n")
            print("已请求 tele 2；须收到有效诊断帧才确认，ERR回复保留在原始记录中")
        while time.monotonic() - t0 < a.sec:
            chunk = ser.read(4096)
            now = time.monotonic()
            if chunk:
                events.append({"offset": len(raw), "length": len(chunk), "t": now-t0})
                raw += chunk
                parser.feed(chunk, now-t0)
            if now - last > 2.0:
                last = now
                print(f"  {now-t0:5.1f}s  帧 {parser.total_frames}  坏 {parser.bad_frames}  丢 {parser.dropped}")
            if a.diag and now-t0 >= 3 and not parser.total_frames:
                print("诊断切换未确认：3秒内无有效V1帧；现役固件不支持tele 2。已保留原始数据。")
                break
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        duration = time.monotonic()-t0
        ser.close()
        save_capture(a.out, raw, events, duration)
    print(f"已保存 {a.out}({len(raw)} 字节, {parser.total_frames} 帧)及接收时间记录")
    return 1 if a.diag and not parser.total_frames else 0


def cmd_capture_v2(a):
    import serial
    if not math.isfinite(a.sec) or a.sec<=0:raise ValueError("采集时长必须大于0")
    import uuid
    from datetime import datetime, timezone
    port=a.port or _auto_pick_port()
    if not port:raise ValueError("请选择串口")
    path=os.path.abspath(a.out);os.makedirs(os.path.dirname(path),exist_ok=True)
    if os.path.exists(path):raise FileExistsError("日志已存在，请换一个--out文件名："+path)
    parser=StreamParser();hasher=hashlib.sha256();offset=0;events=[]
    start=time.monotonic();formal=None;end_reason="complete";error=None
    meta={"run_uuid":str(uuid.uuid4()),"started_utc":datetime.now(timezone.utc).isoformat(),
          "host_version":"v2.1","host_sha256":hashlib.sha256(Path(__file__).read_bytes()+Path(v2.__file__).read_bytes()).hexdigest(),"protocol":2,"port":port,"baud":a.baud,"requested_rate":a.rate}
    serial_port=serial.Serial(port,a.baud,timeout=.03)
    def send(cmd):
        events.append({"t":time.monotonic()-start,"command":cmd.strip()})
        serial_port.write(cmd.encode("ascii"));return True
    handshake=v2.Handshake(send,parser,a.rate)
    try:
        with open(path,"wb") as raw,open(path+".rx.jsonl","w",encoding="utf-8") as rx:
            handshake.start(time.monotonic())
            last_rx=time.monotonic()
            while True:
                if a.stop_file and os.path.exists(a.stop_file):
                    end_reason="user_stop";break
                chunk=serial_port.read(4096);now=time.monotonic()
                if chunk:
                    raw.write(chunk);hasher.update(chunk)
                    rx.write(json.dumps({"offset":offset,"length":len(chunk),"t":now-start,"host_monotonic_ns":time.monotonic_ns()})+"\n")
                    offset+=len(chunk);parser.feed(chunk,now-start);last_rx=now
                if formal is None:
                    handshake.poll(now)
                    if handshake.done:
                        formal=now;meta["pre_roll_s"]=now-start
                        meta["session"]=handshake.session;meta["info"]=handshake.info
                        print("V2协商及配置完整，开始正式采集",flush=True)
                else:
                    if any(m["session"]!=handshake.session for m in parser.v2.messages[-10:]):
                        raise ValueError("设备会话改变，停止当前run并重新协商")
                    # Unknown parameter revision requires a new exact snapshot.
                    unknown=any(r.get("version")==2 and (r["session"],r["param_revision"]) not in parser.v2.configs for r in parser.records[-1:])
                    if unknown and now-getattr(handshake,"last_config_request",0)>2:
                        send("getcfg\n");handshake.last_config_request=now
                    if now-last_rx>3:raise TimeoutError("链路或设备无响应")
                    if now-formal>=a.sec:break
    except KeyboardInterrupt:end_reason="user_interrupt"
    except Exception as exc:end_reason="error";error=str(exc)
    finally:
        serial_port.close();duration=time.monotonic()-start
        meta.update(version=1,clock="host_monotonic",duration_s=duration,bytes=offset,
                    sha256=hasher.hexdigest(),ended_utc=datetime.now(timezone.utc).isoformat(),
                    end_reason=end_reason,error=error,commands=events,
                    configs={f"{sid}:{rev}":cfg for (sid,rev),cfg in parser.v2.configs.items()})
        with open(path+".meta.json","w",encoding="utf-8") as f:json.dump(meta,f,ensure_ascii=False,indent=2)
    print("已保存 "+path+("；错误："+error if error else ""),flush=True)
    return 1 if error else 0


def cmd_report(a):
    for path in a.bin:
        base = os.path.splitext(path)[0]
        p = parse_file(path)
        res = analyze(p)
        write_csv(p, base + ".csv")
        write_report(res, base + "_report.md", os.path.basename(path))
        try:
            write_plot(p, res, base + ".png")
        except Exception as e:
            print(f"(绘图跳过: {e})")
        print(f"{path} → {base}.csv / {base}_report.md / {base}.png")


def cmd_demo(a):
    os.makedirs(a.outdir, exist_ok=True)
    binp = os.path.join(a.outdir, "demo.bin")
    make_demo_bin(binp, seconds=a.sec)
    p = parse_file(binp)
    res = analyze(p)
    write_csv(p, os.path.join(a.outdir, "demo.csv"))
    write_report(res, os.path.join(a.outdir, "demo_report.md"), "demo.bin")
    write_plot(p, res, os.path.join(a.outdir, "demo.png"))
    print(f"模拟数据 {binp} → demo.csv / demo_report.md / demo.png")
    print(f"解析结果: {res['n_frames']} 帧, 丢 {res['dropped']}, 坏 {res['bad_frames']}, "
          f"方向异常 {res['dir_anomaly']['runs_ge3']} 段, "
          f"残留 {res['carry']['frames']} 帧, 可疑片段 {len(res['segments'])} 个")


def main():
    ap = argparse.ArgumentParser(description="RoboCup 诊断遥测上位机(核心库/CLI)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="列出串口").set_defaults(func=cmd_list)

    c = sub.add_parser("capture", help="采集原始字节流")
    c.add_argument("--port", default=None)
    c.add_argument("--baud", type=int, default=9600)
    c.add_argument("--sec", type=float, default=30)
    c.add_argument("--out", default="log/run.bin")
    c.add_argument("--diag", action="store_true", help="先发 tele 2 切到诊断流")
    c.set_defaults(func=cmd_capture)

    cv = sub.add_parser("capture-v2",help="协商会话/配置后采集V2，流式保存")
    cv.add_argument("--port",default=None)
    cv.add_argument("--baud",type=int,default=115200)
    cv.add_argument("--sec",type=float,default=30)
    cv.add_argument("--rate",type=int,choices=range(1,11),default=1)
    cv.add_argument("--out",default="log/v2.bin")
    cv.add_argument("--stop-file",default=None,help=argparse.SUPPRESS)
    cv.set_defaults(func=cmd_capture_v2)

    r = sub.add_parser("report", help="解析 → CSV + 报告 + 图")
    r.add_argument("bin", nargs="+")
    r.set_defaults(func=cmd_report)

    v = sub.add_parser("vofa", help="解析老遥测(VOFA+ JustFloat 8通道流, 看雷达转速)")
    v.add_argument("bin")
    v.add_argument("--sec", type=float, default=0.0, help="采集时长(秒), 用于算帧周期")
    v.add_argument("--nch", type=int, default=8, help="通道数(默认8)")
    v.set_defaults(func=cmd_vofa)

    d = sub.add_parser("demo", help="生成模拟数据并出报告(无车验证工具)")
    d.add_argument("--sec", type=float, default=60)
    d.add_argument("--outdir", default="log")
    d.set_defaults(func=cmd_demo)

    a = ap.parse_args()
    return a.func(a) or 0


if __name__ == "__main__":
    sys.exit(main())
