# -*- coding: utf-8 -*-
"""用真实点云坐实"车身偏转 -> 分区关联断裂"。

1) 把 cloud.csv 的真实前向点（就是 LEIDA_front_points，与遥测同源）喂进 boundary() 的
   忠实复现，逐区打印：期望位置 / 最近点距离 / 容差 / 结果
2) 与该帧遥测记录的 left_near_count/right_near_count 对比 —— 一致才说明复现忠实
3) 画图：点在车体坐标里的分布 + 分区带 + 断链处
"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\tmp\cloud_probe'
STRIPS, STRIP_MM, MAX_GAP = 12, 100.0, 310.0
WIDTH, CX = 500.0, 10.0
RUNS = [('④194802', 'v2_1013_194802_3717046000000_bd535409', (117, 156)),
        ('⑥195157', 'v2_1013_195147_3941359000000_fd8a625e', (94, 129))]


def rows(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


def num(v):
    try:
        return float(v)
    except Exception:
        return None


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def boundary(points, side, trace=None):
    """path_track.c:56-109 的忠实复现。side=-1 左, +1 右。"""
    out, slope = [], 0.0
    for strip in range(1, STRIPS + 1):
        lo, hi = strip * STRIP_MM, strip * STRIP_MM + STRIP_MM
        expected = CX + side * WIDTH * 0.5
        best, cx_, cy_ = 1e9, 0.0, 0.0
        if out:
            expected = out[-1][0] + slope * (lo + 50.0 - out[-1][1])
        tol = 220.0 if out else 300.0
        for (x, y) in points:
            if not (y >= lo and y < hi and abs(x) < 2200.0):
                continue
            if not out and (side * (x - CX) < 40.0 or lo > 400.0):
                continue
            d = abs(x - expected)
            if d < best:
                best, cx_, cy_ = d, x, y
        if best > tol:
            if trace is not None:
                trace.append((strip, lo, hi, expected, best, tol, '无候选超容差', None))
            continue
        if out and cy_ - out[-1][1] > MAX_GAP:
            if trace is not None:
                trace.append((strip, lo, hi, expected, best, tol, 'y间隔>310 断链', None))
            break
        sx = sy = n = 0
        for (x, y) in points:
            if y >= lo and y < hi and abs(x - cx_) < 65.0:
                sx += x; sy += y; n += 1
        if not n:
            continue
        out.append((sx / n, sy / n))
        if len(out) > 1:
            dy = out[-1][1] - out[-2][1]
            slope = clamp((out[-1][0] - out[-2][0]) / dy, -1.5, 1.5) if dy else 0.0
        if trace is not None:
            trace.append((strip, lo, hi, expected, best, tol, '接受',
                          (round(out[-1][0], 1), round(out[-1][1], 1), round(slope, 3))))
    return out


os.makedirs(OUT, exist_ok=True)
for tag, cap, blind in RUNS:
    cf = rows(os.path.join(LOG, cap + '.cloud_frames.csv'))
    cl = rows(os.path.join(LOG, cap + '.cloud.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
    snaps = {}
    for r in cl:
        try:
            snaps.setdefault(r['control_seq'], []).append((float(r['x_mm']), float(r['y_mm'])))
        except Exception:
            pass
    print('=' * 100)
    print('%s  点云快照 control_seq = %s' % (tag, [r['control_seq'] for r in cf]))
    print('  盲段帧号 %d~%d' % blind)
    for r in cf:
        sq = r['control_seq']
        d = dD.get((sq, r.get('param_revision')))
        pts = snaps.get(sq, [])
        if not d or not pts:
            print('  seq=%-6s 无对应遥测/点云' % sq); continue
        tl, tr = [], []
        L = boundary(pts, -1, tl)
        R = boundary(pts, +1, tr)
        rl, rr = d.get('left_near_count'), d.get('right_near_count')
        print('  seq=%-6s 点数=%-4d  遥测 左近=%s 右近=%s | 复现 左近=%d 右近=%d  %s' %
              (sq, len(pts), rl, rr, len(L), len(R),
               '复现一致' if (str(len(L)) == rl and str(len(R)) == rr) else '★不一致'))
        if len(L) <= 2 or len(R) <= 2:
            ys = [p[1] for p in pts]
            print('        点云 y 范围 %.0f~%.0f，x 范围 %.0f~%.0f' %
                  (min(ys), max(ys), min(p[0] for p in pts), max(p[0] for p in pts)))
            for side, tr_, nm in ((-1, tl, '左'), (+1, tr, '右')):
                print('        %s: %s' % (nm, '; '.join(
                    '区%d(y%.0f-%.0f) 期望x=%.0f 最近差=%.0f 容差=%.0f %s' % t for t in tr_[:4])))
        # 画图
        fig, ax = plt.subplots(figsize=(6.4, 7.2))
        A = np.array(pts)
        ax.scatter(A[:, 0], A[:, 1], s=6, c='#888', label='raw front points')
        if L:
            ax.plot([p[0] for p in L], [p[1] for p in L], 'o-', c='b', lw=1.6, ms=5, label='left strips (%d)' % len(L))
        if R:
            ax.plot([p[0] for p in R], [p[1] for p in R], 's-', c='r', lw=1.6, ms=5, label='right strips (%d)' % len(R))
        for k in range(1, 13):
            ax.axhline(k * 100, color='#ddd', lw=.5)
        ax.axhline(400, color='g', ls='--', lw=1)
        ax.axvline(CX - 40, color='b', ls=':', lw=.8)
        ax.axvline(CX + 40, color='r', ls=':', lw=.8)
        ax.set_xlim(-700, 700); ax.set_ylim(0, 1350)
        ax.set_xlabel('x mm (right +)'); ax.set_ylabel('y mm (forward)')
        ax.set_title('%s  seq=%s  pts=%d\nL=%d R=%d  (tele L=%s R=%s)' %
                     (tag, sq, len(pts), len(L), len(R), rl, rr), fontsize=9)
        ax.legend(fontsize=7, loc='upper right'); ax.grid(alpha=.2)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, '%s_%s.png' % (tag[:5], sq)), dpi=95)
        plt.close(fig)
print('\n图输出到', OUT)
