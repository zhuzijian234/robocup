# -*- coding: utf-8 -*-
"""用真实点云坐实机制。忠实复现 path_track.c 的 boundary() 与 fit()，
先用 fit() 的 count 与遥测对齐（证明复现忠实），再看 boundary() 的分区链断在哪。"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\tmp\cloud_probe'
STRIPS, STRIP_MM, MAX_GAP, MIN_SPAN, MAX_RMS = 12, 100.0, 310.0, 250.0, 60.0
CX = 10.0
RUNS = [('4_194802', 'v2_1013_194802_3717046000000_bd535409', 117, 156),
        ('6_195147', 'v2_1013_195147_3941359000000_fd8a625e', 94, 129)]


def rows(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def one_pass(pts, low, high):
    sx = sy = yy = xy = 0.0
    n = 0; last = None; gap = 0.0; mn = high; mx = low
    for (x, y) in pts:
        if y < low or y > high:
            continue
        sx += x; sy += y; yy += y * y; xy += x * y
        if last is not None and y - last > gap:
            gap = y - last
        last = y; n += 1
        mn = min(mn, y); mx = max(mx, y)
    return sx, sy, yy, xy, n, gap, (mn if n else 0.0), (mx if n else 0.0)


def fit(pts, low, high):
    """返回 (count, span, rms, a, rejected)。count 取第2遍的 n，与遥测 out.count 同义。"""
    a = b = 0.0
    for p in (0, 1):
        sx, sy, yy, xy, n, gap, mn, mx = one_pass(pts, low, high) if p == 0 else None, None, None, None, None, None, None, None
        # 第2遍要按第1遍的线剔除
        if p == 1:
            f = [(x, y) for (x, y) in pts if low <= y <= high and abs(x - (a * y + b)) <= 100.0]
            sx, sy, yy, xy, n, gap, mn, mx = one_pass(f, low, high)
        rej = (1 if n < 4 else 0) | (2 if (mx - mn) < MIN_SPAN else 0) | (4 if gap > MAX_GAP else 0)
        if rej:
            return n, mx - mn, None, None, rej
        yy -= sy * sy / n; xy -= sx * sy / n
        if yy <= 1.0:
            return n, mx - mn, None, None, 8
        a = xy / yy; b = (sx - a * sy) / n
    f = [(x, y) for (x, y) in pts if low <= y <= high and abs(x - (a * y + b)) <= 100.0]
    res = sum((x - (a * y + b)) ** 2 for (x, y) in f)
    n2 = len(f)
    rms = (res / n2) ** .5 if n2 else 1e9
    rej = (1 if n2 < 4 else 0) | (16 if rms > MAX_RMS else 0) | (32 if abs(a) > 1.8 else 0)
    return n2, 0.0, rms, a, rej


def boundary(pts, side, width, trace=None):
    out, slope = [], 0.0
    for strip in range(1, STRIPS + 1):
        lo, hi = strip * STRIP_MM, strip * STRIP_MM + STRIP_MM
        expected = CX + side * width * 0.5
        best, cx_, cy_ = 1e9, 0.0, 0.0
        if out:
            expected = out[-1][0] + slope * (lo + 50.0 - out[-1][1])
        tol = 220.0 if out else 300.0
        for (x, y) in pts:
            if not (y >= lo and y < hi and abs(x) < 2200.0):
                continue
            if (not out) and (side * (x - CX) < 40.0 or lo > 400.0):
                continue
            d = abs(x - expected)
            if d < best:
                best, cx_, cy_ = d, x, y
        if best > tol:
            if trace is not None:
                trace.append((strip, lo, expected, best, tol, 'no-cand'))
            continue
        if out and cy_ - out[-1][1] > MAX_GAP:
            if trace is not None:
                trace.append((strip, lo, expected, best, tol, 'GAP-break'))
            break
        sub = [(x, y) for (x, y) in pts if lo <= y < hi and abs(x - cx_) < 65.0]
        if not sub:
            continue
        ax = sum(p[0] for p in sub) / len(sub); ay = sum(p[1] for p in sub) / len(sub)
        out.append((ax, ay))
        if len(out) > 1 and (out[-1][1] - out[-2][1]) != 0:
            slope = clamp((out[-1][0] - out[-2][0]) / (out[-1][1] - out[-2][1]), -1.5, 1.5)
        if trace is not None:
            trace.append((strip, lo, expected, best, tol, 'accept x=%.0f slope=%.2f' % (ax, slope)))
    return out


os.makedirs(OUT, exist_ok=True)
for tag, cap, b0, b1 in RUNS:
    cl = rows(os.path.join(LOG, cap + '.cloud.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
    snaps = {}
    for r in cl:
        try:
            snaps.setdefault(r['control_seq'], []).append((float(r['x_mm']), float(r['y_mm'])))
        except Exception:
            pass
    print('=' * 104)
    print('%s  盲段=第%d~%d帧' % (tag, b0, b1))
    print('  %-6s %5s %-26s %-26s %s' % ('seq', '点数', '遥测 fit点数 左/右(跨度)', '复现 fit点数 左/右', '校验'))
    for sq in sorted(snaps, key=lambda z: int(z)):
        pts = snaps[sq]
        d = dD.get((sq, '1')) or dD.get((sq, '0')) or {}
        if not d:
            for k, v in dD.items():
                if k[0] == sq:
                    d = v; break
        lc, ls, lr, la, lrej = fit(pts, 100, 750)
        rc, rs, rr, ra, rrej = fit(pts, 100, 750) if False else (None,) * 5
        # 右：需要按右墙拟合 —— fit 与 side 无关，只是输入点子集不同，这里两壁都用全部点不行。
        # 遥测的 near_fit[0/1] 是【同一侧分区链】的拟合，所以必须用 boundary 先分侧再 fit。
        # => 直接看 boundary 的分区数即可；fit点数只对“左”做近似校验。
        print('  %-6s %5d  遥测 L=%s/%s R=%s/%s' %
              (sq, len(pts), d.get('left_near_count'), d.get('left_span'),
               d.get('right_near_count'), d.get('right_span')))
        if len(pts) and (int(sq) in range(7596, 7700) or int(sq) in range(10290, 10400)):
            tl, tr = [], []
            L = boundary(pts, -1, 500.0, tl)
            R = boundary(pts, +1, 500.0, tr)
            print('         复现 boundary: 左 %d 区 %s | 右 %d 区' %
                  (len(L), [(round(p[0]), round(p[1])) for p in L], len(R)))
            print('         左前4区: %s' % ('; '.join('区%d y%.0f 期望x=%.0f 差=%.0f 容差=%.0f %s' % t
                                                     for t in tl[:4])))
            fig, ax = plt.subplots(figsize=(6.2, 7.0))
            A = np.array(pts)
            ax.scatter(A[:, 0], A[:, 1], s=5, c='#999')
            if L:
                ax.plot([p[0] for p in L], [p[1] for p in L], 'o-', c='b', lw=1.5, ms=5, label='left %d' % len(L))
            if R:
                ax.plot([p[0] for p in R], [p[1] for p in R], 's-', c='r', lw=1.5, ms=5, label='right %d' % len(R))
            for k in range(1, 13):
                ax.axhline(k * 100, color='#eee', lw=.5)
            ax.axhline(750, color='m', ls='--', lw=.9)
            ax.axhline(400, color='g', ls='--', lw=.9)
            ax.axvline(CX - 40, color='b', ls=':', lw=.8)
            ax.axvline(CX + 40, color='r', ls=':', lw=.8)
            ax.set_xlim(-750, 750); ax.set_ylim(0, 1350)
            ax.set_xlabel('x mm (right +)'); ax.set_ylabel('y mm (forward)')
            ax.set_title('%s seq=%s pts=%d L=%d R=%d teleL=%s teleR=%s' %
                         (tag, sq, len(pts), len(L), len(R),
                          d.get('left_near_count'), d.get('right_near_count')), fontsize=8)
            ax.legend(fontsize=7, loc='upper right'); ax.grid(alpha=.2)
            fig.tight_layout(); fig.savefig(os.path.join(OUT, '%s_%s.png' % (tag, sq)), dpi=95)
            plt.close(fig)
print('\n图 ->', OUT)
