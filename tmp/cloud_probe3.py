# -*- coding: utf-8 -*-
"""忠实复现 boundary() -> separate_boundaries() -> fit()，用遥测的 near_count/span 验证。"""
import csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\tmp\cloud_probe'
STRIPS, STRIP_MM, MAX_GAP, MIN_SPAN, MAX_RMS, CX = 12, 100.0, 310.0, 250.0, 60.0, 10.0
RUNS = [('4_194802', 'v2_1013_194802_3717046000000_bd535409'),
        ('6_195147', 'v2_1013_195147_3941359000000_fd8a625e'),
        ('3_194631', 'v2_1013_194631_3625625000000_8da28e07')]


def rows(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


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
                trace.append((strip, lo, round(expected), round(best), tol, 'no-cand'))
            continue
        if out and cy_ - out[-1][1] > MAX_GAP:
            if trace is not None:
                trace.append((strip, lo, round(expected), round(best), tol, 'GAP-break'))
            break
        sub = [p for p in pts if lo <= p[1] < hi and abs(p[0] - cx_) < 65.0]
        if not sub:
            continue
        out.append((sum(p[0] for p in sub) / len(sub), sum(p[1] for p in sub) / len(sub)))
        if len(out) > 1 and out[-1][1] != out[-2][1]:
            slope = clamp((out[-1][0] - out[-2][0]) / (out[-1][1] - out[-2][1]), -1.5, 1.5)
        if trace is not None:
            trace.append((strip, lo, round(expected), round(best), tol,
                          'accept x=%.0f slope=%.2f' % (out[-1][0], slope)))
    return out


def separate(s0, s1):
    if not s0 or not s1:
        return s0, s1
    first = [len(s0), len(s1)]; shared = 0
    for i, a in enumerate(s0):
        for j, b in enumerate(s1):
            if abs(a[0] - b[0]) < 12 and abs(a[1] - b[1]) < 12:
                first[0] = min(first[0], i); first[1] = min(first[1], j); shared += 1
    if not shared:
        return s0, s1
    if abs(s0[0][1] - s1[0][1]) > 20:
        keep = 0 if s0[0][1] < s1[0][1] else 1
    elif first[0] != first[1]:
        keep = 0 if first[0] > first[1] else 1
    elif len(s0) != len(s1):
        keep = 0 if len(s0) > len(s1) else 1
    else:
        return s0[:first[0]], s1[:first[1]]
    if keep == 0:
        s1 = s1[:first[1]]
    else:
        s0 = s0[:first[0]]
    return s0, s1


def fit(samples, low, high):
    a = b = 0.0; cnt = 0
    for p in (0, 1):
        pts = samples if p == 0 else [s for s in samples
                                      if low <= s[1] <= high and abs(s[0] - (a * s[1] + b)) <= 100.0]
        sx = sy = yy = xy = 0.0; n = 0; last = None; gap = 0.0; mn = high; mx = low
        for (x, y) in pts:
            if y < low or y > high:
                continue
            sx += x; sy += y; yy += y * y; xy += x * y
            if last is not None and y - last > gap:
                gap = y - last
            last = y; n += 1; mn = min(mn, y); mx = max(mx, y)
        span = (mx - mn) if n else 0.0
        rej = (1 if n < 4 else 0) | (2 if span < MIN_SPAN else 0) | (4 if gap > MAX_GAP else 0)
        cnt = n
        if rej:
            return dict(n=n, span=span, rms=None, a=None, rej=rej)
        yy -= sy * sy / n; xy -= sx * sy / n
        if yy <= 1.0:
            return dict(n=n, span=span, rms=None, a=None, rej=8)
        a = xy / yy; b = (sx - a * sy) / n
    f = [s for s in samples if low <= s[1] <= high and abs(s[0] - (a * s[1] + b)) <= 100.0]
    n2 = len(f)
    rms = (sum((x - (a * y + b)) ** 2 for (x, y) in f) / n2) ** .5 if n2 else 1e9
    rej = (1 if n2 < 4 else 0) | (16 if rms > MAX_RMS else 0) | (32 if abs(a) > 1.8 else 0)
    return dict(n=cnt, span=span, rms=rms, a=a, rej=rej)


os.makedirs(OUT, exist_ok=True)
for tag, cap in RUNS:
    cl = rows(os.path.join(LOG, cap + '.cloud.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    dD = {}
    for r in det:
        dD.setdefault(r.get('control_seq'), r)
    snaps = {}
    for r in cl:
        try:
            snaps.setdefault(r['control_seq'], []).append((float(r['x_mm']), float(r['y_mm'])))
        except Exception:
            pass
    print('=' * 108)
    print('%s   验证：复现的 near_count/span 是否等于遥测' % tag)
    print('  %-6s %5s %-24s %-24s %s' % ('seq', '点数', '遥测 左 n/span', '复现 左 n/span', '右 n/span 遥测->复现'))
    ok = bad = 0
    for sq in sorted(snaps, key=lambda z: int(z)):
        pts = snaps[sq]; d = dD.get(sq) or {}
        L = boundary(pts, -1, 500.0)
        R = boundary(pts, +1, 500.0)
        L, R = separate(L, R)
        fl = fit(L, 100, 750); fr = fit(R, 100, 750)
        tel_l = '%s/%s' % (d.get('left_near_count'), d.get('left_span'))
        my_l = '%d/%.0f' % (fl['n'], fl['span'])
        tel_r = '%s' % d.get('right_near_count')
        match = (str(fl['n']) == str(d.get('left_near_count')))
        ok += 1 if match else 0; bad += 0 if match else 1
        print('  %-6s %5d %-24s %-24s %s->%s  %s' %
              (sq, len(pts), tel_l, my_l, tel_r, fr['n'], 'match' if match else 'DIFF'))
        fig, ax = plt.subplots(figsize=(6.0, 6.8))
        A = np.array(pts)
        ax.scatter(A[:, 0], A[:, 1], s=5, c='#999')
        if L:
            ax.plot([p[0] for p in L], [p[1] for p in L], 'o-', c='b', lw=1.5, ms=5, label='L strips %d' % len(L))
        if R:
            ax.plot([p[0] for p in R], [p[1] for p in R], 's-', c='r', lw=1.5, ms=5, label='R strips %d' % len(R))
        for k in range(1, 13):
            ax.axhline(k * 100, color='#eee', lw=.5)
        ax.axhline(750, color='m', ls='--', lw=.9); ax.axhline(400, color='g', ls='--', lw=.9)
        ax.axvline(CX - 40, color='b', ls=':', lw=.8); ax.axvline(CX + 40, color='r', ls=':', lw=.8)
        ax.set_xlim(-750, 750); ax.set_ylim(0, 1350)
        ax.set_title('%s seq=%s pts=%d  teleL=%s myL=%s' % (tag, sq, len(pts), tel_l, my_l), fontsize=8)
        ax.legend(fontsize=7); ax.grid(alpha=.2)
        fig.tight_layout(); fig.savefig(os.path.join(OUT, '%s_%s.png' % (tag, sq)), dpi=92); plt.close(fig)
    print('  -> 左 near_count 匹配 %d / 不匹配 %d' % (ok, bad))
print('\n图 ->', OUT)
