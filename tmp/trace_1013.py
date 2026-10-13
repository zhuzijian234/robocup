# -*- coding: utf-8 -*-
"""打印 boundary() 的逐区判定轨迹（含被选点的坐标），定位断链的那一格。"""
import csv, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
STRIPS, STRIP_MM, MAX_GAP, CX = 12, 100.0, 310.0, 10.0
CASES = [('④健康 seq7608', 'v2_1013_194802_3717046000000_bd535409', '7608'),
         ('④盲  seq7677', 'v2_1013_194802_3717046000000_bd535409', '7677'),
         ('⑥盲  seq10346', 'v2_1013_195147_3941359000000_fd8a625e', '10346')]


def rows(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def trace(pts, side, width=500.0):
    out, slope = [], 0.0
    print('   区  y窗口        期望x   最近点(x,y)      差    容差  判定           接受后 slope')
    for strip in range(1, STRIPS + 1):
        lo, hi = strip * STRIP_MM, strip * STRIP_MM + STRIP_MM
        expected = CX + side * width * 0.5
        best, cx_, cy_ = 1e9, None, None
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
        nsub = len([p for p in pts if lo <= p[1] < hi])
        if best > tol:
            if cx_ is None:
                print('   %-3d %4.0f-%-4.0f %8.0f  (该区无满足身份的点, 区内共%d点)  %s' %
                      (strip, lo, hi, expected, nsub, '跳过'))
            else:
                print('   %-3d %4.0f-%-4.0f %8.0f  (%7.0f,%6.0f) %7.0f %5.0f  ***超容差跳过*** (区内%d点)' %
                      (strip, lo, hi, expected, cx_, cy_, best, tol, nsub))
            continue
        if out and cy_ - out[-1][1] > MAX_GAP:
            print('   %-3d %4.0f-%-4.0f %8.0f  (%7.0f,%6.0f) %7.0f %5.0f  ***y间隔>310 断链*** 区内%d点' %
                  (strip, lo, hi, expected, cx_, cy_, best, tol, nsub))
            break
        sub = [p for p in pts if lo <= p[1] < hi and abs(p[0] - cx_) < 65.0]
        if not sub:
            continue
        ax = sum(p[0] for p in sub) / len(sub); ay = sum(p[1] for p in sub) / len(sub)
        out.append((ax, ay))
        if len(out) > 1 and out[-1][1] != out[-2][1]:
            slope = clamp((out[-1][0] - out[-2][0]) / (out[-1][1] - out[-2][1]), -1.5, 1.5)
        print('   %-3d %4.0f-%-4.0f %8.0f  (%7.0f,%6.0f) %7.0f %5.0f  接受(聚合%d点) -> x=%.0f  slope=%.2f' %
              (strip, lo, hi, expected, cx_, cy_, best, tol, len(sub), ax, slope))
    return out


for tag, cap, sq in CASES:
    cl = rows(os.path.join(LOG, cap + '.cloud.csv'))
    pts = []
    for r in cl:
        if r.get('control_seq') == sq:
            try:
                pts.append((float(r['x_mm']), float(r['y_mm'])))
            except Exception:
                pass
    print('=' * 120)
    print('%s   点数=%d   x范围 %.0f~%.0f  y范围 %.0f~%.0f' %
          (tag, len(pts), min(p[0] for p in pts), max(p[0] for p in pts),
           min(p[1] for p in pts), max(p[1] for p in pts)))
    for side, nm in ((-1, '左墙'), (+1, '右墙')):
        print('  --- %s ---' % nm)
        out = trace(pts, side)
        print('  => %s 得到 %d 个分区' % (nm, len(out)))
