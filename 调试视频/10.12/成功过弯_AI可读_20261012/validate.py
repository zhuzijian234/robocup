# -*- coding: utf-8 -*-
"""用原始点云独立验证 y_corner（排除窗口交界假象）。

独立方法（完全不用固件的拟合窗口）：
  1) 取该侧的点，用 y∈[100,600] 的点自算最小二乘直线 x = a*y + b
  2) 从 y=600 往上扫，找第一个使 |x_point - (a*y+b)| > DEV(150mm) 的 y -> y_depart
  3) 与 distance.py 的 y_corner 比较

判读：若 y_depart 与 y_corner 接近（|差|<=200mm），说明 y_corner 反映真实的壁面转折；
      若 y_corner 恒在 650 而 y_depart 随帧变化很大，说明是窗口假象。
"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\成功过弯_AI可读_20261012'
CAPS = ['v2_1012_164512_3089484000000', 'v2_1012_164616_3154078000000',
        'v2_1012_164757_3254406000000', 'v2_1012_164942_3359656000000',
        'v2_1012_165050_3428000000000', 'v2_1012_165134_3471765000000']
DEV, YLO, YHI = 150.0, 100.0, 600.0


def rd(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


def find_xy(cols):
    cx = next((c for c in cols if c.lower() in ('x', 'point_x', 'px', 'x_mm')), None)
    cy = next((c for c in cols if c.lower() in ('y', 'point_y', 'py', 'y_mm')), None)
    return cx, cy


def fitline(pts):
    n = len(pts)
    if n < 4:
        return None
    sy = sum(p[1] for p in pts); sx = sum(p[0] for p in pts)
    syy = sum(p[1] * p[1] for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    den = n * syy - sy * sy
    if abs(den) < 1e-6:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sx - a * sy) / n
    return a, b


def main():
    ds = json.load(open(os.path.join(OUT, 'distance_summary.json'), encoding='utf-8'))
    for cap in CAPS:
        cl = rd(os.path.join(LOG, cap + '.cloud.csv'))
        fr = rd(os.path.join(LOG, cap + '.cloud_frames.csv'))
        if not cl or not fr:
            print('%s  无点云' % cap); continue
        cx, cy = find_xy(list(cl[0].keys()))
        print('=' * 92)
        print('%s  点云 %d 行 / %d 个快照   列名 x=%s y=%s' % (cap, len(cl), len(fr), cx, cy))
        if not cx or not cy:
            print('  未识别到坐标列，全部列名：', list(cl[0].keys())); continue
        # 按 snapshot_id 分组
        snaps = {}
        for r in cl:
            k = r.get('control_seq')
            pts = snaps.setdefault(k, [])
            try:
                pts.append((float(r[cx]), float(r[cy])))
            except Exception:
                pass
        # distance.py 里同 control_seq 的 y_corner
        want = {}
        for tag, v in ds.items():
            if v.get('capture') == cap:
                for k in ('turnin', 'hug'):
                    if v.get(k):
                        want[tag.split('_')[1]] = want.get(tag.split('_')[1], [])
        # 直接打印每个快照的独立结果
        dcsv = os.path.join(OUT, 'distance_frames.csv')
        print('   %-10s %6s %6s %8s %8s %8s %8s' %
              ('control_seq', '点数', '左点数', '左:y_depart', '左a', '右点数', '右:y_depart'))
        for k in sorted(snaps, key=lambda s: int(s) if s and s.isdigit() else 0):
            pts = snaps[k]
            left = [p for p in pts if p[0] < 0]
            right = [p for p in pts if p[0] >= 0]
            out = []
            for side in (left, right):
                base = [p for p in side if YLO <= p[1] <= YHI]
                fb = fitline(base)
                if not fb:
                    out.append((len(side), None, None, 'n<4')); continue
                a, b = fb
                cand = sorted((p[1] for p in side if p[1] > YHI and abs(p[0] - (a * p[1] + b)) > DEV))
                out.append((len(side), (cand[0] if cand else None), a, 'ok'))
            print('   %-10s %6d %6d %8s %8s %8d %8s' %
                  (k, len(pts), out[0][0], ('%.0f' % out[0][1]) if out[0][1] else out[0][3],
                   ('%.3f' % out[0][2]) if out[0][2] is not None else '-',
                   out[1][0], ('%.0f' % out[1][1]) if out[1][1] else out[1][3]))


if __name__ == '__main__':
    main()
