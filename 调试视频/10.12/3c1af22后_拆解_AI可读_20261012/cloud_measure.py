# -*- coding: utf-8 -*-
"""独立点云测量：可见弯墙偏离近切线的位置（我自算，供与 acute_feature_y_mm 对照）。

⚠️ 口径声明（照抄作者文档 M10P_急弯提前布位修改与验证_20261012.md §2）：
   "这里的距离是可见弯墙偏离近切线的位置，不是真实拐点、前轴到弯口距离或预测碰撞距离。"
   本脚本量的正是同一类量（墙偏离近切线处），**不是**真实弯口距离。
   我的偏离阈值 150mm 与固件 acute_wall() 的 "连续3个分区偏离近切线至少40mm" 不同，
   所以数值不会与 acute_feature_y_mm 逐位一致。
"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\3c1af22后_拆解_AI可读_20261012'
DEV, YLO, YHI = 150.0, 100.0, 600.0


def rd(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as fh:
        return list(csv.DictReader(fh))


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
    return a, (sx - a * sy) / n


def main():
    S = json.load(open(os.path.join(OUT, 'summary.json'), encoding='utf-8'))
    out = {}
    for s in S:
        cap = s['capture']
        cl = rd(os.path.join(LOG, cap + '.cloud.csv'))
        if not cl:
            print('%s 无点云' % cap); continue
        snaps = {}
        for r in cl:
            try:
                snaps.setdefault(r['control_seq'], []).append((float(r['x_mm']), float(r['y_mm'])))
            except Exception:
                pass
        print('=' * 92)
        print('%s  (%s)  快照 %d 个' % (s['tag'], cap, len(snaps)))
        print('   %-8s %5s %10s %10s %8s' % ('ctrl_seq', '点数', '左:偏离处y', '右:偏离处y', '左a/右a'))
        rows = []
        for k in sorted(snaps, key=lambda x: int(x) if x.isdigit() else 0):
            pts = snaps[k]
            res = []
            for sel in (lambda p: p[0] < 0, lambda p: p[0] >= 0):
                side = [p for p in pts if sel(p)]
                base = [p for p in side if YLO <= p[1] <= YHI]
                fb = fitline(base)
                if not fb:
                    res.append((None, None)); continue
                a, b = fb
                cand = sorted(p[1] for p in side if p[1] > YHI and abs(p[0] - (a * p[1] + b)) > DEV)
                res.append((cand[0] if cand else None, a))
            print('   %-8s %5d %10s %10s %8s' %
                  (k, len(pts),
                   ('%.0f' % res[0][0]) if res[0][0] else '-',
                   ('%.0f' % res[1][0]) if res[1][0] else '-',
                   '%s/%s' % (('%.3f' % res[0][1]) if res[0][1] is not None else '-',
                              ('%.3f' % res[1][1]) if res[1][1] is not None else '-')))
            rows.append(dict(control_seq=k, n=len(pts),
                             left_y=res[0][0], left_a=res[0][1],
                             right_y=res[1][0], right_a=res[1][1]))
        out[s['tag']] = rows
    json.dump(out, open(os.path.join(OUT, 'cloud_departure.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n写出 cloud_departure.json')


if __name__ == '__main__':
    main()
