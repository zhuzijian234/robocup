# -*- coding: utf-8 -*-
"""10/13 六组 + 10/12 六次手推成功：速度、舵量饱和、几何有效性的对照数字。

只用 (session, control_seq, param_revision) 精确关联；缺失留空不补 0。
接近舵量边界沿用评审口径：PWM<=1180 或 >=1710。
"""
import csv, os
from collections import Counter

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
NEW = [('194302 左大弯', 'v2_1013_194302_3416906000000_195e6842'),
       ('194450 左弯', 'v2_1013_194450_3524718000000_b41f5f1e'),
       ('194631 左弯', 'v2_1013_194631_3625625000000_8da28e07'),
       ('194802 右弯', 'v2_1013_194802_3717046000000_bd535409'),
       ('195030 右弯', 'v2_1013_195030_3865031000000_676aafed'),
       ('195147 s弯', 'v2_1013_195147_3941359000000_fd8a625e')]
OLD = [('164512 手推左', 'v2_1012_164512_3089484000000'),
       ('164616 手推左', 'v2_1012_164616_3154078000000'),
       ('164757 手推左', 'v2_1012_164757_3254406000000'),
       ('164942 手推右', 'v2_1012_164942_3359656000000'),
       ('165050 手推右', 'v2_1012_165050_3428000000000'),
       ('165134 手推右', 'v2_1012_165134_3471765000000')]


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


def med(a):
    a = sorted(a)
    return a[len(a) // 2] if a else None


def report(title, group):
    print('=' * 104)
    print(title)
    print('  %-14s %5s %7s %7s %9s %9s %7s %9s %s' %
          ('采集', 'CTRL', '速度中位', '速度max', '舵<=1180', '舵>=1710', '感知无效', '几何冲突', 'source 分布'))
    tot = dict(ctl=0, sat=0, inv=0)
    for tag, cap in group:
        ctl, det, pat = (rows(os.path.join(LOG, cap + f)) for f in ('.csv', '.detail.csv', '.path.csv'))
        sp = [num(r.get('Speed_now')) for r in ctl]
        sp = [v for v in sp if v is not None]
        pw = [num(r.get('servo_pwm')) for r in ctl]
        pw = [v for v in pw if v is not None]
        lo = sum(1 for v in pw if v <= 1180)
        hi = sum(1 for v in pw if v >= 1710)
        inv = sum(1 for r in ctl if num(r.get('action_reason')) == 5)
        dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
        geo = Counter()
        src = Counter()
        for r in ctl:
            p = dP.get((r.get('control_seq'), r.get('param_revision')))
            if p:
                geo[int(num(p.get('geometry_reason')) or 0)] += 1
        for r in det:
            src[r.get('source')] += 1
        print('  %-14s %5d %7s %7s %9d %9d %7d %9d %s' %
              (tag, len(ctl), ('%.2f' % med(sp)) if sp else '-', ('%.2f' % max(sp)) if sp else '-',
               lo, hi, inv, geo.get(3, 0),
               ' '.join('%s×%d' % (k, n) for k, n in sorted(src.items(), key=lambda z: -z[1]))))
        tot['ctl'] += len(ctl); tot['sat'] += lo + hi; tot['inv'] += inv
    print('  合计: CONTROL %d, 接近边界 %d (%.0f%%), 感知无效 %d' %
          (tot['ctl'], tot['sat'], 100.0 * tot['sat'] / max(1, tot['ctl']), tot['inv']))


report('10/13 六组（新固件，带 acute）', NEW)
report('10/12 六次人工推车成功（回退固件 fa544667）', OLD)

print('\n=== 10/13 六组的 fit_rejected 高频值（4×8 位：左近|右近<<8|左远<<16|右远<<24）===')
for tag, cap in NEW:
    pat = rows(os.path.join(LOG, cap + '.path.csv'))
    c = Counter()
    for r in pat:
        try:
            c[int(float(r.get('fit_rejected')))] += 1
        except Exception:
            pass
    print('  %-14s %s' % (tag, ' '.join('0x%08X×%d' % (k, n) for k, n in c.most_common(5))))
