# -*- coding: utf-8 -*-
"""在 3c1af22 自己的数据（10/12 22:29 T01~T07, fw=9b8c0a52）上验证：
   急弯分支不启动，是不是同一个"预滤波要求远段 |a|>0.30"造成的。

只用 race.csv 的 acute 列 + path.csv 的拟合列，按 (control_seq,param_revision) 精确关联。
"""
import csv, os
from collections import Counter

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('T01_222930 左弯', 'v2_1012_222930_23745609000000'),
        ('T02_223105 左弯', 'v2_1012_223105_23840687000000'),
        ('T03_223130 右弯', 'v2_1012_223130_23865359000000'),
        ('T04_223154 右弯', 'v2_1012_223154_23889031000000'),
        ('T05_223226 右弯', 'v2_1012_223226_23921656000000'),
        ('T06_223303 右弯', 'v2_1012_223303_23958609000000'),
        ('T07_224402 左弯', 'v2_1012_224402_24616921000000')]
PH = {0: '普通', 1: '确认', 2: '准备', 3: '主弯', 4: '出口'}
RSN = {0: '无入口证据', 1: '确认中', 2: '准备中', 3: '主弯接管', 4: '余量不足',
       5: 'S弯/锥桶优先', 6: '证据丢失', 7: '出口证据'}


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


tot = Counter()
th = {0.15: 0, 0.20: 0, 0.25: 0, 0.30: 0}
print('%-16s %5s %5s %6s %6s %6s %7s %7s %8s %8s' %
      ('采集', 'race', 'ph>0', '|近a|中', '|近a|max', '|远a|中', 'far有效', 'model', 'goal中', 'pos中'))
for tag, cap in CAPS:
    rac, pat = rows(os.path.join(LOG, cap + '.race.csv')), rows(os.path.join(LOG, cap + '.path.csv'))
    if not rac:
        print(tag, '无 race'); continue
    dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
    ph = Counter(); rs = Counter()
    npos = 0
    na, fa, go, po = [], [], [], []
    farok = 0
    for r in rac:
        p = int(num(r.get('acute_phase')) or 0)
        ph[p] += 1
        if p > 0:
            npos += 1
        rs[int(num(r.get('acute_reason')) or 0)] += 1
        pp = dP.get((r.get('control_seq'), r.get('param_revision')))
        if pp:
            nmax = max(abs(num(pp.get('left_near_a')) or 0), abs(num(pp.get('right_near_a')) or 0))
            fmax = max(abs(num(pp.get('left_far_a')) or 0), abs(num(pp.get('right_far_a')) or 0))
            f0v = (num(pp.get('left_far_max_y')) or 0) > 0
            f1v = (num(pp.get('right_far_max_y')) or 0) > 0
            if f0v or f1v:
                farok += 1
            na.append(nmax); fa.append(fmax)
            if p == 0:      # 只统计"没进阶段"的帧：这批才是被预滤波挡掉的候选
                for t in th:
                    if nmax > t:
                        th[t] += 1
        for k, lst in (('acute_goal_mm', go), ('acute_position_mm', po)):
            v = num(r.get(k))
            if v is not None:
                lst.append(v)
    tot['race'] += len(rac); tot['pos'] += npos
    def md(a):
        return sorted(a)[len(a) // 2] if a else 0
    print('%-16s %5d %5d %6.3f %6.3f %6.3f %7d %7d %8.2f %8.2f' %
          (tag, len(rac), npos, md(na), max(na or [0]), md(fa), farok,
           sum(1 for r in rac if num(r.get('model'))), md(go), md(po)))
    print('        phase: %s | reason: %s' %
          (' '.join('%s×%d' % (PH[k], v) for k, v in sorted(ph.items())),
           ' '.join('%s×%d' % (RSN[k], v) for k, v in rs.most_common(4))))
print()
print('七组合计: race 行 %d，其中 acute_phase>0 的 %d 行 (%.1f%%)' %
      (tot['race'], tot['pos'], 100.0 * tot['pos'] / max(1, tot['race'])))
print('**如果预滤波额外接受"近段 |a| > T"，有多少"没进阶段"的帧会变成可扫描：**')
for t in sorted(th):
    print('   T=%.2f -> %3d 帧' % (t, th[t]))
