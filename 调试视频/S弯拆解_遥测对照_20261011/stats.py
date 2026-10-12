# -*- coding: utf-8 -*-
"""从 aligned.csv 汇总每段的遥测分布（只统计，不解释）。"""
import csv, os
from collections import Counter

OUT = r'D:\OneDrive\Desktop\robocup\调试视频\S弯拆解_遥测对照_20261011'
TAGS = ['01_215921', '02_220025', '03_220116_表现良好', '04_220417_表现良好']


def cnt(vals):
    c = Counter(v for v in vals if v not in ('', None))
    return ' '.join('%s×%d' % (k, n) for k, n in sorted(c.items(), key=lambda kv: -kv[1]))


for tag in TAGS:
    p = os.path.join(OUT, tag, 'aligned.csv')
    if not os.path.exists(p):
        print(tag, '缺 aligned.csv'); continue
    with open(p, encoding='utf-8-sig', newline='') as fh:
        rows = list(csv.DictReader(fh))
    ok = [r for r in rows if r['status'] == 'ok']
    def f(r, k):
        try:
            return float(r[k])
        except Exception:
            return None
    pw = [f(r, 'servo_pwm') for r in ok]
    pw = [x for x in pw if x is not None]
    sp = [x for x in (f(r, 'speed_now') for r in ok) if x is not None]
    tg = sorted(set(r['speed_target'] for r in ok if r['speed_target']))
    hot = sum(1 for x in pw if abs(x - 1445) > 150)
    print('=' * 76)
    print('%s   对齐帧 %d/%d  ok=%d' % (tag, len(ok), len(rows), len(ok)))
    print('  servo_pwm : %.0f ~ %.0f   中位附近(%d)  |偏差|>150 的帧 %d (%.0f%%)  打到限位(<=1170 或 >=1720): %d' %
          (min(pw), max(pw), sum(1 for x in pw if abs(x - 1445) <= 150), hot, 100.0 * hot / max(1, len(pw)),
           sum(1 for x in pw if x <= 1170 or x >= 1720)))
    print('  Speed_now : %.2f ~ %.2f   Speed_mubiao 取值 %s' % (min(sp), max(sp), tg))
    for col in ['action_reason', 'path_reason', 'command_applied', 'far_valid', 'source',
                'bend_code', 'geometry_reason', 'fit_rejected', 'avoid_state', 'cone_candidates',
                'speed_limit', 'effective_target', 'speed_reason']:
        print('  %-18s %s' % (col, cnt([r[col] for r in ok])))
