# -*- coding: utf-8 -*-
"""定 P0-A 的阈值：看 PREFILTER(512) 发生时，近段/远段斜率各是多少。

目的：把"明显在弯"的触发条件从"远段 |a|>0.30"扩展到近段时，近段阈值该取多少。
判据来自数据，不猜。
"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = ['v2_1013_194302_3416906000000_195e6842', 'v2_1013_194450_3524718000000_b41f5f1e',
        'v2_1013_194631_3625625000000_8da28e07', 'v2_1013_194802_3717046000000_bd535409',
        'v2_1013_195030_3865031000000_676aafed', 'v2_1013_195147_3941359000000_fd8a625e']


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


tot = {'512': 0, 'other': 0, 'all': 0}
th = {0.15: 0, 0.20: 0, 0.25: 0, 0.30: 0}
near_vals, far_vals, fmaxy = [], [], []
near_valid_any = 0
print('%-34s %6s %6s %10s %10s %10s %10s %8s' %
      ('capture', '512', '其它', '|近a|中位', '|近a|max', '|远a|中位', '|远a|max', '近段有效'))
for cap in CAPS:
    ac = rows(os.path.join(LOG, cap + '.acute.csv'))
    pat = rows(os.path.join(LOG, cap + '.path.csv'))
    dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
    n512 = nother = 0
    na, fa, fy = [], [], []
    nv = 0
    for r in ac:
        tot['all'] += 1
        k = (r.get('control_seq'), r.get('param_revision'))
        p = dP.get(k)
        if not p:
            continue
        try:
            iv = int(float(r.get('acute_checks')))
        except Exception:
            continue
        # 近段/远段"有效且明显在弯"的判定：用 min_y/max_y 非退化近似 valid
        n0a, n1a = num(p.get('left_near_a')), num(p.get('right_near_a'))
        f0a, f1a = num(p.get('left_far_a')), num(p.get('right_far_a'))
        f0y, f1y = num(p.get('left_far_max_y')), num(p.get('right_far_max_y'))
        nmax = max(abs(n0a or 0), abs(n1a or 0))
        fmax = max(abs(f0a or 0), abs(f1a or 0))
        nvalid = 1 if ((num(p.get('left_near_min_y')) or 0) and (num(p.get('left_near_max_y')) or 0)
                       and (num(p.get('left_near_max_y')) - num(p.get('left_near_min_y')) >= 250)) or \
                      ((num(p.get('right_near_max_y')) or 0) - (num(p.get('right_near_min_y')) or 0) >= 250) else 0
        if iv == 512:
            n512 += 1
            na.append(nmax); fa.append(fmax)
            fy.append(max(f0y or 0, f1y or 0))
            near_valid_any += nvalid
            for t in th:
                if nmax > t:
                    th[t] += 1
        else:
            nother += 1
    tot['512'] += n512; tot['other'] += nother
    print('%-34s %6d %6d %10s %10s %10s %10s %8d' %
          (cap[-6:], n512, nother,
           ('%.3f' % sorted(na)[len(na) // 2]) if na else '-', ('%.3f' % max(na)) if na else '-',
           ('%.3f' % sorted(fa)[len(fa) // 2]) if fa else '-', ('%.3f' % max(fa)) if fa else '-',
           near_valid_any))
print()
print('全批: %d 个 acute 事件, 其中 512 有 %d, 其它 %d' % (tot['all'], tot['512'], tot['other']))
print('**如果预滤波改成"近段 |a| > T 也算证据"，能救回多少个 512：**')
for t in sorted(th):
    print('   T=%.2f -> %3d / %3d  (%.0f%%)' % (t, th[t], tot['512'], 100.0 * th[t] / max(1, tot['512'])))
