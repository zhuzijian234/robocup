# -*- coding: utf-8 -*-
"""逐层深入：10/13 六组（fw=73299d3）为什么感知会断、弧模型为什么不接管。

层1  fit_rejected 逐字节逐位：哪一侧、哪一条门限
层2  detail 的分区数/跨度/间隙/RMS：门限背后的真实量（按 source 分组）
层3  race 的 curve_checks：弧模型被拒在哪一条
层4  交叉表：geometry_reason × 分区数/跨度
"""
import csv, os
from collections import Counter, defaultdict

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('①194302 左大弯', 'v2_1013_194302_3416906000000_195e6842', '左大弯'),
        ('②194450 左弯', 'v2_1013_194450_3524718000000_b41f5f1e', '左弯'),
        ('③194631 左弯', 'v2_1013_194631_3625625000000_8da28e07', '左弯'),
        ('④194802 右弯差劲', 'v2_1013_194802_3717046000000_bd535409', '右弯_s弯后大急弯表现差劲'),
        ('⑤195030 右弯差劲', 'v2_1013_195030_3865031000000_676aafed', '右弯,表现差劲'),
        ('⑥195157 s弯失败', 'v2_1013_195147_3941359000000_fd8a625e', 's弯没成功通过')]
FITB = [(1, 'PTS'), (2, 'SPAN'), (4, 'GAP'), (8, 'DEGEN'), (16, 'RMS'), (32, 'SLOPE')]
SIDES = [(0, '左近'), (8, '右近'), (16, '左远'), (24, '右远')]
CVB = [(1, 'SHORT'), (2, 'SEED'), (4, 'TANGENT'), (8, 'RESIDUAL'), (16, 'JUMP'),
       (32, 'BEND'), (64, 'ENTRY'), (128, 'BLOCKED'), (256, 'SUPPORT'), (512, 'AMBIG')]


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
    a = [x for x in a if x is not None]
    return sorted(a)[len(a) // 2] if a else None


D = {}
for tag, cap, note in CAPS:
    D[tag] = dict(cap=cap, note=note,
                  ctl=rows(os.path.join(LOG, cap + '.csv')),
                  det=rows(os.path.join(LOG, cap + '.detail.csv')),
                  pat=rows(os.path.join(LOG, cap + '.path.csv')),
                  rac=rows(os.path.join(LOG, cap + '.race.csv')))

print('=' * 110)
print('层1  fit_rejected 逐层拆：哪一侧的哪一条门限')
print('%-16s %-34s %s' % ('采集', '按侧统计(该侧有拒绝的帧数)', '按位统计'))
for tag, d in D.items():
    side_c, bit_c = Counter(), Counter()
    for r in d['pat']:
        try:
            v = int(float(r.get('fit_rejected')))
        except Exception:
            continue
        for sh, nm in SIDES:
            if (v >> sh) & 0xFF:
                side_c[nm] += 1
                for b, bn in FITB:
                    if ((v >> sh) & 0xFF) & b:
                        bit_c['%s:%s' % (nm, bn)] += 1
    print('%-16s %-34s %s' % (tag,
          ' '.join('%s×%d' % (k, v) for k, v in side_c.most_common()),
          ' '.join('%s×%d' % (k, v) for k, v in bit_c.most_common(6))))

print()
print('=' * 110)
print('层2  门限背后的真实量（detail 的每帧统计，中位数 / 最大）')
print('%-16s %-12s %-24s %-24s' % ('采集', 'source', '近段 分区数 / 跨度 / RMS', '远段 分区数 / 跨度 / RMS'))
for tag, d in D.items():
    g = defaultdict(list)
    for r in d['det']:
        for k, f in (('L', 'left'), ('R', 'right')):
            g[(r.get('source'), k)].append(
                (num(r.get(f + '_near_count')), num(r.get(f + '_span')), num(r.get(f + '_rms')),
                 num(r.get(f + '_far_count')), num(r.get(f + '_far_span')), num(r.get(f + '_far_rms'))))
    for src in ('0', '1', '2', '3'):
        for k in ('L', 'R'):
            rowsx = g.get((src, k))
            if not rowsx or len(rowsx) < 3:
                continue
            print('%-16s src=%-8s 近 %s/%s/%s (n=%d)   远 %s/%s/%s' %
                  (tag, src,
                   med([x[0] for x in rowsx]), med([x[1] for x in rowsx]), med([x[2] for x in rowsx]),
                   len(rowsx),
                   med([x[3] for x in rowsx]), med([x[4] for x in rowsx]), med([x[5] for x in rowsx])))
    print()

print('=' * 110)
print('层3  弧模型被拒在哪一条（race 的 curve_checks）')
print('%-16s %-28s %-20s %s' % ('采集', 'model / curve_rejected', 'curve_checks 位', '宽候选/支持'))
for tag, d in D.items():
    md = Counter(int(num(r.get('model')) or 0) for r in d['rac'])
    cr = Counter(int(num(r.get('curve_rejected')) or 0) for r in d['rac'])
    cb = Counter()
    for r in d['rac']:
        for b, nm in CVB:
            if int(num(r.get('curve_checks')) or 0) & b:
                cb[nm] += 1
    print('%-16s model %-20s %-28s sup=%s' %
          (tag, ' '.join('%d×%d' % (k, v) for k, v in sorted(md.items())),
           ' '.join('%s×%d' % (k, v) for k, v in cb.most_common(5)),
           med([num(r.get('support_mm')) for r in d['rac']])))
    print('%-16s curve_rejected %s' % ('', ' '.join('%d×%d' % (k, v) for k, v in sorted(cr.items()))))

print()
print('=' * 110)
print('层4  交叉表：geometry_reason × 近段分区数（看 NEAR_MISSING/CONFLICT 时到底几个分区）')
for tag, d in D.items():
    dD = {(r.get('control_seq'), r.get('param_revision')): r for r in d['det']}
    x = defaultdict(Counter)
    for r in d['pat']:
        g = int(num(r.get('geometry_reason')) or 0)
        p = dD.get((r.get('control_seq'), r.get('param_revision')))
        if not p:
            continue
        nl = num(p.get('left_near_count')) or 0
        nr = num(p.get('right_near_count')) or 0
        x[g]['左%d右%d' % (nl, nr)] += 1
        x[g]['__pk'] += 1
    print('%-16s %s' % (tag, ' | '.join(
        'geo=%d(n=%d): %s' % (g, c['__pk'], ' '.join('%s×%d' % (k, v) for k, v in c.most_common(3) if k != '__pk'))
        for g, c in sorted(x.items()))))
