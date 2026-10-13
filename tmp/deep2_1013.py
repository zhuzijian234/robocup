# -*- coding: utf-8 -*-
"""分辨"点没了"还是"点还在但分区关联断了"；并补出层3/层4。"""
import csv, os
from collections import Counter

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('①194302', 'v2_1013_194302_3416906000000_195e6842'),
        ('②194450', 'v2_1013_194450_3524718000000_b41f5f1e'),
        ('③194631', 'v2_1013_194631_3625625000000_8da28e07'),
        ('④194802', 'v2_1013_194802_3717046000000_bd535409'),
        ('⑤195030', 'v2_1013_195030_3865031000000_676aafed'),
        ('⑥195157', 'v2_1013_195147_3941359000000_fd8a625e')]
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
    a = sorted(x for x in a if x is not None)
    return a[len(a) // 2] if a else None


print('=' * 104)
print('层5  前向点数 vs 近段分区数：点还在吗？')
print('%-10s %-26s %-26s %s' % ('采集', '有来源帧: 前向点数/左近/右近', '无来源帧: 前向点数/左近/右近', '结论'))
for tag, cap in CAPS:
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    ok_fp, ok_l, ok_r, no_fp, no_l, no_r = [], [], [], [], [], []
    for r in det:
        fp = num(r.get('front_points'))
        l = num(r.get('left_near_count'))
        rr = num(r.get('right_near_count'))
        if r.get('source') == '3':
            no_fp.append(fp); no_l.append(l); no_r.append(rr)
        else:
            ok_fp.append(fp); ok_l.append(l); ok_r.append(rr)
    same = med(no_fp) is not None and med(ok_fp) is not None and med(no_fp) > 0.6 * med(ok_fp)
    print('%-10s %-26s %-26s %s' % (tag,
          '%s / %s / %s' % (med(ok_fp), med(ok_l), med(ok_r)),
          '%s / %s / %s' % (med(no_fp), med(no_l), med(no_r)),
          '点还在，是关联链断了' if same else '点本身就少了'))
    print('%-10s   无来源帧数=%d' % ('', len(no_fp)))

print()
print('=' * 104)
print('层3（补全）  弧模型被拒在哪一条（race 的 curve_checks）')
print('%-10s %-16s %-14s %s' % ('采集', 'model', 'curve_rejected', 'curve_checks 位'))
for tag, cap in CAPS:
    rac = rows(os.path.join(LOG, cap + '.race.csv'))
    md = Counter(int(num(r.get('model')) or 0) for r in rac)
    cr = Counter(int(num(r.get('curve_rejected')) or 0) for r in rac)
    cb = Counter()
    for r in rac:
        for b, nm in CVB:
            if int(num(r.get('curve_checks')) or 0) & b:
                cb[nm] += 1
    print('%-10s %-16s %-14s %s' % (tag,
          ' '.join('%d×%d' % (k, v) for k, v in sorted(md.items())),
          ' '.join('%d×%d' % (k, v) for k, v in sorted(cr.items())),
          ' '.join('%s×%d' % (k, v) for k, v in cb.most_common())))

print()
print('=' * 104)
print('层4（补全）  NEAR缺失/无来源 时，两侧近段分区数与跨度的中位数')
print('%-10s %-34s %s' % ('采集', '有来源: 左近(分区/跨度) 右近(分区/跨度)', '无来源: 左近 右近'))
for tag, cap in CAPS:
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    g = {0: [], 1: []}
    for r in det:
        k = 1 if r.get('source') == '3' else 0
        g[k].append((num(r.get('left_near_count')), num(r.get('left_span')),
                     num(r.get('right_near_count')), num(r.get('right_span'))))
    def s(k):
        a = g[k]
        if not a:
            return '-'
        return '%s/%s  %s/%s (n=%d)' % (med([x[0] for x in a]), med([x[1] for x in a]),
                                        med([x[2] for x in a]), med([x[3] for x in a]), len(a))
    print('%-10s %-34s %s' % (tag, s(0), s(1)))
