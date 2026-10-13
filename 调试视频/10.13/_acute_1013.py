# -*- coding: utf-8 -*-
"""10/13 六组：解出 acute 阶段的"卡在哪一条"，并验证 PREFILTER(512) 的成因。

口径来源（代码，不是猜）：
  acute_checks 是**拒绝位图**（path_track.c 15 个赋值点全为 |= 失败条件）
    NEAR=1 PARALLEL=2 DIRECTION=4 RANGE=8 ROOM=16 PRIORITY=32
    GAP=64 LINKED=128 CONE=256 PREFILTER=512
  path_track.c:1233  PREFILTER 条件 = out->model || (phase==0 且两侧远段都
                     !valid 或 |a|<=0.30)
  path_track.c:980   wall->gap=1 是在通过 tip 测试后**无条件**置位
  path_track.c:1306  连续 2 帧才从 ACUTE_CONFIRM(1) 升到 ACUTE_PREPARE(2)
"""
import csv, json, os
from collections import Counter

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('194302_左大弯', 'v2_1013_194302_3416906000000_195e6842'),
        ('194450_左弯', 'v2_1013_194450_3524718000000_b41f5f1e'),
        ('194631_左弯', 'v2_1013_194631_3625625000000_8da28e07'),
        ('194802_右弯_s弯后大急弯', 'v2_1013_194802_3717046000000_bd535409'),
        ('195030_右弯_表现差劲', 'v2_1013_195030_3865031000000_676aafed'),
        ('195147_s弯没成功', 'v2_1013_195147_3941359000000_fd8a625e')]
BITS = [(512, 'PREFILTER'), (256, 'CONE'), (128, 'LINKED'), (64, 'GAP'), (32, 'PRIORITY'),
        (16, 'ROOM'), (8, 'RANGE'), (4, 'DIRECTION'), (2, 'PARALLEL'), (1, 'NEAR')]
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


def dec(v):
    if v is None:
        return '?'
    try:
        i = int(float(v))
    except Exception:
        return '?'
    if i == 0:
        return '0(全过)'
    return '|'.join(n for b, n in BITS if i & b) or str(i)


def main():
    rep = {}
    for tag, cap in CAPS:
        ac, ctl, pat, rac = (rows(os.path.join(LOG, cap + f)) for f in
                             ('.acute.csv', '.csv', '.path.csv', '.race.csv'))
        if not ac:
            print('%s 无 .acute.csv' % tag); continue
        dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
        dR = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}
        ck, ph, rs = Counter(), Counter(), Counter()
        pref_why = Counter()
        rowsout = []
        for r in ac:
            k = (r.get('control_seq'), r.get('param_revision'))
            v = r.get('acute_checks')
            ck[dec(v)] += 1
            ph[int(num(r.get('acute_phase')) or 0)] += 1
            rs[int(num(r.get('acute_reason')) or 0)] += 1
            p, x = dP.get(k) or {}, dR.get(k) or {}
            try:
                iv = int(float(v))
            except Exception:
                iv = -1
            if iv == 512:
                mdl = num(x.get('model'))
                f0a = num(p.get('left_far_a')); f1a = num(p.get('right_far_a'))
                f0 = num(p.get('left_far_max_y')); f1 = num(p.get('right_far_max_y'))
                why = []
                if mdl:
                    why.append('model=%g' % mdl)
                if not f0 or (f0a is not None and abs(f0a) <= .30):
                    why.append('左远弱')
                if not f1 or (f1a is not None and abs(f1a) <= .30):
                    why.append('右远弱')
                pref_why['+'.join(why) or '未解'] += 1
            rowsout.append(dict(t=num(r.get('host_t')), phase=int(num(r.get('acute_phase')) or 0),
                                reason=int(num(r.get('acute_reason')) or 0),
                                dirn=int(num(r.get('acute_direction_code')) or 0),
                                checks=dec(v), model=num(x.get('model')),
                                left_far_a=f0a if False else num(p.get('left_far_a')),
                                right_far_a=num(p.get('right_far_a'))))
        print('=' * 96)
        print('%s   %s' % (tag, cap))
        print('  .acute.csv %d 行   phase: %s' %
              (len(ac), ' '.join('%s×%d' % (PH[k], n) for k, n in sorted(ph.items()))))
        print('  reason: %s' % ' '.join('%s×%d' % (RSN[k], n) for k, n in sorted(rs.items(), key=lambda z: -z[1])))
        print('  checks: %s' % '  '.join('%s×%d' % (k, n) for k, n in ck.most_common()))
        if pref_why:
            print('  **512(PREFILTER) 的成因分布:', ' '.join('%s×%d' % (k, n) for k, n in pref_why.most_common()))
        rep[tag] = dict(capture=cap, n=len(ac), phase=dict(ph), reason=dict(rs), checks=dict(ck),
                        prefilter_why=dict(pref_why), rows=rowsout)
    json.dump(rep, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'acute_1013_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n写出 acute_1013_summary.json')


if __name__ == '__main__':
    main()
