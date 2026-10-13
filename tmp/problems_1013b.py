# -*- coding: utf-8 -*-
"""把 action_reason=5（无有效转向）按"车在动 / 车停着"分开，避免把停车段的无效当故障。"""
import csv, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('①194302 左大弯', 'v2_1013_194302_3416906000000_195e6842', '左大弯'),
        ('②194450 左弯', 'v2_1013_194450_3524718000000_b41f5f1e', '左弯'),
        ('③194631 左弯', 'v2_1013_194631_3625625000000_8da28e07', '左弯'),
        ('④194802 右弯', 'v2_1013_194802_3717046000000_bd535409', '右弯_s弯后大急弯表现差劲'),
        ('⑤195030 右弯', 'v2_1013_195030_3865031000000_676aafed', '右弯,表现差劲'),
        ('⑥195147 s弯', 'v2_1013_195147_3941359000000_fd8a625e', 's弯没成功通过')]
MOVE = 5.0


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


def longest(flags):
    out = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        out = max(out, cur)
    return out


print('%-16s %-22s %5s %6s %8s %8s %10s %10s' %
      ('采集', '视频标注', 'CTRL', '在动帧', '动+无转向', '停+无转向', '最长动-盲段', '最长盲段'))
tot = dict(ctl=0, mv=0, inv_mv=0, inv_st=0)
for tag, cap, note in CAPS:
    ctl = rows(os.path.join(LOG, cap + '.csv'))
    inv = [int(num(r.get('action_reason')) or 0) == 5 for r in ctl]
    mv = [(num(r.get('Speed_now')) or 0) > MOVE for r in ctl]
    inv_mv = [a and b for a, b in zip(inv, mv)]
    inv_st = [a and not b for a, b in zip(inv, mv)]
    print('%-16s %-22s %5d %6d %8d %8d %10d %10d' %
          (tag, note, len(ctl), sum(mv), sum(inv_mv), sum(inv_st),
           longest(inv_mv), longest(inv)))
    tot['ctl'] += len(ctl); tot['mv'] += sum(mv)
    tot['inv_mv'] += sum(inv_mv); tot['inv_st'] += sum(inv_st)
print('%-16s %-22s %5d %6d %8d %8d' % ('合计', '', tot['ctl'], tot['mv'], tot['inv_mv'], tot['inv_st']))
print()
print('  在动帧里的无转向比例 = %d/%d = %.0f%%   ← 这才是"跑着却瞎了"'
      % (tot['inv_mv'], tot['mv'], 100.0 * tot['inv_mv'] / max(1, tot['mv'])))
print('  停车帧里的无转向     = %d/%d = %.0f%%   ← 这部分是正常的'
      % (tot['inv_st'], tot['ctl'] - tot['mv'],
         100.0 * tot['inv_st'] / max(1, tot['ctl'] - tot['mv'])))
