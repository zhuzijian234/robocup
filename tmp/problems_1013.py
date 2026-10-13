# -*- coding: utf-8 -*-
"""10/13 六组（fw=73299d3 / sha256-59f2128e）问题清点。

只用 (session, control_seq, param_revision) 精确关联；缺字段留空不补 0。
口径：
  action_reason  1=本帧PD执行  2=算出但保舵  5=无有效转向
  geometry_reason 0=OK 1=ARGUMENT 2=NEAR_MISSING 3=CONFLICT 4=SUPPORT
  source         0=双侧 1=仅左 2=仅右 3=无
  fit_rejected   4条拟合各8位：byte0左近 byte1右近 byte2左远 byte3右远；位=1点数2跨度4间隙8退化16RMS32斜率
  acute_checks   拒绝位图：NEAR1 PARALLEL2 DIRECTION4 RANGE8 ROOM16 PRIORITY32 GAP64 LINKED128 CONE256 PREFILTER512
"""
import csv, os
from collections import Counter

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('①194302 左大弯', 'v2_1013_194302_3416906000000_195e6842', '左大弯'),
        ('②194450 左弯', 'v2_1013_194450_3524718000000_b41f5f1e', '左弯'),
        ('③194631 左弯', 'v2_1013_194631_3625625000000_8da28e07', '左弯'),
        ('④194802 右弯', 'v2_1013_194802_3717046000000_bd535409', '右弯_s弯后大急弯表现差劲'),
        ('⑤195030 右弯', 'v2_1013_195030_3865031000000_676aafed', '右弯,表现差劲'),
        ('⑥195147 s弯', 'v2_1013_195147_3941359000000_fd8a625e', 's弯没成功通过')]
ACB = [(512, 'PREFILTER'), (256, 'CONE'), (128, 'LINKED'), (64, 'GAP'), (32, 'PRIORITY'),
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


def runs_of(flags):
    """连续 True 的长度列表"""
    out, cur = [], 0
    for f in flags:
        if f:
            cur += 1
        elif cur:
            out.append(cur); cur = 0
    if cur:
        out.append(cur)
    return out


print('=' * 112)
print('一、控制与感知总表（视频标注取自文件名）')
print('%-16s %-22s %5s %6s %6s %6s %6s %6s %6s' %
      ('采集', '视频标注', 'CTRL', 'PD', '保舵', '无转向', 'PWMmin', 'PWMmax', '速度max'))
ALL = dict(ctl=0, pd=0, hold=0, inv=0, sat_lo=0, sat_hi=0)
per = {}
for tag, cap, note in CAPS:
    ctl, det, pat, rac, ac = (rows(os.path.join(LOG, cap + f)) for f in
                              ('.csv', '.detail.csv', '.path.csv', '.race.csv', '.acute.csv'))
    dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
    ar = Counter(int(num(r.get('action_reason')) or 0) for r in ctl)
    pw = [num(r.get('servo_pwm')) for r in ctl]; pw = [v for v in pw if v is not None]
    sp = [num(r.get('Speed_now')) for r in ctl]; sp = [v for v in sp if v is not None]
    pd = ar.get(1, 0); hd = ar.get(2, 0); iv = ar.get(5, 0)
    lo = sum(1 for v in pw if v <= 1180); hi = sum(1 for v in pw if v >= 1710)
    ALL['ctl'] += len(ctl); ALL['pd'] += pd; ALL['hold'] += hd; ALL['inv'] += iv
    ALL['sat_lo'] += lo; ALL['sat_hi'] += hi
    print('%-16s %-22s %5d %6d %6d %6d %6.0f %6.0f %6.2f' %
          (tag, note, len(ctl), pd, hd, iv, min(pw or [0]), max(pw or [0]), max(sp or [0])))
    per[tag] = dict(cap=cap, ctl=ctl, det=det, pat=pat, rac=rac, ac=ac, dP=dP, note=note,
                    pw=pw, sp=sp, lo=lo, hi=hi, ar=ar)
print('%-16s %-22s %5d %6d %6d %6d %6s %6s %6s' %
      ('合计', '', ALL['ctl'], ALL['pd'], ALL['hold'], ALL['inv'], '', '', ''))
print('  合计比例: PD %.0f%%  保舵 %.0f%%  无有效转向 %.0f%%    贴舵量边界 %d (%.0f%%)'
      % (100.0 * ALL['pd'] / ALL['ctl'], 100.0 * ALL['hold'] / ALL['ctl'],
         100.0 * ALL['inv'] / ALL['ctl'], ALL['sat_lo'] + ALL['sat_hi'],
         100.0 * (ALL['sat_lo'] + ALL['sat_hi']) / ALL['ctl']))

print()
print('=' * 112)
print('二、感知为什么失效（geometry_reason / source / 远段拟合）')
print('%-16s %-28s %-30s %s' % ('采集', 'geometry_reason', 'source', 'fit_rejected 前2位'))
for tag, d in per.items():
    g = Counter(int(num(x.get('geometry_reason')) or 0) for x in d['pat'])
    s = Counter(x.get('source') for x in d['det'])
    fr = Counter()
    for x in d['pat']:
        try:
            fr['0x%08X' % int(float(x.get('fit_rejected')))] += 1
        except Exception:
            pass
    print('%-16s %-28s %-30s %s' % (tag,
          ' '.join('%d×%d' % (k, v) for k, v in sorted(g.items())),
          ' '.join('%s×%d' % (k, v) for k, v in sorted(s.items(), key=lambda z: -z[1])),
          ' '.join('%s×%d' % (k, v) for k, v in fr.most_common(2))))

print()
print('=' * 112)
print('三、急弯新逻辑（73299d3 的全部新增机制）到底有没有起作用')
print('%-16s %-30s %-34s %s' % ('采集', 'acute_phase', 'acute_reason', 'acute_checks 出现的位'))
for tag, d in per.items():
    ph = Counter(int(num(x.get('acute_phase')) or 0) for x in d['ac'])
    rs = Counter(int(num(x.get('acute_reason')) or 0) for x in d['ac'])
    bits = Counter()
    nolink = 0
    for x in d['ac']:
        try:
            v = int(float(x.get('acute_checks')))
        except Exception:
            continue
        for b, n in ACB:
            if v & b:
                bits[n] += 1
    print('%-16s %-30s %-34s %s' % (tag,
          ' '.join('%s×%d' % (PH[k], v) for k, v in sorted(ph.items())),
          ' '.join('%s×%d' % (RSN[k], v) for k, v in rs.most_common(3)),
          ' '.join('%s×%d' % (k, v) for k, v in bits.most_common())))

print()
print('=' * 112)
print('四、感知"断流"长度 + 曲线模型 + 速度调度')
print('%-16s %-24s %-22s %s' % ('采集', '连续无有效转向(帧)', 'model/curve_rejected', 'speed_reason / 直道占比'))
for tag, d in per.items():
    inv = runs_of([int(num(x.get('action_reason')) or 0) == 5 for x in d['ctl']])
    md = Counter(int(num(x.get('model')) or 0) for x in d['rac'])
    cr = Counter(int(num(x.get('curve_rejected')) or 0) for x in d['rac'])
    sr = Counter(int(num(x.get('speed_reason')) or 0) for x in d['rac'])
    print('%-16s %-24s model %-16s rsn %s' % (tag,
          ('最长%d帧, 共%d段' % (max(inv or [0]), len(inv))) if inv else '无',
          ' '.join('%d×%d' % (k, v) for k, v in sorted(md.items())),
          ' '.join('%d×%d' % (k, v) for k, v in sorted(sr.items()))))
    print('%-16s   curve_rejected: %s' % ('', ' '.join('%d×%d' % (k, v) for k, v in sorted(cr.items()))))
