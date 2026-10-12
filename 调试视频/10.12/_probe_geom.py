# -*- coding: utf-8 -*-
"""看清成功过弯采集里"几何"到底给了什么：逐帧打印关键列，判断哪个量能定"弯道在哪"。

设计这个脚本是因为 turn_distance_mm 的定义被固件注释否掉了
（path_track.h:75 "尚非标定的真实拐点距离"）。先看数据，再决定怎么算，不猜。
"""
import csv, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = ['v2_1012_164512_3089484000000', 'v2_1012_164942_3359656000000']
FITS = ['left_near_a', 'left_near_b', 'left_near_min_y', 'left_near_max_y',
        'right_near_a', 'right_near_b', 'right_near_min_y', 'right_near_max_y',
        'left_far_a', 'left_far_b', 'left_far_min_y', 'left_far_max_y',
        'right_far_a', 'right_far_b', 'right_far_min_y', 'right_far_max_y']


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


for cap in CAPS:
    ctl = rows(os.path.join(LOG, cap + '.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    pat = rows(os.path.join(LOG, cap + '.path.csv'))
    rac = rows(os.path.join(LOG, cap + '.race.csv'))
    print('=' * 100)
    print('%s  ctl=%d det=%d path=%d race=%d' % (cap, len(ctl), len(det), len(pat), len(rac)))
    dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
    dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
    dR = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}
    print('  n=%3s %6s %5s %5s %4s %4s %4s %8s %8s %8s %7s %7s %6s %6s %6s' %
          ('', 't', 'pwm', 'spd', 'src', 'bend', 'far', 'near_x', 'far_x', 'ref_y',
           'near_a', 'far_a', 'width', 'tdist', 'supp'))
    for i, r in enumerate(ctl):
        if i % 6:
            continue
        k = (r.get('control_seq'), r.get('param_revision'))
        d, p, x = dD.get(k) or {}, dP.get(k) or {}, dR.get(k) or {}
        print('  n=%3d %6.2f %5s %5s %4s %4s %4s %8s %8s %8s %7s %7s %6s %6s %6s' %
              (i, num(r.get('host_t')) or 0, r.get('servo_pwm'), r.get('Speed_now'),
               d.get('source'), d.get('bend_code'), d.get('far_valid'),
               d.get('near_x'), d.get('far_x'), d.get('ref_y'),
               d.get('near_a'), d.get('far_a'), d.get('track_width'),
               x.get('turn_distance_mm'), x.get('support_mm')))
    print('  --- path.csv 的 4 条拟合（前 8 行里非零的）---')
    shown = 0
    for p in pat:
        vals = {f: num(p.get(f)) for f in FITS}
        if not any(v for v in vals.values() if v):
            continue
        print('   t=%.2f  ' % (num(p.get('host_t')) or 0) +
              '  '.join('%s=%.1f' % (f.replace('_near', 'N').replace('_far', 'F').replace('left', 'L').replace('right', 'R'), vals[f])
                        for f in FITS if vals[f]))
        shown += 1
        if shown >= 8:
            break
    print('  --- race.csv 全部行 ---')
    print('   %6s %8s %8s %8s %8s %7s %7s %6s %6s' %
          ('t', 'tdist', 'support', 'bend_rad', 'head_rad', 'prev_hd', 'lim', 'crej', 'sprev'))
    for x in rac:
        print('   %6.2f %8s %8s %8s %8s %7s %7s %6s %6s' %
              (num(x.get('host_t')) or 0, x.get('turn_distance_mm'), x.get('support_mm'),
               x.get('bend_rad'), x.get('heading_rad'), x.get('preview_heading_rad'),
               x.get('speed_limit'), x.get('curve_rejected'), x.get('s_preview')))
