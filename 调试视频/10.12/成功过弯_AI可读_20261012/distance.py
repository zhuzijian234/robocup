# -*- coding: utf-8 -*-
"""成功过弯的"参考距离"求解。**所有定义都写在下面，可复核。**

已知：turn_distance_mm 不是到弯距离
  path_track.h:75  turn_distance_mm /* S预告时为近段参考距离，尚非标定的真实拐点距离。 */
  path_track.c:1058 out->turn_distance_mm = out->s_preview ? out->near_ref_y : out->support_mm;
  => 它就是 near_ref_y(近段参考高度) 或 support_mm(拟合窗口上界)，本批 s_preview 恒为 0。

所以自己定义并求解：
  弯道前向距离 y_corner(side) = 同一侧"近段拟合直线"与"远段拟合直线"的交点 y
      近段: x = a1*y + b1 , y in [min_y1, max_y1]
      远段: x = a2*y + b2 , y in [min_y2, max_y2]
      交点: y = (b2 - b1) / (a1 - a2)
  门限（全部满足才算有效，否则该侧记 none）：
    G1 两侧拟合都"未被拒"：fit_rejected 的对应字节 == 0
       字节序来自 ble_diag.c:440  near[0] | near[1]<<8 | far[0]<<16 | far[1]<<24
       数组下标 0=左 1=右（path_track.c boundary() 里 side=0 走 -1）
    G2 |a1 - a2| >= 0.05        （两条线近乎平行时交点无意义）
    G3 min_y1 < max_y2           （近段在远段之前，顺序正常）
    G4 交点落在 [min_y1 - 50, max_y2 + 50] 内（不许大幅外插）
  一致性检查（不参与门限，只报出来）：
    C1 交点应 >= max_y1 - 50     （拐点在近段可见范围之外/边缘）
    C2 |y_left - y_right| <= 300 mm 时记 agree，否则记 disagree

车的位置口径（同样有代码依据）：
  target_x = 路径中线的横向位置（车体坐标，+x = 车右），车在 x=0
  error = cx - target_x   (main.c 侧 CENTER_X_TARGET_MM = -27)
  road_error 在无避让时 == error
  => road_error 就是"车相对目标路径的横向偏差(mm)"，>0 表示车偏在路径右侧
"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\成功过弯_AI可读_20261012'
CAPS = [('SUC01_164512_左弯_电机关闭', 'v2_1012_164512_3089484000000', 'left'),
        ('SUC02_164616_左弯', 'v2_1012_164616_3154078000000', 'left'),
        ('SUC03_164757_左弯', 'v2_1012_164757_3254406000000', 'left'),
        ('SUC04_164942_右弯', 'v2_1012_164942_3359656000000', 'right'),
        ('SUC05_165050_右弯', 'v2_1012_165050_3428000000000', 'right'),
        ('SUC06_165134_右弯', 'v2_1012_165134_3471765000000', 'right')]
MID, TH = 1445.0, 150.0
DA_MIN, SLACK = 0.05, 50.0
AGR_TOL = 300.0


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


def corner(near, far, rej, shift):
    """near/far: dict with a,b,min_y,max_y ; rej: fit_rejected int ; shift: 0=左 8=右
    返回 (y_corner or None, 说明)"""
    if rej is None:
        return None, 'no_fit_rejected_field'
    if ((rej >> shift) & 0xFF) or ((rej >> (shift + 16)) & 0xFF):
        return None, 'G1_side_rejected'
    a1, b1 = num(near.get('a')), num(near.get('b'))
    a2, b2 = num(far.get('a')), num(far.get('b'))
    y1lo, y1hi = num(near.get('min_y')), num(near.get('max_y'))
    y2lo, y2hi = num(far.get('min_y')), num(far.get('max_y'))
    if None in (a1, b1, a2, b2, y1lo, y1hi, y2lo, y2hi):
        return None, 'G1_missing'
    if y1hi <= y1lo or y2hi <= y2lo:
        return None, 'G1_degenerate_range'
    if abs(a1 - a2) < DA_MIN:
        return None, 'G2_parallel'
    if not (y1lo < y2hi):
        return None, 'G3_order'
    y = (b2 - b1) / (a1 - a2)
    if not (y1lo - SLACK <= y <= y2hi + SLACK):
        return None, 'G4_extrapolated'
    return y, 'ok'


def main():
    allrep, allframes = {}, []
    for tag, cap, turn in CAPS:
        ctl, det, pat, rac, mot = (rows(os.path.join(LOG, cap + f)) for f in
                                   ('.csv', '.detail.csv', '.path.csv', '.race.csv', '.motor.csv'))
        if not ctl:
            print(tag, '无 CONTROL'); continue
        dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
        dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
        dR = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}
        # 运动窗口（与 extract 同法）
        sp = [(num(r.get('host_t')), num(r.get('speed_float'))) for r in mot]
        sp = [(t, s) for t, s in sp if t is not None and s is not None]
        runs, s0, prev = [], None, None
        for t, s in sp:
            m = abs(s) > 1.0
            if m and s0 is None:
                s0 = t
            if not m and s0 is not None:
                runs.append((s0, prev)); s0 = None
            prev = t
        if s0 is not None:
            runs.append((s0, prev))
        runs = [r for r in runs if r[1] and r[1] - r[0] > 0.3]
        t_start = runs[0][0] if runs else 0.0

        recs = []
        for r in ctl:
            t = num(r.get('host_t'))
            if t is None:
                continue
            k = (r.get('control_seq'), r.get('param_revision'))
            d, p, x = dD.get(k) or {}, dP.get(k) or {}, dR.get(k) or {}
            rej = None
            try:
                rej = int(float(p.get('fit_rejected')))
            except Exception:
                pass
            yl, why_l = corner(dict(a=p.get('left_near_a'), b=p.get('left_near_b'),
                                    min_y=p.get('left_near_min_y'), max_y=p.get('left_near_max_y')),
                               dict(a=p.get('left_far_a'), b=p.get('left_far_b'),
                                    min_y=p.get('left_far_min_y'), max_y=p.get('left_far_max_y')),
                               rej, 0)
            yr, why_r = corner(dict(a=p.get('right_near_a'), b=p.get('right_near_b'),
                                    min_y=p.get('right_near_min_y'), max_y=p.get('right_near_max_y')),
                               dict(a=p.get('right_far_a'), b=p.get('right_far_b'),
                                    min_y=p.get('right_far_min_y'), max_y=p.get('right_far_max_y')),
                               rej, 8)
            agree = ''
            if yl is not None and yr is not None:
                agree = 'agree' if abs(yl - yr) <= AGR_TOL else 'disagree'
            recs.append(dict(t=t, pwm=num(r.get('servo_pwm')), spd=num(r.get('Speed_now')),
                             bend=d.get('bend_code'), src=d.get('source'), far=d.get('far_valid'),
                             path_reason=d.get('path_reason'), near_x=num(d.get('near_x')),
                             far_x=num(d.get('far_x')), ref_y=num(d.get('ref_y')),
                             near_a=num(d.get('near_a')), far_a=num(d.get('far_a')),
                             width=num(d.get('track_width')), road_error=num(p.get('road_error')),
                             y_left=yl, why_left=why_l, y_right=yr, why_right=why_r, agree=agree))

        moving = [x for x in recs if (x['spd'] or 0) > 1.0]
        # 入弯摆舵机：运动开始后首个 |pwm-1445|>150
        turnin = next((x for x in moving if x['pwm'] is not None and abs(x['pwm'] - MID) > TH), None)
        # 入弯前"贴近外侧"：turnin 之前 |road_error| 最大的一帧（>0=车偏路径右）
        pre = [x for x in moving if turnin and x['t'] < turnin['t'] and x['road_error'] is not None]
        hug = max(pre, key=lambda x: abs(x['road_error'])) if pre else None
        cover = sum(1 for x in moving if x['y_left'] is not None or x['y_right'] is not None)

        def show(label, x):
            if not x:
                print('   %-22s 无' % label); return
            print('   %-22s t=%6.2fs pwm=%5.0f spd=%5.1f bend=%s src=%s far=%s' %
                  (label, x['t'], x['pwm'] or 0, x['spd'] or 0, x['bend'], x['src'], x['far']))
            print('   %-22s 到弯距离: 左=%s 右=%s   (%s / %s)  %s' %
                  ('', ('%.0f' % x['y_left']) if x['y_left'] is not None else 'n/a',
                   ('%.0f' % x['y_right']) if x['y_right'] is not None else 'n/a',
                   x['why_left'], x['why_right'], x['agree']))
            print('   %-22s 车的位置: near_x=%s far_x=%s road_error=%s ref_y=%s 宽=%s' %
                  ('', x['near_x'], x['far_x'], x['road_error'], x['ref_y'], x['width']))
        print('=' * 96)
        print('%s  (%s弯)  CONTROL %d 帧  运动 %d 帧  可解 y_corner 的帧 %d (%.0f%%)' %
              (tag, turn, len(ctl), len(moving), cover, 100.0 * cover / max(1, len(moving))))
        show('入弯摆舵机时刻', turnin)
        show('入弯前最贴外侧', hug)
        allrep[tag] = dict(capture=cap, turn=turn, control_rows=len(ctl), moving_frames=len(moving),
                           corner_solvable_frames=cover,
                           turnin=None if not turnin else {k: turnin[k] for k in
                                                           ('t', 'pwm', 'spd', 'bend', 'src', 'far',
                                                            'near_x', 'road_error', 'y_left', 'y_right',
                                                            'why_left', 'why_right', 'agree', 'near_a', 'far_a')},
                           hug=None if not hug else {k: hug[k] for k in
                                                     ('t', 'pwm', 'spd', 'bend', 'src', 'far',
                                                      'near_x', 'road_error', 'y_left', 'y_right',
                                                      'why_left', 'why_right', 'agree', 'near_a', 'far_a')})
        for x in recs:
            allframes.append(dict(tag=tag, **{k: ('' if v is None else v) for k, v in x.items()}))
    json.dump(allrep, open(os.path.join(OUT, 'distance_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    if allframes:
        with open(os.path.join(OUT, 'distance_frames.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
            wr = csv.DictWriter(fh, fieldnames=list(allframes[0].keys()))
            wr.writeheader()
            wr.writerows(allframes)
    print('\n写出 distance_summary.json / distance_frames.csv')


if __name__ == '__main__':
    main()
