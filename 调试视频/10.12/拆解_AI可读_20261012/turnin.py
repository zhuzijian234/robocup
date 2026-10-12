# -*- coding: utf-8 -*-
"""算"入弯前"和"入弯摆舵机时"的参照距离 —— 全部来自遥测，单位 mm。

数据来源与口径：
  servo_pwm / Speed_now        <- CONTROL (.csv)
  path_reason / source / bend_code / near_ref_y / far_x  <- detail.csv
  turn_distance_mm / bend_rad / heading_rad / preview_heading_rad
  / speed_limit / curve_rejected / support_mm / outer_clearance <- race.csv
注意：race.csv 约 0.5s 一行（速度规划节拍），比 CONTROL(约80ms) 稀。
      "该时刻的 race 值"取的是时间上最接近的 race 行，不是插值。
"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = [('01_160711 右转弯', 'v2_1012_160711_808515000000'),
        ('02_161100 右转弯', 'v2_1012_161100_1037562000000'),
        ('03_161233 右转弯', 'v2_1012_161233_1130140000000'),
        ('04_161426 左转弯', 'v2_1012_161426_1243328000000'),
        ('05_161746 左转弯', 'v2_1012_161746_1443265000000'),
        ('06_162029 左转弯', 'v2_1012_162029_1606890000000')]
TH = 150.0
MID = 1445.0


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


def main():
    report = {}
    for tag, cap in CAPS:
        ctl, det, pat, rac = (rows(os.path.join(LOG, cap + f)) for f in
                              ('.csv', '.detail.csv', '.path.csv', '.race.csv'))
        if not ctl:
            print(tag, '无 CONTROL'); continue
        dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
        dR = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}
        cr = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                    key=lambda kv: kv[0])
        rr = sorted([(num(r.get('host_t')), r) for r in rac if num(r.get('host_t')) is not None],
                    key=lambda kv: kv[0])

        def race_at(t):
            if not rr:
                return {}
            return min(rr, key=lambda kv: abs(kv[0] - t))[1]

        def det_of(r):
            return dD.get((r.get('control_seq'), r.get('param_revision'))) or {}

        # 入弯摆舵机：首个 |pwm-1445|>150
        turnin = next(((t, r) for t, r in cr
                       if num(r.get('servo_pwm')) is not None and abs(num(r.get('servo_pwm')) - MID) > TH), None)
        # 弯道模型首次被拒（说明前方出现真正需要建模的弯）
        rej = next(((t, r) for t, r in cr
                    if num(race_at(t).get('curve_rejected')) == 1.0), None)
        # 入弯前最后一个"中线仍在双侧/单侧、且弯模型还没拒"的帧
        pre = None
        if turnin:
            for t, r in cr:
                if t < turnin[0]:
                    pre = (t, r)
        print('=' * 74)
        print('%s   CONTROL %d 帧 / race %d 行  跨度 %.2f~%.2f s' %
              (tag, len(ctl), len(rac), cr[0][0], cr[-1][0]))
        for label, item in (('入弯摆舵机(首个|pwm-1445|>150)', turnin),
                            ('弯道模型首次被拒(curve_rejected=1)', rej),
                            ('摆舵机前一帧', pre)):
            if not item:
                print('  %-32s 无' % label); continue
            t, r = item
            d, x = det_of(r), race_at(t)
            print('  %-32s t=%6.2fs  pwm=%4s  spd=%5s  到弯距离=%7s mm  弯曲率=%6s rad' %
                  (label, t, r.get('servo_pwm'), r.get('Speed_now'),
                   x.get('turn_distance_mm', '?'), x.get('bend_rad', '?')))
            print('  %-32s   heading=%s  preview_heading=%s  speed_limit=%s  curve_rejected=%s' %
                  ('', x.get('heading_rad', '?'), x.get('preview_heading_rad', '?'),
                   x.get('speed_limit', '?'), x.get('curve_rejected', '?')))
            print('  %-32s   path_reason=%s far_valid=%s source=%s bend_code=%s' %
                  ('', d.get('path_reason'), d.get('far_valid'), d.get('source'), d.get('bend_code')))
        report[tag] = dict(
            control_rows=len(ctl), race_rows=len(rac),
            turnin=None if not turnin else dict(
                t=turnin[0], pwm=num(turnin[1].get('servo_pwm')), speed=num(turnin[1].get('Speed_now')),
                turn_distance_mm=num(race_at(turnin[0]).get('turn_distance_mm')),
                bend_rad=num(race_at(turnin[0]).get('bend_rad')),
                heading_rad=num(race_at(turnin[0]).get('heading_rad')),
                speed_limit=num(race_at(turnin[0]).get('speed_limit'))),
            curve_rejected_first=None if not rej else rej[0])
    json.dump(report, open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                        'turnin_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n写出 turnin_summary.json')


if __name__ == '__main__':
    main()
