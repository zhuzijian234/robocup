# -*- coding: utf-8 -*-
"""3c1af22 后 7 段：急弯阶段时间线 + 参考量 + 对齐可信度检验。

口径全部来自 md文档/M10P_急弯提前布位修改与验证_20261012.md（作者口径）：
  acute_phase 0普通/1确认/2准备/3主弯/4出口
  acute_reason 0无入口证据/1确认中/2准备中/3主弯接管/4余量不足/5S弯锥桶优先/6证据丢失/7出口证据
  acute_direction_code 0左/1无/2右
  acute_position_mm  前轴处局部道路法向位置，向弯外为正
  acute_goal_mm      本帧向外目标，上限40mm
  acute_feature_y_mm 可见弯墙偏离近切线的位置 —— **文档明确：不能当真实弯口距离**
  位置/目标/距离/航向 **仅在阶段1/2 作为有效测量**，其他阶段零值不得当"居中/距离为0"
  文档 §1：TURN 在"可见弯墙特征进入 700mm 范围"时启动；950mm 以上用准备目标

对齐检验（本批两锚点差 1.69~4.25s，需判断哪个可信）：
  预测：撞车后电机仍在转 -> 视频结束之后编码器速度仍非零 -> 结尾锚被拉长
  检验：看 video_end 之后的 speed_float 是否仍 >1
"""
import csv, json, os

OUT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\3c1af22后_拆解_AI可读_20261012'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
PH = {0: '普通', 1: '确认', 2: '准备', 3: '主弯', 4: '出口'}
RSN = {0: '无入口证据', 1: '确认中', 2: '准备中', 3: '主弯接管', 4: '余量不足',
       5: 'S弯/锥桶优先', 6: '证据丢失', 7: '出口证据'}
DIR = {0: '左弯', 1: '无方向', 2: '右弯'}


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
    S = {r['tag']: r for r in json.load(open(os.path.join(OUT, 'summary.json'), encoding='utf-8'))}
    rep = {}
    for tag, s in S.items():
        cap = s['capture']
        rac = rows(os.path.join(LOG, cap + '.race.csv'))
        mot = rows(os.path.join(LOG, cap + '.motor.csv'))
        ctl = rows(os.path.join(LOG, cap + '.csv'))
        if not rac:
            print(tag, '无 race'); continue
        offA, vdur = s['offset_A'], s['video_duration']
        vid_end = offA + vdur          # 视频结束对应的遥测时刻
        # ---------- 对齐检验 ----------
        after = [(num(r.get('host_t')), num(r.get('speed_float'))) for r in mot]
        after = [(t, v) for t, v in after if t is not None and v is not None and t > vid_end]
        nz = [t for t, v in after if abs(v) > 1.0]
        # ---------- 阶段时间线 ----------
        timeline, cur = [], None
        for r in rac:
            p = num(r.get('acute_phase'))
            if p is None:
                continue
            t = num(r.get('host_t'))
            if cur is None or p != cur['phase']:
                if cur:
                    timeline.append(cur)
                cur = dict(phase=int(p), t0=t, t1=t, n=0, rsn=set(), dirs=set(),
                           feat=[], pos=[], goal=[], head=[], in_video=None)
            cur['t1'] = t
            cur['n'] += 1
            cur['rsn'].add(int(num(r.get('acute_reason')) or 0))
            cur['dirs'].add(int(num(r.get('acute_direction_code')) or 0))
            for key, lst in (('acute_feature_y_mm', 'feat'), ('acute_position_mm', 'pos'),
                             ('acute_goal_mm', 'goal'), ('acute_heading_rad', 'head')):
                v = num(r.get(key))
                if v is not None:
                    cur[lst].append(v)
            if cur['in_video'] is None:
                cur['in_video'] = (vid_end - 0.2 <= t <= vid_end + 0.2)
        if cur:
            timeline.append(cur)

        # ---------- 阶段1/2 的有效测量（文档：只有这两阶段有效） ----------
        valid12 = [r for r in rac if num(r.get('acute_phase')) in (1, 2)]
        pos12 = [num(r.get('acute_position_mm')) for r in valid12]
        pos12 = [v for v in pos12 if v is not None]
        goal12 = [num(r.get('acute_goal_mm')) for r in valid12]
        goal12 = [v for v in goal12 if v is not None]
        feat12 = [(num(r.get('host_t')), num(r.get('acute_feature_y_mm'))) for r in valid12]
        feat12 = [(t, v) for t, v in feat12 if v is not None]
        # TURN 接管帧（阶段 3 的第一帧）
        turn = next((r for r in rac if num(r.get('acute_phase')) == 3), None)
        # 该时刻最近的 CONTROL 帧
        ctl_t = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                       key=lambda kv: kv[0])
        pwm_at = None
        if turn and ctl_t:
            tt = num(turn.get('host_t'))
            pwm_at = num(min(ctl_t, key=lambda kv: abs(kv[0] - tt))[1].get('servo_pwm'))

        print('=' * 100)
        print('%s   %s' % (tag, cap))
        print('  对齐: offA %+.2f  offB %+.2f  差 %.2f s | 视频结束对应遥测 t=%.2f' %
              (offA, s['offset_B'], abs(offA - s['offset_B']), vid_end))
        print('  检验(视频结束之后的编码器速度): 共 %d 个样本, 其中 |speed|>1 的有 %d 个 (%.0f%%)  %s' %
              (len(after), len(nz), 100.0 * len(nz) / max(1, len(after)),
               '-> 撞后轮子仍在转，结尾锚被拉长，应用 offA' if len(nz) > 0.5 * max(1, len(after)) else ''))
        print('  阶段时间线:')
        for c in timeline:
            print('    ph=%d(%s) t=%.2f~%.2f  帧%2d  rsn=%s  dir=%s' %
                  (c['phase'], PH.get(c['phase'], '?'), c['t0'], c['t1'], c['n'],
                   '/'.join(RSN.get(x, str(x)) for x in sorted(c['rsn'])),
                   '/'.join(DIR.get(x, str(x)) for x in sorted(c['dirs']))))
            if c['phase'] in (1, 2) and c['feat']:
                print('         feat_y=%.0f~%.0f  pos=%.0f~%.0f  goal=%.1f~%.1f  head=%.3f~%.3f' %
                      (min(c['feat']), max(c['feat']), min(c['pos']), max(c['pos']),
                       min(c['goal']), max(c['goal']), min(c['head']), max(c['head'])))
        print('  阶段1/2 有效测量: pos=%.1f~%.1f mm (n=%d)  goal=%.1f~%.1f mm (n=%d)' %
              (min(pos12, default=0), max(pos12, default=0), len(pos12),
               min(goal12, default=0), max(goal12, default=0), len(goal12)))
        if feat12:
            print('  阶段1/2 feat_y 序列: %s' % ', '.join('%.2fs:%.0f' % (t, v) for t, v in feat12[:14]))
        print('  TURN 接管: %s' %
              ('无（全程未进入阶段3）' if not turn else
               't=%.2f  rsn=%s dir=%s feat_y=%s pos=%s goal=%s servo_pwm_at=%s' %
               (num(turn.get('host_t')), RSN.get(int(num(turn.get('acute_reason')) or 0)),
                DIR.get(int(num(turn.get('acute_direction_code')) or 0)),
                turn.get('acute_feature_y_mm'), turn.get('acute_position_mm'),
                turn.get('acute_goal_mm'), ('%.0f' % pwm_at) if pwm_at else '?')))
        rep[tag] = dict(capture=cap, offset_A=offA, offset_B=s['offset_B'],
                        spread=round(abs(offA - s['offset_B']), 3), video_end_tele=round(vid_end, 2),
                        after_end_samples=len(after), after_end_moving=len(nz),
                        timeline=[dict(phase=c['phase'], t0=round(c['t0'], 2), t1=round(c['t1'], 2),
                                       n=c['n'], rsn=sorted(c['rsn']), dirs=sorted(c['dirs']))
                                  for c in timeline],
                        pos12=[min(pos12, default=None), max(pos12, default=None), len(pos12)],
                        goal12=[min(goal12, default=None), max(goal12, default=None), len(goal12)],
                        turn=None if not turn else dict(
                            t=num(turn.get('host_t')),
                            reason=int(num(turn.get('acute_reason')) or 0),
                            direction=int(num(turn.get('acute_direction_code')) or 0),
                            feature_y=num(turn.get('acute_feature_y_mm')),
                            position=num(turn.get('acute_position_mm')),
                            goal=num(turn.get('acute_goal_mm')),
                            servo_pwm=pwm_at))
    json.dump(rep, open(os.path.join(OUT, 'acute_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n写出 acute_summary.json')


if __name__ == '__main__':
    main()
