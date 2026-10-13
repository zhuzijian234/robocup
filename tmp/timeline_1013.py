# -*- coding: utf-8 -*-
"""最深层：④194802 与 ⑥195157 的失败时序。看是"先瞎后撞"还是"先撞后瞎"。

每帧输出：t / 速度 / 电机PWM / 舵机PWM / 左近分区 / 右近分区 / geometry / path_reason / 阶段
只打印"连续动-盲段"及其前后各若干帧。
"""
import csv, os, sys

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
TARGETS = [('④194802 右弯差劲', 'v2_1013_194802_3717046000000_bd535409'),
           ('⑥195157 s弯失败', 'v2_1013_195147_3941359000000_fd8a625e')]
GEO = {0: 'OK', 1: 'ARG', 2: 'NEAR缺失', 3: '冲突', 4: '无支持'}
PTH = {0: 'APPLY', 1: 'NOGEOM', 2: '等出弯', 3: '等反向', 4: '出弯', 5: '反向', 6: '坏扫描'}
SRC = {'0': '双侧', '1': '仅左', '2': '仅右', '3': '无'}


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


for tag, cap in TARGETS:
    ctl = rows(os.path.join(LOG, cap + '.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    pat = rows(os.path.join(LOG, cap + '.path.csv'))
    mot = rows(os.path.join(LOG, cap + '.motor.csv'))
    dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
    dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
    # 电机PWM按时间就近
    mt = sorted([(num(r.get('host_t')), num(r.get('motor_pwm'))) for r in mot
                 if num(r.get('host_t')) is not None], key=lambda z: z[0])

    def motor_at(t):
        if not mt:
            return None
        return min(mt, key=lambda z: abs(z[0] - t))[1]

    inv = [int(num(r.get('action_reason')) or 0) == 5 and (num(r.get('Speed_now')) or 0) > 5 for r in ctl]
    # 找最长连续段
    best = (0, 0); cur = 0; start = 0
    for i, f in enumerate(inv):
        if f:
            if cur == 0:
                start = i
            cur += 1
            if cur > best[0]:
                best = (cur, start)
        else:
            cur = 0
    n, s0 = best
    if n == 0:
        print('%s 无"动-盲"段' % tag); continue
    a = max(0, s0 - 6); b = min(len(ctl), s0 + n + 6)
    print('=' * 128)
    print('%s  最长"动-盲"段：第 %d~%d 帧，共 %d 帧（约 %.1f s @80ms）' %
          (tag, s0, s0 + n - 1, n, n * 0.08))
    print('  %-4s %6s %6s %6s %6s %5s %5s %-9s %-8s %-6s' %
          ('帧', 't', '速度', '电机PWM', '舵机PWM', '左近', '右近', 'geometry', 'path', 'source'))
    for i in range(a, b):
        r = ctl[i]
        t = num(r.get('host_t')) or 0
        k = (r.get('control_seq'), r.get('param_revision'))
        p = dD.get(k) or {}
        q = dP.get(k) or {}
        mark = ' <--盲' if inv[i] else ''
        print('  %-4d %6.2f %6.1f %6.0f %6s %5s %5s %-9s %-8s %-6s%s' %
              (i, t, num(r.get('Speed_now')) or 0, motor_at(t) or 0, r.get('servo_pwm'),
               p.get('left_near_count'), p.get('right_near_count'),
               GEO.get(int(num(q.get('geometry_reason')) or 0), '?'),
               PTH.get(int(num(p.get('path_reason')) or 0), '?'),
               SRC.get(p.get('source'), '?'), mark))
    sys.stdout.flush()
