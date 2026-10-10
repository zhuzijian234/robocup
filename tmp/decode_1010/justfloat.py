# -*- coding: utf-8 -*-
"""把 tele 1 的 VOFA JustFloat 36 字节流解码成 CSV。

帧格式(ble_tune.c:68-91): 8×f32 小端 + 帧尾 00 00 80 7F (=+inf)
通道: err, pwm, kp, kd, mode, speed_now, speed_target, radar_dps
mode>=10 时 err 被写成 qNaN(0x7fc00000) -> 该帧没有可信误差。
帧率约 9Hz(每 ~110ms 一帧)。
"""
import csv, json, os, struct

LOG = r'D:\OneDrive\Desktop\robocup\2-LXY传承\LXY传承-m10p\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\tmp\decode_1010'
TAIL = b'\x00\x00\x80\x7f'
COLS = ['t', 'dt', 'seq', 'err', 'pwm', 'kp', 'kd', 'mode', 'speed', 'target', 'radar_dps']
NAMES = {0: 'DUAL(双侧)', 1: 'LEFT(仅左)', 2: 'RIGHT(仅右)', 10: 'HOLD(保持)', 11: 'INVALID(无效)', 12: 'FORCED'}


def frames(path, rxpath):
    data = open(path, 'rb').read()
    segs = []
    if os.path.exists(rxpath):
        for line in open(rxpath, encoding='utf-8'):
            line = line.strip()
            if line:
                segs.append(json.loads(line))
    out = []
    base = None
    i = 0
    while True:
        j = data.find(TAIL, i)
        if j < 0 or j < 32:
            break
        payload = data[j - 32:j]
        # 该帧尾所在偏移 -> 用 rx.jsonl 的段起点时间近似(110ms 分辨率)
        end = j + 4
        t = None
        for s in segs:
            if s['offset'] <= j < s['offset'] + s['length']:
                t = s['t']
                break
        if t is None:
            t = segs[-1]['t'] if segs else 0.0
        if base is None:
            base = t
        vals = struct.unpack('<8f', payload)
        out.append((t - base, vals))
        i = end
    return out


for tag in ['run_1010_171850_90260062000000', 'run_1010_175052_92181875000000']:
    b = os.path.join(LOG, tag + '.bin')
    if not os.path.exists(b):
        print('missing', tag); continue
    fr = frames(b, os.path.join(LOG, tag + '.bin.rx.jsonl'))
    print('=' * 78)
    print('%s   解出 %d 帧, 覆盖 %.2f s' % (tag, len(fr), fr[-1][0] if fr else 0))
    if not fr:
        continue
    rows = []
    prev = None
    for k, (t, v) in enumerate(fr):
        dt = '' if prev is None else round(t - prev, 3)
        prev = t
        rows.append([round(t, 3), dt, k] + [None if v[i] != v[i] else round(v[i], 3) for i in range(8)])
    with open(os.path.join(OUT, tag + '.justfloat.csv'), 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f); w.writerow(COLS); w.writerows(rows)

    modes = {}
    for t, v in fr:
        modes[int(v[4])] = modes.get(int(v[4]), 0) + 1
    pw = [v[1] for _, v in fr]
    sp = [v[5] for _, v in fr]
    tg = [v[6] for _, v in fr]
    nan = sum(1 for _, v in fr if v[0] != v[0])
    print('  mode 分布: ' + ', '.join('%s×%d' % (NAMES.get(k, k), c) for k, c in sorted(modes.items())))
    print('  err: NaN %d 帧(无可信误差) / 有效 %d 帧' % (nan, len(fr) - nan))
    print('  舵机CCR: %.0f ~ %.0f   (中位1445, 限位1170/1720)' % (min(pw), max(pw)))
    print('  车速  : %.2f ~ %.2f   目标: %.2f ~ %.2f' % (min(sp), max(sp), min(tg), max(tg)))
    print('  雷达dps: %.0f ~ %.0f' % (min(v[7] for _, v in fr), max(v[7] for _, v in fr)))
    # 粗时间线：每 ~1s 取一帧
    print('  --- 时间线(每~1s) ---')
    last = -9
    for t, v in fr:
        if t - last >= 1.0:
            last = t
            e = ' NaN ' if v[0] != v[0] else '%5.1f' % v[0]
            print('    t=%5.2f  err=%s pwm=%4.0f mode=%-12s spd=%5.2f' %
                  (t, e, v[1], NAMES.get(int(v[4]), int(v[4])), v[5]))
print('\nCSV 已写入', OUT)
