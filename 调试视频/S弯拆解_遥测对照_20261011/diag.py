# -*- coding: utf-8 -*-
"""诊断：1) 遥测里"车真的在动"的时间窗口  2) CONTROL/detail/path/race 的主键重叠。"""
import csv, json, os

LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
CAPS = ['v2_1011_215921_18542828000000', 'v2_1011_220025_18606281000000',
        'v2_1011_220116_18657781000000', 'v2_1011_220417_18838921000000']
VDUR = {'v2_1011_215921_18542828000000': 7.27, 'v2_1011_220025_18606281000000': 7.50,
        'v2_1011_220116_18657781000000': 7.10, 'v2_1011_220417_18838921000000': 7.00}


def rows(p):
    with open(p, encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def f(v):
    try:
        return float(v)
    except Exception:
        return None


for cap in CAPS:
    print('=' * 78)
    meta = json.load(open(os.path.join(LOG, cap + '.bin.meta.json'), encoding='utf-8'))
    ctl = rows(os.path.join(LOG, cap + '.csv'))
    det = rows(os.path.join(LOG, cap + '.detail.csv'))
    pat = rows(os.path.join(LOG, cap + '.path.csv'))
    rac = rows(os.path.join(LOG, cap + '.race.csv'))
    mot = rows(os.path.join(LOG, cap + '.motor.csv'))
    print('%s  cap_dur=%.3f  video_dur=%.2f  end_align_offset=%+.2f' %
          (cap[-6:], meta['duration_s'], VDUR[cap], meta['duration_s'] - VDUR[cap]))
    print('  rows: ctl=%d det=%d path=%d race=%d motor=%d' % (len(ctl), len(det), len(pat), len(rac), len(mot)))

    # --- 车速运动窗口（用 motor.csv 的 speed_float，host_t 为轴）---
    sp = [(f(r.get('host_t')), f(r.get('speed_float'))) for r in mot]
    sp = [(t, s) for t, s in sp if t is not None and s is not None]
    mv = [(t, s) for t, s in sp if abs(s) > 1.0]
    if mv:
        t_first, t_last = mv[0][0], mv[-1][0]
        print('  motor: t=%.2f..%.2f  |speed|>1 首次 %.2f 末次 %.2f  跨度 %.2fs  (视频 %.2fs)'
              % (sp[0][0], sp[-1][0], t_first, t_last, t_last - t_first, VDUR[cap]))
        # 连续段
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
        runs = [r for r in runs if r[1] - r[0] > 0.3]
        print('  motor 连续运动段(>0.3s): ' + ', '.join('%.2f~%.2f(%.2fs)' % (a, b, b - a) for a, b in runs))
    else:
        print('  motor: 没有 |speed|>1 的样本')

    # --- 主键重叠 ---
    kc = set((r.get('control_seq'), r.get('param_revision')) for r in ctl)
    for nm, dat in (('detail', det), ('path', pat), ('race', rac)):
        k = set((r.get('control_seq'), r.get('param_revision')) for r in dat)
        print('  %-6s keys=%3d  与ctl重叠=%3d   样例key=%s' %
              (nm, len(k), len(k & kc), sorted(k)[:2]))
    # race 的 host_t 范围
    if rac:
        hs = [f(r.get('host_t')) for r in rac]
        print('  race host_t: %.2f..%.2f  control_seq 样例 %s' %
              (min(hs), max(hs), [r.get('control_seq') for r in rac[:4]]))
    if pat:
        print('  path host_t: %.2f..%.2f' % (min(f(r.get('host_t')) for r in pat),
                                             max(f(r.get('host_t')) for r in pat)))
    if det:
        print('  det  host_t: %.2f..%.2f' % (min(f(r.get('host_t')) for r in det),
                                             max(f(r.get('host_t')) for r in det)))
    if ctl:
        print('  ctl  host_t: %.2f..%.2f   Speed_mubiao 取值=%s' %
              (f(ctl[0].get('host_t')), f(ctl[-1].get('host_t')),
               sorted(set(r.get('Speed_mubiao') for r in ctl))[:6]))
