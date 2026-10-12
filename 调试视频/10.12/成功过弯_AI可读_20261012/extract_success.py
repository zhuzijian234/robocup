# -*- coding: utf-8 -*-
"""8 个"成功过弯"视频统一拆解 -> 调试视频\10.12\成功过弯_AI可读_20261012\

六个有遥测（人工推车，文件名带 v2_1012_xxxxxx）；两个没有（仅 人工推车_成功过弯_左/右转弯.mp4）。
对齐沿用已验证的"运动起点锚"：motor.csv 里 |speed|>1 的连续段(>0.3s) 起点 = video_t 0。
  ↑ "电机关闭"不影响：编码器是被动计数，推车照样出速度。
只做抽帧/对齐/标注，不做原因推断。中文路径：读 fromfile+imdecode，写 imencode+tofile。
"""
import csv, json, os, shutil
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = os.path.join(ROOT, '成功过弯_AI可读_20261012')
FPS_PICK, CELL_W, MOVE_EPS, RUN_MIN_S = 6.0, 320, 1.0, 0.3

RUNS = [
    ('SUC01_164512_左弯_电机关闭', '人工推车,成功过弯,左转弯_v2_1012_164512_3089484000000_电机关闭.mp4', 'v2_1012_164512_3089484000000'),
    ('SUC02_164616_左弯', '人工推车_成功过弯_左转弯_v2_1012_164616_3154078000000.mp4', 'v2_1012_164616_3154078000000'),
    ('SUC03_164757_左弯', '人工推车_成功过弯_左转弯_v2_1012_164757_3254406000000.mp4', 'v2_1012_164757_3254406000000'),
    ('SUC04_164942_右弯', '人工推车_成功过弯_右转弯-v2_1012_164942_3359656000000.mp4', 'v2_1012_164942_3359656000000'),
    ('SUC05_165050_右弯', '人工推车_成功过弯_右转弯v2_1012_165050_3428000000000.mp4', 'v2_1012_165050_3428000000000'),
    ('SUC06_165134_右弯', '人工推车_成功过弯_右转弯v2_1012_165134_3471765000000.mp4', 'v2_1012_165134_3471765000000'),
    ('SUC07_左弯_无遥测', '人工推车_成功过弯_左转弯.mp4', None),
    ('SUC08_右弯_无遥测', '人工推车_成功过弯_右转弯.mp4', None),
]


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


def save(p, img, q=84):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def motion_window(mot):
    sp = [(num(r.get('host_t')), num(r.get('speed_float'))) for r in mot]
    sp = [(t, s) for t, s in sp if t is not None and s is not None]
    if not sp:
        return None, None, []
    runs, s0, prev = [], None, None
    for t, s in sp:
        m = abs(s) > MOVE_EPS
        if m and s0 is None:
            s0 = t
        if not m and s0 is not None:
            runs.append((s0, prev)); s0 = None
        prev = t
    if s0 is not None:
        runs.append((s0, prev))
    runs = [r for r in runs if r[1] is not None and r[1] - r[0] > RUN_MIN_S]
    if not runs:
        return None, None, []
    return runs[0][0], max(b for _, b in runs), runs


def main():
    os.makedirs(OUT, exist_ok=True)
    summary = []
    for tag, vname, cap in RUNS:
        src = os.path.join(ROOT, vname)
        if not os.path.exists(src):
            print('MISSING:', vname); continue
        d = os.path.join(OUT, tag)
        for sub in ('frames', 'sheets'):
            os.makedirs(os.path.join(d, sub), exist_ok=True)
        meta, ctl, det, pat, rac, mot = {}, [], [], [], [], []
        if cap:
            os.makedirs(os.path.join(d, 'telemetry'), exist_ok=True)
            mp = os.path.join(LOG, cap + '.bin.meta.json')
            meta = json.load(open(mp, encoding='utf-8')) if os.path.exists(mp) else {}
            ctl, det = rows(os.path.join(LOG, cap + '.csv')), rows(os.path.join(LOG, cap + '.detail.csv'))
            pat, rac = rows(os.path.join(LOG, cap + '.path.csv')), rows(os.path.join(LOG, cap + '.race.csv'))
            mot = rows(os.path.join(LOG, cap + '.motor.csv'))
            for f in ['.csv', '.detail.csv', '.path.csv', '.race.csv', '.motor.csv', '.m10p.csv',
                      '.cloud_frames.csv', '.bin.meta.json', '_report.md']:
                p = os.path.join(LOG, cap + f)
                if os.path.exists(p):
                    shutil.copy2(p, os.path.join(d, 'telemetry', os.path.basename(p)))
        c = cv2.VideoCapture(src)
        fps = c.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(c.get(cv2.CAP_PROP_FRAME_COUNT))
        w, h = int(c.get(cv2.CAP_PROP_FRAME_WIDTH)), int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
        vdur = (total - 1) / fps
        step = max(1, int(round(fps / FPS_PICK)))
        kept, k = [], 0
        while True:
            ok, fr = c.read()
            if not ok:
                break
            if k % step == 0:
                kept.append((k, k / fps, fr))
            k += 1
        c.release()

        m_first, m_last, m_runs = motion_window(mot)
        off_a = m_first if cap else None
        off_b = (m_last - vdur) if (cap and m_last is not None) else None
        offset = off_a if off_a is not None else 0.0
        dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
        dP = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}
        ctl_t = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                       key=lambda kv: kv[0])

        def at(t):
            if not cap or not ctl_t:
                return None, None, None, None
            tt = t + offset
            if tt < ctl_t[0][0] - 0.25 or tt > ctl_t[-1][0] + 0.25:
                return tt, None, None, None
            cr = min(ctl_t, key=lambda kv: abs(kv[0] - tt))[1]
            key = (cr.get('control_seq'), cr.get('param_revision'))
            return tt, cr, dD.get(key), dP.get(key)

        for fi, t, fr in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        nmove = 0
        if cap:
            COLS = ['frame', 'video_t', 'tele_t', 'status', 'servo_pwm', 'action_reason',
                    'speed_now', 'speed_target', 'path_reason', 'command_applied', 'far_valid',
                    'source', 'bend_code', 'candidate_pwm', 'final_pwm', 'geometry_reason',
                    'fit_rejected', 'avoid_state', 'near_x', 'far_x', 'ref_y', 'near_a', 'far_a',
                    'track_width', 'road_error', 'avoid_offset', 'near_ref_y',
                    'left_near_a', 'left_near_b', 'left_near_min_y', 'left_near_max_y',
                    'right_near_a', 'right_near_b', 'right_near_min_y', 'right_near_max_y',
                    'left_far_a', 'left_far_b', 'left_far_min_y', 'left_far_max_y',
                    'right_far_a', 'right_far_b', 'right_far_min_y', 'right_far_max_y',
                    'speed_limit', 'effective_target', 'support_mm', 'turn_distance_mm', 's_preview',
                    'bend_rad', 'heading_rad', 'preview_heading_rad', 'curve_rejected']
            with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
                wr = csv.writer(fh); wr.writerow(COLS)
                for fi in range(total):
                    t = fi / fps
                    tt, cr, dr, pr = at(t)
                    if cr is None:
                        wr.writerow([fi, round(t, 3), '' if tt is None else round(tt, 3),
                                     'out_of_range'] + [''] * (len(COLS) - 4)); continue
                    if (num(cr.get('Speed_now')) or 0) > MOVE_EPS:
                        nmove += 1
                    key = (cr.get('control_seq'), cr.get('param_revision'))
                    rc = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}.get(key) or {}
                    wr.writerow([fi, round(t, 3), round(tt, 3), 'ok',
                                 cr.get('servo_pwm'), cr.get('action_reason'),
                                 cr.get('Speed_now'), cr.get('Speed_mubiao'),
                                 (dr or {}).get('path_reason'), (dr or {}).get('command_applied'),
                                 (dr or {}).get('far_valid'), (dr or {}).get('source'),
                                 (dr or {}).get('bend_code'), (dr or {}).get('candidate_pwm'),
                                 (dr or {}).get('final_pwm'),
                                 (pr or {}).get('geometry_reason'), (pr or {}).get('fit_rejected'),
                                 (pr or {}).get('avoid_state'),
                                 (dr or {}).get('near_x'), (dr or {}).get('far_x'),
                                 (dr or {}).get('ref_y'), (dr or {}).get('near_a'), (dr or {}).get('far_a'),
                                 (dr or {}).get('track_width'),
                                 (pr or {}).get('road_error'), (pr or {}).get('avoid_offset'),
                                 (pr or {}).get('near_ref_y'),
                                 (pr or {}).get('left_near_a'), (pr or {}).get('left_near_b'),
                                 (pr or {}).get('left_near_min_y'), (pr or {}).get('left_near_max_y'),
                                 (pr or {}).get('right_near_a'), (pr or {}).get('right_near_b'),
                                 (pr or {}).get('right_near_min_y'), (pr or {}).get('right_near_max_y'),
                                 (pr or {}).get('left_far_a'), (pr or {}).get('left_far_b'),
                                 (pr or {}).get('left_far_min_y'), (pr or {}).get('left_far_max_y'),
                                 (pr or {}).get('right_far_a'), (pr or {}).get('right_far_b'),
                                 (pr or {}).get('right_far_min_y'), (pr or {}).get('right_far_max_y'),
                                 rc.get('speed_limit'), rc.get('effective_target'),
                                 rc.get('support_mm'), rc.get('turn_distance_mm'), rc.get('s_preview'),
                                 rc.get('bend_rad'), rc.get('heading_rad'), rc.get('preview_heading_rad'),
                                 rc.get('curve_rejected')])

        ROWS, PER = 4, 20
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, t, fr = kept[int(i)]
            ch = int(CELL_W * h / w)
            img = cv2.resize(fr, (CELL_W, ch))
            tt, cr, dr, pr = at(t)
            lab = ['f%04d v=%.2fs%s' % (fi, t, '' if tt is None else ' t=%.2fs' % tt)]
            if not cap:
                lab.append('NO TELEMETRY'); col = (255, 255, 255)
            elif cr is None:
                lab.append('NO TELEMETRY IN RANGE'); col = (120, 170, 255)
            else:
                lab.append('pwm=%s act=%s spd=%s' % (cr.get('servo_pwm'), cr.get('action_reason'),
                                                     cr.get('Speed_now')))
                lab.append('path=%s far=%s src=%s bend=%s' % ((dr or {}).get('path_reason'),
                                                              (dr or {}).get('far_valid'),
                                                              (dr or {}).get('source'),
                                                              (dr or {}).get('bend_code')))
                lab.append('geo=%s near_x=%s rerr=%s' % ((pr or {}).get('geometry_reason'),
                                                         (dr or {}).get('near_x'),
                                                         (pr or {}).get('road_error')))
                col = (170, 230, 170)
                if (num(cr.get('Speed_now')) or 0) <= MOVE_EPS:
                    col = (140, 140, 255)
            bar = np.zeros((len(lab) * 18 + 6, CELL_W, 3), np.uint8)
            for j, tx in enumerate(lab):
                cv2.putText(bar, tx[:46], (3, 14 + j * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                            (255, 255, 255) if j == 0 else col, 1, cv2.LINE_AA)
            cells.append(np.vstack([bar, img]))
        while len(cells) < PER:
            cells.append(np.zeros_like(cells[0]))
        ch = max(x.shape[0] for x in cells)
        cells = [x if x.shape[0] == ch else np.vstack([x, np.zeros((ch - x.shape[0], CELL_W, 3), np.uint8)])
                 for x in cells]
        grid = np.vstack([np.hstack(cells[r * 5:(r + 1) * 5]) for r in range(ROWS)])
        hdr = np.zeros((52, grid.shape[1], 3), np.uint8)
        cv2.putText(hdr, '%s  %dx%d %.1ffps %d frames %.2fs | capture %s' %
                    (tag, w, h, fps, total, vdur, ('%.3fs' % meta['duration_s']) if cap else 'NONE'),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(hdr, 'offset_A=%s offset_B=%s | moving %d/%d | runs %s' %
                    ('n/a' if off_a is None else '%+.2fs' % off_a,
                     'n/a' if off_b is None else '%+.2fs' % off_b, nmove, total if cap else 0,
                     ','.join('%.2f-%.2f' % r for r in m_runs)),
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 230, 200), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview_tele.jpg' if cap else 'overview.jpg'),
             np.vstack([hdr, grid]))

        if not cap:
            kd = os.path.join(d, 'keyframes'); os.makedirs(kd, exist_ok=True)
            c = cv2.VideoCapture(src); k = 0
            while True:
                ok, fr = c.read()
                if not ok:
                    break
                save(os.path.join(kd, 'f%05d.jpg' % k), fr); k += 1
            c.release()
        cfg = {}
        for _, v2 in (meta.get('configs') or {}).items():
            cfg = v2; break
        summary.append(dict(tag=tag, capture=cap, video=vname, frames=total,
                            video_duration=round(vdur, 3),
                            capture_duration=None if not cap else round(meta['duration_s'], 3),
                            offset_A=None if off_a is None else round(off_a, 3),
                            offset_B=None if off_b is None else round(off_b, 3),
                            motion_runs=[(round(a, 2), round(b, 2)) for a, b in m_runs],
                            moving_frames=nmove, ctl_rows=len(ctl), path_rows=len(pat),
                            race_rows=len(rac), spd=cfg.get('spd'), bendspd=cfg.get('bendspd')))
        print('%-26s %6.2fs  cap %s  offA %s  offB %s  动帧 %3d/%3d' %
              (tag, vdur, ('%.3f' % meta['duration_s']) if cap else 'NONE',
               ('%+.2f' % off_a) if off_a is not None else 'n/a',
               ('%+.2f' % off_b) if off_b is not None else 'n/a', nmove, total if cap else 0))

    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
