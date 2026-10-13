# -*- coding: utf-8 -*-
"""3c1af22 后调试视频（7 段）拆解 + 遥测对齐 + 急弯阶段列。

固件：全部 fw=sha256-9b8c0a520fdcfee2（= commit 3c1af22 "大弯" / 急弯提前布位 r2）
对齐：运动起点锚（motor.csv |speed|>1 连续段>0.3s 起点 = video_t 0），并同时算结尾锚作对照。
只做抽帧/对齐/标注，不做原因推断。中文路径：读 fromfile+imdecode，写 imencode+tofile。
"""
import csv, json, os, re, shutil
import cv2
import numpy as np

SRC = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\3c1af2204be9967e8ec5bcbb33a689cf3cfb207f后调试视频'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\3c1af22后_拆解_AI可读_20261012'
FPS_PICK, CELL_W, MOVE_EPS, RUN_MIN_S = 6.0, 320, 1.0, 0.3

RUNS = [
    ('T01_222930_左弯_偏移略小略不及时', '左弯_v2_1012_222930_23745609000000_偏移量略小,略微不及时.mp4', 'v2_1012_222930_23745609000000'),
    ('T02_223105_左弯_同上次', '左弯_同上次_v2_1012_223105_23840687000000.mp4', 'v2_1012_223105_23840687000000'),
    ('T03_223130_右弯_表现良好', '右弯_表现良好v2_1012_223130_23865359000000.mp4', 'v2_1012_223130_23865359000000'),
    ('T04_223154_右弯_略微勉强', '右弯_略微勉强,偏移位置可调整v2_1012_223154_23889031000000.mp4', 'v2_1012_223154_23889031000000'),
    ('T05_223226_右弯_反应太慢直接撞', '右弯_重点排查,对弯道反应太慢,直接撞向赛道_v2_1012_223226_23921656000000.mp4', 'v2_1012_223226_23921656000000'),
    ('T06_223303_右弯_过弯勉强', '右弯_偏移位置可调整,过弯比较勉强_v2_1012_223303_23958609000000.mp4', 'v2_1012_223303_23958609000000'),
    ('T07_224402_左弯_略微刮蹭', '左弯_略微刮蹭赛道,偏移位置可调整,左弯整体表现不如右弯v2_1012_224402_24616921000000.mp4', 'v2_1012_224402_24616921000000'),
]
ACUTE = ['acute_phase', 'acute_reason', 'acute_direction_code', 'acute_position_mm', 'acute_goal_mm',
         'acute_feature_y_mm', 'acute_heading_rad']


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
        src = os.path.join(SRC, vname)
        if not os.path.exists(src):
            print('MISSING VIDEO:', vname); continue
        d = os.path.join(OUT, tag)
        for sub in ('frames', 'sheets', 'telemetry'):
            os.makedirs(os.path.join(d, sub), exist_ok=True)
        mp = os.path.join(LOG, cap + '.bin.meta.json')
        # 采集ID 校验：文件名里的 ID 必须与预期一致，且 meta 必须存在
        mfile = re.search(r'v2_1012_\d{6}_\d+', vname)
        if not mfile or mfile.group(0) != cap:
            print('  !! 采集ID 不匹配: 文件名=%s 预期=%s' % (mfile and mfile.group(0), cap))
        if not os.path.exists(mp):
            print('  !! meta 缺失，跳过', cap); continue
        meta = json.load(open(mp, encoding='utf-8'))
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
        off_a, off_b = m_first, (None if m_last is None else m_last - vdur)
        offset = off_a if off_a is not None else 0.0
        dD = {(r.get('control_seq'), r.get('param_revision')): r for r in det}
        dR = {(r.get('control_seq'), r.get('param_revision')): r for r in rac}
        ctl_t = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                       key=lambda kv: kv[0])

        def at(t):
            tt = t + offset
            if not ctl_t or tt < ctl_t[0][0] - 0.25 or tt > ctl_t[-1][0] + 0.25:
                return tt, None, None, None
            cr = min(ctl_t, key=lambda kv: abs(kv[0] - tt))[1]
            key = (cr.get('control_seq'), cr.get('param_revision'))
            return tt, cr, dD.get(key), dR.get(key)

        for fi, t, fr in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        COLS = ['frame', 'video_t', 'tele_t', 'status', 'servo_pwm', 'action_reason', 'speed_now',
                'speed_target', 'path_reason', 'command_applied', 'far_valid', 'source', 'bend_code',
                'geometry_reason', 'fit_rejected', 'near_x', 'far_x', 'ref_y', 'near_a', 'far_a',
                'track_width', 'road_target', 'target_x', 'road_error', 'avoid_offset', 'outer_offset',
                'left_near_a', 'left_near_b', 'left_near_min_y', 'left_near_max_y',
                'right_near_a', 'right_near_b', 'right_near_min_y', 'right_near_max_y',
                'left_far_a', 'left_far_b', 'left_far_min_y', 'left_far_max_y',
                'right_far_a', 'right_far_b', 'right_far_min_y', 'right_far_max_y',
                'speed_limit', 'effective_target', 'support_mm', 'turn_distance_mm', 's_preview',
                'bend_rad', 'heading_rad', 'preview_heading_rad', 'curve_rejected'] + ACUTE
        nmove = 0
        with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
            wr = csv.writer(fh); wr.writerow(COLS)
            for fi in range(total):
                t = fi / fps
                tt, cr, dr, rc = at(t)
                if cr is None:
                    wr.writerow([fi, round(t, 3), round(tt, 3), 'out_of_range'] + [''] * (len(COLS) - 4))
                    continue
                if (num(cr.get('Speed_now')) or 0) > MOVE_EPS:
                    nmove += 1
                pr = {(r.get('control_seq'), r.get('param_revision')): r for r in pat}.get(
                    (cr.get('control_seq'), cr.get('param_revision'))) or {}
                wr.writerow([fi, round(t, 3), round(tt, 3), 'ok',
                             cr.get('servo_pwm'), cr.get('action_reason'), cr.get('Speed_now'),
                             cr.get('Speed_mubiao'),
                             (dr or {}).get('path_reason'), (dr or {}).get('command_applied'),
                             (dr or {}).get('far_valid'), (dr or {}).get('source'),
                             (dr or {}).get('bend_code'),
                             pr.get('geometry_reason'), pr.get('fit_rejected'),
                             (dr or {}).get('near_x'), (dr or {}).get('far_x'), (dr or {}).get('ref_y'),
                             (dr or {}).get('near_a'), (dr or {}).get('far_a'),
                             (dr or {}).get('track_width'),
                             pr.get('road_target'), pr.get('target_x'), pr.get('road_error'),
                             pr.get('avoid_offset'), (rc or {}).get('outer_offset'),
                             pr.get('left_near_a'), pr.get('left_near_b'),
                             pr.get('left_near_min_y'), pr.get('left_near_max_y'),
                             pr.get('right_near_a'), pr.get('right_near_b'),
                             pr.get('right_near_min_y'), pr.get('right_near_max_y'),
                             pr.get('left_far_a'), pr.get('left_far_b'),
                             pr.get('left_far_min_y'), pr.get('left_far_max_y'),
                             pr.get('right_far_a'), pr.get('right_far_b'),
                             pr.get('right_far_min_y'), pr.get('right_far_max_y'),
                             (rc or {}).get('speed_limit'), (rc or {}).get('effective_target'),
                             (rc or {}).get('support_mm'), (rc or {}).get('turn_distance_mm'),
                             (rc or {}).get('s_preview'), (rc or {}).get('bend_rad'),
                             (rc or {}).get('heading_rad'), (rc or {}).get('preview_heading_rad'),
                             (rc or {}).get('curve_rejected')] +
                            [(rc or {}).get(k) for k in ACUTE])

        ROWS, PER = 4, 20
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, t, fr = kept[int(i)]
            ch = int(CELL_W * h / w)
            img = cv2.resize(fr, (CELL_W, ch))
            tt, cr, dr, rc = at(t)
            lab = ['f%04d v=%.2fs t=%.2fs' % (fi, t, tt)]
            if cr is None:
                lab.append('NO TELEMETRY IN RANGE'); col = (120, 170, 255)
            else:
                lab.append('pwm=%s act=%s spd=%s' % (cr.get('servo_pwm'), cr.get('action_reason'),
                                                     cr.get('Speed_now')))
                lab.append('path=%s far=%s src=%s bend=%s' % ((dr or {}).get('path_reason'),
                                                              (dr or {}).get('far_valid'),
                                                              (dr or {}).get('source'),
                                                              (dr or {}).get('bend_code')))
                lab.append('ACUTE ph=%s rsn=%s dir=%s' % ((rc or {}).get('acute_phase'),
                                                          (rc or {}).get('acute_reason'),
                                                          (rc or {}).get('acute_direction_code')))
                lab.append('pos=%s goal=%s featY=%s' % ((rc or {}).get('acute_position_mm'),
                                                        (rc or {}).get('acute_goal_mm'),
                                                        (rc or {}).get('acute_feature_y_mm')))
                col = (170, 230, 170)
                if (num(cr.get('Speed_now')) or 0) <= MOVE_EPS:
                    col = (140, 140, 255)
                try:
                    if float((rc or {}).get('acute_phase')) >= 1:
                        col = (80, 200, 255)
                except Exception:
                    pass
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
        cv2.putText(hdr, '%s  %dx%d %.1ffps %d frames %.2fs | %s | fw=%s' %
                    (tag, w, h, fps, total, vdur, cap, (meta.get('info') or {}).get('fw', '?')
                     if isinstance(meta.get('info'), dict) else '?'),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(hdr, 'offset_A=%s offset_B=%s | moving %d/%d | runs %s | orange=ACUTE phase>=1' %
                    ('n/a' if off_a is None else '%+.2fs' % off_a,
                     'n/a' if off_b is None else '%+.2fs' % off_b, nmove, total,
                     ','.join('%.2f-%.2f' % r for r in m_runs)),
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 230, 200), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview_tele.jpg'), np.vstack([hdr, grid]))

        summary.append(dict(tag=tag, capture=cap, video=vname, frames=total,
                            video_duration=round(vdur, 3), capture_duration=round(meta['duration_s'], 3),
                            offset_A=None if off_a is None else round(off_a, 3),
                            offset_B=None if off_b is None else round(off_b, 3),
                            motion_runs=[(round(a, 2), round(b, 2)) for a, b in m_runs],
                            moving_frames=nmove, ctl_rows=len(ctl), path_rows=len(pat),
                            race_rows=len(rac), end_reason=meta.get('end_reason'),
                            fw=(meta.get('info') or {}).get('fw') if isinstance(meta.get('info'), dict) else None))
        print('%-34s %6.2fs cap %6.3f offA %+6.2f offB %+6.2f 动帧 %3d/%3d path=%3d race=%2d' %
              (tag, vdur, meta['duration_s'], off_a or 0, off_b if off_b is not None else 0,
               nmove, total, len(pat), len(rac)))

    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
