# -*- coding: utf-8 -*-
"""10.12 批次拆解：6 个实跑（带遥测）+ 2 个"人工推车_成功过弯"演示（无遥测）。

对齐：沿用已验证的"运动起点锚"——
  从 motor.csv 找 |speed|>1 的连续段(>0.3s)，把 video_t=0 对齐到段起点；
  同时算"结尾锚"(段末−视频时长) 作为参考，两者之差 = 对齐不确定度。
  （上一批实测：视频每帧都落在 |Speed_now|>1 的阶段，219/219 等）

无遥测的视频（人工推车）只抽帧 + 概览，并按原始 30fps 全片抽到 keyframes/ 供量距离。
只做抽帧/对齐/标注，不做原因推断。中文路径：读 fromfile+imdecode，写 imencode+tofile。
"""
import csv, json, os, shutil
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

VID = r'D:\OneDrive\Desktop\robocup\调试视频\10.12'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = os.path.join(VID, '拆解_AI可读_20261012')
FPS_PICK = 6.0
CELL_W = 320
MOVE_EPS = 1.0
RUN_MIN_S = 0.3

# (目录名, 视频文件名, 采集名 or None)
RUNS = [
    ('01_160711_右转弯_出弯刮蹭外侧', 'e86v2_1012_160711_808515000000_右转弯,出弯时刮蹭外侧.mp4', 'v2_1012_160711_808515000000'),
    ('02_161100_右转弯', 'v2_1012_161100_1037562000000_右转弯.mp4', 'v2_1012_161100_1037562000000'),
    ('03_161233_右转弯_出弯略刮蹭', 'v2_1012_161233_1130140000000_右转弯,出弯时略微刮蹭赛道边界,出弯仍旧不能完整过去.mp4', 'v2_1012_161233_1130140000000'),
    ('04_161426_左转弯_贴外侧不明显', 'v2_1012_161426_1243328000000_左转弯_入弯时贴向外侧不明显.mp4', 'v2_1012_161426_1243328000000'),
    ('05_161746_左转弯_入弯不贴外侧致出弯碰撞', 'v2_1012_161746_1443265000000_左转弯,入弯时不太能贴向赛道外侧导致出弯时碰撞.mp4', 'v2_1012_161746_1443265000000'),
    ('06_162029_左转弯_入弯后才贴外侧', 'v2_1012_162029_1606890000000_左转弯,表现不如右转弯,入弯后才贴向外侧,实际应该入弯前贴向外侧然后正常转大弯就能过去mp4.mp4', 'v2_1012_162029_1606890000000'),
    ('DEMO_左转弯_人工推车成功过弯', '人工推车_成功过弯_左转弯.mp4', None),
    ('DEMO_右转弯_人工推车成功过弯', '人工推车_成功过弯_右转弯.mp4', None),
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
        src = os.path.join(VID, vname)
        if not os.path.exists(src):
            print('MISSING VIDEO:', vname); continue
        d = os.path.join(OUT, tag)
        for sub in ('frames', 'sheets'):
            os.makedirs(os.path.join(d, sub), exist_ok=True)
        if cap:
            os.makedirs(os.path.join(d, 'telemetry'), exist_ok=True)

        meta, ctl, det, pat, rac, mot = {}, [], [], [], [], []
        if cap:
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
        cap_dur = meta.get('duration_s')

        def byk(dat):
            return {(r.get('control_seq'), r.get('param_revision')): r for r in dat}

        dD, dP, dR = byk(det), byk(pat), byk(rac)
        ctl_t = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                       key=lambda kv: kv[0])

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

        def at(t):
            if not cap or not ctl_t:
                return None, None, None, None, None
            tt = t + offset
            if tt < ctl_t[0][0] - 0.25 or tt > ctl_t[-1][0] + 0.25:
                return tt, None, None, None, None
            cr = min(ctl_t, key=lambda kv: abs(kv[0] - tt))[1]
            key = (cr.get('control_seq'), cr.get('param_revision'))
            return tt, cr, dD.get(key), dP.get(key), dR.get(key)

        for fi, t, fr in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        nmove = 0
        if cap:
            COLS = ['frame', 'video_t', 'tele_t', 'status', 'servo_pwm', 'action_reason',
                    'speed_now', 'speed_target', 'path_reason', 'command_applied', 'far_valid',
                    'source', 'bend_code', 'candidate_pwm', 'final_pwm', 'geometry_reason',
                    'fit_rejected', 'avoid_state', 'cone_candidates', 'road_error', 'avoid_offset',
                    'near_ref_y', 'width_candidate', 'left_near_a', 'left_far_a',
                    'speed_limit', 'effective_target', 'speed_reason', 'support_mm', 'outer_clearance']
            with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
                wr = csv.writer(fh); wr.writerow(COLS)
                for fi in range(total):
                    t = fi / fps
                    tt, cr, dr, pr, rr = at(t)
                    if cr is None:
                        wr.writerow([fi, round(t, 3), '' if tt is None else round(tt, 3),
                                     'out_of_range'] + [''] * (len(COLS) - 4)); continue
                    if (num(cr.get('Speed_now')) or 0) > MOVE_EPS:
                        nmove += 1
                    wr.writerow([fi, round(t, 3), round(tt, 3), 'ok',
                                 cr.get('servo_pwm'), cr.get('action_reason'),
                                 cr.get('Speed_now'), cr.get('Speed_mubiao'),
                                 (dr or {}).get('path_reason'), (dr or {}).get('command_applied'),
                                 (dr or {}).get('far_valid'), (dr or {}).get('source'),
                                 (dr or {}).get('bend_code'), (dr or {}).get('candidate_pwm'),
                                 (dr or {}).get('final_pwm'),
                                 (pr or {}).get('geometry_reason'), (pr or {}).get('fit_rejected'),
                                 (pr or {}).get('avoid_state'), (pr or {}).get('cone_candidates'),
                                 (pr or {}).get('road_error'), (pr or {}).get('avoid_offset'),
                                 (pr or {}).get('near_ref_y'), (pr or {}).get('width_candidate'),
                                 (pr or {}).get('left_near_a'), (pr or {}).get('left_far_a'),
                                 (rr or {}).get('speed_limit'), (rr or {}).get('effective_target'),
                                 (rr or {}).get('speed_reason'), (rr or {}).get('support_mm'),
                                 (rr or {}).get('outer_clearance')])

        # ---- 概览（带遥测；无遥测的只标 NO TELEMETRY） ----
        ROWS, PER = 4, 20
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, t, fr = kept[int(i)]
            ch = int(CELL_W * h / w)
            img = cv2.resize(fr, (CELL_W, ch))
            tt, cr, dr, pr, rr = at(t)
            lab = ['f%04d v=%.2fs%s' % (fi, t, '' if tt is None else ' t=%.2fs' % tt)]
            if not cap:
                lab.append('NO TELEMETRY (manual push, no capture id)'); col = (255, 255, 255)
            elif cr is None:
                lab.append('NO TELEMETRY IN RANGE'); col = (120, 170, 255)
            else:
                lab.append('pwm=%s act=%s spd=%s/%s' % (cr.get('servo_pwm'), cr.get('action_reason'),
                                                        cr.get('Speed_now'), cr.get('Speed_mubiao')))
                lab.append('path=%s far=%s src=%s bend=%s' % ((dr or {}).get('path_reason'),
                                                              (dr or {}).get('far_valid'),
                                                              (dr or {}).get('source'),
                                                              (dr or {}).get('bend_code')))
                lab.append('geo=%s fit=%s avoid=%s' % ((pr or {}).get('geometry_reason'),
                                                       (pr or {}).get('fit_rejected'),
                                                       (pr or {}).get('avoid_state')))
                lab.append('lim=%s eff=%s rsn=%s' % ((rr or {}).get('speed_limit'),
                                                     (rr or {}).get('effective_target'),
                                                     (rr or {}).get('speed_reason')))
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
                    (tag, w, h, fps, total, vdur, ('%.3fs' % cap_dur) if cap_dur else 'NONE'),
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(hdr, 'offset_A=%s offset_B=%s  |  moving frames %d/%d  |  runs %s' %
                    ('n/a' if off_a is None else '%+.2fs' % off_a,
                     'n/a' if off_b is None else '%+.2fs' % off_b, nmove, total if cap else 0,
                     ','.join('%.2f-%.2f' % r for r in m_runs)),
                    (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 230, 200), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview_tele.jpg' if cap else 'overview.jpg'),
             np.vstack([hdr, grid]))

        # ---- 无遥测的演示视频：按原始 30fps 全片抽，供量距离 ----
        if not cap:
            kd = os.path.join(d, 'keyframes')
            os.makedirs(kd, exist_ok=True)
            c = cv2.VideoCapture(src); k = 0
            while True:
                ok, fr = c.read()
                if not ok:
                    break
                save(os.path.join(kd, 'f%05d.jpg' % k), fr)
                k += 1
            c.release()
            print('%-38s %6.2fs  %3d帧(演示) -> 6fps %2d + 30fps 全片 %d' % (tag, vdur, total, len(kept), k))
        else:
            # 曲线
            if ctl_t:
                ts = [t for t, _ in ctl_t]
                pw = [num(r.get('servo_pwm')) for _, r in ctl_t]
                sp = [num(r.get('Speed_now')) for _, r in ctl_t]
                tg = [num(r.get('Speed_mubiao')) for _, r in ctl_t]
                rl = [num((dR.get((r.get('control_seq'), r.get('param_revision'))) or {}).get('speed_limit')) for _, r in ctl_t]
                geo = [num((dP.get((r.get('control_seq'), r.get('param_revision'))) or {}).get('geometry_reason')) for _, r in ctl_t]
                fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
                ax[0].plot(ts, pw, lw=1.2); ax[0].axhline(1445, ls='--', c='gray', lw=1)
                ax[0].axhline(1170, ls=':', c='r', lw=1); ax[0].axhline(1720, ls=':', c='r', lw=1)
                ax[0].set_ylabel('servo CCR'); ax[0].grid(alpha=.25)
                ax[1].plot(ts, sp, label='speed_now'); ax[1].plot(ts, tg, label='target(effective)')
                ax[1].plot(ts, rl, label='speed_limit', ls='--'); ax[1].legend(fontsize=8)
                ax[1].set_ylabel('encoder units'); ax[1].grid(alpha=.25)
                ax[2].plot(ts, geo, lw=1.0, drawstyle='steps-post')
                ax[2].set_ylabel('geometry_reason'); ax[2].set_xlabel('telemetry host_t (s)'); ax[2].grid(alpha=.25)
                if off_a is not None:
                    ax[0].axvspan(off_a, off_a + vdur, color='orange', alpha=0.12)
                fig.suptitle('%s   orange = video coverage' % cap)
                fig.tight_layout(); fig.savefig(os.path.join(d, 'telemetry_plot.png'), dpi=110); plt.close(fig)
            print('%-38s %6.2fs  %3d帧  offA %+6.2f offB %+6.2f  动帧 %3d/%3d  pwm %.0f~%.0f spd<=%.1f' %
                  (tag, vdur, total, off_a or 0, off_b if off_b is not None else 0, nmove, total,
                   min([x for x in pw if x is not None], default=0), max([x for x in pw if x is not None], default=0),
                   max([x for x in sp if x is not None], default=0)))

        cfg = {}
        for _, v2 in (meta.get('configs') or {}).items():
            cfg = v2; break
        summary.append(dict(tag=tag, capture=cap, video=vname, w=w, h=h, fps=round(fps, 2),
                            frames=total, video_duration=round(vdur, 3),
                            capture_duration=None if cap_dur is None else round(cap_dur, 3),
                            offset_A=None if off_a is None else round(off_a, 3),
                            offset_B=None if off_b is None else round(off_b, 3),
                            motion_runs=[(round(a, 2), round(b, 2)) for a, b in m_runs],
                            moving_frames=nmove, end_reason=meta.get('end_reason'),
                            ctl_rows=len(ctl), spd=cfg.get('spd'), bendspd=cfg.get('bendspd')))

    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
