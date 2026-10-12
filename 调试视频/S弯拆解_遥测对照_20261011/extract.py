# -*- coding: utf-8 -*-
"""s弯视频拆解 + 遥测对照（修正版）。

对齐（两套都算出来并报告，因为没有任何共同时间标记）：
  A) 运动锚点(primary)：把 video_t=0 对齐到遥测里"车开始持续运动"的时刻
       offset_A = motion_first
  B) 结尾锚点(参考)：把视频结尾对齐到"持续运动段结束"
       offset_B = motion_last_sustained - video_duration
  两者之差 = 对齐不确定度。aligned.csv 用 A，并把 B 也写给出来。

列来源（上一版在这里搞错了）：
  CONTROL(.csv) : servo_pwm, action_reason, Speed_now, Speed_mubiao
  DETAIL        : path_reason, command_applied, far_valid, source, bend_code, candidate_pwm, final_pwm
  PATH          : geometry_reason, fit_rejected, avoid_state, cone_candidates, road_error,
                  avoid_offset, near_ref_y, width_candidate, left_near_a, left_far_a
  RACE          : speed_limit, effective_target, speed_reason, support_mm, outer_clearance
只做抽帧/对齐/标注，不做原因推断。中文路径：读 fromfile+imdecode，写 imencode+tofile。
"""
import csv, json, os, shutil
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

VID = r'D:\OneDrive\Desktop\robocup\调试视频'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = os.path.join(VID, 'S弯拆解_遥测对照_20261011')
FPS_PICK = 6.0
CELL_W = 320
MOVE_EPS = 1.0        # |speed|>1 视为在动
RUN_MIN_S = 0.3       # 连续段最短

RUNS = [
    ('01_215921', 's弯_v2_1011_215921_18542828000000.mp4', 'v2_1011_215921_18542828000000'),
    ('02_220025', 's弯_v2_1011_220025_18606281000000_.mp4', 'v2_1011_220025_18606281000000'),
    ('03_220116_表现良好', 's弯_v2_1011_220116_18657781000000_表现良好.mp4', 'v2_1011_220116_18657781000000'),
    ('04_220417_表现良好', 's弯_v2_1011_220417_18838921000000_表现良好.mp4', 'v2_1011_220417_18838921000000'),
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
    """返回 (first, last_sustained, 段列表)。轴用 host_t，值用 speed_float。"""
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
            runs.append((s0, prev))
            s0 = None
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
        for sub in ('frames', 'sheets', 'telemetry'):
            os.makedirs(os.path.join(d, sub), exist_ok=True)

        meta_p = os.path.join(LOG, cap + '.bin.meta.json')
        meta = json.load(open(meta_p, encoding='utf-8')) if os.path.exists(meta_p) else {}
        cap_dur = meta.get('duration_s')
        ctl, det = rows(os.path.join(LOG, cap + '.csv')), rows(os.path.join(LOG, cap + '.detail.csv'))
        pat, rac = rows(os.path.join(LOG, cap + '.path.csv')), rows(os.path.join(LOG, cap + '.race.csv'))
        mot = rows(os.path.join(LOG, cap + '.motor.csv'))

        def byk(dat):
            return {(r.get('control_seq'), r.get('param_revision')): r for r in dat}

        dD, dP, dR = byk(det), byk(pat), byk(rac)
        ctl_t = sorted([(num(r.get('host_t')), r) for r in ctl if num(r.get('host_t')) is not None],
                       key=lambda kv: kv[0])
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
        off_a = m_first                                   # 运动起点锚
        off_b = None if m_last is None else (m_last - vdur)   # 结尾锚(参考)
        offset = off_a
        if offset is None:
            offset = 0.0 if cap_dur is None else (cap_dur - vdur)

        def at(t):
            """返回 (tele_t, ctl行, detail行, path行, race行)"""
            tt = t + offset
            if not ctl_t or tt < ctl_t[0][0] - 0.25 or tt > ctl_t[-1][0] + 0.25:
                return tt, None, None, None, None
            cr = min(ctl_t, key=lambda kv: abs(kv[0] - tt))[1]
            key = (cr.get('control_seq'), cr.get('param_revision'))
            return tt, cr, dD.get(key), dP.get(key), dR.get(key)

        for fi, t, fr in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        # ---- 逐帧对齐表（30fps 全片）----
        COLS = ['frame', 'video_t', 'tele_t', 'status', 'servo_pwm', 'action_reason',
                'speed_now', 'speed_target', 'path_reason', 'command_applied', 'far_valid',
                'source', 'bend_code', 'candidate_pwm', 'final_pwm', 'geometry_reason',
                'fit_rejected', 'avoid_state', 'cone_candidates', 'road_error', 'avoid_offset',
                'near_ref_y', 'width_candidate', 'left_near_a', 'left_far_a',
                'speed_limit', 'effective_target', 'speed_reason', 'support_mm', 'outer_clearance']
        nmove = 0
        with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
            wr = csv.writer(fh); wr.writerow(COLS)
            for fi in range(total):
                t = fi / fps
                tt, cr, dr, pr, rr = at(t)
                if cr is None:
                    wr.writerow([fi, round(t, 3), round(tt, 3), 'out_of_range'] + [''] * (len(COLS) - 4))
                    continue
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

        # ---- 概览（带遥测）----
        ROWS, PER = 4, 20
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, t, fr = kept[int(i)]
            ch = int(CELL_W * h / w)
            img = cv2.resize(fr, (CELL_W, ch))
            tt, cr, dr, pr, rr = at(t)
            lab = ['f%04d v=%.2fs t=%.2fs' % (fi, t, tt)]
            if cr is None:
                lab.append('NO TELEMETRY'); col = (120, 170, 255)
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
                    col = (140, 140, 255)   # 车未在动 -> 偏红，便于一眼看出对齐是否落在静止段
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
        cv2.putText(hdr, '%s  %dx%d %.1ffps  %d frames %.2fs | capture %.3fs' %
                    (tag, w, h, fps, total, vdur, cap_dur or -1), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(hdr, 'offset_A(motion_start)=%s  offset_B(motion_end-vdur)=%s  used=A  |  moving frames %d/%d  |  motion runs %s' %
                    ('n/a' if off_a is None else '%+.2fs' % off_a,
                     'n/a' if off_b is None else '%+.2fs' % off_b, nmove, total,
                     ','.join('%.2f-%.2f' % r for r in m_runs)), (8, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 230, 200), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview_tele.jpg'), np.vstack([hdr, grid]))

        # ---- 曲线 ----
        if ctl_t:
            ts = [t for t, _ in ctl_t]
            pw = [num(r.get('servo_pwm')) for _, r in ctl_t]
            sp = [num(r.get('Speed_now')) for _, r in ctl_t]
            tg = [num(r.get('Speed_mubiao')) for _, r in ctl_t]
            rl = [num((dR.get((r.get('control_seq'), r.get('param_revision'))) or {}).get('speed_limit')) for _, r in ctl_t]
            geo = [num((dP.get((r.get('control_seq'), r.get('param_revision'))) or {}).get('geometry_reason')) for _, r in ctl_t]
            fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
            ax[0].plot(ts, pw, lw=1.2); ax[0].axhline(1445, ls='--', c='gray', lw=1)
            ax[0].axhline(1170, ls=':', c='r', lw=1); ax[0].axhline(1720, ls=':', c='r', lw=1)
            ax[0].set_ylabel('servo CCR'); ax[0].grid(alpha=.25)
            ax[1].plot(ts, sp, label='speed_now'); ax[1].plot(ts, tg, label='speed_target(effective)')
            ax[1].plot(ts, rl, label='speed_limit', ls='--'); ax[1].legend(fontsize=8)
            ax[1].set_ylabel('encoder units'); ax[1].grid(alpha=.25)
            ax[2].plot(ts, geo, lw=1.0, drawstyle='steps-post')
            ax[2].set_ylabel('geometry_reason'); ax[2].grid(alpha=.25)
            if off_a is not None:
                for a in (off_a, None if off_b is None else off_b):
                    if a is not None:
                        ax[0].axvspan(a, a + vdur, color='orange', alpha=0.12)
            ax[3].plot(ts, geo, alpha=0)  # 占位保持 4 行
            ax[3].set_xlabel('telemetry host_t (s)'); ax[3].grid(alpha=.25)
            fig.suptitle('%s   orange band = video coverage under offset A / B' % cap)
            fig.tight_layout(); fig.savefig(os.path.join(d, 'telemetry_plot.png'), dpi=110); plt.close(fig)

        cfg = {}
        for _, v2 in (meta.get('configs') or {}).items():
            cfg = v2; break
        summary.append(dict(tag=tag, capture=cap, video=vname, w=w, h=h, fps=round(fps, 2),
                            frames=total, video_duration=round(vdur, 3),
                            capture_duration=None if cap_dur is None else round(cap_dur, 3),
                            offset_A_motion_start=None if off_a is None else round(off_a, 3),
                            offset_B_motion_end=None if off_b is None else round(off_b, 3),
                            alignment_spread_s=None if (off_a is None or off_b is None) else round(abs(off_a - off_b), 3),
                            motion_runs=[(round(a, 2), round(b, 2)) for a, b in m_runs],
                            moving_frames=nmove, end_reason=meta.get('end_reason'),
                            ctl_rows=len(ctl), det_rows=len(det), path_rows=len(pat),
                            race_rows=len(rac), motor_rows=len(mot),
                            spd=cfg.get('spd'), bendspd=cfg.get('bendspd'),
                            pwm_min=min([x for x in pw if x is not None], default=None),
                            pwm_max=max([x for x in pw if x is not None], default=None),
                            spd_now_max=max([x for x in sp if x is not None], default=None)))
        print('%-20s %6.2fs  cap %6.3f  offA %+6.2f  offB %+6.2f  动帧 %3d/%3d  pwm %.0f~%.0f spd<=%.1f' %
              (tag, vdur, cap_dur or -1, off_a or 0, off_b if off_b is not None else 0,
               nmove, total, summary[-1]['pwm_min'] or 0, summary[-1]['pwm_max'] or 0,
               summary[-1]['spd_now_max'] or 0))

    json.dump(summary, open(os.path.join(OUT, 'summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
