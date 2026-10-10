# -*- coding: utf-8 -*-
"""修正版对齐：不再用 mp4 的 mtime（那更像拷贝时间）。

锚点假设：录制与采集都在"撞停"时结束 -> 视频结尾对齐采集结尾
          offset = cap_duration - video_duration,  tele_t = video_t + offset
交叉验证：视频逐帧差分信号 × 遥测编码器速度  的归一化互相关，扫描 offset。
数据说明：run1/run3 的 motor 通道轮速恒为 0（遥测有损），互相关不可用，
          只以 CONTROL 通道(servo_pwm/action_reason/valid_couter)提供逐帧固件状态。
"""
import csv, json, os
import cv2
import numpy as np

ROOT = r'D:\OneDrive\Desktop\robocup'
UP = os.path.join(ROOT, '2-LXY传承', 'LXY传承-m10p', '上位机')
LOG = os.path.join(UP, 'log')
DEC = os.path.join(ROOT, 'video_ai_20261009', 'decoded')
VID = os.path.join(ROOT, '调试视频')
OUT = os.path.dirname(os.path.abspath(__file__))

RUNS = [
    dict(idx=1, label='第一次', dirname='run1_first',
         video='第一次_面对s弯大湾道入口反应迟钝,几乎没转向径直撞向赛道后停止采集.mp4',
         tele_dir=LOG, tele='v2_1009_173009_4538453000000', cap_dur=31.98399999999947),
    dict(idx=2, label='第二次', dirname='run2_second',
         video='第二次调试成功进入s弯大弯入口,面对s弯道的小s弯反应不够灵敏,剐蹭赛道边界,同时出弯道转弯不及时,力度不够,撞向边界后停止,注意,此次采集可能遥测数据有损.mp4',
         tele_dir=DEC, tele='run_1009_173143_4632265000000', cap_dur=7.125),
    dict(idx=3, label='第三次', dirname='run3_third',
         video='第三次_与第二次问题相同,同样遥测数据可能有损.mp4',
         tele_dir=DEC, tele='run_1009_173614_4903375000000', cap_dur=7.782),
]

FIELDS = ['servo_pwm', 'action_reason', 'pid_select', 'valid_couter',
          'path_reason', 'command_applied', 'far_valid', 'source', 'final_pwm',
          'candidate_pwm', 'width_source', 'bend_code', 'near_x', 'ref_y', 'near_a', 'far_a']


def rows(p):
    if not os.path.exists(p):
        return []
    with open(p, encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    try:
        return float(r[k])
    except Exception:
        return None


def main():
    report = []
    for r in RUNS:
        d = os.path.join(OUT, r['dirname'])
        ctl = rows(os.path.join(r['tele_dir'], r['tele'] + '.csv'))
        det = rows(os.path.join(r['tele_dir'], r['tele'] + '.detail.csv'))
        mot = rows(os.path.join(r['tele_dir'], r['tele'] + '.motor.csv'))

        def axis(dat):
            out = []
            for x in dat:
                t = fnum(x, 't')
                if t is None:
                    t = fnum(x, 'host_t')
                if t is None:
                    t = (fnum(x, 'sample_us') or 0) / 1e6
                out.append((t, x))
            return sorted(out, key=lambda kv: (kv[0] is None, kv[0]))

        ctl_t, mot_t = axis(ctl), axis(mot)
        ctl_span = (ctl_t[0][0], ctl_t[-1][0]) if ctl_t else (None, None)

        # 视频：帧数 + 运动信号
        vpath = os.path.join(VID, r['video'])
        c = cv2.VideoCapture(vpath)
        fps = c.get(cv2.CAP_PROP_FPS) or 30.0
        sig, prev, nf = [], None, 0
        while True:
            ok, fr = c.read()
            if not ok:
                break
            g = cv2.cvtColor(cv2.resize(fr, (180, 320)), cv2.COLOR_BGR2GRAY).astype(np.float32)
            sig.append(0.0 if prev is None else float(np.mean(np.abs(g - prev))))
            prev = g
            nf += 1
        c.release()
        sig = np.array(sig)
        vdur = (nf - 1) / fps

        # 主锚点：结尾对齐
        off_end = r['cap_dur'] - vdur

        # 交叉验证：运动互相关
        tv = np.arange(nf) / fps
        tt = np.array([t for t, _ in mot_t], dtype=float)
        ss = np.array([abs(fnum(x, 'speed_float') or 0.0) for _, x in mot_t], dtype=float)
        have_speed = len(tt) > 3 and ss.max() > 0.2
        off_best, cc_best, curve = None, None, {}
        if have_speed:
            grid = np.arange(0, max(tv[-1], tt[-1]) + 0.1, 0.05)
            hv = np.interp(grid, tv, sig / (sig.max() or 1))
            hv -= hv.mean()
            for o in np.arange(-1.5, 1.51, 0.05):
                ht = np.interp(grid, tt + o, ss / ss.max())
                ht -= ht.mean()
                den = (np.linalg.norm(hv) * np.linalg.norm(ht)) or 1
                cc = float(np.dot(hv, ht) / den)
                curve[round(float(o), 2)] = cc
                if cc_best is None or cc > cc_best:
                    cc_best, off_best = cc, float(o)

        off = off_end  # 采用主锚点

        def tele_at(t):
            if not ctl_t or t < ctl_span[0] - 0.15 or t > ctl_span[1] + 0.15:
                return None, None
            cr = min(ctl_t, key=lambda kv: abs(kv[0] - t))[1]
            dr = None
            cs = cr.get('control_seq')
            if cs is not None:
                dr = next((x for x in det if x.get('control_seq') == cs), None)
            return cr, dr

        # 每帧遥测行
        per = []
        for f in range(nf):
            vt = f / fps
            cr, dr = tele_at(vt + off)
            per.append((f, vt, cr, dr))

        nok = sum(1 for _, _, cr, _ in per if cr is not None)
        with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
            w = csv.writer(fh)
            w.writerow(['frame', 'video_t', 'tele_t', 'status'] + FIELDS)
            for f, vt, cr, dr in per:
                if cr is None:
                    w.writerow([f, round(vt, 3), round(vt + off, 3), 'out_of_range'] + [''] * len(FIELDS))
                    continue
                vals = []
                for k in FIELDS:
                    v = dr.get(k) if (dr is not None and k in dr) else (cr.get(k) if k in cr else None)
                    vals.append('' if v is None else v)
                w.writerow([f, round(vt, 3), round(vt + off, 3), 'ok'] + vals)

        # 重建联络表
        SHEET = 16
        for s in range(0, nf, SHEET):
            ids = list(range(s, min(s + SHEET, nf)))
            cells = []
            for f, vt, cr, dr in per[s:s + SHEET]:
                img = cv2.imread(os.path.join(d, 'frames', 'f%05d.jpg' % f))
                if img is None:
                    img = np.zeros((533, 300, 3), np.uint8)
                img = cv2.resize(img, (300, 533))
                if cr is None:
                    lab = ['f%03d v=%.2fs  tele_t=%.2fs' % (f, vt, vt + off), 'NO TELEMETRY IN RANGE']
                    col = (120, 170, 255)
                else:
                    rr = dr.get('path_reason') if dr else None
                    lab = ['f%03d v=%.2fs  tele_t=%.2fs' % (f, vt, vt + off),
                           'pwm=%s  act=%s  vc=%s' % (cr.get('servo_pwm'), cr.get('action_reason'),
                                                      cr.get('valid_couter')),
                           'reason=%s  src=%s  far=%s  bend=%s' % (rr, (dr or {}).get('source'),
                                                                   (dr or {}).get('far_valid'),
                                                                   (dr or {}).get('bend_code')),
                           'cand=%s  ref_y=%s  near_a=%s' % ((dr or {}).get('candidate_pwm'),
                                                             (dr or {}).get('ref_y'),
                                                             (dr or {}).get('near_a'))]
                    col = (170, 230, 170)
                bar = np.zeros((len(lab) * 21 + 6, 300, 3), np.uint8)
                for j, tx in enumerate(lab):
                    cv2.putText(bar, tx, (4, 17 + j * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                                (255, 255, 255) if j == 0 else col, 1, cv2.LINE_AA)
                cells.append(np.vstack([bar, img]))
            hmax = max(c.shape[0] for c in cells)
            cells = [c if c.shape[0] == hmax else np.vstack([c, np.zeros((hmax - c.shape[0], 300, 3), np.uint8)])
                     for c in cells]
            while len(cells) < SHEET:
                cells.append(np.zeros_like(cells[0]))
            grid = np.vstack([np.hstack(cells[i:i + 4]) for i in range(0, SHEET, 4)])
            hdr = np.zeros((46, grid.shape[1], 3), np.uint8)
            cv2.putText(hdr, 'run%d %s  frames %d-%d  offset=%.2fs (end-aligned)%s' %
                        (r['idx'], r['label'], ids[0], ids[-1], off,
                         '' if not have_speed else '  xcorr best=%.2fs(cc=%.2f)' % (off_best, cc_best)),
                        (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imwrite(os.path.join(d, 'sheets', 'sheet_%02d.jpg' % (s // SHEET)),
                        np.vstack([hdr, grid]), [cv2.IMWRITE_JPEG_QUALITY, 84])

        report.append(dict(idx=r['idx'], label=r['label'], dir=r['dirname'],
                           telemetry_source=os.path.join(r['tele_dir'], r['tele'] + '.csv'),
                           video_frames=nf, fps=round(fps, 3), video_duration=round(vdur, 3),
                           cap_duration=r['cap_dur'], offset_end_aligned=round(off_end, 3),
                           delay_after_reset_video_t0=round(off_end, 3),
                           xcorr_available=bool(have_speed),
                           xcorr_best_offset=None if off_best is None else round(off_best, 2),
                           xcorr_best_cc=None if cc_best is None else round(cc_best, 4),
                           xcorr_agreement_s=None if off_best is None else round(off_end - off_best, 3),
                           frames_with_telemetry=nok, frames_without=nf - nok,
                           ctl_rows=len(ctl), det_rows=len(det), motor_rows=len(mot),
                           speed_max=round(float(ss.max()), 3) if len(ss) else None))
        print('run%d %s: off=%.2f  xcorr=%s  frames_ok=%d/%d' %
              (r['idx'], r['label'], off_end,
               'n/a' if off_best is None else '%.2f(cc=%.2f)' % (off_best, cc_best), nok, nf))

    json.dump(report, open(os.path.join(OUT, 'alignment_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('done')


if __name__ == '__main__':
    main()
