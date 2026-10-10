# -*- coding: utf-8 -*-
"""把 10/9 三次调试图拆成 AI 可读格式，并与遥测按绝对墙钟对齐。

对齐原理：
  遥测 side:  bin.meta.json 里 started_utc (绝对UTC) + CONTROL/motor 的 host_t
             -> 每一帧遥测都有绝对墙钟时间
  视频 side:  mp4 的 mtime = 录制停止时刻(手机录制结束才落盘)
             -> 视频第 f 帧的绝对墙钟 = mtime - (总帧数-1-f)/fps
  两边都有绝对时间 -> 直接映射；再用"运动信号互相关"求最佳微调量并报告。
  没有共同发车标记，所以对齐精度取决于 mtime 是否等于录制停止时刻，
  互相关结果与 0 偏移的差就是这条假设的检验。
"""
import csv, json, os, shutil, sys
from datetime import datetime, timezone, timedelta

import cv2
import numpy as np

ROOT = r'D:\OneDrive\Desktop\robocup'
LOG = os.path.join(ROOT, '2-LXY传承', 'LXY传承-m10p', '上位机', 'log')
VID = os.path.join(ROOT, '调试视频')
OUT = os.path.dirname(os.path.abspath(__file__))
CN = timezone(timedelta(hours=8))

RUNS = [
    dict(idx=1, label='第一次', video='第一次_面对s弯大湾道入口反应迟钝,几乎没转向径直撞向赛道后停止采集.mp4',
         cap='v2_1009_173009_4538453000000', raw='run_1009_173009_4538453000000.bin', dirname='run1_first'),
    dict(idx=2, label='第二次', video='第二次调试成功进入s弯大弯入口,面对s弯道的小s弯反应不够灵敏,剐蹭赛道边界,同时出弯道转弯不及时,力度不够,撞向边界后停止,注意,此次采集可能遥测数据有损.mp4',
         cap='v2_1009_173144_4633921000000', raw='run_1009_173143_4632265000000.bin', dirname='run2_second'),
    dict(idx=3, label='第三次', video='第三次_与第二次问题相同,同样遥测数据可能有损.mp4',
         cap='v2_1009_173619_4908953000000', raw='run_1009_173614_4903375000000.bin', dirname='run3_third'),
]


def rows(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def num(r, k, d=None):
    try:
        return float(r[k])
    except Exception:
        return d


def utc_of(iso):
    return datetime.fromisoformat(iso).astimezone(CN)


def motion_signal(video_path, every=1):
    """逐帧与前一帧的差分强度（灰度缩小后），用于找运动区间并做互相关。"""
    c = cv2.VideoCapture(video_path)
    fps = c.get(cv2.CAP_PROP_FPS) or 30.0
    sig, frames, prev = [], [], None
    i = 0
    while True:
        ok, fr = c.read()
        if not ok:
            break
        if i % every == 0:
            g = cv2.cvtColor(cv2.resize(fr, (180, 320)), cv2.COLOR_BGR2GRAY).astype(np.float32)
            sig.append(0.0 if prev is None else float(np.mean(np.abs(g - prev))))
            prev = g
            frames.append(i)
        i += 1
    total = i
    c.release()
    return fps, total, frames, np.array(sig)


def main():
    summary = []
    for r in RUNS:
        vpath = os.path.join(VID, r['video'])
        d = os.path.join(OUT, r['dirname'])
        os.makedirs(os.path.join(d, 'frames'), exist_ok=True)
        os.makedirs(os.path.join(d, 'sheets'), exist_ok=True)
        os.makedirs(os.path.join(d, 'telemetry'), exist_ok=True)

        meta_p = os.path.join(LOG, r['cap'] + '.bin.meta.json')
        meta = json.load(open(meta_p, encoding='utf-8')) if os.path.exists(meta_p) else {}
        start = utc_of(meta['started_utc']) if meta.get('started_utc') else None
        end = utc_of(meta['ended_utc']) if meta.get('ended_utc') else None

        ctl = rows(os.path.join(LOG, r['cap'] + '.csv'))
        det = rows(os.path.join(LOG, r['cap'] + '.detail.csv'))
        mot = rows(os.path.join(LOG, r['cap'] + '.motor.csv'))
        m10 = rows(os.path.join(LOG, r['cap'] + '.m10p.csv'))

        for f in [r['cap'] + '.csv', r['cap'] + '.detail.csv', r['cap'] + '.motor.csv',
                  r['cap'] + '.m10p.csv', r['cap'] + '.bin.meta.json', r['cap'] + '_report.md']:
            p = os.path.join(LOG, f)
            if os.path.exists(p):
                shutil.copy2(p, os.path.join(d, 'telemetry', f))
        for f in [r['raw'], r['raw'] + '.meta.json']:
            p = os.path.join(LOG, f)
            if os.path.exists(p):
                shutil.copy2(p, os.path.join(d, 'telemetry', f))

        fps, total, fidx, sig = motion_signal(vpath)
        mtime = datetime.fromtimestamp(os.path.getmtime(vpath), CN)
        video_end = mtime
        video_start = mtime - timedelta(seconds=(total - 1) / fps)

        # 遥测运动区间（编码器速度）。spd 给信号用，mot_t 给查表用。
        spd, mot_t = [], []
        for m in mot:
            t = num(m, 'host_t')
            if t is None:
                t = (num(m, 'sample_us', 0) or 0) / 1e6
            spd.append((t, num(m, 'speed_float', 0.0)))
            mot_t.append((t, m))
        moving = [(t, s) for t, s in spd if abs(s) > 0.5]
        tele_motion = (min(t for t, _ in moving), max(t for t, _ in moving)) if moving else (None, None)

        ctl_t = [(num(c, 'host_t'), c) for c in ctl if num(c, 'host_t') is not None]

        # 互相关：把视频运动曲线搬到 host_t 轴上，搜索最佳平移
        best_shift, best_corr = 0.0, -2.0
        corr_curve = []
        if len(sig) > 5 and len(spd) > 5:
            # 统一到 0.1s 栅格做互相关
            tv = np.arange(total) / fps
            tt = np.array([t for t, _ in spd], dtype=float)
            ss = np.array([abs(s) for _, s in spd], dtype=float)
            grid = np.arange(0, max(tv[-1], tt[-1] if len(tt) else 0) + 0.1, 0.1)
            hv = np.interp(grid, tv, sig / (sig.max() or 1))
            hv = hv - hv.mean()
            for sh in np.arange(-3.0, 3.01, 0.1):
                ht2 = np.interp(grid, tt + sh, ss / (ss.max() or 1)) if len(tt) > 1 else np.zeros_like(grid)
                ht2 = ht2 - ht2.mean()
                den = (np.linalg.norm(hv) * np.linalg.norm(ht2)) or 1
                cc = float(np.dot(hv, ht2) / den)
                corr_curve.append((round(float(sh), 2), cc))
                if cc > best_corr:
                    best_corr, best_shift = cc, float(sh)

        # 逐帧绝对时间 -> host_t：假定 mtime = 录制结束
        def frame_host_t(f):
            wall = video_start + timedelta(seconds=f / fps)
            if start is None:
                return None
            return (wall - start).total_seconds()

        # 取最接近的 CONTROL / motor 行
        def nearest(pairs, t):
            if not pairs or t is None:
                return None
            return min(pairs, key=lambda kv: abs(kv[0] - t))[1]

        # 导出全部帧
        c = cv2.VideoCapture(vpath)
        k = 0
        while True:
            ok, fr = c.read()
            if not ok:
                break
            cv2.imwrite(os.path.join(d, 'frames', 'f%05d.jpg' % k), fr, [cv2.IMWRITE_JPEG_QUALITY, 88])
            k += 1
        c.release()

        # 对齐表
        with open(os.path.join(d, 'aligned.csv'), 'w', newline='', encoding='utf-8-sig') as fh:
            w = csv.writer(fh)
            w.writerow(['frame', 'video_t', 'video_wall_cn', 'tele_host_t', 'tele_t_device',
                        'servo_pwm', 'action_reason', 'pid_select', 'valid_couter', 'speed',
                        'path_reason', 'command_applied', 'far_valid', 'source', 'final_pwm',
                        'candidate_pwm', 'width_source', 'bend_code', 'ref_x', 'ref_y', 'near_a', 'far_a'])
            for f in range(total):
                ht = frame_host_t(f)
                cr = nearest(ctl_t, ht)
                mr = nearest(mot_t, ht)
                dr = None
                if cr is not None:
                    cs = cr.get('control_seq')
                    dr = next((x for x in det if x.get('control_seq') == cs), None)
                w.writerow([
                    f, round(f / fps, 3),
                    (video_start + timedelta(seconds=f / fps)).strftime('%H:%M:%S.%f')[:-3],
                    None if ht is None else round(ht, 3),
                    None if cr is None else cr.get('t'),
                    None if cr is None else cr.get('servo_pwm'),
                    None if cr is None else cr.get('action_reason'),
                    None if cr is None else cr.get('pid_select'),
                    None if cr is None else cr.get('valid_couter'),
                    None if mr is None else mr.get('speed_float'),
                    None if dr is None else dr.get('path_reason'),
                    None if dr is None else dr.get('command_applied'),
                    None if dr is None else dr.get('far_valid'),
                    None if dr is None else dr.get('source'),
                    None if dr is None else dr.get('final_pwm'),
                    None if dr is None else dr.get('candidate_pwm'),
                    None if dr is None else dr.get('width_source'),
                    None if dr is None else dr.get('bend_code'),
                    None if dr is None else dr.get('near_x'),
                    None if dr is None else dr.get('ref_y'),
                    None if dr is None else dr.get('near_a'),
                    None if dr is None else dr.get('far_a'),
                ])

        # 联络表：4x4=16 帧/张，带烧入标注
        SHEET = 16
        nsheets = 0
        for s in range(0, total, SHEET):
            ids = list(range(s, min(s + SHEET, total)))
            cells = []
            for f in ids:
                img = cv2.imread(os.path.join(d, 'frames', 'f%05d.jpg' % f))
                if img is None:
                    img = np.zeros((320, 180, 3), np.uint8)
                img = cv2.resize(img, (300, 533))
                ht = frame_host_t(f)
                cr = nearest(ctl_t, ht)
                mr = nearest(mot_t, ht)
                dr = None
                if cr is not None:
                    dr = next((x for x in det if x.get('control_seq') == cr.get('control_seq')), None)
                lab = ['f%03d  v=%.2fs' % (f, f / fps)]
                if ht is not None and cr is not None:
                    lab.append('tel=%.2fs pwm=%s' % (ht, cr.get('servo_pwm')))
                    lab.append('a=%s r=%s vc=%s' % (cr.get('action_reason'),
                                                    None if dr is None else dr.get('path_reason'),
                                                    cr.get('valid_couter')))
                    lab.append('spd=%s' % (None if mr is None else mr.get('speed_float')))
                bar = np.zeros((len(lab) * 22 + 6, 300, 3), np.uint8)
                for j, tx in enumerate(lab):
                    cv2.putText(bar, tx, (4, 18 + j * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                (255, 255, 255) if j == 0 else (170, 230, 170), 1, cv2.LINE_AA)
                cells.append(np.vstack([bar, img]))
            while len(cells) < SHEET:
                cells.append(np.zeros_like(cells[0]))
            grid = np.vstack([np.hstack(cells[i:i + 4]) for i in range(0, SHEET, 4)])
            cv2.imwrite(os.path.join(d, 'sheets', 'sheet_%02d.jpg' % (s // SHEET)), grid,
                        [cv2.IMWRITE_JPEG_QUALITY, 82])
            nsheets += 1

        summary.append(dict(
            idx=r['idx'], label=r['label'], dir=r['dirname'], video=r['video'],
            fps=round(fps, 3), frames=total, duration=round((total - 1) / fps, 2),
            video_mtime=video_end.strftime('%Y-%m-%d %H:%M:%S'),
            video_span=[video_start.strftime('%H:%M:%S.%f')[:-3], video_end.strftime('%H:%M:%S.%f')[:-3]],
            cap_start=start.strftime('%H:%M:%S.%f')[:-3] if start else None,
            cap_end=end.strftime('%H:%M:%S.%f')[:-3] if end else None,
            cap_duration=meta.get('duration_s'), end_reason=meta.get('end_reason'),
            fw=(meta.get('info') or '')[:200],
            ctl_rows=len(ctl), detail_rows=len(det), motor_rows=len(mot), m10p_rows=len(m10),
            speed_min=None if not spd else round(min(s for _, s in spd), 3),
            speed_max=None if not spd else round(max(s for _, s in spd), 3),
            moving_samples=len(moving),
            tele_motion=[None if tele_motion[0] is None else round(tele_motion[0], 2),
                         None if tele_motion[1] is None else round(tele_motion[1], 2)],
            best_shift_s=round(best_shift, 2), best_corr=round(best_corr, 4),
            corr_at_zero=next((round(cc, 4) for sh, cc in corr_curve if abs(sh) < 1e-9), None),
            sheets=nsheets,
            telemetry_files=sorted(os.listdir(os.path.join(d, 'telemetry'))),
        ))
        print('run%d %s: %d frames, %d sheets, shift=%.2f corr=%.3f' %
              (r['idx'], r['label'], total, nsheets, best_shift, best_corr))

    json.dump(summary, open(os.path.join(OUT, 'alignment_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('done ->', OUT)


if __name__ == '__main__':
    main()
