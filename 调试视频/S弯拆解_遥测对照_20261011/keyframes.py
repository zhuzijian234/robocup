# -*- coding: utf-8 -*-
"""S 弯动作窗口按 30fps 全抽 + 每 24 帧(0.8s) 一张带遥测标注的密表。

窗口**由遥测决定**（不靠眼看）：
  action window = 首个到末个 |servo_pwm - 1445| > 150 的控制帧
  再换算成视频帧号（用 extract.py 的 offset_A 运动锚点）。
"""
import csv, json, os
import cv2
import numpy as np

VID = r'D:\OneDrive\Desktop\robocup\调试视频'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'
OUT = os.path.join(VID, 'S弯拆解_遥测对照_20261011')
CELL_W = 250
PER = 24
DEV = 150.0
MID = 1445.0

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


def save(p, img, q=82):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def align_rows(path, cap):
    """读 extract.py 产出的 aligned.csv，返回 {frame: row}"""
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            out[int(r['frame'])] = r
    return out


def main():
    sm = {r['tag']: r for r in json.load(open(os.path.join(OUT, 'summary.json'), encoding='utf-8'))}
    for tag, vname, cap in RUNS:
        src = os.path.join(VID, vname)
        d = os.path.join(OUT, tag)
        if not (os.path.exists(src) and os.path.isdir(d)):
            print('skip', tag); continue
        S = sm.get(tag)
        if not S:
            print('no summary for', tag); continue
        offset, fps, vdur = S['offset_A_motion_start'], S['fps'], S['video_duration']

        # 动作窗口：由 CONTROL 的 servo_pwm 决定
        ctl = rows(os.path.join(LOG, cap + '.csv'))
        hot = [num(r.get('host_t')) for r in ctl
               if num(r.get('servo_pwm')) is not None and abs(num(r.get('servo_pwm')) - MID) > DEV]
        if not hot:
            print('%-20s 无 |偏差|>150 的控制帧，跳过' % tag); continue
        t0, t1 = min(hot), max(hot)
        f0 = max(0, int((t0 - offset) * fps))
        f1 = min(S['frames'] - 1, int(np.ceil((t1 - offset) * fps)))
        # 前後各留 0.3s
        f0 = max(0, f0 - int(0.3 * fps))
        f1 = min(S['frames'] - 1, f1 + int(0.3 * fps))

        alp = align_rows(os.path.join(d, 'aligned.csv'), cap)
        kd = os.path.join(d, 'keyframes')
        os.makedirs(kd, exist_ok=True)
        c = cv2.VideoCapture(src)
        w, h = int(c.get(cv2.CAP_PROP_FRAME_WIDTH)), int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ch = int(CELL_W * h / w)
        frames, k = [], 0
        while True:
            ok, fr = c.read()
            if not ok:
                break
            if f0 <= k <= f1:
                save(os.path.join(kd, 'f%05d.jpg' % k), fr)
                frames.append((k, fr))
            k += 1
        c.release()

        nsh = 0
        for s in range(0, len(frames), PER):
            chunk = frames[s:s + PER]
            cells = []
            for fi, fr in chunk:
                img = cv2.resize(fr, (CELL_W, ch))
                r = alp.get(fi, {})
                lab = ['f%04d v=%.2fs t=%s' % (fi, fi / fps, r.get('tele_t', '?'))]
                lab.append('pwm=%s spd=%s/%s' % (r.get('servo_pwm', '?'), r.get('speed_now', '?'),
                                                 r.get('speed_target', '?')))
                lab.append('path=%s far=%s src=%s bend=%s' % (r.get('path_reason', '?'), r.get('far_valid', '?'),
                                                              r.get('source', '?'), r.get('bend_code', '?')))
                lab.append('geo=%s fit=%s adv=%s' % (r.get('geometry_reason', '?'), r.get('fit_rejected', '?'),
                                                     r.get('avoid_state', '?')))
                bar = np.zeros((len(lab) * 17 + 5, CELL_W, 3), np.uint8)
                for j, tx in enumerate(lab):
                    col = (255, 255, 255) if j == 0 else (170, 230, 170)
                    try:
                        if j == 1 and abs(float(r.get('servo_pwm')) - MID) > DEV:
                            col = (80, 200, 255)
                    except Exception:
                        pass
                    cv2.putText(bar, tx[:42], (3, 13 + j * 17), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                                col, 1, cv2.LINE_AA)
                cells.append(np.vstack([bar, img]))
            while len(cells) < PER:
                cells.append(np.zeros_like(cells[0]))
            hh = max(x.shape[0] for x in cells)
            cells = [x if x.shape[0] == hh else np.vstack([x, np.zeros((hh - x.shape[0], CELL_W, 3), np.uint8)])
                     for x in cells]
            grid = np.vstack([np.hstack(cells[r * 6:(r + 1) * 6]) for r in range(4)])
            hdr = np.zeros((30, grid.shape[1], 3), np.uint8)
            cv2.putText(hdr, 'DENSE 30fps %s  f%04d-f%04d (%.2f-%.2f s)  offset_A=%+.2fs  |  |PWM-1445|>150 帧标黄' %
                        (tag[:9], chunk[0][0], chunk[-1][0], chunk[0][0] / fps, chunk[-1][0] / fps, offset),
                        (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            save(os.path.join(d, 'sheets', 'dense_%02d.jpg' % nsh), np.vstack([hdr, grid]))
            nsh += 1
        print('%-20s 动作窗口 t=%.2f~%.2f s -> 帧 %d~%d  %3d 帧  %d 张密表' %
              (tag, t0, t1, f0, f1, len(frames), nsh))


if __name__ == '__main__':
    main()
