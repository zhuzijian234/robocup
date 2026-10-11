# -*- coding: utf-8 -*-
"""本批 8 个视频的概览联络表 + 标记色框。

只标"画面里确实看得见"的：锥桶 / 车与管壁接触 / 弯道 / 人手人脚。
中文一律不进图（cv2.putText 的 Hershey 字体不支持），图内用 ASCII。
"""
import os
import cv2
import numpy as np

OUT = r'D:\OneDrive\Desktop\robocup\调试视频\视频解析_AI可读_20261010_二批'
CELL_W = 320
COL = {'cone': (0, 140, 255), 'touch': (0, 0, 255), 'curve': (255, 210, 0), 'human': (255, 255, 255)}
ASCII = {'cone': 'CONE', 'touch': 'CAR-AT-TUBE', 'curve': 'CURVE', 'human': 'HAND/FOOT'}

MARKS = {
 '01_212056_出S弯刮蹭_大弯迟钝卡连接处': {
   0:['human'], 195:['cone','curve'], 230:['touch'], 270:['touch'], 310:['touch'], 350:['touch'],
   385:['touch'], 425:['touch'], 465:['touch'], 505:['touch'], 540:['curve'], 580:['curve'],
   620:['human','curve'], 660:['curve'], 695:['curve'], 735:['curve']},
 '02_212730_大弯不及时_S弯入弯刮蹭_出弯改善': {
   0:['human'], 15:['curve'], 35:['curve'], 50:['curve'], 70:['curve'], 85:['curve'], 105:['curve'],
   120:['curve'], 140:['curve'], 155:['curve'], 175:['curve'], 190:['curve'], 210:['curve'],
   225:['curve'], 245:['touch'], 260:['curve'], 280:['curve'], 295:['curve'], 315:['touch'], 330:['touch']},
 '03_213339_加急大弯不及时_出弯冲撞': {
   0:['human'], 10:['curve'], 20:['curve'], 30:['curve'], 40:['curve'], 50:['curve'], 60:['curve'],
   70:['curve'], 80:['curve'], 90:['curve'], 100:['curve'], 110:['curve'], 120:['curve'],
   130:['curve'], 140:['curve'], 150:['curve'], 160:['curve'], 170:['touch','human'], 180:['curve'], 190:['curve']},
 '04_213609_大急弯拐不过_入弯不及时': {
   0:['human'], 45:['curve'], 65:['curve'], 90:['curve'], 110:['curve'], 130:['curve'],
   150:['curve'], 160:['curve'], 170:['curve']},
 '05_213856_放缓_进S弯舵机几乎没动撞入弯口': {
   0:['human'], 30:['curve'], 65:['curve'], 80:['curve'], 95:['curve'], 110:['curve'], 130:['curve'],
   145:['curve'], 160:['curve'], 175:['curve'], 195:['curve'], 210:['curve'], 225:['curve'],
   240:['curve'], 255:['touch'], 275:['human'], 290:['curve'], 305:['touch','human']},
 '06_214152_成功进S弯_出弯反摆不及时撞击': {
   0:['human'], 40:['curve'], 55:['curve'], 65:['curve'], 75:['curve'], 85:['curve'], 95:['curve'],
   105:['curve'], 115:['curve'], 125:['curve'], 135:['curve'], 140:['curve'], 150:['human','curve'],
   160:['curve'], 170:['touch','human'], 180:['touch']},
 '07_214444_S弯略急_右侧刮蹭': {
   0:['human'], 40:['curve'], 45:['curve'], 55:['curve'], 60:['curve'], 70:['curve'], 75:['curve'],
   85:['curve'], 90:['curve'], 100:['curve'], 105:['curve'], 115:['human'], 120:['human'],
   130:['touch'], 135:['touch'], 145:['touch']},
 '08_220019_加长变缓速度12_出弯略刮蹭': {
   0:['human'], 25:['curve'], 35:['curve'], 45:['curve'], 60:['curve'], 70:['curve'], 80:['curve'],
   95:['curve'], 105:['human'], 115:['curve'], 125:['curve'], 140:['curve'], 150:['curve'],
   160:['curve'], 175:['curve'], 185:['curve'], 195:['curve'], 210:['curve'], 220:['human']},
}


def imread_u(p):
    try:
        return cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def save(p, img, q=84):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def main():
    for tag, marks in MARKS.items():
        fdir = os.path.join(OUT, tag, 'frames')
        if not os.path.isdir(fdir):
            print('skip', tag); continue
        kept = [(int(f[1:6]), os.path.join(fdir, f)) for f in sorted(os.listdir(fdir))]
        COLS, ROWS = 5, 4
        PER = COLS * ROWS
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, p = kept[int(i)]
            fr = imread_u(p)
            if fr is None:
                print('  read fail', p); continue
            h, w = fr.shape[:2]
            ch = int(320 * h / w)
            img = cv2.resize(fr, (320, ch))
            mk = marks.get(fi, [])
            border = COL[mk[0]] if mk else (60, 60, 60)
            img = cv2.copyMakeBorder(img, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=border)
            img = cv2.resize(img, (320, ch))
            extra = '  '.join(ASCII[t] for t in mk)
            bar = np.zeros(((2 if extra else 1) * 20 + 6, 320, 3), np.uint8)
            cv2.putText(bar, 'f%04d  %.2fs' % (fi, fi / 30.0), (4, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if extra:
                cv2.putText(bar, extra[:40], (4, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            COL[mk[0]], 1, cv2.LINE_AA)
            cells.append(np.vstack([bar, img]))
        while len(cells) < PER:
            cells.append(np.zeros_like(cells[0]))
        ch = max(c.shape[0] for c in cells)
        cells = [c if c.shape[0] == ch else np.vstack([c, np.zeros((ch - c.shape[0], 320, 3), np.uint8)])
                 for c in cells]
        grid = np.vstack([np.hstack(cells[r * COLS:(r + 1) * COLS]) for r in range(ROWS)])
        hdr = np.zeros((34, grid.shape[1], 3), np.uint8)
        cv2.putText(hdr, 'MARKED  orange=CONE  red=CAR-AT-TUBE  cyan=CURVE  white=HAND/FOOT   (%s)' % tag[:2],
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        save(os.path.join(OUT, tag, 'sheets', 'overview_marked.jpg'), np.vstack([hdr, grid]))
        print('marked:', tag[:20], len(kept), 'frames')


if __name__ == '__main__':
    main()
