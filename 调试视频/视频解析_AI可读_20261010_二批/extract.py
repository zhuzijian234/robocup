# -*- coding: utf-8 -*-
"""第二批（10-10 21:20~22:01，8 个视频）抽帧 + 概览联络表。

只做抽帧与"帧号/时间"标注，不做遥测对齐、不做推断。
Windows 中文路径：读 numpy.fromfile+imdecode、写 imencode+tofile。
"""
import os
import cv2
import numpy as np

VID = r'D:\OneDrive\Desktop\robocup\调试视频'
OUT = os.path.join(VID, '视频解析_AI可读_20261010_二批')
FPS_PICK = 6.0
CELL_W = 320

VIDEOS = [
    ('01_212056_出S弯刮蹭_大弯迟钝卡连接处',
     'v2_1010_212056_104784296000000_小车s弯表现提高,虽然在出s弯时还是有刮蹭,这次的问题是小车在面对大弯时反应略微有些迟钝,转弯力度差不多,但是因为反应不够快所以撞向赛道连接处卡住,这次遥测结束时间比视频慢了两三秒,不过这两三秒都是卡住后的时间.mp4'),
    ('02_212730_大弯不及时_S弯入弯刮蹭_出弯改善',
     'v2_1010_212730_105177968000000_这次同样有大弯道反应不及时,刮蹭赛道边界,s弯道还是出现入弯时刮蹭边界,转弯不及时,出弯改善了挺多.mp4'),
    ('03_213339_加急大弯不及时_出弯冲撞',
     'v2_1010_213339_105547062000000_面对加急大弯(比真是赛道急一点)转弯反应不及时,出弯后冲撞赛道边界.mp4'),
    ('04_213609_大急弯拐不过_入弯不及时',
     'v2_1010_213609_105696843000000_在和真实赛道接近的大急弯中仍然无法完全拐过,主要原因是入弯时转向反应不及时.mp4'),
    ('05_213856_放缓_进S弯舵机几乎没动撞入弯口',
     'v2_1010_213856_105863968000000_这次弯道放缓一点,但是进入s弯时舵机几乎没动,撞向s弯入弯口赛道边界.mp4'),
    ('06_214152_成功进S弯_出弯反摆不及时撞击',
     'v2_1010_214152_106040062000000_这次成功进入s弯,刮蹭一下赛道边界后出弯时面对倾向于垂直前进方向的赛道边界反摆舵机不及时,撞击赛道边界.mp4'),
    ('07_214444_S弯略急_右侧刮蹭',
     'v2_1010_214444_106212328000000_s弯比真实赛道略急,进入后刮蹭小车右侧赛道边界后到出弯口停止记录.mp4'),
    ('08_220019_加长变缓速度12_出弯略刮蹭',
     'v2_1010_220019_107147375000000_弯道后s弯(加长变缓,速度12,接近真实赛道),出弯时略微刮蹭小车右侧赛道边界.mp4'),
]


def save(path, img, q=86):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(path)
    return ok


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for tag, name in VIDEOS:
        src = os.path.join(VID, name)
        if not os.path.exists(src):
            print('missing:', name[:50]); continue
        d = os.path.join(OUT, tag)
        os.makedirs(os.path.join(d, 'frames'), exist_ok=True)
        os.makedirs(os.path.join(d, 'sheets'), exist_ok=True)

        cap = cv2.VideoCapture(src)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        step = max(1, int(round(fps / FPS_PICK)))
        kept, k = [], 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if k % step == 0:
                kept.append((k, k / fps, fr))
            k += 1
        cap.release()

        for fi, t, fr in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        COLS, ROWS = 5, 4
        PER = COLS * ROWS
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, t, fr = kept[int(i)]
            ch = int(CELL_W * h / w)
            img = cv2.resize(fr, (CELL_W, ch))
            bar = np.zeros((24, CELL_W, 3), np.uint8)
            cv2.putText(bar, 'f%04d  %.2fs' % (fi, t), (4, 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cells.append(np.vstack([bar, img]))
        while len(cells) < PER:
            cells.append(np.zeros_like(cells[0]))
        ch = max(c.shape[0] for c in cells)
        cells = [c if c.shape[0] == ch else np.vstack([c, np.zeros((ch - c.shape[0], CELL_W, 3), np.uint8)])
                 for c in cells]
        grid = np.vstack([np.hstack(cells[r * COLS:(r + 1) * COLS]) for r in range(ROWS)])
        hdr = np.zeros((40, grid.shape[1], 3), np.uint8)
        cv2.putText(hdr, 'overview  %dx%d  %.1ffps  %d frames  %.2fs   (%s)' %
                    (w, h, fps, total, total / fps, tag), (8, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview.jpg'), np.vstack([hdr, grid]))
        rows.append((tag, w, h, total, round(total / fps, 2), len(kept), step))
        print('%-46s %4dx%-4d %5d帧 %6.2fs -> kept %3d' % (tag, w, h, total, total / fps, len(kept)))
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
