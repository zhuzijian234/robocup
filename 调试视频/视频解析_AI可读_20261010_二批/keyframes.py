# -*- coding: utf-8 -*-
"""把每个视频的"关键窗口"按原始 30fps 全抽，并每 24 帧（0.8s）拼一张密表。

窗口按 §标记 的关键处选取（撞击/刮蹭/大弯/锥桶 前后）。
只抽帧与标帧号时间，不做推断、不做遥测对齐。
"""
import os
import cv2
import numpy as np

VID = r'D:\OneDrive\Desktop\robocup\调试视频'
OUT = os.path.join(VID, '视频解析_AI可读_20261010_二批')
CELL_W = 260
PER = 24          # 6 列 x 4 行 = 0.8 秒 @30fps

# tag -> (文件名, 窗口起帧, 窗口止帧(含), 窗口说明)
WINDOWS = {
 '01_212056_出S弯刮蹭_大弯迟钝卡连接处':
   ('v2_1010_212056_104784296000000_小车s弯表现提高,虽然在出s弯时还是有刮蹭,这次的问题是小车在面对大弯时反应略微有些迟钝,转弯力度差不多,但是因为反应不够快所以撞向赛道连接处卡住,这次遥测结束时间比视频慢了两三秒,不过这两三秒都是卡住后的时间.mp4',
    180, 280, 'cone_at_195__contact_at_230'),
 '02_212730_大弯不及时_S弯入弯刮蹭_出弯改善':
   ('v2_1010_212730_105177968000000_这次同样有大弯道反应不及时,刮蹭赛道边界,s弯道还是出现入弯时刮蹭边界,转弯不及时,出弯改善了挺多.mp4',
    230, 332, 'narrow_gap_245__rub_315'),
 '03_213339_加急大弯不及时_出弯冲撞':
   ('v2_1010_213339_105547062000000_面对加急大弯(比真是赛道急一点)转弯反应不及时,出弯后冲撞赛道边界.mp4',
    140, 191, 'impact_170'),
 '04_213609_大急弯拐不过_入弯不及时':
   ('v2_1010_213609_105696843000000_在和真实赛道接近的大急弯中仍然无法完全拐过,主要原因是入弯时转向反应不及时.mp4',
    140, 174, 'end_of_run'),
 '05_213856_放缓_进S弯舵机几乎没动撞入弯口':
   ('v2_1010_213856_105863968000000_这次弯道放缓一点,但是进入s弯时舵机几乎没动,撞向s弯入弯口赛道边界.mp4',
    230, 306, 'wall_ahead_255__stop'),
 '06_214152_成功进S弯_出弯反摆不及时撞击':
   ('v2_1010_214152_106040062000000_这次成功进入s弯,刮蹭一下赛道边界后出弯时面对倾向于垂直前进方向的赛道边界反摆舵机不及时,撞击赛道边界.mp4',
    130, 184, 'impact_170'),
 '07_214444_S弯略急_右侧刮蹭':
   ('v2_1010_214444_106212328000000_s弯比真实赛道略急,进入后刮蹭小车右侧赛道边界后到出弯口停止记录.mp4',
    90, 149, 'human_115__closeup'),
 '08_220019_加长变缓速度12_出弯略刮蹭':
   ('v2_1010_220019_107147375000000_弯道后s弯(加长变缓,速度12,接近真实赛道),出弯时略微刮蹭小车右侧赛道边界.mp4',
    170, 223, 'exit_scrape'),
}


def save(p, img, q=86):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


def main():
    for tag, (name, f0, f1, note) in WINDOWS.items():
        src = os.path.join(VID, name)
        if not os.path.exists(src):
            print('missing', tag); continue
        d = os.path.join(OUT, tag, 'keyframes')
        os.makedirs(d, exist_ok=True)
        cap = cv2.VideoCapture(src)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ch = int(CELL_W * h / w)

        frames = []
        k = 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if f0 <= k <= f1:
                save(os.path.join(d, 'f%05d.jpg' % k), fr)
                frames.append((k, fr))
            k += 1
        cap.release()

        nsheet = 0
        for s in range(0, len(frames), PER):
            chunk = frames[s:s + PER]
            cells = []
            for fi, fr in chunk:
                img = cv2.resize(fr, (CELL_W, ch))
                bar = np.zeros((20, CELL_W, 3), np.uint8)
                cv2.putText(bar, 'f%04d %.2fs' % (fi, fi / fps), (3, 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
                cells.append(np.vstack([bar, img]))
            while len(cells) < PER:
                cells.append(np.zeros_like(cells[0]))
            grid = np.vstack([np.hstack(cells[r * 6:(r + 1) * 6]) for r in range(4)])
            hdr = np.zeros((32, grid.shape[1], 3), np.uint8)
            cv2.putText(hdr, 'DENSE 30fps  %s   f%04d-f%04d  (%.2f-%.2f s)   %s' %
                        (tag[:2], chunk[0][0], chunk[-1][0], chunk[0][0] / fps, chunk[-1][0] / fps, note),
                        (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            save(os.path.join(OUT, tag, 'sheets', 'dense_%02d.jpg' % nsheet),
                 np.vstack([hdr, grid]), 82)
            nsheet += 1
        print('%-44s window f%04d-f%04d  %3d frames  %d sheets  (%s)' %
              (tag[:24], f0, f1, len(frames), nsheet, note))
    print('\n输出:', OUT)


if __name__ == '__main__':
    main()
