# -*- coding: utf-8 -*-
"""把 调试视频 里的视频抽帧成 AI 可读格式（联络表 + 单帧）。

原则：只做抽帧和标注"帧号/时间"，不做遥测对齐、不做推断。
用 cv2.imencode + ndarray.tofile 写文件，避免 Windows 下中文路径写入失败。
"""
import os
import cv2
import numpy as np

VID = r'D:\OneDrive\Desktop\robocup\调试视频'
OUT = os.path.join(VID, '视频解析_AI可读_20261010')

VIDEOS = [
    dict(dir='01_180345_大弯后刮蹭', fps_pick=6.0,
         name='v2_1010_180345_92954359000000-小车对大弯道表现可以,但是出大弯道后对后面的大弯+小s弯道刮蹭到边界.mp4'),
    dict(dir='02_181529_撞边界停止', fps_pick=6.0,
         name='v2_1010_181529_93659046000000.m10p_大弯道过后对弯道接s弯,没有成功进入s弯,舵机没有摆动,径直撞向赛道边界后停止,疑似对垂直于前进方向赛道感知灵敏,导致冲到近点无法再做出控制.mp4'),
    dict(dir='03_181925_刮蹭', fps_pick=6.0,
         name='v2_1010_181925_93894812000000_和180345问题近似,刮蹭赛道边界.mp4'),
    dict(dir='04_182048_进S弯不及时刮蹭', fps_pick=6.0,
         name='v2_1010_182048_93978031000000_和181925问题近似,进入s弯后不能及时转向进入下一弯道导致刮蹭赛道边界成为普遍问题,!!!同时要记住真实赛道s弯每个弯道长度略长于当前赛道大概10-20cm左右!!!.mp4'),
    dict(dir='05_赛道布局', fps_pick=2.0, name='赛道布局.mp4'),
]

CELL_W = 320          # 联络表单元格宽（保持原比例）


def save(path, img, q=88):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(path)
    return ok


def main():
    os.makedirs(OUT, exist_ok=True)
    index = []
    for v in VIDEOS:
        src = os.path.join(VID, v['name'])
        if not os.path.exists(src):
            print('missing:', v['name'][:40]); continue
        d = os.path.join(OUT, v['dir'])
        os.makedirs(os.path.join(d, 'frames'), exist_ok=True)
        os.makedirs(os.path.join(d, 'sheets'), exist_ok=True)

        cap = cv2.VideoCapture(src)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        step = max(1, int(round(fps / v['fps_pick'])))

        kept = []          # (frame_index, t, image)
        k = 0
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            if k % step == 0:
                kept.append((k, k / fps, fr))
            k += 1
        cap.release()

        # 单帧落盘
        for (fi, t, fr) in kept:
            save(os.path.join(d, 'frames', 'f%05d.jpg' % fi), fr)

        # 概览联络表：5 列，覆盖整段
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
        ch = cells[0].shape[0]
        grid = np.vstack([np.hstack(cells[r * COLS:(r + 1) * COLS]) for r in range(ROWS)])
        hdr = np.zeros((40, grid.shape[1], 3), np.uint8)
        cv2.putText(hdr, 'overview  %s   %dx%d %.1ffps  %d frames %.2fs' %
                    (v['dir'], w, h, fps, total, total / fps), (8, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 1, cv2.LINE_AA)
        save(os.path.join(d, 'sheets', 'overview.jpg'), np.vstack([hdr, grid]), 84)

        index.append((v['dir'], v['name'], w, h, fps, total, round(total / fps, 2),
                      step, len(kept), round(1.0 / v['fps_pick'], 3)))
        print('%-26s %4dx%-4d %d frames %6.2fs -> kept %3d (每 %d 帧取 1)' %
              (v['dir'], w, h, total, total / fps, len(kept), step))

    print('\n输出目录:', OUT)
    print('dir | 源文件 | WxH | fps | 总帧 | 时长 | 抽帧步长 | 保留帧数')


if __name__ == '__main__':
    main()
