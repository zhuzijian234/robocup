# -*- coding: utf-8 -*-
"""在概览联络表上按帧号画标记。只标"画面里确实看得见"的东西。

标记只做三件事：给单元格加色框 + 在标签栏补一句可见事实。
不做任何推断（不写"转向不足""感知失败"这类结论）。
"""
import os
import cv2
import numpy as np

OUT = r'D:\OneDrive\Desktop\robocup\调试视频\视频解析_AI可读_20261010'
CELL_W = 320
COL = {'cone': (0, 140, 255), 'touch': (0, 0, 255), 'curve': (255, 210, 0), 'human': (255, 255, 255),
       'stop': (0, 0, 255)}
# cv2.putText 的 Hershey 字体不支持中文(会渲染成 ?)，图内一律用 ASCII；
# 中文事实写在同目录/上级 README.md 里。
ASCII = {'cone': 'CONE', 'touch': 'CAR-TOUCH-TUBE', 'curve': 'CURVE', 'human': 'HAND', 'stop': 'STOPPED'}

# 每个视频：帧号 -> [(类型, 文字)]   —— 只在"看得见"处标
MARKS = {
    '01_180345_大弯后刮蹭': {
        0: [('human', '人手在车旁')],
        15: [('cone', '红色锥桶')], 35: [('cone', '锥桶在通道内')], 50: [('cone', '锥桶在右侧')],
        70: [('cone', '锥桶在右后方')], 85: [('cone', '锥桶(画面左侧)')],
        105: [('curve', '管子成弯')], 120: [('curve', '弯道内')], 135: [('curve', '弯道内')],
        155: [('curve', '弯道内')], 170: [('curve', '弯道内')],
        190: [('touch', '左侧管壁紧贴车身')], 205: [('touch', '左侧管壁贴近')], 220: [('touch', '左侧管壁贴近')],
        240: [('touch', '左侧管壁贴近')],
        255: [('curve', '弯道内')], 275: [('curve', '弯道内')], 290: [('touch', '贴住管壁')],
        310: [('curve', '弯道内')], 325: [('cone', '锥桶(右上)')],
    },
    '02_181529_撞边界停止': {
        0: [('human', '人手在车旁')],
        15: [('cone', '红色锥桶(右)')], 30: [('cone', '锥桶在右前')], 45: [('cone', '锥桶在右')],
        60: [('cone', '锥桶在后方')], 80: [('cone', '锥桶(左上)')],
        95: [('curve', '前方管壁成弯')], 110: [('curve', '弯道内')], 125: [('curve', '弯道内')],
        140: [('curve', '弯道内')], 155: [('curve', '弯道内')], 170: [('touch', '车贴左侧管壁')],
        185: [('curve', '车头前方出现横向管壁')], 200: [('curve', '前方横向管壁')],
        215: [('touch', '车头接近横向管壁')],
        235: [('stop', '车头抵在管壁上')], 250: [('stop', '位置基本不变')],
        265: [('stop', '位置基本不变')], 280: [('stop', '静止')],
        295: [('stop', '静止; 人脚入镜')],
    },
    '03_181925_刮蹭': {
        0: [('human', '人手在车旁')],
        50: [('cone', '红色锥桶(右上)')], 80: [('cone', '锥桶(左下)')],
        100: [('curve', '管子成弯')], 115: [('curve', '弯道内')], 130: [('curve', '弯道内')],
        145: [('curve', '通道内')], 165: [('curve', '通道内')], 180: [('curve', '通道内')],
        195: [('curve', '管子自上方弯下')], 210: [('touch', '右侧管壁紧贴车身')],
        230: [('curve', '管子右侧弯曲')], 245: [('curve', '管子左侧弯过')],
        260: [('curve', '弯道内')], 275: [('curve', '弯道内')], 295: [('curve', '弯道内')],
        310: [('human', '人脚入镜')],
    },
    '04_182048_进S弯不及时刮蹭': {
        0: [('human', '人手在车旁')],
        15: [('cone', '红色锥桶(右上)')], 30: [('cone', '锥桶(右)')], 50: [('cone', '锥桶(右)')],
        65: [('cone', '锥桶在左后')], 80: [('cone', '锥桶(左下)')],
        95: [('curve', '弯道内')], 110: [('curve', '弯道内')], 130: [('curve', '弯道内')],
        145: [('curve', '弯道内')], 160: [('curve', '通道内')], 175: [('curve', '管子向右弯')],
        195: [('curve', '弯道内')],
        210: [('touch', '左侧管壁紧贴车身')], 225: [('touch', '左侧管壁贴近')],
        240: [('touch', '左侧管壁贴近')], 255: [('touch', '左侧管壁贴近')],
        275: [('curve', '前方管子弯过来')], 290: [('curve', '弯道内')],
        305: [('human', '弯道内; 人脚入镜')],
    },
    '05_赛道布局': {
        0: [('curve', '管段分叉/并行')], 60: [('curve', '并行段')], 120: [('curve', '并行/分叉')],
        165: [('curve', '大U形回头弯')], 225: [('curve', '大U形回头弯')], 285: [('curve', '大U形回头弯')],
        345: [('curve', 'S形弯')], 405: [('curve', 'S形弯')], 450: [('curve', 'S形弯')],
        510: [('curve', '弯道')], 570: [('curve', '直道段')], 630: [('curve', '直道段')],
        675: [('cone', '红色锥桶(通道内)')],
        735: [('cone', '红色锥桶(通道内)')],
        795: [('cone', '锥桶(通道内)')], 855: [('cone', '红色锥桶(通道内)')],
        915: [('cone', '锥桶(通道内)')], 960: [('cone', '锥桶(通道内)')],
        1020: [('curve', '弯道')], 1080: [('curve', '弯道')],
    },
}


def imread_u(path):
    """cv2.imread 在中文路径下会返回 None；用 fromfile+imdecode 绕过。"""
    try:
        return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def save(path, img, q=86):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(path)


def main():
    for d, marks in MARKS.items():
        fdir = os.path.join(OUT, d, 'frames')
        if not os.path.isdir(fdir):
            print('skip', d); continue
        files = sorted(os.listdir(fdir))
        kept = [(int(f[1:6]), os.path.join(fdir, f)) for f in files]
        # 概览：5x4 覆盖全段
        COLS, ROWS = 5, 4
        PER = COLS * ROWS
        idxs = np.linspace(0, len(kept) - 1, min(PER, len(kept))).round().astype(int)
        cells = []
        for i in idxs:
            fi, p = kept[int(i)]
            fr = imread_u(p)
            if fr is None:
                print('  读不到:', p); continue
            h, w = fr.shape[:2]
            ch = int(320 * h / w)
            img = cv2.resize(fr, (320, ch))
            mk = marks.get(fi, [])
            cols = [COL[t] for t, _ in mk]
            border = cols[0] if cols else (60, 60, 60)
            img = cv2.copyMakeBorder(img, 4, 4, 4, 4, cv2.BORDER_CONSTANT, value=border)
            img = cv2.resize(img, (320, ch))
            extra = '  '.join(ASCII.get(t, t) for t, _ in mk)
            nlab = 2 if extra else 1
            bar = np.zeros((20 * nlab + 6, 320, 3), np.uint8)
            cv2.putText(bar, 'f%04d  %.2fs' % (fi, fi / 30.0), (4, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if extra:
                cv2.putText(bar, extra[:38], (4, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            cols[0], 1, cv2.LINE_AA)
            cells.append(np.vstack([bar, img]))
        while len(cells) < PER:
            cells.append(np.zeros_like(cells[0]))
        ch = max(c.shape[0] for c in cells)
        cells = [c if c.shape[0] == ch else np.vstack([c, np.zeros((ch - c.shape[0], 320, 3), np.uint8)])
                 for c in cells]
        grid = np.vstack([np.hstack(cells[r * COLS:(r + 1) * COLS]) for r in range(ROWS)])
        hdr = np.zeros((34, grid.shape[1], 3), np.uint8)
        cv2.putText(hdr, 'MARKED overview  orange=CONE  red=CAR-TOUCH-TUBE / STOPPED  cyan=CURVE  white=HAND   (%s)' % d.split('_')[0],
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        save(os.path.join(OUT, d, 'sheets', 'overview_marked.jpg'), np.vstack([hdr, grid]))
        print('marked:', d, len(kept), 'frames kept')


if __name__ == '__main__':
    main()
