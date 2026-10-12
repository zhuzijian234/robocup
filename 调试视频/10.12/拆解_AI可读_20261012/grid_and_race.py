# -*- coding: utf-8 -*-
"""1) 给演示视频的关键帧加"像素网格"，供人工/我读坐标（标尺=已知通道宽500mm）。
   2) 汇总 6 次实跑里 race.csv 报的"到弯距离/弯半径/外壁余量"，作为参照。"""
import csv, json, os
import cv2
import numpy as np

BASE = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\拆解_AI可读_20261012'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'

# 演示视频：导出这些原片帧并加网格
GRID = {
    'DEMO_左转弯_人工推车成功过弯': [40, 80, 110, 120, 130, 140, 150, 165],
    'DEMO_右转弯_人工推车成功过弯': [40, 60, 80, 88, 95, 105, 120, 130],
}
CAPS = ['v2_1012_160711_808515000000', 'v2_1012_161100_1037562000000',
        'v2_1012_161233_1130140000000', 'v2_1012_161426_1243328000000',
        'v2_1012_161746_1443265000000', 'v2_1012_162029_1606890000000']
FIELDS = ['turn_distance_mm', 'bend_rad', 'support_mm', 'outer_clearance', 'outer_offset',
          'speed_limit', 'effective_target', 'speed_reason', 's_preview', 'curve_rejected',
          'heading_rad', 'preview_heading_rad']


def save(p, img, q=92):
    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
    if ok:
        buf.tofile(p)


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


def grid_overlay(img, step=50):
    h, w = img.shape[:2]
    out = img.copy()
    for x in range(0, w, step):
        c = (0, 0, 255) if x % 200 == 0 else (0, 200, 255)
        cv2.line(out, (x, 0), (x, h), c, 1, cv2.LINE_AA)
        if x % 100 == 0:
            cv2.putText(out, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
    for y in range(0, h, step):
        c = (0, 0, 255) if y % 200 == 0 else (0, 200, 255)
        cv2.line(out, (0, y), (w, y), c, 1, cv2.LINE_AA)
        if y % 100 == 0:
            cv2.putText(out, str(y), (2, y + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
    return out


def main():
    # ---- 1) 演示视频加网格 ----
    for tag, frames in GRID.items():
        kd = os.path.join(BASE, tag, 'keyframes')
        gd = os.path.join(BASE, tag, 'grid')
        os.makedirs(gd, exist_ok=True)
        n = 0
        for f in frames:
            p = os.path.join(kd, 'f%05d.jpg' % f)
            if not os.path.exists(p):
                print('  缺帧', tag, f); continue
            img = cv2.imdecode(np.fromfile(p, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            save(os.path.join(gd, 'grid_f%05d.jpg' % f), grid_overlay(img))
            n += 1
        print('%-38s 网格图 %d 张 -> grid\\' % (tag, n))

    # ---- 2) 实跑遥测：到弯距离等 ----
    print('\n=== 6 次实跑的 race.csv 关键量（激光雷达量出来的，单位 mm/rad）===')
    print('%-24s %-14s %8s %8s %8s %8s %8s' % ('capture', 'field', 'min', 'median', 'max', 'n', 'nonzero'))
    out = {}
    for cap in CAPS:
        dat = rows(os.path.join(LOG, cap + '.race.csv'))
        if not dat:
            print('  %s 无 race.csv' % cap); continue
        out[cap] = {}
        for f in FIELDS:
            v = [num(r.get(f)) for r in dat]
            v = [x for x in v if x is not None]
            if not v:
                continue
            sv = sorted(v)
            med = sv[len(sv) // 2]
            out[cap][f] = dict(min=min(v), median=med, max=max(v), n=len(v),
                               nonzero=sum(1 for x in v if abs(x) > 1e-9))
            print('%-24s %-14s %8.1f %8.1f %8.1f %8d %8d' %
                  (cap[-6:], f, min(v), med, max(v), len(v), out[cap][f]['nonzero']))
    json.dump(out, open(os.path.join(BASE, 'race_distance_summary.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=2)
    print('\n写出 race_distance_summary.json')


if __name__ == '__main__':
    main()
