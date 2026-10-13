# -*- coding: utf-8 -*-
"""探明 3c1af22 后调试视频 的规格 + 对应采集 + **每段用的固件ID**。"""
import cv2, glob, json, os, csv, datetime

D = r'D:\OneDrive\Desktop\robocup\调试视频\10.12\3c1af2204be9967e8ec5bcbb33a689cf3cfb207f后调试视频'
LOG = r'D:\OneDrive\Desktop\robocup\robocup-新建文件夹队\robocup-新建文件夹\上位机\log'

print('=== 视频规格 ===')
for f in sorted(glob.glob(os.path.join(D, '*.mp4'))):
    c = cv2.VideoCapture(f)
    n = int(c.get(cv2.CAP_PROP_FRAME_COUNT)); fps = c.get(cv2.CAP_PROP_FPS)
    w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH)); h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
    c.release()
    print('  %4dx%-4d %5.1ffps %4d帧 %6.2fs  %s' % (w, h, fps, n, (n / fps if fps else 0),
                                                   os.path.basename(f)))

print('\n=== 对应采集与固件 ===')
import re
ids = []
for f in sorted(glob.glob(os.path.join(D, '*.mp4'))):
    m2 = re.search(r'v2_1012_\d{6}_\d+', os.path.basename(f))
    if m2:
        ids.append(m2.group(0))
print('  文件名里的采集ID:', ids)
for i in ids:
    files = glob.glob(os.path.join(LOG, i + '.*'))
    mp = os.path.join(LOG, i + '.bin.meta.json')
    if not os.path.exists(mp):
        print('  %s  **缺 meta**（匹配到 %d 个文件）' % (i, len(files))); continue
    m = json.load(open(mp, encoding='utf-8'))
    cfg = {}
    for _, v in (m.get('configs') or {}).items():
        cfg = v; break
    info = m.get('info') or ''
    fw = ''
    if isinstance(info, dict):
        fw = info.get('fw') or info.get('build') or json.dumps(info, ensure_ascii=False)[:150]
    else:
        fw = str(info)[:150]
    print('  %-32s dur=%7.3fs end=%-10s 文件=%2d' % (i, m['duration_s'], m.get('end_reason'), len(files)))
    print('       fw/info = %s' % fw)
    print('       spd=%s bendspd=%s  configs键=%s' %
          (cfg.get('spd'), cfg.get('bendspd'), list(cfg.keys())[:8]))
    for ext in ('.csv', '.detail.csv', '.path.csv', '.race.csv', '.motor.csv', '.cloud.csv', '.cloud_frames.csv'):
        p = os.path.join(LOG, i + ext)
        if os.path.exists(p):
            with open(p, encoding='utf-8-sig', newline='') as fh:
                nr = sum(1 for _ in fh) - 1
            print('       %-20s 行=%d' % (ext, nr))
