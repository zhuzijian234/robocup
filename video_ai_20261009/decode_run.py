# -*- coding: utf-8 -*-
"""把 run_*.bin 原始记录解码成与控制台 GUI 相同的 CSV 产物。

GUI 的调用序列(robocup_gui.py:356-362):
    p = core.parse_file(binp); res = core.analyze(p)
    core.write_csv(p, base+'.csv'); core.write_report(res, base+'_report.md', name)
这里照抄，并额外尝试 telemetry_v2 的扩展写出(motor/detail/m10p)。
"""
import os, sys, traceback

UP = r'D:\OneDrive\Desktop\robocup\2-LXY传承\LXY传承-m10p\上位机'
LOG = os.path.join(UP, 'log')
OUT = r'D:\OneDrive\Desktop\robocup\video_ai_20261009\decoded'
sys.path.insert(0, UP)

import robocup_telemetry as core
try:
    import telemetry_v2 as tv2
except Exception:
    tv2 = None

os.makedirs(OUT, exist_ok=True)

BINS = ['run_1009_173143_4632265000000.bin', 'run_1009_173614_4903375000000.bin']
for b in BINS:
    src = os.path.join(LOG, b)
    if not os.path.exists(src):
        print('missing', b); continue
    base = os.path.join(OUT, b[:-4])
    try:
        p = core.parse_file(src)
        res = core.analyze(p)
        core.write_csv(p, base + '.csv')
        core.write_report(res, base + '_report.md', os.path.basename(src))
        made = ['.csv', '_report.md']
        if tv2 is not None:
            for fn, tag in [('write_extensions', '_ext')]:
                f = getattr(tv2, fn, None)
                if f is None:
                    continue
                try:
                    f(p, base)
                    made.append(tag)
                except Exception:
                    traceback.print_exc()
        print('%s -> ok  %s' % (b, made))
    except Exception:
        print('%s -> FAILED' % b)
        traceback.print_exc()

print('files in', OUT)
for f in sorted(os.listdir(OUT)):
    print('  %-60s %8d' % (f, os.path.getsize(os.path.join(OUT, f))))
