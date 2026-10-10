# -*- coding: utf-8 -*-
"""解码 10/10 的两个 run_*.bin（与 robocup_gui.py 相同的调用序列）。"""
import os, sys, traceback
UP = r'D:\OneDrive\Desktop\robocup\2-LXY传承\LXY传承-m10p\上位机'
LOG = os.path.join(UP, 'log')
OUT = r'D:\OneDrive\Desktop\robocup\tmp\decode_1010'
sys.path.insert(0, UP)
import robocup_telemetry as core
try:
    import telemetry_v2 as tv2
except Exception:
    tv2 = None
os.makedirs(OUT, exist_ok=True)

BINS = ['run_1010_171850_90260062000000.bin', 'run_1010_175052_92181875000000.bin']
for b in BINS:
    src = os.path.join(LOG, b)
    if not os.path.exists(src):
        print('missing', b); continue
    base = os.path.join(OUT, b[:-4])
    try:
        p = core.parse_file(src)
        res = core.analyze(p)
        core.write_csv(p, base + '.csv')
        core.write_report(res, base + '_report.md', b)
        if tv2 is not None and hasattr(tv2, 'write_extensions'):
            try:
                tv2.write_extensions(p, base)
            except Exception:
                traceback.print_exc()
        print('%s -> ok' % b)
    except Exception:
        print('%s -> FAILED' % b); traceback.print_exc()

print('\n--- decoded files ---')
for f in sorted(os.listdir(OUT)):
    print('  %-58s %8d' % (f, os.path.getsize(os.path.join(OUT, f))))
