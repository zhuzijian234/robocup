"""用两份真实采集逐点核对生产C解析器；时间为模拟值，不声称是板上测试。"""
from pathlib import Path
import hashlib
import importlib.util
import json
import math
import struct
import subprocess
import tempfile

from check_m10p import P, OUT, cl, env

ROOT = P.parents[1]
spec = importlib.util.spec_from_file_location('capture_parser', ROOT / 'm10p/check_m10p.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)
work = Path(tempfile.mkdtemp(prefix='replay_', dir=OUT))
exe = work / 'replay.exe'
build = subprocess.run([cl, '/nologo', '/std:c11', '/utf-8', '/W4', '/Od',
                        str(P / 'test/replay_m10p.c'), '/I' + str(P / 'HARDWARE/LEIDA_DATA'),
                        '/Fe:' + str(exe), '/Fo:' + str(work / 'replay.obj')], env=env, capture_output=True)
if build.returncode:
    raise RuntimeError((build.stdout + build.stderr).decode(errors='replace'))

records = []
captures = [
    ('hardware_check_verified.bin', 4311, 'e57aa16fa97db1f6998819e32fa96e27ed529e6ad28247ceafadb6712189ed5e'),
    ('latest_capture.bin', 8635, 'cc8fcc5d075bf7f56652b666048566dd65681deb505af3f9992019f1afe9ef34'),
]
for name, expected_packets, expected_hash in captures:
    path = ROOT / 'm10p/reports' / name
    raw = path.read_bytes()  # 缺少原始bin应明确失败，不能跳过后宣称回放通过。
    assert hashlib.sha256(raw).hexdigest() == expected_hash, '采集文件已变化：' + name
    parser = capture.Parser()
    frames = parser.feed(raw)
    assert parser.bad == 0 and len(frames) == expected_packets

    # 使用独立Python采集工具的浮点点云作参照；仅量化到固件的0.01度。
    # 首圈/末圈不完整：按车后切点取中间完整扫描范围，不比较DMA时间戳。
    points = []
    for _, _, frame_points, _ in frames:
        points.extend((int(math.floor(a * 100 + 0.5)) % 36000, d, int(reflect))
                      for a, d, reflect in frame_points)
    cuts = [i for i in range(1, len(points))
            if (points[i][0] + 18000) % 36000 < (points[i - 1][0] + 18000) % 36000]
    expected = b''.join(struct.pack('<HHB', *p) for p in points[cuts[0]:cuts[-1]])
    # 由每包报告转速估算采集时长；不把1MB文件瞬间喂入，否则整圈时效检查必然拒绝。
    duration = sum(60.0 / rpm / 24 for _, rpm, _, _ in frames)
    byte_rate = len(raw) / duration
    for chunk in (1, 7, 160, 512):
        output = work / f'{path.stem}_{chunk}.points'
        run = subprocess.run([str(exe), str(path), str(output), str(chunk), str(byte_rate)],
                             capture_output=True, check=True)
        stats = json.loads(run.stdout)
        assert stats['packets'] == expected_packets, stats
        assert stats['scans'] == len(cuts) - 1, stats
        assert not any(stats[k] for k in ('bad', 'discontinuities', 'overflow', 'rejected')), stats
        actual = output.read_bytes()
        assert actual == expected, (name, chunk, '点数/角度/距离/高反标志不一致')
        record = dict(file=name, chunk=chunk, **stats, point_sha256=hashlib.sha256(actual).hexdigest())
        records.append(record)
        print(json.dumps(record))
(OUT / 'replay_summary.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
print('PASS real captures: 4 chunk sizes, every published point matches Python reference')
