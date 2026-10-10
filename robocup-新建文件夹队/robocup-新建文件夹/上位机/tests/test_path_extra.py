"""点云丢块不能伪装成完整快照；扩展数据保持旧协议兼容。"""
import csv
import struct
import tempfile
import unittest
import sys
import json
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path

import robocup_telemetry as core
import telemetry_v2 as v2


def cloud(seq, offset, points, total=3, **changes):
    values = dict(zip(v2.CLOUD_U, [1, 7, 0, 123, 2, 99, 123456, 4, 1, total, offset, len(points)]))
    values.update(changes)
    payload = struct.pack('<12I', *(values[k] for k in v2.CLOUD_U))
    payload += b''.join(struct.pack('<hh', *p) for p in points)
    return v2.packet(9, payload, seq, 55)


class PathExtraTests(unittest.TestCase):
    def export(self, packets):
        parser = core.StreamParser()
        for packet in packets:
            for b in packet:
                parser.feed(bytes([b]), 0)
        self.assertEqual(parser.bad_frames, 0)
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)/'sample'
            v2.write_clouds(parser.v2.messages, base)
            with Path(str(base)+'.cloud_frames.csv').open(encoding='utf-8-sig') as stream:
                frames = list(csv.DictReader(stream))
            with Path(str(base)+'.cloud.csv').open(encoding='utf-8-sig') as stream:
                points = list(csv.DictReader(stream))
        return frames, points

    def test_complete_out_of_order_and_identical_repeat(self):
        frames, points = self.export([cloud(1, 2, [(30, 500)]), cloud(2, 0, [(-10, 100), (20, 300)]),
                                      cloud(3, 2, [(30, 500)])])
        self.assertEqual(frames[0]['complete'], '1')
        self.assertEqual(len(points), 3)
        self.assertEqual(points[0]['x_mm'], '-10')

    def test_missing_and_conflicting_metadata(self):
        frames, points = self.export([cloud(1, 0, [(-10, 100)])])
        self.assertEqual(frames[0]['complete'], '0')
        self.assertEqual(len(points), 1)
        frames, _ = self.export([cloud(1, 0, [(10, 100), (20, 300)]),
                                 cloud(2, 2, [(30, 500)], input_seq=100)])
        self.assertEqual(frames[0]['conflict'], '1')
        self.assertEqual(frames[0]['complete'], '0')

    def test_conflicting_point_and_empty_frame(self):
        frames, _ = self.export([cloud(1, 0, [(10, 100)], total=1), cloud(2, 0, [(11, 100)], total=1)])
        self.assertEqual(frames[0]['conflict'], '1')
        frames, points = self.export([cloud(1, 0, [], total=0)])
        self.assertEqual(frames[0]['complete'], '1')
        self.assertEqual(points, [])

    def test_malformed_cloud_and_extra(self):
        decoder = v2.Decoder()
        for packet in [cloud(1, 3, [(1, 2)]), cloud(1, 0, [], total=5),
                       cloud(1, 0, [(1, 2)], frame_index=4), cloud(1, 0, [(1, 2)], total=401)]:
            with self.assertRaises(ValueError):
                decoder.accept(packet, [], [], 0)
        u = [1]+[0]*11
        f = [0.0]*28
        f[2] = float('nan')
        with self.assertRaises(ValueError):
            decoder.accept(v2.packet(8, struct.pack('<12I28f', *u, *f)), [], [], 0)

    def test_capture_waits_for_cloud_after_completion_ack(self):
        from test_v2 import config_frames, control
        class Serial:
            def __init__(self, *args, **kwargs):
                self.buffer=b''; self.session=0; self.seq=0; self.tail=b''; self.dump=False
            def write(self, command):
                text=command.decode().strip()
                if text=='info':
                    self.buffer+=f'INFO proto=1,2 layout={v2.LAYOUT} fw=fake cloud=1\r\n'.encode()
                elif text.startswith('cloud '):
                    if text=='cloud dump':
                        self.dump=True
                        self.buffer+=b'CLOUD id=7 state=2 frames=1 sending=1\r\nCLOUD id=7 state=2 frames=1 sending=0\r\n'
                        payload=struct.pack('<12Ihh',1,7,0,123,2,99,123456,4,1,1,0,1,-10,100)
                        self.tail=v2.packet(9,payload,self.seq,self.session);self.seq+=1
                    else:self.buffer+=b'CLOUD id=7 state=2 frames=1 sending=0\r\n'
                else:
                    self.buffer+=('OK '+text+'\r\n').encode()
                    if text.startswith('session '):self.session=int(text.split()[1])
                    if text=='getcfg':
                        frames=config_frames(session=self.session)
                        self.buffer+=b''.join(frames);self.seq=len(frames)
            def read(self, size):
                if self.buffer:
                    data=self.buffer[:size];self.buffer=self.buffer[size:];return data
                if self.tail:
                    data=self.tail;self.tail=b'';return data
                data=control(self.seq,session=self.session);self.seq+=1;return data
            def close(self):pass
        with tempfile.TemporaryDirectory() as temp, patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=Serial)}):
            path=Path(temp)/'capture.bin'
            args=SimpleNamespace(port='FAKE',baud=115200,sec=.02,rate=1,out=str(path),stop_file=None)
            self.assertEqual(core.cmd_capture_v2(args),0)
            parsed=core.parse_file(str(path))
            self.assertTrue(v2.cloud_dump_complete(parsed.v2.messages,parsed.meta['session'],7,1))
            self.assertEqual(parsed.meta['cloud_dump'],'sent_check_csv_completeness')
            self.assertIn('formal_end_s',parsed.meta)


if __name__ == '__main__':
    unittest.main()
