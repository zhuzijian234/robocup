"""急弯扩展必须兼容旧帧，并保留采集期间的会话边界。"""
import csv
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robocup_telemetry as core
import telemetry_v2 as v2


class AcuteDiagnostics(unittest.TestCase):
    def test_entry_race_schema_and_missing_fields(self):
        parser = core.StreamParser()
        old = struct.pack('<19I14f', 2, *([0]*18), *([0.0]*14))
        ints = [3]+[0]*18+[2,2,0]
        floats = [0.0]*14+[12.5,40,925,-.1]
        new = struct.pack('<22I18f', *ints, *floats)
        for byte in v2.packet(10,old,0)+v2.packet(10,new,1):
            parser.feed(bytes([byte]),0)
        self.assertEqual(parser.bad_frames,0)
        self.assertNotIn('acute_phase',parser.v2.messages[0])
        self.assertEqual(parser.v2.messages[1]['acute_phase'],2)
        self.assertEqual(parser.v2.messages[1]['acute_goal_mm'],40)
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'entry.csv'
            v2.write_extensions(parser,file)
            with file.with_suffix('.race.csv').open(encoding='utf-8-sig') as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(rows[0]['acute_phase'],'')
            self.assertEqual(rows[1]['acute_phase'],'2')
        for index,bad in ((19,5),(20,8),(21,3)):
            invalid=ints.copy();invalid[index]=bad
            with self.assertRaises(ValueError):
                v2.Decoder().accept(v2.packet(10,struct.pack('<22I18f',*invalid,*floats)),[],[],0)
        for length in (112,148,172,180):
            malformed=(new+bytes(180))[:length-16]
            with self.assertRaises(ValueError):
                v2.Decoder().accept(v2.packet(10,malformed),[],[],0)

    def test_old_and_new_race_fragmented_and_csv(self):
        parser = core.StreamParser()
        old = struct.pack('<14I10f', 1, *([0] * 13), *([0.0] * 10))
        integers = [2] + [0] * 13 + [8, 1, 1, 1, 64]
        floats = [0.0] * 10 + [-25.0, -492.0, .6, 220.0]
        new = struct.pack('<19I14f', *integers, *floats)
        for byte in v2.packet(10, old, 0) + v2.packet(10, new, 1):
            parser.feed(bytes([byte]), 0)
        self.assertEqual(parser.bad_frames, 0)
        self.assertEqual(len(parser.v2.messages), 2)
        self.assertNotIn('outer_offset', parser.v2.messages[0])
        self.assertEqual(parser.v2.messages[1]['outer_offset'], -25)
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'acute.csv'
            v2.write_extensions(parser, file)
            with file.with_suffix('.race.csv').open(encoding='utf-8-sig') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]['identity_corrected'], '')
            self.assertEqual(rows[1]['identity_corrected'], '1')

    def test_race_schema_bounds_and_cloud_event(self):
        integers = [2] + [0] * 18
        floats = [0.0] * 14
        for index, bad in ((14, 13), (15, 2), (18, 1024)):
            values = integers.copy()
            values[index] = bad
            with self.assertRaises(ValueError):
                v2.Decoder().accept(v2.packet(10, struct.pack('<19I14f', *values, *floats)), [], [], 0)
        for schema, accepted in ((2, False), (3, True)):
            packet = v2.packet(9, struct.pack('<12Ihh', schema, 7, 0, 5, 1, 5, 100, 1,
                                             64, 1, 0, 1, 0, 500))
            if accepted:
                decoder = v2.Decoder()
                decoder.accept(packet, [], [], 0)
                self.assertEqual(decoder.messages[0]['trigger_flags'], 64)
            else:
                with self.assertRaises(ValueError):
                    v2.Decoder().accept(packet, [], [], 0)

    def test_handshake_detects_session_lost_after_current_session_seen(self):
        from test_v2 import control
        parser = core.StreamParser()
        handshake = v2.Handshake(lambda command: True, parser, session=123)
        handshake.start(0)
        parser.feed(control(0, session=0), 0)
        handshake.poll(.1)  # 连接前残留帧不误判为重启。
        parser.feed(control(0, session=123), .2)
        handshake.poll(.2)
        parser.feed(control(1, session=0), .3)
        with self.assertRaisesRegex(ValueError, '会话改变'):
            handshake.poll(.3)

    def test_dump_retries_not_started_and_verifies_received_points(self):
        from test_v2 import config_frames, control

        class Serial:
            instance = None

            def __init__(self, *args, **kwargs):
                Serial.instance = self
                self.buffer = b''
                self.session = 0
                self.seq = 0
                self.attempts = 0

            def write(self, command):
                text = command.decode().strip()
                if text == 'info':
                    self.buffer += f'INFO proto=1,2 layout={v2.LAYOUT} fw=fake cloud=2\r\n'.encode()
                elif text.startswith('cloud '):
                    if text == 'cloud dump':
                        self.attempts += 1
                        if self.attempts == 1:
                            return  # 模拟首次dump命令未被设备接受。
                        # 即使sending=1确认丢失，也以实际点完整性为准。
                        payload = struct.pack('<12Ihh', 3, 7, 0, 1, 1, 1, 10, 1, 64, 1, 0, 1, 0, 500)
                        self.buffer += v2.packet(9, payload, self.seq, self.session)
                        self.seq += 1
                    self.buffer += b'CLOUD id=7 state=2 frames=1 sending=0\r\n'
                else:
                    self.buffer += ('OK ' + text + '\r\n').encode()
                    if text.startswith('session '):
                        self.session = int(text.split()[1])
                    if text == 'getcfg':
                        frames = config_frames(session=self.session)
                        self.buffer += b''.join(frames)
                        self.seq = len(frames)

            def read(self, size):
                if self.buffer:
                    data = self.buffer[:size]
                    self.buffer = self.buffer[size:]
                    return data
                data = control(self.seq, session=self.session)
                self.seq += 1
                return data

            def close(self):
                pass

        clock = [0.0]

        def monotonic():
            clock[0] += .05
            return clock[0]

        with tempfile.TemporaryDirectory() as directory, \
                patch.dict(sys.modules, {'serial': SimpleNamespace(Serial=Serial)}), \
                patch.object(core.time, 'monotonic', monotonic):
            file = Path(directory) / 'retry.bin'
            args = SimpleNamespace(port='FAKE', baud=115200, sec=.2, rate=1, out=str(file), stop_file=None)
            self.assertEqual(core.cmd_capture_v2(args), 0)
            parsed = core.parse_file(str(file))
            self.assertEqual(Serial.instance.attempts, 2)
            self.assertTrue(v2.cloud_dump_complete(parsed.v2.messages, parsed.meta['session'], 7, 1))
            self.assertEqual(parsed.meta['cloud_dump'], 'sent_check_csv_completeness')


if __name__ == '__main__':
    unittest.main()
