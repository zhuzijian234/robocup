"""Extension interoperability, signed encoder evidence and legacy report regression."""
import csv
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robocup_telemetry as core
import telemetry_v2 as v2
from test_v2 import control


def detail(seq=1, session=1, cs=1):
    ints = [0]*24
    ints[0:3] = [1, cs, 0]
    ints[4] = 1 | 8 | 64
    ints[12:14] = [38, 2]
    floats = [0.0]*16
    floats[3] = -337.5
    return v2.packet(5, struct.pack('<24I16f', *ints, *floats), seq, session)


def motor(seq=2, session=1, count=8, first=100):
    prefix = struct.pack('<HHIII', 1, count, first, 0, 3)
    samples = b''.join(struct.pack('<IHHffffI', (0xfffffff0+i*10000)&0xffffffff,
                    65535 if i==0 else 28, 0 if i==0 else 45,
                    23831.0 if i==0 else 10.0, 10.0, -200.0, -99.0 if i==0 else 45.0, 7)
                    for i in range(count))
    return v2.packet(6, prefix+samples, seq, session)


class ExtensionTests(unittest.TestCase):
    def test_fragmented_full_motor_frame_and_raw_float(self):
        frame = motor()
        self.assertEqual(len(frame), 256)
        p = core.StreamParser()
        for b in frame: p.feed(bytes([b]), .2)
        samples = p.v2.messages[0]['samples']
        self.assertEqual(len(samples), 8)
        self.assertEqual(samples[0]['encoder_signed'], -1)
        self.assertEqual(samples[0]['speed_float'], 23831)
        self.assertEqual(samples[-1]['sample_seq'], 107)
        self.assertEqual(samples[1]['sample_us'], 9984)

    def test_pairing_uses_session_control_sequence_and_revision(self):
        p = core.StreamParser()
        p.feed(control()+detail()+motor())
        p.feed(detail(seq=3, session=2))
        p.capture_duration = 1
        result = core.analyze(p)
        d = result['diagnosis']
        self.assertEqual(d['detail_frames'], 1)
        self.assertEqual(d['unpaired_controls'], 0)
        self.assertEqual(d['encoder_negative'], 1)
        self.assertEqual(d['empty_reference'], 1)
        self.assertEqual(d['radar_crc_bad'], 2)
        self.assertEqual(d['pd_d_abs_max'], 337.5)
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/'run.csv')
            core.write_csv(p, path)
            with Path(folder,'run.motor.csv').open(encoding='utf-8-sig') as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 8)
            self.assertEqual(rows[0]['encoder_signed'], '-1')
            core.write_report(result, str(Path(folder)/'report.md'))

    def test_missing_detail_not_zero_faults_and_malformed_resync(self):
        p = core.StreamParser()
        p.feed(v2.packet(6, struct.pack('<HHIII',1,9,0,0,0)))
        p.feed(control(seq=1))
        result = core.analyze(p)
        self.assertGreater(p.bad_frames, 0)
        self.assertEqual(result['diagnosis']['unpaired_controls'], 1)
        self.assertEqual(result['diagnosis']['motor_samples'], 0)

    def test_speed_clipping_excluded_from_mean(self):
        payload = bytearray(control()[14:-2])
        struct.pack_into('<h', payload, 36, 32767)
        struct.pack_into('<I', payload, 76, 1 << 18)
        p = core.StreamParser(); p.feed(v2.packet(1,payload))
        result = core.analyze(p)
        self.assertIsNone(result['speed_error'])
        self.assertEqual(result['speed_error_samples'], 0)
        self.assertEqual(result['clipped_fields']['Speed_now'], 1)

    def test_unknown_schema_rejected(self):
        payload = bytearray(detail()[14:-2]); struct.pack_into('<I', payload, 0, 2)
        p = core.StreamParser();p.feed(v2.packet(5,payload)+control(seq=1))
        self.assertEqual(len(p.records), 1)
        self.assertGreater(p.bad_frames, 0)

    def test_motor_invalid_flags_rejected(self):
        payload = bytearray(motor(count=1)[14:-2]); struct.pack_into('<I',payload,40,32)
        p = core.StreamParser();p.feed(v2.packet(6,payload))
        self.assertFalse(p.v2.messages)
        self.assertGreater(p.bad_frames, 0)


if __name__ == '__main__': unittest.main()
