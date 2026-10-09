"""New firmware schema must remain distinct from legacy DS mode diagnostics."""
import csv
import re
import struct
import tempfile
import unittest
from pathlib import Path
import robocup_telemetry as core
import telemetry_v2 as v2


def path_packet(seq=0):
    integers=[0]*24
    integers[0]=2; integers[4]=1; integers[7]=1; integers[8]=2
    integers[17]=0; integers[19]=1245; integers[20]=1445
    return v2.packet(5,struct.pack('<24I24f',*integers,*([0.0]*24)),seq)


class PathDiagnostics(unittest.TestCase):
    def test_fragmented_path_and_csv(self):
        parser=core.StreamParser()
        for byte in path_packet(): parser.feed(bytes([byte]),0)
        self.assertEqual(parser.bad_frames,0)
        message=parser.v2.messages[0]
        self.assertEqual(message['schema'],2)
        self.assertEqual(message['command_applied'],0)
        self.assertNotIn('radar_crc_bad',message)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'path.csv'
            v2.write_extensions(parser,path)
            with (Path(directory)/'path.detail.csv').open(encoding='utf-8-sig') as stream:
                row=next(csv.DictReader(stream))
            self.assertEqual(row['candidate_pwm'],'1245')
            self.assertEqual(row['final_pwm'],'1445')
            self.assertEqual(row['radar_crc_bad'],'')

    def test_new_registry_matches_production(self):
        project=Path(__file__).resolve().parents[2]
        source=(project/'HARDWARE/hc-05/ble_tune.c').read_text(encoding='utf-8')
        entries=re.findall(r'\{"(\w+)"\s*,\s*&[\w.]+\s*,\s*-?\d+\s*,\s*\d+\s*,\s*(\d+)\s*,\s*\d+\}',source)
        keys={int(key):name for name,key in entries}
        self.assertEqual(set(keys),v2.PATH_KEYS-set(range(1,15)))
        self.assertEqual(len(keys),15)
        for key,name in keys.items(): self.assertEqual(v2.KEYS[key],name)
        config={key:0 for key in v2.PATH_KEYS}
        config.update({1:v2.LAYOUT,2:'test-path',3:3,4:4,5:1,6:115200,7:1})
        parser=core.StreamParser()
        for i,(key,value) in enumerate(config.items()):
            parser.feed(v2.packet(3,struct.pack('<IIHH',1,0,i,len(config))+v2.tlv(key,value),i))
        parsed=parser.v2.configs[(1,0)]
        self.assertEqual(parsed['algorithm'],4)
        self.assertIn('preview',parsed)
        self.assertNotIn('disr',parsed)

    def test_schema_size_mismatch_rejected(self):
        parser=core.StreamParser()
        short=struct.pack('<24I16f',2,*([0]*23),*([0.0]*16))
        parser.feed(v2.packet(5,short)+path_packet(1))
        self.assertGreater(parser.bad_frames,0)
        self.assertEqual(len(parser.v2.messages),1)

    def test_labels_follow_algorithm(self):
        self.assertEqual(core.mode_name(1,4),'左侧路径')
        self.assertEqual(core.mode_name(1,3),'小右')
