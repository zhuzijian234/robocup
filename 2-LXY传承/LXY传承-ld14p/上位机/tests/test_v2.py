import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robocup_telemetry as core
import telemetry_v2 as v2


def control(seq=0,session=1,ms=100,rev=0,valid=0x7fffffff,action=1):
    data=bytearray(92)
    values=[0]*26
    values[0]=5000;values[1]=1445;values[2]=7;values[3]=500
    values[18]=80;values[19]=8
    struct.pack_into('<26h',data,0,*values)
    struct.pack_into('<7I',data,52,seq+1,ms-10 if ms>=10 else 0,ms,114800,valid,valid,0)
    struct.pack_into('<4BIHH',data,80,action,2,1,1,rev,444,2160)
    return v2.packet(1,data,seq,session)


def config_frames(session=1,rev=0,seq=0,split=23):
    values={key:0 for key in v2.KEYS}
    values.update({1:v2.LAYOUT,2:'test-source',3:1,4:1,5:0,6:115200,7:1,8:500,9:1000,
                   10:'units',11:1798,12:800,13:'geometry',14:'lidar'})
    values.update({100+i:.0395 for i in range(16)})
    data=b''.join(v2.tlv(k,v) for k,v in values.items())
    chunks=[data[i:i+split] for i in range(0,len(data),split)]
    return [v2.packet(3,struct.pack('<IIHH',1,rev,i,len(chunks))+chunk,seq+i,session)
            for i,chunk in enumerate(chunks)]


class V2Tests(unittest.TestCase):
    def test_crc_and_lengths(self):
        self.assertEqual(v2.crc16(b'123456789'),0x29b1)
        self.assertEqual(len(control()),108)
        self.assertEqual(len(v2.packet(2,bytes(64))),80)

    def test_bytewise_and_invalid_mask(self):
        p=core.StreamParser()
        frame=control(valid=0x7ffffffe)
        for b in frame:p.feed(bytes([b]),.5)
        self.assertEqual(len(p.records),1)
        self.assertTrue(math.isnan(p.records[0]['err']))
        self.assertEqual(p.records[0]['pid_select'],7)
        self.assertEqual(p.records[0]['Midline.k'],.5)

    def test_corruption_resync(self):
        p=core.StreamParser();bad=bytearray(control());bad[10]^=1
        p.feed(bytes(bad)+control(1))
        self.assertEqual(len(p.records),1)
        self.assertGreater(p.bad_frames,0)
        for _ in range(100):p.feed(b'\xaa\x55\x02\x01\xff\xff'+b'garbage'*30)
        self.assertLess(len(p.buf),256)
        self.assertLessEqual(len(p._ascii),256)

    def test_crc_protects_sequence_and_reserved_bits(self):
        p=core.StreamParser();p.feed(control(valid=0xffffffff))
        self.assertEqual(len(p.records),0)
        self.assertGreater(p.bad_frames,0)

    def test_configs_cross_tlv_and_event(self):
        p=core.StreamParser();frames=config_frames()
        for f in frames[:-1]:p.feed(f)
        self.assertFalse(p.v2.configs)
        p.feed(frames[-1])
        self.assertAlmostEqual(p.v2.configs[(1,0)]['kp'],.0395,places=7)
        event=struct.pack('<IIHHHBBff',200,1,1,12,100,3,4,.0395,.045)
        p.feed(v2.packet(4,event,len(frames)))
        self.assertAlmostEqual(p.v2.configs[(1,1)]['kp'],.045,places=7)

    def test_time_wrap_session_and_duplicates(self):
        p=core.StreamParser()
        p.feed(control(0,ms=0xfffffff0)+control(1,ms=84)+control(1,ms=84))
        self.assertEqual(len(p.records),2)
        self.assertAlmostEqual(p.records[-1]['t'],.1)
        self.assertEqual(p.v2.duplicates,1)
        p.feed(control(0,session=2,ms=100))
        self.assertNotEqual(p.records[-1]['segment'],p.records[-2]['segment'])

    def test_unknown_type_and_missing_sequence(self):
        p=core.StreamParser();p.feed(v2.packet(99,b'future',0)+control(2))
        self.assertEqual(p.v2.messages[0]['type'],99)
        self.assertEqual(p.v2.missing,1)

    def test_handshake_requires_ack_and_config(self):
        p=core.StreamParser();sent=[]
        h=v2.Handshake(lambda c:sent.append(c),p,session=7)
        h.start(0)
        replies=[f'INFO proto=1,2 layout={v2.LAYOUT} fw=test','OK session 7','OK tele 3','OK rate 1','OK getcfg']
        for i,line in enumerate(replies):p.feed((line+'\r\n').encode());h.poll(i+1)
        self.assertFalse(h.done)
        for frame in config_frames(session=7):p.feed(frame)
        h.poll(6);self.assertTrue(h.done)
        self.assertEqual(sent,['info\n','session 7\n','tele 3\n','rate 1\n','getcfg\n'])
        p2=core.StreamParser();h2=v2.Handshake(lambda c:True,p2);h2.start(0)
        with self.assertRaises(TimeoutError):h2.poll(9)

    def test_info_after_default_vofa(self):
        p=core.StreamParser()
        old=struct.pack('<8f',.5,1445,.035,.075,7,8,8,2160)+core.VOFA_TAIL
        reply=f'INFO proto=1,2 layout={v2.LAYOUT}\r\n'.encode()
        for b in old+reply:p.feed(bytes([b]))
        self.assertEqual(p.ascii_lines[-1]['text'],reply.decode().strip())

    def test_report_and_csv(self):
        p=core.StreamParser();frames=config_frames()
        for f in frames:p.feed(f,.1)
        p.feed(control(len(frames),ms=100),.2)
        p.capture_duration=1
        with tempfile.TemporaryDirectory() as d:
            result=core.analyze(p);self.assertTrue(result['v2'])
            core.write_csv(p,str(Path(d)/'data.csv'))
            core.write_report(result,str(Path(d)/'report.md'))
            core.write_plot(p,result,str(Path(d)/'plot.png'))
            self.assertIn('valid_mask',(Path(d)/'data.csv').read_text(encoding='utf-8-sig'))
            self.assertTrue((Path(d)/'data.events.jsonl').exists())

    def test_capture_cli_serial_roundtrip(self):
        class Serial:
            def __init__(self,*args,**kwargs):self.buffer=b'';self.session=0;self.seq=0
            def write(self,command):
                text=command.decode().strip()
                if text=='info':self.buffer+=f'INFO proto=1,2 layout={v2.LAYOUT} fw=fake\r\n'.encode()
                else:
                    self.buffer+=('OK '+text+'\r\n').encode()
                    if text.startswith('session '):self.session=int(text.split()[1])
                    if text=='getcfg':
                        frames=config_frames(session=self.session)
                        self.buffer+=b''.join(frames);self.seq=len(frames)
            def read(self,size):
                if self.buffer:
                    data=self.buffer[:size];self.buffer=self.buffer[size:];return data
                data=control(self.seq,session=self.session);self.seq+=1;return data
            def close(self):pass
        with tempfile.TemporaryDirectory() as d,patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=Serial)}):
            path=str(Path(d)/'live.bin')
            args=SimpleNamespace(port='FAKE',baud=115200,sec=.03,rate=1,out=path,stop_file=None)
            self.assertEqual(core.cmd_capture_v2(args),0)
            parsed=core.parse_file(path)
            self.assertEqual(parsed.meta['end_reason'],'complete')
            self.assertIn('pre_roll_s',parsed.meta)
            self.assertTrue(parsed.v2.configs)


if __name__=='__main__':unittest.main()
