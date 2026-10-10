"""新导航DETAIL、协商撤驱动及历史日志报告口径回归。"""
import sys, struct, unittest, tempfile, time
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import robocup_telemetry as core
import telemetry_v2 as v2
from test_v2 import control,config_frames

class NavigationProtocolTests(unittest.TestCase):
    def test_schema2_keeps_navigation_and_schema1_compatible(self):
        ints=[0]*24;ints[0]=2;ints[1]=42
        floats=[0.0]*16
        nav=[3,1,1292,2,41,64,62,180,2,0]
        payload=struct.pack('<24I16f10I4f',*ints,*floats,*nav,8.,0.,20.,250.)
        p=core.StreamParser()
        frame=v2.packet(5,payload)
        self.assertEqual(len(frame),232)
        for b in frame:p.feed(bytes([b]))
        self.assertEqual(p.bad_frames,0)
        self.assertEqual(p.v2.messages[0]['reject_reason'],3)
        self.assertEqual(p.v2.messages[0]['effective_speed'],0)
        self.assertEqual(p.v2.messages[0]['obstacle_y'],250)

    def test_new_handshake_stops_before_session_and_does_not_arm(self):
        p=core.StreamParser();sent=[]
        h=v2.Handshake(sent.append,p,session=7);h.start(0)
        replies=[f'INFO proto=1,2 layout={v2.LAYOUT} drive=1','OK drive 0','OK session 7','OK tele 3','OK rate 1','OK getcfg']
        for i,line in enumerate(replies):p.feed((line+'\r\n').encode());h.poll(i+1)
        self.assertFalse(h.done)
        for frame in config_frames(session=7):p.feed(frame)
        h.poll(7)
        self.assertTrue(h.done)
        self.assertEqual(sent[1],'drive 0\n')
        self.assertNotIn('drive 1\n',sent)

    def test_capture_arms_after_config_and_collects_after_stop(self):
        commands=[]
        class Serial:
            def __init__(self,*args,**kwargs):self.buffer=b'';self.session=0;self.seq=0
            def write(self,raw):
                text=raw.decode().strip();commands.append(text)
                if text=='info':self.buffer+=f'INFO proto=1,2 layout={v2.LAYOUT} drive=1\r\n'.encode()
                else:
                    self.buffer+=('OK '+text+'\r\n').encode()
                    if text.startswith('session '):self.session=int(text.split()[1])
                    if text=='getcfg':
                        frames=config_frames(session=self.session)
                        self.buffer+=b''.join(frames);self.seq=len(frames)
            def read(self,size):
                time.sleep(.005)
                if self.buffer:
                    data=self.buffer[:size];self.buffer=self.buffer[size:];return data
                data=control(self.seq,session=self.session);self.seq+=1;return data
            def close(self):pass
        with tempfile.TemporaryDirectory() as folder,patch.dict(sys.modules,{'serial':SimpleNamespace(Serial=Serial)}):
            path=str(Path(folder)/'capture.bin')
            args=SimpleNamespace(port='TEST',baud=115200,sec=.03,rate=1,out=path,stop_file=None)
            self.assertEqual(core.cmd_capture_v2(args),0)
            parsed=core.parse_file(path)
            self.assertLess(commands.index('drive 0'),commands.index('getcfg'))
            self.assertLess(commands.index('getcfg'),commands.index('drive 1'))
            self.assertEqual(commands[-1],'drive 0')
            self.assertGreater(parsed.meta['duration_s']-parsed.meta['post_roll_start_s'],1.9)

    def test_report_exposes_preroll_and_failed_negotiation(self):
        p=core.StreamParser();p.feed(control(0),.5);p.feed(control(1),2)
        p.meta={'pre_roll_s':1};p.capture_duration=3
        r=core.analyze(p)
        self.assertEqual((r['n_frames'],r['all_frames']),(1,2))
        self.assertTrue(r['formal_known'])
        p.meta={'end_reason':'error'}
        self.assertFalse(core.analyze(p)['formal_known'])

if __name__=='__main__':unittest.main()
