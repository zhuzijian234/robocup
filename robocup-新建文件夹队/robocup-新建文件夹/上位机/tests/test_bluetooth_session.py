"""无电机实连：验证真实V2收发循环、命令覆盖、超时和串口所有权。"""
import json
import queue
import struct
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bluetooth_session as link
import robocup_telemetry as core
import telemetry_v2 as v2
from test_v2 import config_frames, control


class FakeSerial:
    def __init__(self, cloud=True):
        self.buffer = b""
        self.session = self.seq = self.revision = 0
        self.commands = []
        self.writers = set()
        self.closed = False
        self.cloud = cloud
        self.on_command = lambda command: None
        self.short_write = False

    def write(self, data):
        text = data.decode().strip()
        self.commands.append(text)
        self.writers.add(threading.get_ident())
        if self.short_write:
            return 0
        if text == "info":
            self.buffer += f'INFO proto=1,2 layout={v2.LAYOUT} fw=fake {"cloud=2" if self.cloud else ""}\r\n'.encode()
        elif text.startswith("cloud "):
            self.buffer += b'CLOUD id=7 state=2 frames=1 sending=0\r\n'
            if text == "cloud dump":
                payload = struct.pack('<12Ihh', 4, 7, 0, 5, self.revision, 5, 100, 1, 128, 1, 0, 1, 0, 500)
                self.buffer += v2.packet(9, payload, self.seq, self.session)
                self.seq += 1
        elif text.startswith("spd "):
            self.revision += 1
            self.buffer += f'OK spd={text.split()[1]}\r\n'.encode()
        else:
            self.buffer += ('OK ' + text + '\r\n').encode()
            if text.startswith("session "):
                self.session = int(text.split()[1])
                self.seq = 0
            if text == "getcfg":
                frames = config_frames(session=self.session, rev=self.revision, seq=self.seq)
                self.buffer += b''.join(frames)
                self.seq += len(frames)
        self.on_command(text)
        return len(data)

    def read(self, size):
        time.sleep(.001)
        if self.buffer:
            chunk, self.buffer = self.buffer[:size], self.buffer[size:]
            return chunk
        packet = control(self.seq, session=self.session, rev=self.revision)
        self.seq += 1
        return packet

    def close(self):
        self.closed = True


class MotorTests(unittest.TestCase):
    def setUp(self):
        self.events, self.sent = [], []
        self.motor = link.MotorCommand(lambda *event: self.events.append(event))
        self.parser = core.StreamParser()

    def poll(self, now):
        self.motor.poll(self.parser, lambda command: self.sent.append(command), now)

    def test_shutdown_supersedes_start_and_ignores_late_ack(self):
        self.motor.request(17)
        self.poll(0)
        self.motor.request(0)
        self.parser.feed(b'OK spd=17\r\n')
        self.poll(.1)
        self.assertEqual(self.sent, ['spd 17\n', 'spd 0\n'])
        self.assertEqual(self.motor.pending, 0)
        self.parser.feed(b'OK spd=0\r\n')
        self.poll(.2)
        self.assertIsNone(self.motor.pending)
        self.assertIn('关闭指令已确认', self.events[-1][1])

    def test_unsent_old_ack_busy_retry_and_bounded_timeout(self):
        self.parser.feed(b'OK spd=0\r\n')
        self.motor.request(0)
        self.poll(0)
        self.assertEqual(self.motor.pending, 0)
        self.parser.feed(b'ERR busy\r\n')
        for now in (.1, 1, 2, 3, 4, 5):
            self.poll(now)
        self.assertEqual(len(self.sent), 4)
        self.assertIsNone(self.motor.pending)
        self.assertTrue(any('未确认' in text for kind, text in self.events))
        self.assertFalse(any('已确认' in text for kind, text in self.events))

    def test_handshake_defers_start_and_queued_stop_wins(self):
        worker = link.BluetoothSession('FAKE', 115200, queue.Queue())
        worker.request_motor(17)
        worker.request_motor(0)
        self.assertFalse(worker._service(self.parser, self.sent.append, 'negotiating'))
        self.assertEqual(self.sent, [])
        self.assertTrue(worker._service(self.parser, self.sent.append, 'idle'))
        self.assertEqual(self.sent, ['spd 0\n'])


class CaptureTests(unittest.TestCase):
    def make(self, path, sec=.04):
        worker = link.BluetoothSession('FAKE', 115200, queue.Queue())
        serial = FakeSerial()
        worker.connection = serial
        args = SimpleNamespace(port='FAKE', baud=115200, sec=sec, rate=1, out=str(path), stop_file=None)
        return worker, serial, args

    def test_one_click_starts_after_config_and_logs_manual_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'auto.bin'
            worker, serial, args = self.make(path)
            def command(text):
                if text == 'spd 17':
                    worker.request_motor(0)
            serial.on_command = command
            worker._capture((args, 17, 0))
            self.assertEqual([c for c in serial.commands if c.startswith('spd ')], ['spd 17', 'spd 0'])
            self.assertLess(serial.commands.index('getcfg'), serial.commands.index('spd 17'))
            self.assertFalse(serial.closed)
            parsed = core.parse_file(str(path))
            self.assertEqual(parsed.meta['end_reason'], 'complete')
            self.assertEqual(parsed.meta['cloud_dump'], 'sent_check_csv_completeness')
            self.assertEqual([e['command'] for e in parsed.meta['commands'] if e['command'].startswith('spd ')], ['spd 17', 'spd 0'])
            self.assertTrue(parsed.v2.configs)

    def test_stop_during_negotiation_cancels_auto_start_and_saves(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cancel.bin'
            worker, serial, args = self.make(path)
            serial.on_command = lambda text: worker.request_motor(0) if text == 'info' else None
            worker._capture((args, 17, 0))
            worker._service(worker.parser, worker._send, 'idle')
            self.assertNotIn('spd 17', serial.commands)
            self.assertEqual(serial.commands[-1], 'spd 0')
            meta = json.loads(Path(str(path)+'.meta.json').read_text(encoding='utf-8'))
            self.assertNotIn('pre_roll_s', meta)
            self.assertEqual(meta['end_reason'], 'user_stop')
            self.assertFalse(serial.closed)

    def test_manual_stop_at_last_handshake_reply_never_restarts(self):
        with tempfile.TemporaryDirectory() as directory:
            worker, serial, args = self.make(Path(directory)/'boundary.bin')
            read = serial.read
            def last_reply(size):
                data = read(size)
                if b'OK getcfg' in data:
                    worker.request_motor(0)
                return data
            serial.read = last_reply
            worker._capture((args, 17, 0))
            self.assertNotIn('spd 17', serial.commands)
            self.assertIn('spd 0', serial.commands)

    def test_stop_during_cloud_export_uses_same_connection(self):
        with tempfile.TemporaryDirectory() as directory:
            worker, serial, args = self.make(Path(directory)/'dump.bin')
            serial.on_command = lambda text: worker.request_motor(0) if text == 'cloud freeze' else None
            worker._capture((args, 17, 0))
            self.assertLess(serial.commands.index('cloud freeze'), serial.commands.index('spd 0'))
            self.assertIn('cloud dump', serial.commands)
            self.assertFalse(serial.closed)
            self.assertEqual(len(serial.writers), 1)

    def test_capture_stop_does_not_send_motor_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'stop.bin'
            worker, serial, args = self.make(path)
            serial.on_command = lambda text: worker.capture_stop.set() if text == 'spd 17' else None
            worker._capture((args, 17, 0))
            self.assertIn('spd 17', serial.commands)
            self.assertNotIn('spd 0', serial.commands)
            meta = json.loads(Path(str(path)+'.meta.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['end_reason'], 'user_stop')
            self.assertIn('formal_end_s', meta)

    def test_short_write_saves_failure_and_does_not_start(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'short.bin'
            worker, serial, args = self.make(path)
            serial.short_write = True
            worker._capture((args, 17, 0))
            self.assertNotIn('spd 17', serial.commands)
            meta = json.loads(Path(str(path)+'.meta.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['end_reason'], 'error')
            self.assertIn('未完整发送', meta['error'])

    def test_two_runs_and_idle_motor_control_only_open_once(self):
        events = queue.Queue()
        instances = []
        def factory(*args, **kwargs):
            serial = FakeSerial();instances.append(serial);return serial
        worker = link.BluetoothSession('FAKE', 115200, events, factory)
        def wait_for(kind, matches=lambda value: True):
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                event, value = events.get(timeout=3)
                if event == kind and matches(value):return value
            self.fail('未收到事件 '+kind)
        with tempfile.TemporaryDirectory() as directory:
            worker.start()
            try:
                wait_for('connected')
                for number in range(2):
                    _, _, args = self.make(Path(directory)/f'run{number}.bin')
                    worker.request_capture(args, 17)
                    self.assertEqual(wait_for('captured')['result'], 0)
                    worker.request_motor(0)
                    wait_for('motor', lambda text: text.startswith('关闭指令已确认'))
                self.assertEqual(len(instances), 1)
                self.assertFalse(instances[0].closed)
                self.assertEqual(len(instances[0].writers), 1)
                self.assertNotIn(threading.get_ident(), instances[0].writers)
            finally:
                worker.closing.set();worker.join(timeout=3)
            self.assertFalse(worker.is_alive())
            self.assertTrue(instances[0].closed)


if __name__ == '__main__':
    unittest.main()
