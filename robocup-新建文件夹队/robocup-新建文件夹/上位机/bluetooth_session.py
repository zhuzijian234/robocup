"""蓝牙串口单线程所有权：空闲、V2采集、点云导出共用同一连接。"""
import math
import queue
import threading
import time

import robocup_telemetry as core


class MotorCommand:
    """只在串口工作线程使用。关闭覆盖尚未执行的启动；收到ACK才显示确认。"""
    def __init__(self, emit):
        self.emit = emit
        self.pending = None
        self.parser = None
        self.cursor = 0
        self.sent_at = 0.0
        self.attempts = 0

    def request(self, speed):
        if not isinstance(speed, int) or not 0 <= speed <= 18:
            raise ValueError("电机速度必须是0～18的整数")
        self.pending = speed
        self.attempts = 0
        self.sent_at = 0.0
        self.emit("motor", "等待关闭确认" if speed == 0 else f"等待启动确认（{speed}）")

    def poll(self, parser, send, now, allow_send=True):
        if self.parser is not parser:
            self.parser, self.cursor = parser, 0
        lines = parser.ascii_lines[self.cursor:]
        self.cursor = len(parser.ascii_lines)
        for item in lines:
            line = item["text"]
            # 不接受发送前的旧ACK；也不让先前启动ACK覆盖后来的关闭请求。
            if self.pending is None or not self.attempts:
                continue
            if line == f"OK spd={self.pending}":
                text = "关闭指令已确认（目标0）" if self.pending == 0 else f"启动指令已确认（上限{self.pending}）"
                self.emit("motor", text)
                self.emit("log", text)
                self.pending = None
            elif line.startswith("ERR "):
                self.emit("log", f"设备回复 {line}；电机命令尚未确认")
        if self.pending is None or not allow_send:
            return
        if self.attempts and now - self.sent_at < .8:
            return
        if self.attempts >= 4:
            self.emit("motor", "指令未确认，请检查蓝牙连接")
            self.emit("log", f"电机命令 spd {self.pending} 未收到确认，未将发送成功视为执行成功")
            self.pending = None
            return
        if send(f"spd {self.pending}\n") is False:
            raise IOError("电机命令发送失败")
        self.sent_at = now
        self.attempts += 1

    def reset_cursor(self):
        self.cursor = 0


class BluetoothSession(threading.Thread):
    def __init__(self, port, baud, events, serial_factory=None):
        super().__init__(name="bluetooth-port", daemon=True)
        self.port, self.baud, self.events = port, baud, events
        self.serial_factory = serial_factory
        self.closing = threading.Event()
        self.capture_stop = threading.Event()
        self.jobs = queue.Queue(maxsize=1)
        self.motor_requests = queue.Queue()
        self.request_lock = threading.Lock()
        self.motor_generation = 0
        self.motor = MotorCommand(self.emit)
        self.parser = core.StreamParser()
        self.connection = None
        self.phase = "idle"
        self.last_progress = 0.0

    def emit(self, kind, value):
        self.events.put((kind, value))

    def request_motor(self, speed):
        if not isinstance(speed, int) or not 0 <= speed <= 18:
            raise ValueError("电机速度必须是0～18的整数")
        with self.request_lock:
            self.motor_generation += 1
            self.motor_requests.put(speed)

    def request_capture(self, args, start_speed):
        # UI在任务结束前禁止重复提交；记录手动命令代次，防止协商结束后误重启。
        with self.request_lock:
            generation = self.motor_generation
        self.capture_stop.clear()
        self.jobs.put_nowait((args, start_speed, generation))

    def _drain_motor_requests(self):
        latest = None
        while True:
            try:
                latest = self.motor_requests.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.motor.request(latest)

    def _send(self, command):
        data = command.encode("ascii")
        if self.connection.write(data) != len(data):
            raise IOError("串口命令未完整发送")
        return True

    def _service(self, parser, send, phase):
        self.parser = parser
        if phase != self.phase:
            self.phase = phase
            self.emit("phase", phase)
        self._drain_motor_requests()
        if self.closing.is_set():
            return False
        # 协商时不混入spd的ERR busy污染握手。关闭请求立即取消协商，
        # 回到同一连接的空闲循环执行，绝不等完8/15秒协商超时才处理。
        if phase == "negotiating" and self.motor.pending == 0:
            return False
        self.motor.poll(parser, send, time.monotonic(), allow_send=phase != "negotiating")
        now = time.monotonic()
        if now - self.last_progress >= .3:
            self.last_progress = now
            row = parser.records[-1] if parser.records else {}
            speed = row.get("Speed_now")
            self.emit("progress", dict(frames=parser.total_frames, bytes=parser.received_bytes,
                                       speed=speed if speed is not None and math.isfinite(speed) else None))
        return True

    def _capture(self, job):
        args, start_speed, generation = job
        def ready():
            # 手动关闭或停止采集优先于原先安排的自动启动。
            with self.request_lock:
                if (self.motor_generation == generation and not self.capture_stop.is_set()
                        and not self.closing.is_set()):
                    self.motor_requests.put(start_speed)
        try:
            result = core.cmd_capture_v2(
                args, connection=self.connection, service=self._service,
                stop_requested=self.capture_stop.is_set, on_ready=ready,
                notify=lambda message: self.emit("log", message))
            self.emit("captured", dict(path=args.out, result=result))
        except Exception as exc:
            self.emit("log", f"采集失败：{exc}")
            self.emit("captured", dict(path=args.out, result=1))
        finally:
            self.phase = "idle"
            self.emit("phase", "idle")

    def _trim_idle(self):
        # 不采集时只保留解析残片与协议状态，长时间连接不累积全量遥测。
        self.parser.records.clear()
        self.parser.ascii_lines.clear()
        self.parser.v2.messages.clear()
        self.motor.reset_cursor()

    def run(self):
        try:
            if self.serial_factory is None:
                import serial
                self.serial_factory = serial.Serial
            self.connection = self.serial_factory(self.port, self.baud, timeout=.03, write_timeout=.3)
            self.emit("connected", self.port)
            while not self.closing.is_set():
                self._drain_motor_requests()
                self.motor.poll(self.parser, self._send, time.monotonic())
                # 先确认手动命令再开始协商，避免两个命令流程竞争串口回复。
                if self.motor.pending is None:
                    try:
                        job = self.jobs.get_nowait()
                    except queue.Empty:
                        job = None
                    if job is not None:
                        if not self.capture_stop.is_set():
                            self._capture(job)
                        else:
                            self.emit("captured", dict(path=job[0].out, result=1))
                        continue
                chunk = self.connection.read(4096)
                if chunk:
                    self.parser.feed(chunk, time.monotonic())
                self.motor.poll(self.parser, self._send, time.monotonic())
                self._trim_idle()
        except Exception as exc:
            self.emit("log", f"蓝牙连接异常：{exc}")
        finally:
            try:
                if self.connection is not None:
                    self.connection.close()
            except Exception as exc:
                self.emit("log", f"关闭串口失败：{exc}")
            self.emit("motor", "连接已断开，电机状态未知")
            self.emit("disconnected", None)
