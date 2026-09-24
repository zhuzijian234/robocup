import importlib
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robocup_telemetry as core


def frame(seq=0, **values):
    payload = b"".join(struct.pack("<h", core.encode_field(n, values.get(n, 0))[0])
                       for n, _ in core.FIELD_TABLE)
    return core.HEADER + bytes([1, seq]) + payload + bytes([core.checksum(payload)])


class TelemetryTests(unittest.TestCase):
    def test_encoding(self):
        cases = [("err", 500, (5000, True, False)),
                 ("err", 650, (6000, True, True)),
                 ("Midline.k", .5, (500, True, False)),
                 ("Midline.k", 40, (30000, True, True)),
                 ("Midline.k", .0345, (35, True, False)),
                 ("Midline.k", -.0345, (-35, True, False)),
                 ("Speed_now", 1e30, (32767, True, True)),
                 ("pid_select", 1.5, (0, False, False))]
        for name, value, expected in cases:
            with self.subTest(name=name, value=value):
                self.assertEqual(core.encode_field(name, value), expected)
        for v in [math.nan, math.inf, -math.inf]:
            self.assertEqual(core.encode_field("err", v), (0, False, False))

    def test_real_time_bytes_and_split_frame(self):
        raw = b"OK kp=35\r\n" + frame(1) + frame(3)
        events = [{"offset": 0, "length": 30, "t": .25},
                  {"offset": 30, "length": len(raw)-30, "t": 1.5}]
        with tempfile.TemporaryDirectory() as d:
            p = str(Path(d)/"run.bin")
            core.save_capture(p, raw, events, 2.0)
            parsed = core.parse_file(p)
            r = core.analyze(parsed)
            self.assertEqual(r["link"]["fps"], 1)
            self.assertEqual(r["link"]["bytes_per_s"], len(raw)/2)
            self.assertEqual(parsed.dropped, 1)
            self.assertEqual(parsed.records[0]["t"], 1.5)
            core.write_report(r, str(Path(d)/"report.md"))
            Path(p).write_bytes(raw+b"X")
            with self.assertRaises(ValueError):
                core.parse_file(p)

    def test_legacy_has_no_measured_rate(self):
        p = core.StreamParser()
        p.feed(frame())
        r = core.analyze(p)
        self.assertFalse(r["measured"])
        self.assertTrue(math.isnan(r["link"]["fps"]))
        self.assertTrue(math.isnan(r["link"]["bytes_per_s"]))
        with tempfile.TemporaryDirectory() as d:
            report = Path(d)/"report.md"
            core.write_report(r, str(report))
            self.assertNotIn("7300", report.read_text(encoding="utf-8"))

    def test_target_changes(self):
        p = core.StreamParser()
        p.feed(frame(0, Speed_now=8, Speed_mubiao=8)+frame(1, Speed_now=10, Speed_mubiao=10))
        self.assertEqual(core.analyze(p)["speed"]["mean_err"], 0)

    def test_vofa_nan_does_not_become_tail(self):
        raw = struct.pack("<8f", math.nan, 1170, .035, .075, 11, 8, 8, 2160)+core.VOFA_TAIL
        rows, bad = core.parse_vofa_bytes(raw*3)
        self.assertEqual((len(rows), bad), (3, 0))
        self.assertTrue(math.isnan(rows[0][0]))
        self.assertEqual(rows[0][4], 11)

    def test_demo_end_to_end(self):
        with tempfile.TemporaryDirectory() as d:
            p = str(Path(d)/"demo.bin")
            core.make_demo_bin(p, 60)
            parsed = core.parse_file(p)
            self.assertGreater(parsed.total_frames, 500)
            self.assertEqual(parsed.dropped, 2)
            self.assertEqual(parsed.bad_frames, 1)
            r = core.analyze(parsed)
            core.write_csv(parsed, str(Path(d)/"demo.csv"))
            core.write_report(r, str(Path(d)/"demo.md"))

    def test_gui_capture_resets_session(self):
        gui = importlib.import_module("robocup_gui")
        app = gui.App.__new__(gui.App)
        app.ser, app.capturing = object(), False
        app.parser = core.StreamParser()
        app.parser.feed(frame())
        app.raw = bytearray(b"old")
        app.sp_sec = type("Value", (), {"get": lambda self: "5"})()
        app.btn_start, app.btn_diag, app.btn_stop = {}, {}, {}
        app.log = lambda s: None
        app.send = lambda s: True
        for _ in range(2):
            gui.App.start_capture(app)
            self.assertEqual(app.raw, b"")
            self.assertEqual(app.rx_events, [])
            self.assertEqual(app.parser.total_frames, 0)
            app.parser.feed(frame())
            app.raw.extend(frame())
            app.capturing = False


if __name__ == "__main__":
    unittest.main()
