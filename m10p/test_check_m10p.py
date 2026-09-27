import struct
import unittest

from check_m10p import Parser, Stats


def packet(words=None, angle=35000, ticks=3472):
    if words is None:
        words = [1000] * 70
    return (b"\xa5\x5a" + struct.pack(">HHH", 20 + len(words)*2, angle, ticks)
            + struct.pack(f">{len(words)}H", *words) + bytes(10) + b"\xfa\xfb")


class ProtocolTests(unittest.TestCase):
    def test_fragmentation_and_concatenation(self):
        p = Parser()
        stream = b"noise" + packet() + packet([2000]*69) + packet([1000]*71)
        frames = []
        for byte in stream:
            frames.extend(p.feed(bytes([byte])))
        self.assertEqual(len(frames), 3)
        self.assertEqual(p.lengths, {160: 1, 158: 1, 162: 1})
        self.assertEqual(p.discarded, 5)

    def test_invalid_reflective_and_interpolated_angles(self):
        frame = Parser().feed(packet([0xFFFF, 0x8000 | 5000, 0, 1000]))[0]
        self.assertEqual(frame[3], 1)
        self.assertEqual(frame[2], [(350, 5000, True), (355, 0, False), (0, 1000, False)])
        self.assertAlmostEqual(frame[1], 720.046, places=2)

    def test_corrupt_frames_resynchronize(self):
        p = Parser()
        bad_tail = packet()[:-2] + b"xx"
        frames = p.feed(b"\xa5\x5a\xff\xff" + bad_tail + packet(ticks=0) + packet(angle=36001) + packet())
        self.assertEqual(len(frames), 1)
        self.assertGreaterEqual(p.bad, 4)

    def test_empty_and_ffff_are_not_pass(self):
        p, s = Parser(), Stats()
        self.assertEqual(s.summary(p, 15, 0)["status"], "FAIL")
        s.add(p.feed(packet([0xFFFF]*70))[0], 0)
        self.assertEqual(s.summary(p, 15, 160)["status"], "WARN")

    def test_healthy_stream_and_dropped_frame(self):
        p, s = Parser(), Stats()
        for i in range(4320):
            s.add(p.feed(packet(angle=(i % 24)*1500))[0], i/288)
        self.assertEqual(s.summary(p, 15, 4320*160)["status"], "PASS")
        self.assertEqual(s.summary(p, 30, 4320*160)["status"], "WARN")
        s.add(p.feed(packet(angle=1500))[0], 16)
        self.assertEqual(s.jumps, 1)
        self.assertEqual(s.summary(p, 16, 4321*160)["status"], "WARN")


if __name__ == "__main__":
    unittest.main()
