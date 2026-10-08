# -*- coding: utf-8 -*-
"""Deep-dive on the 10-byte TIME region of the M10P packet."""
import collections
import io
import os
import struct

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
REP = os.path.join(BASE, "reports")

FILES = [
    ("A1 hardware_check_verified.bin", os.path.join(REP, "hardware_check_verified.bin"), 15.031),
    ("A10 latest_capture.bin", os.path.join(REP, "latest_capture.bin"), 30.031),
]

out = io.StringIO()


def w(s=""):
    out.write(s + "\n")
    print(s)


for label, path, dur in FILES:
    blob = open(path, "rb").read()
    w("=" * 76)
    w("%s  declared capture duration %.3f s" % (label, dur))
    frames = []
    pos = blob.find(b"\xa5\x5a")
    while pos >= 0 and pos + 4 <= len(blob):
        L = struct.unpack(">H", blob[pos + 2:pos + 4])[0]
        if L < 22 or L > 512 or L % 2 or pos + L > len(blob):
            break
        raw = blob[pos:pos + L]
        if raw[-2:] != b"\xfa\xfb":
            break
        angle, speed = struct.unpack(">HH", raw[4:8])
        frames.append((L, angle, speed, raw[L - 12:L - 2]))
        pos = blob.find(b"\xa5\x5a", pos + L)
    w("frames parsed: %d" % len(frames))

    # byte-wise variability of the 10-byte region
    w("per-offset byte statistics inside the 10-byte region (offset 0 = L-12):")
    for k in range(10):
        col = [f[3][k] for f in frames]
        zero_frac = sum(1 for v in col if v == 0) / len(col)
        w("   off %2d (abs L%+d): min=%3d max=%3d zero_frac=%.3f"
          % (k, -12 + k, min(col), max(col), zero_frac))

    w1 = [struct.unpack(">H", f[3][6:8])[0] for f in frames]
    w2 = [struct.unpack(">H", f[3][8:10])[0] for f in frames]
    dw1 = collections.Counter((w1[i + 1] - w1[i]) for i in range(len(w1) - 1))
    w("word @L-6..L-5: first=%d last=%d span=%d  ; /1000 = %.3f s (capture %.3f s)"
      % (w1[0], w1[-1], w1[-1] - w1[0], (w1[-1] - w1[0]) / 1000.0, dur))
    w("   delta histogram (top 10): %s" % dw1.most_common(10))
    w("word @L-4..L-3: first=%d last=%d min=%d max=%d" % (w2[0], w2[-1], min(w2), max(w2)))
    w("   delta histogram (top 10): %s"
      % collections.Counter((w2[i + 1] - w2[i]) for i in range(len(w2) - 1)).most_common(10))
    # does w1 relate to frame index linearly?
    # expected ms per frame = 1000/(12*24) = 3.4722
    w("   frames/revolution check: angle delta mode = %s"
      % collections.Counter((frames[i + 1][1] - frames[i][1]) % 36000
                            for i in range(len(frames) - 1)).most_common(3))
    w("   implied ms per frame from w1: %.4f ; from rpm: %.4f"
      % ((w1[-1] - w1[0]) / max(len(w1) - 1, 1),
         60000.0 / (24 * (sum(f[2] for f in frames) / len(frames)))))
    # Is w2 = fractional / sub-ms or independent?
    pair = collections.Counter((w1[i] - w1[i - 1], w2[i] - w2[i - 1]) for i in range(1, len(w1)))
    w("   (dw1, dw2) joint top 12: %s" % pair.most_common(12))

with io.open(os.path.join(BASE, "_verify_time.txt"), "w", encoding="utf-8") as f:
    f.write(out.getvalue())
print("\nwritten -> _verify_time.txt")
