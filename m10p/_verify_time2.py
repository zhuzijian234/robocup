# -*- coding: utf-8 -*-
"""Final characterisation of the two 16-bit values in the TIME region."""
import collections
import io
import os
import struct
import statistics

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
REP = os.path.join(BASE, "reports")
out = io.StringIO()


def w(s=""):
    out.write(s + "\n")
    print(s)


for label, fn, dur in [("A1", "hardware_check_verified.bin", 15.031),
                       ("A10", "latest_capture.bin", 30.031)]:
    blob = open(os.path.join(REP, fn), "rb").read()
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
        slots = struct.unpack(">%dH" % ((L - 20) // 2), raw[8:L - 12])
        w1 = struct.unpack(">H", raw[L - 6:L - 4])[0]
        w2 = struct.unpack(">H", raw[L - 4:L - 2])[0]
        frames.append((L, angle, speed, w1, w2, slots))
        pos = blob.find(b"\xa5\x5a", pos + L)

    w("=" * 74)
    w("%s  frames=%d  capture=%.3f s" % (label, len(frames), dur))
    w1s = [f[3] for f in frames]
    w2s = [f[4] for f in frames]
    # count wraps treating each value as mod-1000
    def wrapcount(vals):
        return sum(1 for i in range(1, len(vals)) if (vals[i] - vals[i - 1]) % 1000 > 500)
    w("W1 wraps (mod 1000): %d -> %.3f wraps/s => unit = %.4f ms"
      % (wrapcount(w1s), wrapcount(w1s) / dur, dur * 1000 / max(wrapcount(w1s), 1)))
    w("W2 wraps (mod 1000): %d -> %.3f wraps/s => unit = %.4f us"
      % (wrapcount(w2s), wrapcount(w2s) / dur, dur * 1e6 / max(wrapcount(w2s), 1)))
    w("W1 range %d..%d   W2 range %d..%d" % (min(w1s), max(w1s), min(w2s), max(w2s)))

    # correlation of W2 with frame length
    bylen = collections.defaultdict(list)
    for L, a, s, x, y, sl in frames:
        bylen[L].append(y)
    for L in sorted(bylen):
        v = bylen[L]
        w("   len=%d n=%4d  W2 mean=%.1f median=%.1f min=%d max=%d"
          % (L, len(v), statistics.mean(v), statistics.median(v), min(v), max(v)))

    # is W2 a function of the number of zero-distance slots?
    zc = [(sum(1 for s in f[5] if (s & 0x7FFF) == 0), f[4]) for f in frames]
    buckets = collections.defaultdict(list)
    for z, y in zc:
        buckets[min(z // 5, 20)].append(y)
    w("   W2 vs count of zero-distance slots (bucketed by 5):")
    for b in sorted(buckets):
        w("      zeros %2d-%2d: n=%4d W2 mean=%.1f" % (b * 5, b * 5 + 4, len(buckets[b]),
                                                       statistics.mean(buckets[b])))
    # W2 delta vs frame period
    d2 = [((w2s[i] - w2s[i - 1]) % 1000) for i in range(1, len(w2s))]
    w("   W2 delta mod1000: mean=%.2f stdev=%.2f min=%d max=%d"
      % (statistics.mean(d2), statistics.pstdev(d2), min(d2), max(d2)))
    d1 = [((w1s[i] - w1s[i - 1]) % 1000) for i in range(1, len(w1s))]
    w("   W1 delta mod1000: mean=%.2f stdev=%.2f min=%d max=%d"
      % (statistics.mean(d1), statistics.pstdev(d1), min(d1), max(d1)))
    # slots*2 bytes vs frame length sanity
    w("   slot count == (L-20)/2 for all frames: %s"
      % all(len(f[5]) == (f[0] - 20) // 2 for f in frames))

with io.open(os.path.join(BASE, "_verify_time2.txt"), "w", encoding="utf-8") as f:
    f.write(out.getvalue())
print("\nwritten -> _verify_time2.txt")
