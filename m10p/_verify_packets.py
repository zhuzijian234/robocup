# -*- coding: utf-8 -*-
"""Independent structural verification of M10P raw captures (vendor bytes)."""
import collections
import hashlib
import io
import os
import struct

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
REP = os.path.join(BASE, "reports")

FILES = [
    ("A1 hardware_check_verified.bin", os.path.join(REP, "hardware_check_verified.bin")),
    ("A10 latest_capture.bin", os.path.join(REP, "latest_capture.bin")),
]

out = io.StringIO()


def w(s=""):
    out.write(s + "\n")
    print(s)


for label, path in FILES:
    if not os.path.exists(path):
        w("MISSING %s" % path)
        continue
    blob = open(path, "rb").read()
    w("=" * 72)
    w("%s  %d bytes  sha256=%s" % (label, len(blob), hashlib.sha256(blob).hexdigest()))

    # --- independent frame walk: 1) by header search, 2) by length field ---
    hdr_offsets = []
    i = blob.find(b"\xa5\x5a")
    while i >= 0:
        hdr_offsets.append(i)
        i = blob.find(b"\xa5\x5a", i + 1)
    w("literal A5 5A occurrences: %d" % len(hdr_offsets))

    lens = collections.Counter()
    lenfield_mismatch = 0
    tail_bad = 0
    slotcounter = collections.Counter()
    nonzero_frame_deltas = collections.Counter()
    gaps = collections.Counter()
    angle_deltas = collections.Counter()
    speed_vals = []
    ffff_total = 0
    hi_ref_total = 0
    zero_total = 0
    point_total = 0
    time_field_samples = []
    frames = 0
    prev = None
    pos = blob.find(b"\xa5\x5a")
    first_start = pos
    while pos >= 0 and pos + 4 <= len(blob):
        L = struct.unpack(">H", blob[pos + 2:pos + 4])[0]
        if L < 22 or L > 512 or L % 2:
            nxt = blob.find(b"\xa5\x5a", pos + 1)
            gaps["badlen"] += 1
            pos = nxt
            continue
        if pos + L > len(blob):
            gaps["tail_incomplete"] += 1
            break
        raw = blob[pos:pos + L]
        if raw[-2:] != b"\xfa\xfb":
            tail_bad += 1
            pos = blob.find(b"\xa5\x5a", pos + 1)
            continue
        angle, speed = struct.unpack(">HH", raw[4:8])
        # header search count between this header and next header must equal L
        nxt = blob.find(b"\xa5\x5a", pos + 1)
        if nxt >= 0 and nxt - pos != L:
            lenfield_mismatch += 1
        if prev is not None:
            angle_deltas[(angle - prev) % 36000] += 1
        prev = angle
        lens[L] += 1
        speed_vals.append(speed)
        slots = struct.unpack(">%dH" % ((L - 20) // 2), raw[8:L - 12])
        slotcounter[(L - 20) // 2] += 1
        point_total += len(slots)
        for s in slots:
            if s == 0xFFFF:
                ffff_total += 1
            else:
                if s & 0x8000:
                    hi_ref_total += 1
                if (s & 0x7FFF) == 0:
                    zero_total += 1
        time_field_samples.append((L, angle, raw[L - 12:L - 2].hex(" ")))
        frames += 1
        pos = nxt if nxt is not None else pos + L

    w("complete frames (header + length field + FA FB tail): %d" % frames)
    w("length-field vs next-header-offset mismatches: %d" % lenfield_mismatch)
    w("bad FA FB tails encountered: %d" % tail_bad)
    w("non-header bytes before first frame: %d" % first_start)
    w("frame-length histogram: %s" % dict(sorted(lens.items())))
    w("distance-slot-count histogram: %s" % dict(sorted(slotcounter.items())))
    w("angle delta histogram (0.01 deg): %s" % dict(sorted(angle_deltas.items())))
    w("total distance slots: %d  (FFFF=%d, bit15 set=%d, value==0=%d)"
      % (point_total, ffff_total, hi_ref_total, zero_total))
    if speed_vals:
        w("speed raw: min=%d max=%d mean=%.2f -> rpm min=%.2f max=%.2f mean=%.2f"
          % (min(speed_vals), max(speed_vals), sum(speed_vals) / len(speed_vals),
             2500000 / max(speed_vals), 2500000 / min(speed_vals),
             2500000 / (sum(speed_vals) / len(speed_vals))))
    # reserved 10-byte region analysis
    w("--- last 12 bytes (10-byte TIME + FA FB) of first 6 frames ---")
    for L, a, t in time_field_samples[:6]:
        w("   len=%d angle=%.2f  TIME= %s" % (L, a / 100.0, t))
    uniq = collections.Counter(t for _, _, t in time_field_samples)
    w("distinct TIME strings: %d over %d frames" % (len(uniq), len(time_field_samples)))
    w("most common TIME strings: %s" % uniq.most_common(5))

with io.open(os.path.join(BASE, "_verify_out.txt"), "w", encoding="utf-8") as f:
    f.write(out.getvalue())
print("\nwritten -> _verify_out.txt")
