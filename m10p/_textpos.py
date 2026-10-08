# -*- coding: utf-8 -*-
"""Dump per-character boxes of the user-manual parameter page (UTF-8 file)."""
import io
import os

import pypdfium2 as pdfium

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
doc = pdfium.PdfDocument(os.path.join(BASE, "M10P_20KHz_用户手册.pdf"))
out = io.StringIO()

for pageno in (8,):
    page = doc[pageno - 1]
    tp = page.get_textpage()
    n = tp.count_chars()
    out.write("=== PDF page %d, %d chars, size %s ===\n" % (pageno, n, page.get_size()))
    rows = []
    for i in range(n):
        ch = tp.get_text_range(i, 1)
        l, b, r, t = tp.get_charbox(i)
        rows.append((round(t, 0), round(l, 1), ch))
    # cluster rows by y within 3 units
    rows.sort(key=lambda x: (-x[0], x[1]))
    cur = None
    for y, x, ch in rows:
        if cur is None or abs(cur[0] - y) > 3:
            if cur is not None:
                out.write("y=%7.1f | %s\n" % (cur[0], cur[1]))
            cur = (y, "")
        cur = (cur[0], cur[1] + ch)
    if cur is not None:
        out.write("y=%7.1f | %s\n" % (cur[0], cur[1]))

doc.close()
with io.open(os.path.join(BASE, "_textpos.txt"), "w", encoding="utf-8") as f:
    f.write(out.getvalue())
print("written ->", os.path.join(BASE, "_textpos.txt"))
