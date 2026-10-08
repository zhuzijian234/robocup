# -*- coding: utf-8 -*-
"""Render specific PDF pages to PNG for visual inspection."""
import os

import pypdfium2 as pdfium

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
OUT = os.path.join(BASE, "_pdf_page")
os.makedirs(OUT, exist_ok=True)

TARGETS = [
    ("STM32读取M10P雷达数据说明.pdf", [3, 5, 6, 7]),
    ("M10P_20KHz_用户手册.pdf", [10, 11, 17]),
    ("M10P_20K_数据输出格式.pdf", [3, 4]),
]

for name, pages in TARGETS:
    path = os.path.join(BASE, name)
    stem = os.path.splitext(name)[0]
    doc = pdfium.PdfDocument(path)
    print("=" * 60)
    print("FILE:", name, "pages:", len(doc))
    for pno in pages:
        if pno > len(doc):
            continue
        page = doc[pno - 1]
        # ~150 dpi: scale = 150/72
        bitmap = page.render(scale=150 / 72)
        pil = bitmap.to_pil()
        dst = os.path.join(OUT, "%s_p%02d.png" % (stem[:24], pno))
        pil.save(dst)
        print("  page %d -> %s  %s" % (pno, os.path.basename(dst), pil.size))
    doc.close()
