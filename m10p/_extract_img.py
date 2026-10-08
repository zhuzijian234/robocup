# -*- coding: utf-8 -*-
"""Dump embedded images from a PDF page range for visual inspection."""
import os
import sys

from pypdf import PdfReader

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
OUT = os.path.join(BASE, "_pdf_img")
os.makedirs(OUT, exist_ok=True)

TARGETS = [
    ("STM32读取M10P雷达数据说明.pdf", [3, 5, 6, 7]),
    ("M10P_20KHz_用户手册.pdf", [10, 11, 17]),
    ("M10P_20K_数据输出格式.pdf", [1, 2, 3, 4]),
    ("M10P雷达测试关键结论完整记录_20260929.pdf", [5, 6]),
]

for name, pages in TARGETS:
    path = os.path.join(BASE, name)
    stem = os.path.splitext(name)[0]
    reader = PdfReader(path)
    print("=" * 60)
    print("FILE:", name, "pages:", len(reader.pages))
    for pno in pages:
        if pno > len(reader.pages):
            continue
        page = reader.pages[pno - 1]
        try:
            imgs = list(page.images)
        except Exception as e:  # noqa: BLE001
            print("  page %d: image error %r" % (pno, e))
            continue
        print("  page %d: %d image(s)" % (pno, len(imgs)))
        for i, im in enumerate(imgs):
            ext = os.path.splitext(im.name)[1] or ".png"
            dst = os.path.join(OUT, "%s_p%02d_%d%s" % (stem[:24], pno, i, ext))
            try:
                with open(dst, "wb") as f:
                    f.write(im.data)
                print("     ->", os.path.basename(dst), len(im.data), "bytes")
            except Exception as e:  # noqa: BLE001
                print("     write error", repr(e))
