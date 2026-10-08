# -*- coding: utf-8 -*-
"""Extract text from M10P PDFs with Chinese preserved."""
import io
import os
import sys

from pypdf import PdfReader

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
OUT = os.path.join(BASE, "_pdf_text")
os.makedirs(OUT, exist_ok=True)

PDFS = [
    "M10P_20K_数据输出格式.pdf",
    "STM32读取M10P雷达数据说明.pdf",
    "M10P_20KHz_用户手册.pdf",
    "M10P雷达测试关键结论完整记录_20260929.pdf",
    "镭神串口雷达Python数据读取应用手册.pdf",
]

for name in PDFS:
    path = os.path.join(BASE, name)
    stem = os.path.splitext(name)[0]
    dst = os.path.join(OUT, stem + ".txt")
    print("=" * 70)
    print("FILE:", name)
    try:
        reader = PdfReader(path)
        n = len(reader.pages)
        print("pages:", n)
        chunks = []
        for i, page in enumerate(reader.pages):
            try:
                t = page.extract_text() or ""
            except Exception as e:  # noqa: BLE001
                t = "<<EXTRACT ERROR page %d: %r>>" % (i + 1, e)
            chunks.append("\n\n===== PAGE %d / %d =====\n" % (i + 1, n) + t)
        text = "".join(chunks)
        with io.open(dst, "w", encoding="utf-8") as f:
            f.write(text)
        print("chars:", len(text), "->", dst)
    except Exception as e:  # noqa: BLE001
        print("FAILED:", repr(e))
