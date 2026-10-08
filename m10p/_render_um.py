# -*- coding: utf-8 -*-
import os
import pypdfium2 as pdfium

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
OUT = os.path.join(BASE, "_pdf_page")
doc = pdfium.PdfDocument(os.path.join(BASE, "M10P_20KHz_用户手册.pdf"))
for pno in [6, 8, 11, 13, 14]:
    page = doc[pno - 1]
    pil = page.render(scale=170 / 72).to_pil()
    dst = os.path.join(OUT, "UM_p%02d.png" % pno)
    pil.save(dst)
    print("page %d -> %s %s" % (pno, os.path.basename(dst), pil.size))
doc.close()
