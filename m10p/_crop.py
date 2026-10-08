# -*- coding: utf-8 -*-
"""Crop and enlarge the parameter-table value column to check hidden text."""
import os

import pypdfium2 as pdfium

BASE = r"D:\OneDrive\Desktop\robocup\m10p"
doc = pdfium.PdfDocument(os.path.join(BASE, "M10P_20KHz_用户手册.pdf"))
page = doc[7]
scale = 6.0
pil = page.render(scale=scale).to_pil()
W, H = pil.size
# value column of the left table: PDF x 255..320, rows y(bottom) 280..340
left = int(255 * scale)
right = int(330 * scale)
top = int((595.32 - 340) * scale)
bottom = int((595.32 - 280) * scale)
crop = pil.crop((left, top, right, bottom))
crop = crop.resize((crop.width * 2, crop.height * 2), 1)
dst = os.path.join(BASE, "_pdf_page", "UM_p08_crop_valuecol.png")
crop.save(dst)
print("saved", dst, crop.size)

# full-width strip covering rows 角度分辨率..扫描频率
left2 = int(60 * scale)
right2 = int(340 * scale)
top2 = int((595.32 - 345) * scale)
bottom2 = int((595.32 - 255) * scale)
crop2 = pil.crop((left2, top2, right2, bottom2))
dst2 = os.path.join(BASE, "_pdf_page", "UM_p08_crop_rows.png")
crop2.save(dst2)
print("saved", dst2, crop2.size)
doc.close()
