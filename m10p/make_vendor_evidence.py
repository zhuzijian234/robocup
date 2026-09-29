"""Render evidence directly from captured bytes; no protocol parser dependency."""
from collections import Counter
from pathlib import Path
import hashlib
from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parent
source = root / 'reports/hardware_check_verified.bin'
data = source.read_bytes()
heads = []
pos = data.find(b'\xa5\x5a')
while pos >= 0:
    heads.append(pos)
    pos = data.find(b'\xa5\x5a', pos + 1)
pairs = list(zip(heads, heads[1:]))
counts = Counter(end-start for start, end in pairs)
tails = sum(data[end-2:end] == b'\xfa\xfb' for start, end in pairs)
lengths = sum(int.from_bytes(data[start+2:start+4], 'big') == end-start for start, end in pairs)
angles = sum((int.from_bytes(data[end+4:end+6], 'big') - int.from_bytes(data[start+4:start+6], 'big')) % 36000 == 1500 for start, end in pairs)
digest = hashlib.sha256(data).hexdigest()
im = Image.new('RGB', (1600, 1870), '#f2f5f9')
d = ImageDraw.Draw(im)
def font(size, bold=False, mono=False):
    return ImageFont.truetype('C:/Windows/Fonts/consola.ttf' if mono else 'C:/Windows/Fonts/msyhbd.ttc' if bold else 'C:/Windows/Fonts/msyh.ttc', size)
def text(x, y, value, size=28, fill='#243449', bold=False, mono=False):
    d.text((x,y), value, font=font(size,bold,mono), fill=fill)
def card(y, h):
    d.rounded_rectangle((48,y,1552,y+h), radius=18, fill='white')

text(55, 32, 'M10P 串口原始数据 · 帧长核对', 48, bold=True)
text(58, 108, '采集：2026-09-27  |  COM11 / CH9102  |  512000 bps，8N1  |  约 15 秒', 27)
text(58, 151, '观察位置：电脑经 USB 串口转接模块收到的数据；尚未直接测量雷达 TX 引脚。', 27, '#a35318')

card(214, 310)
text(78, 234, '01  不依赖长度字段：直接测量相邻 A5 5A 帧头之间的字节数', 31, bold=True)
cols = (90, 410, 775, 1120)
for x, title in zip(cols, ('实际帧头间距', '出现次数', '对应长度字段', '长度字段十进制值')):
    text(x, 296, title, 26, '#64748b')
for i, n in enumerate(sorted(counts)):
    y = 342+i*40
    for x, value in zip(cols, (f'{n} 字节', str(counts[n]), n.to_bytes(2,'big').hex(' ').upper(), str(n))):
        text(x, y, value, 28, bold=True)

card(548, 190)
text(78, 568, '02  对全部连续帧边界交叉核验', 31, bold=True)
text(90, 624, f'长度字段 = 实际间距：{lengths}/{len(pairs)}    帧头前为 FA FB：{tails}/{len(pairs)}', 29)
text(90, 673, f'相邻起始角增加 15°：{angles}/{len(pairs)}    内部包边界未见额外缺口', 29)

start, end = next((a,b) for a,b in pairs if b-a == 162)
prev = heads[heads.index(start)-1]
angle = lambda off: int.from_bytes(data[off+4:off+6], 'big') / 100
card(762, 272)
text(78, 782, '03  一个 162 字节包的实际边界（偏移从 0 开始，单位：字节）', 31, bold=True)
text(90, 838, f'本帧起点：{start}    下一帧起点：{end}    间距：{end-start}', 29)
text(90, 885, '本帧头部：', 29)
text(260, 888, data[start:start+8].hex(' ').upper(), 30, mono=True)
text(90, 931, f'00 A2 = 162；相邻包起始角：{angle(prev):g}° → {angle(start):g}° → {angle(end):g}°', 29)
text(90, 976, '边界：FA FB  |  A5 5A 00 9E ...    （本帧结束 | 下一帧开始）', 27, '#166c56')

card(1058, 490)
text(78, 1078, '04  上述 162 字节包的完整原始十六进制数据', 31, bold=True)
text(90, 1130, '左侧是包内偏移；00–01 为帧头，02–03 为长度，A0–A1 为帧尾。', 26, '#64748b')
raw = data[start:end]
for offset in range(0,len(raw),16):
    text(90, 1176+(offset//16)*31, f'{offset:04X}   '+raw[offset:offset+16].hex(' ').upper(), 26, mono=True)

card(1572, 178)
text(78, 1592, '请厂家确认', 31, bold=True)
text(90, 1646, '该型号 / 固件是否支持 156、158、160、162 字节包？测距区与时间区如何划分？', 27)
text(90, 1693, '本图仅展示捕获事实；未确认变长原因，亦不代表已排除转接模块或固件问题。', 26, '#a35318')
text(58, 1770, f'原始文件：{source.name}（{len(data)} 字节）', 24)
text(58, 1810, 'SHA256: '+digest, 21, mono=True)
out = root/'reports/M10P_厂家核对_原始帧长.png'
im.save(out)
print(out)
