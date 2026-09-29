"""Vendor-facing packet images, always sourced from saved raw bytes."""
from pathlib import Path
from collections import defaultdict
import hashlib
import json
import zipfile
from PIL import Image, ImageDraw, ImageFont

root=Path(__file__).resolve().parent
source=root/'reports/hardware_check_verified.bin'
b=source.read_bytes()
out=root/'reports/厂家原始包样本'
out.mkdir(exist_ok=True)
digest=hashlib.sha256(b).hexdigest()
heads=[]
i=b.find(b'\xa5\x5a')
while i>=0:
    heads.append(i)
    i=b.find(b'\xa5\x5a',i+1)
groups=defaultdict(list)
for index,(start,end) in enumerate(zip(heads,heads[1:])):
    n=end-start
    assert int.from_bytes(b[start+2:start+4],'big')==n
    assert b[end-2:end]==b'\xfa\xfb'
    groups[n].append((index,start,end))

def font(size,bold=False,mono=False):
    return ImageFont.truetype('C:/Windows/Fonts/consola.ttf' if mono else 'C:/Windows/Fonts/msyhbd.ttc' if bold else 'C:/Windows/Fonts/msyh.ttc',size)
def label(draw,xy,s,size=27,color='#243449',bold=False,mono=False):
    draw.text(xy,s,font=font(size,bold,mono),fill=color)

manifest={'source':source.name,'sha256':digest,'samples':[]}
alltext=['M10P raw packet samples','Source: '+source.name,'SHA256: '+digest,
         'Offsets are zero-based; captured through CH9102 at 512000 baud / 8N1.',
         '2026-09-27 capture. Actual firmware layout remains unconfirmed.','']
for length in (156,158,160,162):
    available=groups[length]
    selected=[available[0],available[len(available)//2]]
    assert selected[0]!=selected[1]
    im=Image.new('RGB',(1600,1630),'#f2f5f9'); d=ImageDraw.Draw(im)
    label(d,(55,28),f'M10P · {length} 字节原始包 / 两个独立样本',43,bold=True)
    label(d,(58,95),'2026-09-27 采集 | COM11 / CH9102 | 512000 bps，8N1',27)
    label(d,(58,137),'电脑接收的原始数据；按相邻帧头间距计数，并核对长度字段和 FA FB 帧尾。',26)
    for sample,(index,start,end) in enumerate(selected):
        top=195+sample*660
        raw=b[start:end]
        angle=int.from_bytes(raw[4:6],'big')/100
        ticks=int.from_bytes(raw[6:8],'big')
        d.rounded_rectangle((48,top,1552,top+632),radius=16,fill='white')
        label(d,(78,top+20),f'样本 {sample+1} | 文件偏移 {start} → {end}，间距 {end-start} 字节',31,bold=True)
        label(d,(78,top+70),f'长度字段 {raw[2:4].hex(" ").upper()} = {length}；起始角 {angle:g}°；转速 {2500000/ticks:.2f} rpm',27)
        label(d,(78,top+113),'完整原始包（HEX）；左侧为包内十六进制偏移，从 0000 开始：',25,color='#64748b')
        for row in range((length+15)//16):
            off=row*16; y=top+160+row*32
            label(d,(86,y),f'{off:04X}',27,color='#64748b',mono=True)
            for col,v in enumerate(raw[off:off+16]):
                idx=off+col; color='#243449'
                if idx<2: color='#006e59'
                elif idx<4: color='#b55105'
                elif idx>=length-2: color='#006e59'
                label(d,(208+col*66,y),f'{v:02X}',27,color=color,mono=True)
        tail=raw[-2:].hex(' ').upper(); nxt=b[end:end+8].hex(' ').upper()
        label(d,(78,top+532),'帧尾 → 下一帧头：',25)
        label(d,(360,top+535),tail+' | '+nxt,26,color='#006e59',mono=True)
        next_angle=int.from_bytes(b[end+4:end+6],'big')/100
        label(d,(78,top+580),f'下一包起始角 {next_angle:g}°；模 360° 后角度增量 {(next_angle-angle)%360:g}°。',25)
        item=dict(length=length,sample=sample+1,offset=start,next_offset=end,angle_deg=angle,rpm=2500000/ticks,hex=raw.hex(' ').upper())
        manifest['samples'].append(item)
        alltext.extend([f'Length={length}; sample={sample+1}; offset={start}; next_offset={end}; angle={angle}',item['hex'],''])
        (out/f'packet_{length}_{sample+1}_offset_{start}.bin').write_bytes(raw)
    label(d,(58,1516),'源文件：'+source.name+' | 颜色：橙色为长度字段，绿色为帧头 / 帧尾。',24)
    label(d,(58,1562),'SHA256: '+digest,21,mono=True)
    im.save(out/f'M10P_{length}字节_两个原始包.png')

# These lengths have report counts only, so do not invent packet bytes.
im=Image.new('RGB',(1600,850),'#f2f5f9');d=ImageDraw.Draw(im)
label(d,(55,32),'M10P · 164 / 166 字节：目前只有统计记录',42,bold=True)
label(d,(58,111),'以下来自 2026-09-28 四次 JSON 报告，并非原始十六进制包截图。',28,color='#a35318')
d.rounded_rectangle((48,185,1552,578),radius=16,fill='white')
for x,t in ((90,'测试开始时间'),(640,'164 字节包数'),(1080,'166 字节包数')):
    label(d,(x,210),t,29,bold=True)
total164=total166=0
for j,p in enumerate(sorted((root/'reports').glob('20260928_*.json'))):
    r=json.loads(p.read_text(encoding='utf-8')); t=p.stem.split('_')[1]
    a=r['frame_lengths'].get('164',0);c=r['frame_lengths'].get('166',0)
    total164+=a;total166+=c
    for x,v in ((90,f'{t[:2]}:{t[2:4]}:{t[4:]}'),(640,str(a)),(1080,str(c))):
        label(d,(x,278+52*j),v,30)
for x,v in ((90,'合计'),(640,str(total164)),(1080,str(total166))):
    label(d,(x,505),v,31,bold=True)
label(d,(58,620),'这四次测试未保存原始 .bin，无法还原长度字段、距离值或帧尾原文。',28)
label(d,(58,673),'需要重新连接雷达并保存原始数据后，才能生成这两种包长的真实样本图。',28)
label(d,(58,748),'python check_m10p.py --port COM11 --seconds 30 --raw reports/retest_30s.bin',24,mono=True)
im.save(out/'M10P_164和166字节_仅统计无原始包.png')
(out/'原始包十六进制.txt').write_text('\n'.join(alltext),encoding='utf-8')
(out/'samples.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
archive=root/'reports/M10P_厂家原始包样本.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
    for p in sorted(out.iterdir()):
        if p.is_file():z.write(p,p.name)
print(str(archive))
print('8 complete samples; 4 raw-byte images; 1 counts-only image.')
