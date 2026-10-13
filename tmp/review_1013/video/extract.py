"""只读取原始录像，完整解码验证并按原视频时间生成图像索引。"""
from pathlib import Path
import cv2
from PIL import Image, ImageDraw, ImageFont
import hashlib
import json
import struct

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT / '调试视频' / '10.13'
FONT = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 22)

def atoms(data, start=0, end=None, route=''):
    end = len(data) if end is None else end
    result=[]
    pos=start
    while pos+8<=end:
        size,kind=struct.unpack_from('>I4s',data,pos)
        head=8
        if size==1 and pos+16<=end:
            size=struct.unpack_from('>Q',data,pos+8)[0]; head=16
        if size==0: size=end-pos
        if size<head or pos+size>end: break
        name=kind.decode('latin1'); full=route+'/'+name
        if name in ('moov','trak','mdia','udta'):
            result.extend(atoms(data,pos+head,pos+size,full))
        elif name in ('mvhd','mdhd'):
            payload=data[pos+head:pos+size]
            version=payload[0]
            if version==0:
                created,modified,timescale,duration=struct.unpack_from('>IIII',payload,4)
            else:
                created,modified,timescale,duration=struct.unpack_from('>QQIQ',payload,4)
            result.append(dict(atom=full,created_1904=created,modified_1904=modified,timescale=timescale,duration=duration))
        pos+=size
    return result

rows=[]
for number,path in enumerate(sorted(SOURCE.glob('*.mp4')),1):
    tag=f'V{number:02d}'
    dest=OUT/tag; dest.mkdir(parents=True,exist_ok=True)
    cap=cv2.VideoCapture(str(path))
    fps=cap.get(cv2.CAP_PROP_FPS); declared=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frames=[]
    while True:
        ok,frame=cap.read()
        if not ok: break
        frames.append(frame)
    cap.release()
    chosen=sorted(set(round(i*(len(frames)-1)/23) for i in range(24)))
    record=dict(tag=tag,name=path.name,source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),fps=fps,declared_frames=declared,decoded_frames=len(frames),last_frame_s=(len(frames)-1)/fps,duration_s=len(frames)/fps,width=frames[0].shape[1],height=frames[0].shape[0],stat_mtime=path.stat().st_mtime,mp4_metadata=atoms(path.read_bytes()),selected=[])
    # 单帧完整保存，拼图采用等比例缩小，无透视修正。
    for part in range(2):
        canvas=Image.new('RGB',(1920,3*390),(20,20,20)); draw=ImageDraw.Draw(canvas)
        for slot,idx in enumerate(chosen[part*12:(part+1)*12]):
            im=Image.fromarray(cv2.cvtColor(frames[idx],cv2.COLOR_BGR2RGB))
            file=dest/f'frame_{idx:04d}_{idx/fps:06.3f}s.jpg'; im.save(file,quality=95)
            im.thumbnail((480,350))
            x=(slot%4)*480; y=(slot//4)*390
            canvas.paste(im,(x+(480-im.width)//2,y+40))
            draw.text((x+8,y+5),f'{tag}  f={idx}  t={idx/fps:.3f} s',font=FONT,fill='white')
            record['selected'].append(dict(frame=idx,time_s=idx/fps,path=str(file)))
        canvas.save(dest/f'overview_{part}.jpg',quality=94)
    rows.append(record)
    print(tag,path.name,len(frames),fps,frames[0].shape)
(OUT/'index.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')

