"""按已观察到的阶段加密抽帧，不添加未经验证的遥测时间。"""
from pathlib import Path
import json
import cv2
from PIL import Image,ImageDraw,ImageFont
OUT=Path(__file__).resolve().parent
info={x['tag']:x for x in json.loads((OUT/'index.json').read_text(encoding='utf-8'))}
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',24)
windows={'V04':[(0.5,1.2,'start'),(3.8,5.2,'entry_contact')],
         'V05':[(2.1,2.8,'start'),(4.6,6.0,'entry_contact')],
         'V06':[(0,0.7,'start'),(2.6,4.0,'entry_contact')],
         'V01':[(0,0.7,'start'),(4.8,6.2,'s_change')],
         'V02':[(0,0.7,'start'),(7.2,8.6,'entry_contact')],
         'V03':[(0,0.7,'start'),(7.0,8.4,'entry_contact')]}
for tag,ws in windows.items():
    cap=cv2.VideoCapture(info[tag]['source'])
    for a,b,name in ws:
        canvas=Image.new('RGB',(4*360,2*670),(20,20,20));draw=ImageDraw.Draw(canvas)
        for slot in range(8):
            t=a+(b-a)*slot/7;idx=round(t*30)
            cap.set(cv2.CAP_PROP_POS_FRAMES,idx);ok,frame=cap.read()
            if not ok:continue
            im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im.thumbnail((360,640))
            x=slot%4*360;y=slot//4*670
            canvas.paste(im,(x,y+30));draw.text((x+3,y),f'{tag} f={idx} t={idx/30:.3f}',font=font,fill='white')
            im2=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im2.save(OUT/tag/f'{name}_{idx:04d}.jpg',quality=96)
        canvas.save(OUT/tag/f'{name}.jpg',quality=94)
    cap.release()
