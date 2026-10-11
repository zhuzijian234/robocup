from pathlib import Path
import cv2,json,math,hashlib,re
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent;OUT.mkdir(exist_ok=True)
records=[]
for path in sorted((ROOT/'调试视频').glob('*.mp4')):
    tag=(re.search(r'v2_1011_(\d+)',path.name).group(1) if 'v2_' in path.name else 'layout')
    cap=cv2.VideoCapture(str(path));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width=int(cap.get(3));height=int(cap.get(4));duration=n/fps
    rec=dict(tag=tag,file=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),fps=fps,frames=n,width=width,height=height,duration_s=duration)
    records.append(rec);print(rec)
    step=2.0 if tag=='layout' else .4
    times=[min(i*step,(n-1)/fps) for i in range(math.ceil(duration/step))]
    images=[]
    for sec in times:
        cap.set(cv2.CAP_PROP_POS_MSEC,sec*1000);ok,frame=cap.read()
        if not ok:continue
        im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));im.thumbnail((400,320))
        tile=Image.new('RGB',(400,350),'white');tile.paste(im,((400-im.width)//2,25));ImageDraw.Draw(tile).text((10,5),f'{tag} {sec:.2f}s',fill='black');images.append(tile)
    for start in range(0,len(images),24):
        subset=images[start:start+24];sheet=Image.new('RGB',(400*4,350*math.ceil(len(subset)/4)),(225,225,225))
        for k,im in enumerate(subset):sheet.paste(im,((k%4)*400,(k//4)*350))
        sheet.save(OUT/f'{tag}_sheet_{start//24}.jpg',quality=92)
    cap.release()
(OUT/'video_metadata.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
