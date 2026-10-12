"""原始 BIN 与 MP4 的独立复核；不使用既有 aligned.csv 推导因果。"""
from pathlib import Path
import sys,json,hashlib,collections,math
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
PC=ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机'
VID=ROOT/'调试视频/10.12'
OUT=VID/'成功过弯_AI可读_20261012/复核与控制方案'
OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(PC));import robocup_telemetry as core

def clean(x):
    if isinstance(x,float) and not math.isfinite(x):return None
    if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    return x

def save(path,obj):path.write_text(json.dumps(clean(obj),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
def stats(values):
    v=[x for x in values if x is not None and math.isfinite(x)]
    return dict(n=len(v),min=min(v),median=float(np.median(v)),max=max(v)) if v else dict(n=0)

summaries=[]
for tag in ('160711','161100','161233','161426','161746','162029','164512','164616','164757','164942','165050','165134'):
    file,=(PC/'log').glob(f'v2_1012_{tag}_*.bin')
    p=core.parse_file(str(file));meta=p.meta
    d=OUT/tag;d.mkdir(exist_ok=True)
    maps={t:{core.v2.detail_key(m):m for m in p.v2.messages if m['type']==t} for t in (5,8,10)}
    lo=meta.get('pre_roll_s');hi=meta.get('formal_end_s');sid=meta.get('session')
    def formal(r):return lo is not None and hi is not None and lo<=r['host_t']<=hi and r['session']==sid
    rows=[];start=p.records[0]['control_end_ms']
    for r in p.records:
        row=dict(r);row['formal']=formal(r);row['device_t_from_first_control']=((r['control_end_ms']-start)&0xffffffff)/1000
        for typ,mp in maps.items():
            extra=mp.get(core.v2.detail_key(r))
            if extra:row.update({k:v for k,v in extra.items() if k not in ('type','schema','tx_seq','host_t')})
        rows.append(row)
    save(d/'joined.json',rows)
    forms=[r for r in rows if r['formal']]
    motors=[s for m in p.v2.messages if m['type']==6 for s in m['samples'] if formal(s)]
    clouds=core.v2.assemble_clouds(p.v2.messages)
    save(d/'clouds.json',[dict(meta=c['meta'],points=[v for _,v in sorted(c['points'].items())],complete=len(c['points'])==c['meta']['total_points'],conflict=c.get('conflict')) for _,c in sorted(clouds.items())])
    health=[m for m in p.v2.messages if m['type']==2]
    limits=[r for r in forms if r['servo_pwm']<=1180 or r['servo_pwm']>=1710]
    jumps=[]
    for a,b in zip(forms,forms[1:]):
        if a['session']==b['session'] and a['param_revision']==b['param_revision'] and b['control_seq']==a['control_seq']+1 and abs(b['servo_pwm']-a['servo_pwm'])>=35:
            jumps.append({k:[a.get(k),b.get(k)] for k in ('control_seq','servo_pwm','target_x','ref_y','near_a','far_a','far_valid','source','pd_p','pd_d','gate_reason','geometry_reason')})
    turn=next((r for r in forms if abs(r['servo_pwm']-1445)>150),None)
    s=dict(tag=tag,bin=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),meta_sha256=meta.get('sha256'),
           all_controls=len(rows),formal_controls=len(forms),formal_seq_range=[forms[0]['control_seq'],forms[-1]['control_seq']] if forms else None,
           bad_candidates=p.bad_frames,tx_gaps=p.v2.missing,session_counts=dict(collections.Counter(r['session'] for r in rows)),
           configs=meta.get('configs'),session=sid,formal_start=lo,formal_end=hi,error=meta.get('error'),
           missing_detail=sum('pd_p' not in r for r in forms),missing_path=sum('target_x' not in r for r in forms),race_count=sum('model'in r for r in forms),
           action_counts=dict(collections.Counter(r['action_reason'] for r in forms)),geometry_counts=dict(collections.Counter(r.get('geometry_reason') for r in forms)),
           cycle_us=stats(r['control_dt_us'] for r in forms),control_seq_gaps=[(a['control_seq'],b['control_seq']) for a,b in zip(forms,forms[1:]) if b['control_seq']!=a['control_seq']+1],
           speed=stats(r['Speed_now'] for r in forms),motor_pwm=stats(r.get('motor_pwm') for r in motors),motor_example=motors[0] if motors else None,
           pd_d=stats(r.get('pd_d') for r in forms),near_limit_controls=len(limits),outer_offset_samples=[{k:r.get(k) for k in ('control_seq','outer_offset','outer_clearance','near_a','far_a','target_x','ref_y','servo_pwm')} for r in forms if 'outer_offset' in r],
           first_formal_large_command=turn,jumps=jumps,health=health,cloud_count=len(clouds),complete_cloud_count=sum(len(c['points'])==c['meta']['total_points'] and not c.get('conflict') for c in clouds.values()))
    summaries.append(s)
    fig,ax=plt.subplots(4,1,figsize=(12,9),sharex=True)
    for a,cols in zip(ax,[('servo_pwm','candidate_pwm'),('target_x','ref_y'),('near_a','far_a'),('pd_p','pd_d')]):
        for col in cols:
            values=[r.get(col,math.nan) if r.get('geometry_reason')==0 or col in ('servo_pwm','candidate_pwm') else math.nan for r in forms]
            a.plot([r['device_t_from_first_control'] for r in forms],values,'.-',lw=.8,markersize=3,label=col)
        a.legend(fontsize=8);a.grid(alpha=.3)
    ax[0].axhline(1445,c='gray',ls=':');ax[0].set_ylim(1150,1750)
    ax[-1].set_xlabel('Device time from first recorded CONTROL; NOT video time')
    fig.suptitle(tag+' formal session only; missing extension values left blank')
    fig.tight_layout();fig.savefig(d/'trace.png',dpi=110);plt.close(fig)
    print(tag,len(forms),s['action_counts'],'near_limit',len(limits),'configs',[(v.get('build'),v.get('cx'),v.get('spd')) for v in (s['configs'] or {}).values()])
save(OUT/'telemetry_audit.json',summaries)

# 14 视频统一无遥测覆盖的概览；像素时间来自解码帧，不猜测串口/视频偏移。
videos=[]
for file in sorted(VID.glob('*.mp4')):
    if not ('转弯' in file.name):continue
    match=[s['tag'] for s in summaries if s['tag'] in file.name]
    tag=match[0] if match else ('manual_left_no_log' if '左' in file.name else 'manual_right_no_log')
    d=OUT/tag;d.mkdir(exist_ok=True)
    cap=cv2.VideoCapture(str(file));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    picks=set(np.linspace(0,n-1,18).round().astype(int));cells=[];decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        if decoded in picks:
            h,w=frame.shape[:2];cell=cv2.resize(frame,(240,round(h*240/w)))
            cell=cv2.copyMakeBorder(cell,32,0,0,0,cv2.BORDER_CONSTANT,value=(255,255,255))
            cv2.putText(cell,f'f{decoded} video {decoded/fps:.2f}s',(6,22),cv2.FONT_HERSHEY_SIMPLEX,.52,(0,0,0),1)
            cells.append(cell)
            cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,94])[1].tofile(str(d/f'frame_{decoded:04}.jpg'))
        decoded+=1
    cap.release();assert decoded==n,(str(file),n,decoded)
    for j in range(0,len(cells),9):
        batch=cells[j:j+9];batch+=[np.zeros_like(cells[0])]*((-len(batch))%3)
        sheet=np.vstack([np.hstack(batch[i:i+3]) for i in range(0,len(batch),3)])
        cv2.imencode('.jpg',sheet,[cv2.IMWRITE_JPEG_QUALITY,90])[1].tofile(str(d/f'overview_{j//9}.jpg'))
    videos.append(dict(tag=tag,file=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),fps=fps,decoded_frames=decoded,last_frame_time_s=(n-1)/fps,duration_s=n/fps,alignment='unverified; no telemetry burned onto video'))
save(OUT/'video_audit.json',videos)
print('videos decoded',len(videos))
