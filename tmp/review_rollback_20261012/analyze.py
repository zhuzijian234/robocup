from pathlib import Path
import sys,json,math,collections,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
PC=ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机'
OUT=Path(__file__).parent;OUT.mkdir(exist_ok=True)
sys.path.insert(0,str(PC));import robocup_telemetry as core
summary=[]
for file in sorted((PC/'log').glob('v2_1012_160711_808515000000.bin')):
    tag=file.name.split('_')[2]
    if tag != '160711': continue
    p=core.parse_file(str(file));meta=p.meta
    maps={typ:{core.v2.detail_key(m):m for m in p.v2.messages if m['type']==typ} for typ in (5,8,10)}
    rows=[];first=p.records[0]['control_end_ms'] if p.records else 0
    lo=meta.get('pre_roll_s');hi=meta.get('formal_end_s');sid=meta.get('session')
    def formal(r):return lo is not None and hi is not None and lo<=r['host_t']<=hi and r['session']==sid
    for r in p.records:
        row=dict(r)
        for typ,mp in maps.items():row.update({k:v for k,v in mp.get(core.v2.detail_key(r),{}).items() if k not in ('tx_seq','host_t','t','type','schema')})
        row.update(device_t=((r['control_end_ms']-first)&0xffffffff)/1000,formal=formal(r))
        rows.append(row)
    (OUT/f'{tag}_joined.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
    forms=[r for r in rows if r['formal']]
    motors=[s for m in p.v2.messages if m['type']==6 for s in m['samples'] if formal(s)]
    health=[m for m in p.v2.messages if m['type']==2]
    rad=[m for m in p.v2.messages if m['type']==7]
    cloud=core.v2.assemble_clouds(p.v2.messages)
    cm=[]
    for key,c in sorted(cloud.items()):
        cm.append(dict(key=key,meta=c['meta'],points=[list(x) for _,x in sorted(c['points'].items())],complete=len(c['points'])==c['meta']['total_points'],conflict=c.get('conflict')))
    (OUT/f'{tag}_clouds.json').write_text(json.dumps(cm,ensure_ascii=False,indent=2),encoding='utf-8')
    changes=[]
    for a,b in zip(rows,rows[1:]):
        if b['control_seq']-a['control_seq']!=1 or a['session']!=b['session']:continue
        if abs(b['servo_pwm']-a['servo_pwm'])>=25:
            changes.append(dict(seq=[a['control_seq'],b['control_seq']],t=[a['device_t'],b['device_t']],pwm=[a['servo_pwm'],b['servo_pwm']],formal=a['formal'] or b['formal'],models=[a.get('model'),b.get('model')],source=[a.get('source'),b.get('source')],ref=[a.get('ref_y'),b.get('ref_y')],target=[a.get('target_x'),b.get('target_x')],near=[a.get('near_a'),b.get('near_a')],far=[a.get('far_a'),b.get('far_a')],p=[a.get('pd_p'),b.get('pd_p')],d=[a.get('pd_d'),b.get('pd_d')],gate=[a.get('gate_reason'),b.get('gate_reason')],geom=[a.get('geometry_reason'),b.get('geometry_reason')]))
    s=dict(tag=tag,file=str(file),sha256=hashlib.sha256(file.read_bytes()).hexdigest(),meta=meta,first_control_ms=first,all_frames=len(rows),formal_frames=len(forms),bad_frames=p.bad_frames,tx_gaps=p.v2.missing,sessions=dict(collections.Counter(r['session'] for r in rows)),formal_actions=dict(collections.Counter(r['action_reason'] for r in forms)),formal_gate=dict(collections.Counter(r.get('gate_reason') for r in forms)),formal_geom=dict(collections.Counter(r.get('geometry_reason') for r in forms)),formal_models=dict(collections.Counter(r.get('model') for r in forms)),formal_source=dict(collections.Counter(r.get('source') for r in forms)),formal_detail_missing=sum('command_applied' not in r for r in forms),formal_extra_missing=sum('target_x' not in r for r in forms),formal_pwm_range=[min((r['servo_pwm'] for r in forms),default=None),max((r['servo_pwm'] for r in forms),default=None)],motor_count=len(motors),motor_speed_range=[min((r['speed_float'] for r in motors),default=None),max((r['speed_float'] for r in motors),default=None)],motor_target_values=sorted(set(round(r['target_float'],2) for r in motors)),health=health,radar=rad,clouds=[{k:v for k,v in c.items() if k!='points'} for c in cm],changes=changes)
    summary.append(s)
    print(tag,'frames',len(rows),'formal',len(forms),'bad',p.bad_frames,'gaps',p.v2.missing,'actions',s['formal_actions'],'gate',s['formal_gate'],'clouds',len(cm),sum(c['complete'] for c in cm))
    fig,ax=plt.subplots(5,1,figsize=(14,11),sharex=True)
    for a,cols in zip(ax,[['servo_pwm','candidate_pwm'],['pd_p','pd_d'],['target_x','ref_y'],['near_a','far_a'],['Speed_now','Speed_mubiao']]):
        for col in cols:
            def val(r):
                if col in ('target_x','ref_y','near_a','far_a','pd_p','pd_d') and r.get('geometry_reason')!=0:return math.nan
                if col=='far_a' and not r.get('far_valid'):return math.nan
                return r.get(col,math.nan)
            a.plot([r['device_t'] for r in rows],[val(r) for r in rows],label=col,marker='.',markersize=2,lw=.9)
        if forms:a.axvspan(forms[0]['device_t'],forms[-1]['device_t'],color='green',alpha=.1)
        a.legend(loc='best',fontsize=8);a.grid(alpha=.3)
    ax[0].axhline(1445,color='gray',ls=':');ax[0].set_ylim(1150,1750)
    ax[-1].set_xlim(0,min(rows[-1]['device_t'],forms[-1]['device_t']+1.5) if forms else rows[-1]['device_t'])
    ax[-1].set_xlabel('Device seconds from first CONTROL; NOT video time')
    fig.suptitle(tag+' | green: formal host receive window | gaps: missing/invalid geometry')
    fig.tight_layout();fig.savefig(OUT/f'{tag}_trace.png');plt.close(fig)
    fig,ax=plt.subplots(2,4,figsize=(16,12),sharex=True,sharey=True)
    byseq={r['control_seq']:r for r in rows}
    for a,c in zip(ax.flat,cm):
        pts=np.array(c['points']);m=c['meta'];r=byseq.get(m['control_seq'],{})
        if len(pts):a.scatter(pts[:,0],pts[:,1],s=3,c='gray')
        a.plot([-130,130],[100,100],c='black');a.plot([-130,130],[-205,-205],c='black')
        a.scatter([0],[0],marker='+',c='black')
        for i,side in enumerate(('left_near','right_near','left_far','right_far')):
            if 'fit_rejected' in r and not ((r['fit_rejected']>>(8*i))&255):
                yy=np.linspace(r[side+'_min_y'],r[side+'_max_y'],40);a.plot(r[side+'_a']*yy+r[side+'_b'],yy,label=side,lw=1)
        if r.get('geometry_reason')==0:
            a.scatter([r['target_x']],[r['ref_y']],marker='x',s=60,c='green');a.plot([0,r['target_x']],[0,r['ref_y']],c='green',ls=':',lw=.7)
        a.set_title(f"seq {m['control_seq']} / geom {r.get('geometry_reason','?')}\n{len(c['points'])}/{m['total_points']} pts flag {m['trigger_flags']}")
        a.set_aspect('equal');a.set_xlim(-1000,1000);a.set_ylim(-250,1350);a.grid(alpha=.3)
    fig.suptitle(tag+' | x right,y forward | black: axles at +100/-205mm, width260mm (not swept envelope)')
    fig.tight_layout();fig.savefig(OUT/f'{tag}_cloud.png');plt.close(fig)
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
