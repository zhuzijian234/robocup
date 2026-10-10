from pathlib import Path
import sys, json, collections, math, csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
PC=ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机'
sys.path.insert(0,str(PC))
import robocup_telemetry as core
OUT=Path(__file__).parent
summaries=[]
for tag in ('180345','181529','181925','182048'):
    file=next((PC/'log').glob(f'v2_1010_{tag}_*.bin'))
    p=core.parse_file(str(file)); meta=p.meta
    control=p.records
    first_ms=control[0]['control_end_ms']
    detail={core.v2.detail_key(m):m for m in p.v2.messages if m['type']==5}
    extra={core.v2.detail_key(m):m for m in p.v2.messages if m['type']==8}
    rows=[]
    for r in control:
        key=core.v2.detail_key(r)
        rows.append({**r,**{k:v for k,v in detail.get(key,{}).items() if k not in ('host_t','t','tx_seq')},
                     **{k:v for k,v in extra.get(key,{}).items() if k not in ('host_t','t','tx_seq')},
                     'device_t':((r['control_end_ms']-first_ms)&0xffffffff)/1000,
                     'formal':meta['pre_roll_s']<=r['host_t']<=meta['formal_end_s']})
    fields=list(dict.fromkeys(k for r in rows for k in r))
    with (OUT/f'{tag}_joined.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    def runs(key, pred):
        groups=[];group=[]
        for r in rows:
            if pred(r.get(key)):
                group.append(r)
            elif group:groups.append(group);group=[]
        if group:groups.append(group)
        return [dict(start=g[0]['device_t'],end=g[-1]['device_t'],n=len(g),seq=[g[0]['control_seq'],g[-1]['control_seq']],
                     pwm=[g[0]['servo_pwm'],g[-1]['servo_pwm']], reasons=dict(collections.Counter(x.get('geometry_reason') for x in g)),
                     err=[g[0].get('road_error'),g[-1].get('road_error')],formal_n=sum(r['formal'] for r in g)) for g in groups]
    formal=[r for r in rows if r['formal']]
    motors=[s for m in p.v2.messages if m['type']==6 for s in m['samples']]
    health=[m for m in p.v2.messages if m['type']==2]
    radar=[m for m in p.v2.messages if m['type']==7]
    summary=dict(tag=tag,meta={k:meta.get(k) for k in ('pre_roll_s','formal_end_s','duration_s','error','info')},
                 first_ms=first_ms,all_frames=len(rows),formal_frames=len(formal),bad=p.bad_frames,missing=p.v2.missing,
                 actions=dict(collections.Counter(r['action_reason'] for r in formal)),
                 geom=dict(collections.Counter(r.get('geometry_reason') for r in formal)),
                 gate=dict(collections.Counter(r.get('gate_reason') for r in formal)),
                 invalid=runs('action_reason',lambda x:x==5),waits=runs('gate_reason',lambda x:x in (2,3)),
                 health_edges=[health[0],health[-1]],radar_edges=[radar[0],radar[-1]],
                 cone=dict(collections.Counter(r.get('avoid_state') for r in formal)))
    summaries.append(summary)
    fig,axs=plt.subplots(5,1,figsize=(14,11),sharex=True)
    t=[r['device_t'] for r in rows]
    for ax,cols in zip(axs,[['servo_pwm','candidate_pwm'],['road_error','pd_err'],['near_a','far_a'],['geometry_reason','gate_reason'],['Speed_now','Speed_mubiao']]):
        for col in cols:ax.plot(t,[r.get(col,math.nan) for r in rows],label=col,lw=1)
        ax.legend(loc='upper right',fontsize=8);ax.grid(alpha=.3)
    axs[0].axhline(1445,color='gray',ls=':');axs[-1].set_xlabel('Device seconds from first CONTROL (not video time)')
    fig.suptitle(tag);fig.tight_layout();fig.savefig(OUT/f'{tag}_trace.png');plt.close(fig)
    print(tag,'formal',len(formal),'bad',p.bad_frames,'gaps',p.v2.missing,'geom',summary['geom'],'gate',summary['gate'],'cone',summary['cone'])
    print('INVALID',summary['invalid']);print('WAIT',summary['waits'])
(OUT/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8')
