"""10.13 原始遥测复核：不修改原始文件，扩展严格按三元键连接。"""
from pathlib import Path
import sys,json,hashlib,math,statistics,csv
from collections import Counter
ROOT=Path(__file__).resolve().parents[3]
HOST=ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机'
OUT=Path(__file__).parent
sys.path.insert(0,str(HOST))
import robocup_telemetry as core
def clean(x):
    if isinstance(x,float) and not math.isfinite(x):return None
    if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    return x
def save(path,x):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(clean(x),ensure_ascii=False,indent=2),encoding='utf-8')
def stats(a):
    a=[x for x in a if isinstance(x,(int,float)) and math.isfinite(x)]
    return {'n':len(a),'min':min(a),'median':statistics.median(a),'max':max(a)} if a else {'n':0}
def ranges(rows,key):
    out=[]
    for r in rows:
        v=r.get(key)
        if not out or out[-1]['value']!=v:
            out.append(dict(value=v,seq0=r['control_seq'],seq1=r['control_seq'],t0=r['run_device_t'],t1=r['run_device_t'],n=1))
        else:out[-1].update(seq1=r['control_seq'],t1=r['run_device_t'],n=out[-1]['n']+1)
    return out
summaries=[]
for file in sorted((HOST/'log').glob('v2_1013_*.bin')):
    tag=file.name.split('_')[2];dest=OUT/tag
    p=core.parse_file(file);meta=p.meta;sid=meta['session'];lo=meta['pre_roll_s'];hi=meta['formal_end_s']
    maps={typ:{} for typ in (5,8,10)};dups=[]
    for m in p.v2.messages:
        if m['type'] in maps:
            k=core.v2.detail_key(m)
            if k in maps[m['type']]:dups.append([m['type'],k])
            maps[m['type']][k]=m
    controls=[r for r in p.records if r['session']==sid]
    events=[m for m in p.v2.messages if m['type']==4 and m['session']==sid]
    start_event=next((m for m in events if m['event_code']==1 and m['key']==108 and m['new']>0),None)
    zero_ms=start_event['event_ms'] if start_event else controls[0]['control_end_ms']
    rows=[]
    for r in controls:
        row=dict(r);row['host_phase']='preroll' if r['host_t']<lo else 'formal' if r['host_t']<=hi else 'dump'
        row['run_device_t']=(r['control_end_ms']-zero_ms)/1000
        row['paired_types']=[]
        for typ,mp in maps.items():
            m=mp.get(core.v2.detail_key(r))
            if m:
                row['paired_types'].append(typ)
                row.update({k:v for k,v in m.items() if k not in ('type','schema','tx_seq','host_t')})
        rows.append(row)
    forms=[r for r in rows if r['host_phase']=='formal']
    clouds=core.v2.assemble_clouds(p.v2.messages)
    cloudrows=[dict(meta=c['meta'],points=[v for _,v in sorted(c['points'].items())],complete=len(c['points'])==c['meta']['total_points'],conflict=c.get('conflict')) for _,c in sorted(clouds.items())]
    acute=[dict(m,run_device_t=(m['event_ms']-zero_ms)/1000,host_phase='preroll' if m['host_t']<lo else 'formal' if m['host_t']<=hi else 'dump') for m in events if m['event_code']==3]
    save(dest/'joined.json',rows);save(dest/'clouds.json',cloudrows);save(dest/'acute.json',acute);save(dest/'meta.json',meta);save(dest/'messages.json',p.v2.messages);save(dest/'ascii.json',p.ascii_lines)
    cols='control_seq run_device_t host_t host_phase param_revision servo_pwm candidate_pwm action_reason pd_err pd_p pd_d ref_y near_a far_a target_x road_target geometry_reason gate_reason path_reason model s_preview outer_offset acute_phase acute_reason acute_direction_code acute_position_mm acute_goal_mm acute_feature_y_mm avoid_state cone_candidates avoid_offset Speed_now Speed_mubiao'.split()
    with (dest/'timeline.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');w.writeheader();w.writerows(rows)
    summary=dict(tag=tag,bin=str(file),sha_matches=True,rx_validated=True,bad_frames=p.bad_frames,tx_missing=p.v2.missing,tx_duplicates=p.v2.duplicates,extension_duplicates=dups,
        session=sid,session_control_counts=Counter(r['session'] for r in p.records),formal_bounds=[lo,hi],all_controls=len(rows),formal_controls=len(forms),
        formal_seq=[forms[0]['control_seq'],forms[-1]['control_seq']],zero_device_ms=zero_ms,start_event=start_event,
        param_events=[m for m in events if m['event_code']==1],configs=p.v2.configs,
        formal_actions=Counter(r['action_reason'] for r in forms),formal_geometry=Counter(r.get('geometry_reason') for r in forms),
        action_ranges=ranges(forms,'action_reason'),geometry_ranges=ranges(forms,'geometry_reason'),
        acute=acute,formal_acute_events=[a for a in acute if a['host_phase']=='formal'],
        pairs={typ:sum(typ in r['paired_types'] for r in forms) for typ in maps},
        phases=Counter(r.get('acute_phase') for r in forms if 10 in r['paired_types']),
        models=Counter(r.get('model') for r in forms if 10 in r['paired_types']),
        avoidance=Counter(r.get('avoid_state') for r in forms),
        control_dt_us=stats(r['control_dt_us'] for r in forms),process_us=stats(r.get('process_us') for r in forms),
        pd_d=stats(r.get('pd_d') for r in forms),pwm=stats(r['servo_pwm'] for r in forms),
        cloud_count=len(cloudrows),clouds_complete=sum(c['complete'] and not c['conflict'] for c in cloudrows),
        cloud_meta=[c['meta'] for c in cloudrows],commands=meta['commands'])
    save(dest/'summary.json',summary);summaries.append(summary)
    print(tag,'session',sid,'formal',len(forms),summary['formal_seq'],'bad',p.bad_frames,'txmiss',p.v2.missing,'actions',dict(summary['formal_actions']),'phase',dict(summary['phases']),'cloud',summary['clouds_complete'])
save(OUT/'summary.json',summaries)
