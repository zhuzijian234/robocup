"""重新解码原始BIN，只在tmp输出；扩展严格按三元组连接，不填充缺失RACE。"""
from pathlib import Path
import sys, json, hashlib, math, statistics
from collections import Counter, defaultdict

ROOT = Path(__file__).resolve().parents[2]
PC = ROOT / 'robocup-新建文件夹队/robocup-新建文件夹/上位机'
OUT = Path(__file__).parent / 'telemetry'
sys.path.insert(0, str(PC))
import robocup_telemetry as core

def clean(x):
    if isinstance(x, float) and not math.isfinite(x): return None
    if isinstance(x, dict): return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [clean(v) for v in x]
    return x

def save(p, x):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(clean(x), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')

def stats(values):
    a = [v for v in values if v is not None and math.isfinite(v)]
    return dict(n=len(a), min=min(a), median=statistics.median(a), max=max(a)) if a else dict(n=0)

def gaps(rows, field):
    return [dict(session=a['session'], previous=a[field], current=b[field], missing=(b[field]-a[field]-1)&0xffffffff)
            for a,b in zip(rows, rows[1:]) if a['session']==b['session'] and (b[field]-a[field])&0xffffffff != 1]

def ranges(rows, key):
    groups=[]
    for r in rows:
        value=r.get(key)
        if not groups or groups[-1]['value'] != value or groups[-1]['session'] != r['session']:
            groups.append(dict(value=value, session=r['session'], first_seq=r['control_seq'], last_seq=r['control_seq'], first_device_ms=r['control_end_ms'], last_device_ms=r['control_end_ms'], n=1))
        else:
            groups[-1].update(last_seq=r['control_seq'], last_device_ms=r['control_end_ms'], n=groups[-1]['n']+1)
    return groups

summaries=[]
for tag in ('222930','223105','223130','223154','223226','223303','224402'):
    file, = (PC/'log').glob(f'v2_1012_{tag}_*.bin')
    p=core.parse_file(str(file)); meta=p.meta; dest=OUT/tag
    lo, hi, sid = meta['pre_roll_s'], meta['formal_end_s'], meta['session']
    def formal(r): return lo <= r['host_t'] <= hi and r['session']==sid
    maps={t: {} for t in (5,8,10)}; duplicates=[]
    for m in p.v2.messages:
        if m['type'] in maps:
            k=core.v2.detail_key(m)
            if k in maps[m['type']]: duplicates.append(dict(type=m['type'], key=k))
            maps[m['type']][k]=m
    rows=[]
    for r in p.records:
        row=dict(r); row['formal']=formal(r); row['key']=core.v2.detail_key(r)
        row['same_formal_session']=r['session']==sid
        row['host_phase']='prior_session' if r['session']!=sid else 'preroll' if r['host_t']<lo else 'formal' if r['host_t']<=hi else 'dump'
        row['paired_types']=[]
        for typ, mp in maps.items():
            extra=mp.get(core.v2.detail_key(r))
            if extra:
                row['paired_types'].append(typ)
                row[f'extension_{typ}_host_t']=extra['host_t']
                row.update({k:v for k,v in extra.items() if k not in ('type','schema','tx_seq','host_t')})
        rows.append(row)
    save(dest/'joined.json', rows)
    forms=[r for r in rows if r['formal']]
    sessionrows=[r for r in rows if r['session']==sid]
    motors=[s for m in p.v2.messages if m['type']==6 for s in m['samples']]
    fm=[s for s in motors if formal(s)]
    save(dest/'motors.json', motors)
    save(dest/'messages.json', p.v2.messages)
    save(dest/'ascii.json', p.ascii_lines)
    save(dest/'meta.json', meta)
    clouds=core.v2.assemble_clouds(p.v2.messages)
    cloudrows=[dict(meta=c['meta'], points=[v for _,v in sorted(c['points'].items())], complete=len(c['points'])==c['meta']['total_points'], conflict=c.get('conflict')) for _,c in sorted(clouds.items())]
    save(dest/'clouds.json', cloudrows)
    race=[r for r in forms if 10 in r['paired_types']]
    eligible=[r for r in forms if r['control_seq']%4==0]
    observed=set(r['control_seq'] for r in race)
    fs=next((r for r in rows if r['session']==sid), None)
    # 跨接收块可能批量到达；此处报告设备时间，不用host_t当控制实际执行时刻。
    points=['control_seq','control_end_ms','param_revision','host_t','host_phase','servo_pwm','candidate_pwm','action_reason','pd_p','pd_d','geometry_reason','gate_reason','path_reason','far_valid','near_a','far_a','target_x','ref_y','acute_phase','acute_reason','acute_direction_code','acute_position_mm','acute_goal_mm','acute_feature_y_mm','outer_offset','Speed_now','Speed_mubiao']
    compact=[{k:r.get(k) for k in points} for r in rows]
    save(dest/'timeline.json', compact)
    rev=[]
    for a,b in zip(rows,rows[1:]):
        if a['session']==b['session'] and a['param_revision']!=b['param_revision']:
            rev.append(dict(previous_seq=a['control_seq'], seq=b['control_seq'], before=a['param_revision'], after=b['param_revision'], device_ms=b['control_end_ms'], host_t=b['host_t'], formal=b['formal']))
    health=[m for m in p.v2.messages if m['type']==2 and m['session']==sid]
    config_summary={str(k):v for k,v in p.v2.configs.items()}
    r=dict(tag=tag, bin=str(file), sha256=hashlib.sha256(file.read_bytes()).hexdigest(), meta_hash_matches=True, rx_offsets_and_times_validated=True,
        bad_frames=p.bad_frames, tx_missing=p.v2.missing, tx_duplicates=p.v2.duplicates, extension_duplicate_keys=duplicates,
        formal_session=sid, session_counts=Counter(r['session'] for r in rows), raw_duration_s=meta['duration_s'], formal_host_bounds=[lo,hi], formal_host_duration_s=hi-lo,
        all_controls=len(rows), formal_controls=len(forms), same_session_controls=len(sessionrows), formal_seq_bounds=[forms[0]['control_seq'],forms[-1]['control_seq']] if forms else None,
        formal_device_ms_bounds=[forms[0]['control_end_ms'],forms[-1]['control_end_ms']] if forms else None,
        control_gaps_all=gaps(rows,'control_seq'), control_gaps_formal=gaps(forms,'control_seq'), formal_control_period_us=stats(r['control_dt_us'] for r in forms),
        paired_detail=sum(5 in r['paired_types'] for r in forms), paired_path=sum(8 in r['paired_types'] for r in forms), paired_race=len(race),
        race_expected_mod4=len(eligible), race_missing_eligible=[r['control_seq'] for r in eligible if r['control_seq'] not in observed],
        race_phases=Counter(r['acute_phase'] for r in race), race_reasons=Counter(r['acute_reason'] for r in race),
        race_samples=[{k:r.get(k) for k in points} for r in race],
        action_counts=Counter(r['action_reason'] for r in forms), geometry_counts=Counter(r.get('geometry_reason') for r in forms), gate_counts=Counter(r.get('gate_reason') for r in forms),
        formal_action_ranges=ranges(forms,'action_reason'), same_session_action_ranges=ranges(sessionrows,'action_reason'), formal_geometry_ranges=ranges(forms,'geometry_reason'),
        servo_pwm=stats(r['servo_pwm'] for r in forms), pd_d=stats(r.get('pd_d') for r in forms), actual_speed=stats(r['Speed_now'] for r in forms),
        formal_motor_samples=len(fm), motor_gaps_formal=gaps(fm,'sample_seq'), motor_pwm=stats(r['motor_pwm'] for r in fm), motor_target=stats(r['target_float'] for r in fm),
        param_revision_transitions=rev, configs=config_summary,
        parameter_events=[m for m in p.v2.messages if m['type']==4 and m['event_code']==1],
        all_events=[m for m in p.v2.messages if m['type']==4], commands=meta['commands'],
        all_race_phases=Counter(m['acute_phase'] for m in p.v2.messages if m['type']==10),
        all_nonzero_phase_rows=[{k:r.get(k) for k in points} for r in rows if r.get('acute_phase',0)!=0],
        health=health, cloud_count=len(cloudrows), complete_clouds=sum(c['complete'] and not c['conflict'] for c in cloudrows),
        cloud_keys=[{k:c['meta'].get(k) for k in ('session','control_seq','param_revision','input_us','trigger_flags')} for c in cloudrows])
    summaries.append(r); save(dest/'summary.json',r)
    print(tag, 'formal', len(forms), 'seq',r['formal_seq_bounds'], 'race',len(race),'/',len(eligible),'phase',dict(r['race_phases']),'actions',dict(r['action_counts']),'bad',p.bad_frames,'gaps',p.v2.missing)
save(OUT/'summary.json',summaries)
