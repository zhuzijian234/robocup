"""Offline distance checks, independent framing, same-sector adjacent-turn comparison."""
from pathlib import Path
from collections import Counter, defaultdict
import json
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'reports/hardware_check_verified.bin'
data = SOURCE.read_bytes()
heads = []
offset = data.find(b'\xa5\x5a')
while offset >= 0:
    heads.append(offset)
    offset = data.find(b'\xa5\x5a', offset + 1)
frames = []
for start, end in zip(heads, heads[1:]):
    size = end-start
    assert size == int.from_bytes(data[start+2:start+4], 'big')
    assert data[end-2:end] == b'\xfa\xfb'
    words = np.frombuffer(data[start+8:end-12], dtype='>u2').astype(int)
    valid = words != 65535
    distances = (words[valid] & 32767).astype(float)
    frames.append(dict(offset=start, size=size,
                       angle=int.from_bytes(data[start+4:start+6], 'big') % 36000,
                       distance=distances, invalid=int((~valid).sum())))
assert all((b['angle']-a['angle']) % 36000 == 1500 for a,b in zip(frames,frames[1:]))

def quantile(values, q):
    return round(float(np.percentile(values,q)),2) if len(values) else None

groups = defaultdict(lambda: defaultdict(list))
details = []
for i, f in enumerate(frames):
    d = f['distance']
    g = groups[f['size']]
    g['distance'].extend(d.tolist())
    g['invalid'].append(f['invalid'])
    adjacent = (d[1:]>0) & (d[:-1]>0)
    jumps = np.abs(np.diff(d))[adjacent]
    g['spatial_steps'].extend(jumps.tolist())
    # Isolated spike: both immediate neighbors agree within 50 mm,
    # while the center differs from each by >200 mm; zeros excluded.
    isolated = ((d[1:-1]>0)&(d[:-2]>0)&(d[2:]>0)&
                (np.abs(d[:-2]-d[2:])<=50)&
                (np.abs(d[1:-1]-d[:-2])>200)&(np.abs(d[1:-1]-d[2:])>200))
    g['isolated'].append(int(isolated.sum()))
    record = dict(index=i, offset=f['offset'], length=f['size'], angle_deg=f['angle']/100,
                  points=len(d), zero_points=int((d==0).sum()),
                  max_adjacent_nonzero_jump_mm=round(float(jumps.max()),2) if len(jumps) else None,
                  spatial_spike_indices=(np.flatnonzero(isolated)+1).tolist())
    if 24 <= i < len(frames)-24:
        prev, nxt = frames[i-24], frames[i+24]
        assert prev['angle'] == f['angle'] == nxt['angle']
        # Compare nearest angle, not raw slot index, as point counts vary.
        def match(other):
            od = other['distance']
            ix = np.minimum(np.floor(np.arange(len(d))*len(od)/len(d)+0.5).astype(int),len(od)-1)
            return od[ix]
        p, n = match(prev), match(nxt)
        usable = (d>0)&(p>0)&(n>0)
        residual = np.minimum(np.abs(d-p),np.abs(d-n))
        g['temporal_residual'].extend(residual[usable].tolist())
        # Conservative transient: both adjacent turns agree, current differs
        # from both >200 mm AND >30% of their mean.
        stable = usable & (np.abs(p-n)<=50)
        threshold = np.maximum(200, 0.3*(p+n)/2)
        transient = stable & (residual>threshold)
        zero_only = (d==0)&(p>0)&(n>0)&(np.abs(p-n)<=50)
        g['stable_comparisons'].append(int(stable.sum()))
        g['transients'].append(int(transient.sum()))
        g['zero_only'].append(int(zero_only.sum()))
        meds = [float(np.median(x[x>0])) if (x>0).any() else None for x in (p,d,n)]
        med_delta = min(abs(meds[1]-meds[0]),abs(meds[1]-meds[2])) if all(x is not None for x in meds) else None
        if med_delta is not None:
            g['median_shift'].append(med_delta)
        record.update(temporal_p95_mm=quantile(residual[usable],95),
                      transient_indices=np.flatnonzero(transient).tolist(),
                      transient_values=[dict(slot=int(j),previous_mm=p[j],current_mm=d[j],next_mm=n[j]) for j in np.flatnonzero(transient)],
                      zero_only_count=int(zero_only.sum()), median_shift_mm=med_delta)
    details.append(record)

summary = {}
for length, g in sorted(groups.items()):
    d=np.array(g['distance']); steps=np.array(g['spatial_steps'])
    summary[length] = dict(frames=sum(f['size']==length for f in frames),points=len(d),
        zero_pct=round(float((d==0).mean()*100),3), invalid_ffff=sum(g['invalid']),
        above_10m=int((d>10000).sum()),above_25m=int((d>25000).sum()),
        nonzero_min_mm=float(d[d>0].min()),max_mm=float(d.max()),
        spatial_jump_gt200_count=int((steps>200).sum()),spatial_pairs=len(steps),
        spatial_jump_gt200_pct=round(float((steps>200).mean()*100),3),
        isolated_spikes=sum(g['isolated']),temporal_points=len(g['temporal_residual']),
        temporal_p50_mm=quantile(g['temporal_residual'],50),
        temporal_p95_mm=quantile(g['temporal_residual'],95),
        temporal_p99_mm=quantile(g['temporal_residual'],99),
        stable_comparisons=sum(g['stable_comparisons']),transient_points=sum(g['transients']),
        transient_zero_points=sum(g['zero_only']),
        frame_median_shift_gt200=sum(x>200 for x in g['median_shift']))

result=dict(source=str(SOURCE),sha256=hashlib.sha256(data).hexdigest(),summary=summary,
    assumptions=['Distance region assumes 8-byte header plus 10-byte reserved field and 2-byte tail, per supplied documentation; firmware layout unconfirmed.',
                 'Adjacent turns aligned by start angle and nearest point angle; not by slot index.',
                 'Scene and sensor motion were not controlled. Flags are candidates, not proof of corruption.',
                 '200 mm / 50 mm / 30% thresholds are exploratory, not manufacturer acceptance limits.'],
    rare_frames=[r for r in details if r['length'] in (156,162)],
    flagged_frames=[r for r in details if r.get('transient_indices') or r['spatial_spike_indices']])
(ROOT/'reports/distance_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('flagged_frames',)},ensure_ascii=False,indent=2))
print('flagged_frames',len(result['flagged_frames']))

plt.rcParams['font.sans-serif']=['Microsoft YaHei']
plt.rcParams['axes.unicode_minus']=False
rare=[i for i,f in enumerate(frames) if f['size'] in (156,162)]
fig,axs=plt.subplots(3,2,figsize=(15,12),layout='constrained')
for ax,i in zip(axs.flat,rare):
    for idx,label,color in ((i-24,'前一圈','#8295ad'),(i,'当前包','#e16b23'),(i+24,'后一圈','#24846b')):
        f=frames[idx]; ds=f['distance'].copy(); ds[ds==0]=np.nan
        a=np.arange(len(ds))*15/len(ds)
        ax.plot(a,ds,'.-',markersize=3,linewidth=0.8,label=f"{label} {f['size']} B",color=color)
    f=frames[i]; r=details[i]
    ax.set_title(f"{f['size']} 字节 | 起始角 {f['angle']/100:g}° | 偏移 {f['offset']}\n零值 {r['zero_points']}/{r['points']}，瞬时尖峰候选 {len(r.get('transient_indices',[]))}")
    ax.set_xlabel('相对包起始角（°）'); ax.set_ylabel('非零距离（mm）')
    ax.grid(alpha=.2); ax.legend(fontsize=9)
fig.suptitle('M10P：156 / 162 字节包与前后扫描圈的距离对照\n零距离留空；邻圈按相同起始角选取。曲线差异也可能来自目标边缘或运动。',fontsize=17)
fig.savefig(ROOT/'reports/M10P_变长帧距离对照.png',dpi=150)
plt.close(fig)

# Preserve same-sector zero baselines to avoid comparing different scene regions.
sector_zeros = {}
for angle in (32300,35300):
    sector_zeros[str(angle/100)] = {}
    for size in sorted(summary):
        fs=[f for f in frames if f['angle']==angle and f['size']==size]
        if fs:
            ds=np.concatenate([f['distance'] for f in fs])
            sector_zeros[str(angle/100)][size]=dict(frames=len(fs),zero_pct=round(float((ds==0).mean()*100),3))
result['same_sector_zero_baseline']=sector_zeros
spatial_examples=[]
for r in result['flagged_frames']:
    if not r['spatial_spike_indices']:
        continue
    i=r['index']; slot=r['spatial_spike_indices'][0]; f=frames[i]
    neighbors=[]
    for delta in (-48,-24,0,24,48):
        other=frames[i+delta]; ds=other['distance']; ix=min(int(slot*len(ds)/r['points']+0.5),len(ds)-1)
        neighbors.append(dict(turn_delta=delta//24,length=other['size'],matched_slot=ix,
                              nearby_distances_mm=ds[max(0,ix-3):ix+4].tolist()))
    spatial_examples.append(dict(frame=r,neighbors=neighbors,
        raw_three_words=data[f['offset']+8+2*(slot-1):f['offset']+8+2*(slot+2)].hex(' ').upper()))
    fig,axs=plt.subplots(2,1,figsize=(12,9),layout='constrained')
    for delta,color in ((-24,'#8295ad'),(0,'#e16b23'),(24,'#24846b')):
        other=frames[i+delta]; ds=other['distance'].copy(); a=np.arange(len(ds))*15/len(ds)
        zeros=ds==0; ds[zeros]=np.nan
        axs[0].plot(a,ds,'.-',label=f"{delta//24:+d} 圈 / {other['size']} 字节",color=color,linewidth=1)
        axs[0].scatter(a[zeros],np.zeros(zeros.sum()),marker='x',color=color,s=22)
    axs[0].set_title(f"158 字节包中的可疑孤立近点：偏移 {f['offset']}，包起始角 {f['angle']/100:g}°")
    axs[0].set_xlabel('相对包起始角（°）'); axs[0].set_ylabel('距离（mm）；底部 × 表示零值')
    axs[0].grid(alpha=.2); axs[0].legend()
    ds=f['distance']; x=np.arange(slot-4,slot+4)
    axs[1].plot(x,ds[x],'o-',color='#e16b23')
    for j in x:
        axs[1].annotate(f'{ds[j]:g}',(j,ds[j]),xytext=(0,10),textcoords='offset points',ha='center')
    axs[1].set_ylim(-150,3550);axs[1].set_xlabel('当前包内点索引（从 0 开始）');axs[1].set_ylabel('距离（mm）')
    axs[1].set_title('3080 → 394 → 3070 mm；相应原始字节为 0C 08 | 01 8A | 0B FE')
    axs[1].grid(alpha=.2)
    fig.suptitle('存在距离跳变，但不能据此确定是通信错误\n后一圈该方向的距离及零值也发生变化；未控制现场物体或雷达运动。',fontsize=16)
    fig.savefig(ROOT/'reports/M10P_158字节_可疑距离跳变.png',dpi=150)
    plt.close(fig)
result['spatial_examples']=spatial_examples
(ROOT/'reports/distance_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
