"""Check the front box motion in the 2026-09-28 13:20 raw capture."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from check_m10p import Parser

root=Path(__file__).resolve().parent
path=root/'reports/latest_capture.bin'
data=path.read_bytes()
# Independently check consecutive header spacing before using the decoder.
heads=[]
i=data.find(b'\xa5\x5a')
while i>=0:
    heads.append(i)
    i=data.find(b'\xa5\x5a',i+1)
for start,end in zip(heads,heads[1:]):
    assert end-start==int.from_bytes(data[start+2:start+4],'big')
    assert data[end-2:end]==b'\xfa\xfb'
    assert (int.from_bytes(data[end+4:end+6],'big')-int.from_bytes(data[start+4:start+6],'big'))%36000==1500
p=Parser(); frames=p.feed(data)
rows=[];t=0;zero=0;total=0
for angle,rpm,points,invalid in frames:
    # At 15 degrees per packet: duration = (60/rpm)/24.
    # Raw capture has no host arrival timestamps; this is estimated scan time.
    t+=2.5/rpm
    near=[(a,d) for a,d,h in points if abs((a+180)%360-180)<=2]
    if not near:continue
    total+=len(near);zero+=sum(d==0 for a,d in near)
    ds=[d for a,d in near if d>0]
    if not ds:continue
    center=min(near,key=lambda pt:abs((pt[0]+180)%360-180))
    rows.append(dict(time_s=t,median_mm=float(np.median(ds)),
                     p10_mm=float(np.percentile(ds,10)),p90_mm=float(np.percentile(ds,90)),
                     nearest_zero_angle_deg=(center[0]+180)%360-180,nearest_zero_mm=center[1]))
times=np.array([r['time_s'] for r in rows]);dist=np.array([r['median_mm'] for r in rows])
seconds=[dict(second=s,median_mm=float(np.median(dist[(times>=s)&(times<s+1)])))
         for s in range(int(t)+1) if ((times>=s)&(times<s+1)).any()]
initial=dist[times<22];final=dist[times>=28]
result=dict(source=path.name,sha256=hashlib.sha256(data).hexdigest(),
            frame_lengths=dict(p.lengths),bad_frames=p.bad,estimated_duration_s=t,
            front_window_deg=[-2,2],front_samples=len(rows),front_points=total,front_zero_points=zero,
            initial_0_to_22s_median_mm=float(np.median(initial)),
            initial_scan_median_range_mm=[float(initial.min()),float(initial.max())],
            final_28s_onward_median_mm=float(np.median(final)),
            final_scan_median_range_mm=[float(final.min()),float(final.max())],
            displacement_mm=float(np.median(final)-np.median(initial)),
            per_second=seconds,
            note='Time estimated from motor speed and 15-degree packet span; not exact host timestamps. Distances assume supplied protocol layout. Not an accuracy calibration.')
(root/'reports/盒子后移_分析.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
fig,ax=plt.subplots(figsize=(12,6.5),layout='constrained')
ax.axhspan(22,25,color='#d5e9d7',alpha=.8,label='用户描述的初始距离 22–25 cm')
ax.fill_between(times,np.array([r['p10_mm'] for r in rows])/10,
                np.array([r['p90_mm'] for r in rows])/10,color='#a9c7e9',alpha=.6,
                label='每次扫描前方 ±2° 内点距离的 P10–P90')
ax.plot(times,dist/10,color='#19579e',linewidth=1.5,label='前方 ±2° 非零距离中位数')
ax.scatter([s['second']+.5 for s in seconds],[s['median_mm']/10 for s in seconds],s=18,color='#df7126',label='每秒中位数',zorder=3)
ax.annotate(f"前 22 秒：{np.median(initial)/10:.1f} cm",xy=(10,22.3),xytext=(5,28),arrowprops=dict(arrowstyle='->',color='#243449'),fontsize=12)
ax.annotate('约第 23 秒起逐渐后移',xy=(24.5,28.3),xytext=(14,32),arrowprops=dict(arrowstyle='->',color='#243449'),fontsize=12)
ax.annotate(f"最后：{np.median(final)/10:.1f} cm",xy=(29,34.6),xytext=(21,37),arrowprops=dict(arrowstyle='->',color='#243449'),fontsize=12)
ax.set(xlim=(0,30),ylim=(19,39),xlabel='相对时间（秒，按转速估算）',ylabel='测距（cm）',
       title='M10P 前方盒子测试：先稳定在约 22.3 cm，随后后移至约 34.6 cm\n2026-09-28 13:20 采集 · 0° ±2° · 359 次前方扫描')
ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=9)
fig.savefig(root/'reports/M10P_盒子后移_距离时间图.png',dpi=160)
plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k!='per_second'},ensure_ascii=False,indent=2))
