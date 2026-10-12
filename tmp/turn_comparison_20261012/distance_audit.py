"""距离口径审计：同帧核对旧交点法/偏离法，结果不是物理拐点真值。"""
from pathlib import Path
import sys, json, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
PC = ROOT / 'robocup-新建文件夹队/robocup-新建文件夹/上位机'
OUT = Path(__file__).parent / 'distance_audit'
OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(PC))
import robocup_telemetry as core

def intersection(row, side):
    """重现旧 G1~G4 门限，仅输出拟合交点，不命名为真实弯道距离。"""
    shift = 0 if side == 'left' else 8
    rej = row.get('fit_rejected')
    if rej is None or ((rej >> shift) & 255) or ((rej >> (shift + 16)) & 255):
        return None
    try:
        a, b, lo, hi = [row[f'{side}_near_{k}'] for k in ('a', 'b', 'min_y', 'max_y')]
        c, d, flo, fhi = [row[f'{side}_far_{k}'] for k in ('a', 'b', 'min_y', 'max_y')]
    except KeyError:
        return None
    if abs(a-c) < .05 or hi <= lo or fhi <= flo or lo >= fhi:
        return None
    value = (d-b)/(a-c)
    return value if lo-50 <= value <= fhi+50 else None

def departure(points, side, top=600, threshold=150):
    """精确重现旧点云方法。x 正负不保证墙面身份，单点越限也不保证连续弯墙。"""
    s = points[points[:, 0] < 0] if side == 'left' else points[points[:, 0] >= 0]
    near = s[(s[:, 1] >= 100) & (s[:, 1] <= top)]
    if len(near) < 4 or np.ptp(near[:, 1]) < 1:
        return None, None
    a, b = np.polyfit(near[:, 1], near[:, 0], 1)
    far = s[(s[:, 1] > top) & (np.abs(s[:, 0]-a*s[:, 1]-b) > threshold)]
    return (float(far[:, 1].min()) if len(far) else None), (float(a), float(b))

records = []
for tag in ('164512', '164616', '164757', '164942', '165050', '165134'):
    file, = (PC/'log').glob(f'v2_1012_{tag}_*.bin')
    p = core.parse_file(str(file))
    paths = {core.v2.detail_key(m): m for m in p.v2.messages if m['type'] == 8}
    clouds = core.v2.assemble_clouds(p.v2.messages)
    side = 'right' if tag in ('164512','164616','164757') else 'left'
    fig, axes = plt.subplots(2,4,figsize=(16,10),sharex=True,sharey=True)
    for ax, (key, c) in zip(axes.flat, sorted(clouds.items())):
        m = c['meta']; pts = np.array([v for _,v in sorted(c['points'].items())], dtype=float)
        assert len(pts) == m['total_points'] and not c.get('conflict')
        row = paths.get(core.v2.detail_key(m), {})
        cross = intersection(row, side)
        depart, fit = departure(pts, side)
        sensitivity = []
        for top in (400,500,600):
            for threshold in (100,150,200):
                y, _ = departure(pts, side, top, threshold)
                sensitivity.append(dict(base_top_mm=top,threshold_mm=threshold,departure_y_mm=y))
        rec = dict(run=tag,session=m['session'],seq=m['control_seq'],revision=m['param_revision'],
                   outer_side=side,fit_intersection_y_mm=cross,departure_y_mm=depart,
                   difference_mm=abs(cross-depart) if cross is not None and depart is not None else None,
                   sensitivity=sensitivity)
        records.append(rec)
        ax.scatter(pts[:,0],pts[:,1],s=3,c='gray')
        if fit:
            yy=np.linspace(100,1300,100); ax.plot(fit[0]*yy+fit[1],yy,'b--',label='old near OLS')
        if cross is not None: ax.axhline(cross,c='green',label='fit intersection')
        if depart is not None: ax.axhline(depart,c='red',label='150 mm departure')
        ax.set_title(f'{tag} seq {m["control_seq"]}\nintersection={cross:.0f}' if cross is not None else f'{tag} seq {m["control_seq"]}\nintersection unavailable')
        ax.set_xlim(-1000,1000);ax.set_ylim(0,1400);ax.set_aspect('equal');ax.grid(alpha=.3)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle('Same-frame comparison; neither estimator is a measured physical corner')
    fig.tight_layout();fig.savefig(OUT/f'{tag}_same_frame.png',dpi=120);plt.close(fig)

# 反例：无任何几何突变的平滑抛物线，两个固定窗口仍稳定交于约 663 mm。
synthetic=[]
for k in (.0002,.0005,.001):
    fits=[]
    for lo,hi in ((100,750),(550,1300)):
        y=np.linspace(lo,hi,1001);x=k*y*y
        fits.append(np.polyfit(y,x,1))
    (a,b),(c,d)=fits
    synthetic.append(dict(curve=f'x={k}*y^2',intersection_y_mm=float((d-b)/(a-c)),physical_corner=False))
paired=[r['difference_mm'] for r in records if r['difference_mm'] is not None]
summary=dict(frames=len(records),paired_frames=len(paired),
             within_50_mm=sum(d<=50 for d in paired),min_difference_mm=min(paired) if paired else None,max_difference_mm=max(paired) if paired else None,
             synthetic_smooth_curve=synthetic,records=records)
(OUT/'audit.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='records'},ensure_ascii=False))
for tag in ('164512','164616','164757','164942','165050','165134'):
    print(tag,[(r['seq'],round(r['fit_intersection_y_mm'],1) if r['fit_intersection_y_mm'] is not None else None,r['departure_y_mm'],round(r['difference_mm'],1) if r['difference_mm'] is not None else None) for r in records if r['run']==tag])
