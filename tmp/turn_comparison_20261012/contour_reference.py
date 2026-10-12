"""离线几何参照，明确区分可见弯墙偏离位置与真实入弯触发距离。

沿相邻射线保留同一连续点簇，允许外墙跨越 x=0；不放行到固件。
近处切线用100~350mm的50mm分区中位点拟合；连续三个分区偏离
近处切线至少30/50/80mm，才记录首个分区的y。三种门限是敏感性检查，
不是真实弯起点误差界，也不是待直接烧录的阈值。
"""
from pathlib import Path
import json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'调试视频/10.12/成功过弯_AI可读_20261012/复核与控制方案'

def contour(points,side,gap=120):
    splits=np.where(np.linalg.norm(np.diff(points,axis=0),axis=1)>gap)[0]+1
    parts=np.split(points,splits)
    scores=[sum((c[:,1]>=100)&(c[:,1]<=350)&(c[:,0]*side>0)) for c in parts]
    if not scores or max(scores)<8:return None
    best=parts[int(np.argmax(scores))]
    return best

def features(points,side,gap=120):
    c=contour(points,side,gap)
    if c is None:return dict(valid=False,reason='no_near_seed')
    bins=[]
    for lo in range(100,1300,50):
        b=c[(c[:,1]>=lo)&(c[:,1]<lo+50)]
        if len(b)>=2:bins.append([lo,float(np.median(b[:,0])),float(np.median(b[:,1]))])
    arr=np.array(bins);n=arr[(arr[:,0]>=100)&(arr[:,0]<350)]
    if len(n)<4:return dict(valid=False,reason='insufficient_near_bins')
    a,b=np.polyfit(n[:,2],n[:,1],1)
    residual=(n[:,1]-a*n[:,2]-b)/math.sqrt(1+a*a)
    rms=float(np.sqrt(np.mean(residual**2)))
    if rms>15:return dict(valid=False,reason='near_not_straight',rms=rms)
    # 外墙朝弯内偏折，符号与其近处左右侧身份相反。
    dev=-side*(arr[:,1]-a*arr[:,2]-b)/math.sqrt(1+a*a)
    refs={}
    for threshold in (30,50,80):
        refs[str(threshold)]=None
        for i in range(len(arr)-2):
            if arr[i,0]<350:continue
            if arr[i+2,0]-arr[i,0]==100 and np.all(dev[i:i+3]>=threshold):
                refs[str(threshold)]=[float(arr[i,0]),float(arr[i,0]+50)]
                break
    return dict(valid=True,near_a=float(a),near_b=float(b),near_rms=rms,
                front_axle_normal_distance_mm=float(side*(a*100+b)/math.sqrt(1+a*a)),
                deviation_y_bins_mm=refs,bin_xy=arr[:,1:].tolist(),bin_deviation_mm=dev.tolist(),cluster=c.tolist())

def main():
    records=[]
    for tag in ('164512','164616','164757','164942','165050','165134'):
        clouds=json.loads((OUT/tag/'clouds.json').read_text(encoding='utf-8'))
        side=1 if tag in ('164512','164616','164757') else -1
        fig,ax=plt.subplots(1,4,figsize=(16,5),sharex=True,sharey=True)
        for a,c in zip(ax,clouds[:4]):
            pts=np.array(c['points']);r=features(pts,side)
            rec=dict(tag=tag,seq=c['meta']['control_seq'],session=c['meta']['session'],revision=c['meta']['param_revision'],**r)
            rec['gap_sensitivity']={str(g):features(pts,side,g).get('deviation_y_bins_mm') for g in (80,120,160)}
            records.append(rec)
            a.scatter(pts[:,0],pts[:,1],s=5,c='gray')
            if r['valid']:
                v=np.array(r['cluster']);a.plot(v[:,0],v[:,1],c='orange',lw=1)
                yy=np.linspace(100,1300,50);a.plot(r['near_a']*yy+r['near_b'],yy,'b--')
                for threshold,color in ((30,'green'),(50,'red'),(80,'purple')):
                    y=r['deviation_y_bins_mm'][str(threshold)]
                    if y:a.axhspan(*y,alpha=.15,color=color,label=f'{threshold} mm departure bin')
            a.set_title(f'{tag} seq {rec["seq"]}');a.set_xlim(-1000,1000);a.set_ylim(0,1400);a.set_aspect('equal');a.grid(alpha=.2)
        ax[0].legend(fontsize=6);fig.suptitle('Observed contour feature; NOT true corner distance or steering trigger')
        fig.tight_layout();fig.savefig(OUT/tag/'contour_reference.png',dpi=120);plt.close(fig)
    (OUT/'contour_reference.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    for tag in ('164512','164616','164757','164942','165050','165134'):
        print(tag,[(r['seq'],r.get('deviation_y_bins_mm'),round(r['front_axle_normal_distance_mm']) if r.get('valid') else r['reason']) for r in records if r['tag']==tag])


if __name__ == "__main__":
    main()
