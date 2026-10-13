from pathlib import Path
import csv,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tmp/turn_comparison_20261012'))
from contour_reference import features
LOG=ROOT/'robocup-新建文件夹队/robocup-新建文件夹/上位机/log'
OLD=ROOT/'调试视频/10.12/成功过弯_AI可读_20261012/复核与控制方案'
plt.rcParams['font.sans-serif']=['Microsoft YaHei']
plt.rcParams['axes.unicode_minus']=False
cases=[('222930',3718,'新左弯 T01',False),('224402',14163,'新左弯 T07',False),
       ('164512',1335,'成功手推左弯',True),('223130',5143,'新右弯 T03（良好）',False),
       ('223226',5823,'新右弯 T05（碰撞）',False),('164942',4568,'成功手推右弯',True)]
fig,axs=plt.subplots(2,3,figsize=(13,10))
for ax,(tag,seq,title,old) in zip(axs.flat,cases):
    if old:
        cs=json.loads((OLD/tag/'clouds.json').read_text(encoding='utf-8'))
        points=next(c['points'] for c in cs if c['meta']['control_seq']==seq)
    else:
        f,=LOG.glob(f'v2_1012_{tag}_*.cloud.csv')
        with f.open(encoding='utf-8-sig',newline='') as fd:
            points=[[float(r['x_mm']),float(r['y_mm'])] for r in csv.DictReader(fd) if int(r['control_seq'])==seq]
    p=np.array(points)
    ax.scatter(p[:,0],p[:,1],s=5,c='#555555')
    for side,color in [(-1,'#3465a4'),(1,'#ce5c00')]:
        f=features(p,side)
        if f['valid']:
            ys=np.array([100,350]);ax.plot(f['near_a']*ys+f['near_b'],ys,c=color,lw=2)
    ax.scatter([0],[100],c='red',s=30,label='前轴中心')
    ax.plot([-130,130],[100,100],c='red',lw=2)
    ax.axhspan(100,350,color='#999999',alpha=.08)
    ax.set_xlim(-750,750);ax.set_ylim(0,1350);ax.set_aspect('equal');ax.grid(alpha=.2)
    ax.set_title(f'{title} / seq {seq}',fontsize=11)
    ax.set_xlabel('横向 x（mm，右为正）');ax.set_ylabel('前向 y（mm）')
fig.suptitle('真实冻结点云与近墙参照：各图不是同一物理时刻，不能据此量出所需横移量',fontsize=13)
fig.tight_layout(rect=(0,0,1,.96))
fig.savefig(Path(__file__).with_name('geometry_comparison.png'),dpi=130)
