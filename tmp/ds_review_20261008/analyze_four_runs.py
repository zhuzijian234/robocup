from pathlib import Path
import csv, json, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
PROJECT = ROOT / '2-LXY传承/LXY传承-m10p'
LOG = PROJECT / '上位机/log'
OUT = Path(__file__).resolve().parent
def read(p):
    with p.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))
def key(r): return r['session'], r['control_seq'], r['param_revision']
summary=[]
fig, axes=plt.subplots(2,2,figsize=(13,7.5),sharey=True)
windows=[(23.8,27.4),(7.3,8.16),(0.4,1.85),(.95,1.65)]
for tag,ax,window in zip(['175104','175252','175407','175457'],axes.flat,windows):
    f=next(x for x in LOG.glob('v2_1008_'+tag+'_*.csv') if x.name.count('.')==1)
    rows=read(f); details={key(r):r for r in read(f.with_suffix('.detail.csv'))}
    health=read(f.with_suffix('.m10p.csv'))
    meta=json.loads(f.with_suffix('.bin.meta.json').read_text(encoding='utf-8'))
    assert len(details)==len(read(f.with_suffix('.detail.csv'))), 'duplicate detail join key'
    bad=[r for r in rows if r['action_reason']=='5']
    record=dict(tag=tag,file=str(f),rows=len(rows),invalid=len(bad),
        point_min=min(float(r['valid_couter']) for r in rows),point_max=max(float(r['valid_couter']) for r in rows),
        invalid_points_le20=sum(float(r['valid_couter'])<=20 for r in bad),
        empty_reference=sum(details.get(key(r),{}).get('ref_count')=='0' for r in bad),
        invalid_missing_detail=sum(key(r) not in details for r in bad),
        error=meta.get('error'),configs=meta.get('configs'),session=sorted(set(r['session'] for r in rows)),
        radar_deltas={k:int(health[-1][k])-int(health[0][k]) for k in
            ['packets','bad_length','bad_tail','bad_angle','bad_speed','discontinuities','rejected','scan_overflow','ready_drop','rx_late']})
    summary.append(record)
    selected=[r for r in rows if window[0]<=float(r['t'])<=window[1]]
    ax.step([float(r['t']) for r in selected],[float(r['servo_pwm']) for r in selected],where='post',color='#24599a',lw=1.6,label='Servo command (CCR)')
    for action,color,marker,label in [('1','#24599a','o','PD output'),('2','#d99316','s','Held output'),('5','#b9333b','x','Rejected geometry')]:
        rr=[r for r in selected if r['action_reason']==action]
        ax.scatter([float(r['t']) for r in rr],[float(r['servo_pwm']) for r in rr],c=color,marker=marker,s=25,zorder=3,label=label)
    ax.axhline(1445,color='gray',ls='--',lw=1)
    ax.set_title('Run '+tag+' | '+('configuration captured' if meta.get('configs') else 'handshake incomplete'))
    ax.set_ylim(1135,1755);ax.set_yticks([1170,1300,1445,1600,1720]);ax.set_xlim(*window)
    ax.set_xlabel('Device-relative time in this CSV (s)');ax.set_ylabel('CCR command; not measured wheel angle')
    ax.grid(alpha=.2)
fig.suptitle('Four captures: commanded steering returns and reversals',fontsize=15)
handles,labels=axes[0,0].get_legend_handles_labels()
fig.legend(handles,labels,loc='lower center',ncol=4,frameon=False)
fig.tight_layout(rect=[0,.06,1,.95]);fig.savefig(OUT/'four_runs_steering.png',dpi=160);plt.close(fig)
(OUT/'four_runs_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')

doc='''# 最近四次M10P弯道遥测复核（2026-10-08）

结论：记录支持“控制程序在弯中反复主动收舵/回正，随后再打回甚至反向打满”。不能仅归因于舵机力度、雷达点数或速度参数。修改2针对了部分症状，但没有覆盖所有触发路径。

## 数据口径与限制

分析四份CONTROL CSV，按(session, control_seq, param_revision)精确关联DETAIL；未配对不补零。统计包含CSV已有的协商阶段记录，与原报告“正式区间”的帧数分母不同。t是各文件设备相对时间，不是采集墙钟。

175104正常结束且配置齐全；175407配置齐全，之后串口读取异常结束；175252、175457握手超时，但保留了旧会话继续发出的有效遥测。后两份不作为参数已核实的独立对照试验。

这些是舵机CCR指令，不是实测车轮角度，也没有同步视频/定位证明每帧的物理位置。记录能确认程序发出的回正命令；“该时刻仍在弯中”来自用户的实车观察。雷达原始点云未随这些遥测完整保存，无法确定某个断点对应哪个锥桶。

## 四份记录概况

|记录|CONTROL帧数|无效动作帧|无效帧中参考窗口为空|有效点数范围|无效帧中点数≤20|
|---|---:|---:|---:|---:|---:|
'''
for r in summary:
    doc+=f"|{r['tag']}|{r['rows']}|{r['invalid']}|{r['empty_reference']}|{r['point_min']:.0f}～{r['point_max']:.0f}|{r['invalid_points_le20']}|\n"
doc+='''
其余2个无效动作帧分别是175104的26.834s、175457的0.333s：参考窗口非空，但中线斜率约-0.057/-0.062，落入mode 0的|k|≤0.1拒绝条件。三份记录352个无效动作帧中350个参考窗口为空，所有这些帧仍有数百个有效点。与“points≤20导致无效”的推测相反。

## 证据一：两帧小误差被当成出弯

175104：

|t(s)|动作/模式|CCR|候选误差|中线k|解释|
|---:|---|---:|---:|---:|---|
|26.497|PD / 9|1644|310.4|—|左转输出|
|26.587|HOLD / 候选0|1644|-0.307|-0.407|第一帧小误差，仍保持|
|26.666|PD / 0|1444|-0.899|-0.486|第二帧小误差，执行回正|
|26.755|PD / 0|1609|205.5|-0.398|下一帧又加左舵|

26.666s的DETAIL明确记录P=-0.31465、D=0、未限幅输出1444.685。因此这次回正不是D项反打，也不是PWM脚失效。按代码，前两帧小误差满足出弯确认；距离26.497s仅169ms，亦非350ms保持到期。

25.748/25.838s出现同类序列，误差35.9→-1.6、k=0.516→0.451、CCR1620→1444。

修改2的sk=2能否决这些帧的直道证据。但175104还有24.249/24.338s，k=-4.527/-6.152；24.507/24.585s，k=-5.339/-5.307。它们同样造成1620→1431、1620→1418，并且会通过新斜率门槛。斜率反映局部拟合朝向，不足以证明整条前方路径已经出弯。

## 证据二：保持到期也能放出反向指令

175252：7.480s mode8输出1270，后续保持；7.908s mode0直接输出1548，误差295.9；7.986s mode8又输出1219。7.908s误差远大于100，不满足“小误差直道确认”，距离7.480s已428ms，与350ms旧状态到期后放行普通候选的代码路径一致。该日志缺少完整参数握手及显式释放原因，故标为代码与时序共同支持的归因，而非直接记录的原因位。

修改2仅修改straight_evidence，未修改该到期路径。不能只提高sk或盲目延长保持时间解决；后者会拖慢真实出弯和反向弯。

## 证据三：左右方向翻转在四份记录中出现

175104的27.000→27.089→27.167→27.257s：mode4→3→4→3，CCR1720→1170→1720→1170，误差+500/-500交替，D均为0。是上游方向判定在切换，不能归因于D过大。

175407的0.583→0.661→0.751s也为1170→1720→1170。175457在1.419s从1170回到1434，1.497s又打回1170。175252在7.908/7.986s则从保持右转跨过中位到左侧再打回右侧。

dy=200可滤掉部分侧方断点，但当前HANDLE8/9先返回首个断点、主循环再清零，会漏掉后续有效开口；同时200～550mm内左右竞争仍无方向一致性仲裁。slew=150只能减小每次幅度，不能纠正错误目标；1170到1720至少4次有效更新。

## 证据四：无效控制主要是几何参考缺失

352个无效动作帧中350个ref_count=0；其中大部分是双侧中线参考为空，少量是mode1/2侧边界参考为空。总点数没有任何一帧≤20。因此降低桶数门槛或把数据年龄放宽至220ms，不会自动补出控制参考。

四份M10P健康快照内包长、包尾、角度、速度、圈不连续、拒绝、扫描溢出、ready丢弃、DMA迟服务计数增量均为0。这支持接收故障不是这些事件的首要证据，但不等于测距/点云绝对正确（当前原始协议没有可用CRC验证）。175407有一次健康快照前方年龄172053us，不能将它推成整个无效区间的原因。

## 对修改2的判定及下一步

1. sk有针对性，但只拦一类误出弯，且mode5引用旧Midline.k的问题仍需修正。
2. dy有针对性，但应在搜索内部跳过不合格候选；入口检测与已入弯的持续跟踪不能共用“断点仍须在200mm以外”的条件。
3. slew属于输出平滑，不是方向判定修复。先消除错误方向，再按时间和转向任务设计限幅。
4. 220ms、12/4桶门槛不是本批大量空参考窗口的直接解法。
5. 优先把转向改成连续路径跟踪：弯道入口触发状态，弯中持续依据边界/路径更新曲率；缺少入口断点不等于出弯。出弯结合近远段方向、边界连续性和偏差，不能仅看某点误差或一条线的k。
6. 真正实现单侧边界推算中线，并检查局部参考距离和拟合跨度。350ms到期用于重新评估来源，不应直接批准弱中线回正，也不应无限固定旧舵角。
7. 增加明确仲裁原因、限幅前后目标、所用边界及拟合质量的低频诊断，先复现这几个事件再提固定目标速度。电机继续固定目标编码器闭环，不恢复自动停车。

本次仅分析，未修改固件。由于日志固件标识沿用了旧值，不能把当前源码/修改2构建与这四份记录严格一一对应；cx/eclamp与250合成误差等记录支持其含修改1行为。修改2效果仍需新固件日志验证。

## 原始文件

'''
for r in summary:
    f=Path(r['file'])
    for source in [f,f.with_suffix('.detail.csv'),f.with_suffix('.m10p.csv'),f.with_suffix('.bin.meta.json')]:
        doc+=f'- [{source.name}]({source.as_posix()})\n'
(OUT/'最近四次弯中回正分析_20261008.md').write_text(doc,encoding='utf-8')
print('Created analysis report, summary and steering chart')
