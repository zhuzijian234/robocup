"""A received batch is not a simultaneous control update.
Map device time by the minimum observed receive lag (unknown fixed lag remains).
Video and capture were started nearly simultaneously per user's confirmation.
"""
from pathlib import Path
import json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
plt.rcParams['font.sans-serif']=['Microsoft YaHei'];plt.rcParams['axes.unicode_minus']=False
tags=('222930','223105','223130','223154','223226','223303','224402')
fig,axs=plt.subplots(4,2,figsize=(14,12)); summary=[]
for ax,tag in zip(axs.flat,tags):
    rows=json.loads((ROOT/'telemetry'/tag/'joined.json').read_text(encoding='utf-8'))
    meta=json.loads((ROOT/'telemetry'/tag/'meta.json').read_text(encoding='utf-8'))
    # Use all monotonic device clock observations: exclude previous sessions from output,
    # but do not replace transmission latency with controls' ordinal count.
    rows=[r for r in rows if r['session']==meta['session']]
    offset=min(r['host_t']-r['control_end_ms']/1000 for r in rows)
    for r in rows:r['estimated_capture_t']=r['control_end_ms']/1000+offset
    stop=meta['formal_end_s']; display=[r for r in rows if -.3<=r['estimated_capture_t']<=stop+.5]
    ax.plot([r['estimated_capture_t'] for r in display],[r['servo_pwm'] for r in display],'.-',lw=1,ms=3,label='PWM（设备时间校正）')
    bad=[r for r in display if r['action_reason']==5]
    ax.scatter([r['estimated_capture_t'] for r in bad],[r['servo_pwm'] for r in bad],s=20,facecolors='none',edgecolors='red',label='感知无效、保持旧舵')
    ax.axhline(1445,c='gray',ls=':',lw=1);ax.axvline(stop,c='green',ls='--',lw=1,label='点停止（主机）')
    ax.set_ylim(1150,1740);ax.set_xlim(-.3,stop+.5);ax.grid(alpha=.2)
    ax.set_title(tag+(' 左弯' if tag in ('222930','223105','224402') else ' 右弯'))
    ax.set_xlabel('约对应采集/视频的秒数；仍含未知固定延迟')
    ax.set_ylabel('舵机 PWM');ax.legend(fontsize=7,loc='best')
    events=[]
    for a,b in zip(display,display[1:]):
        if abs(b['servo_pwm']-a['servo_pwm'])>=60 or b['action_reason']!=a['action_reason']:
            events.append({k:[a.get(k),b.get(k)] for k in ('control_seq','estimated_capture_t','host_t','servo_pwm','action_reason','geometry_reason','gate_reason','target_x','near_a','far_a','far_valid','param_revision')})
    summary.append(dict(tag=tag,min_receive_offset=offset,formal_end=stop,events=events))
    if tag=='223226':
        for r in display:
            if .8<r['estimated_capture_t']<2.8:
                print({k:round(r[k],3) if isinstance(r.get(k),float) else r.get(k) for k in ('control_seq','estimated_capture_t','host_t','servo_pwm','candidate_pwm','action_reason','geometry_reason','gate_reason','target_x','near_a','far_a','far_valid')})
axs.flat[-1].axis('off')
fig.suptitle('7 次实跑的舵机指令：逐序号关联；红圈是感知无效，不能解释为正在有效转向',fontsize=14)
fig.tight_layout(rect=(0,0,1,.965));fig.savefig(ROOT/'steering_trace.png',dpi=130)
(ROOT/'event_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
