"""一次性从修改前模块生成黄金结果，测试本身不依赖临时备份或用户日志。"""
from pathlib import Path
import sys,csv,json,hashlib,math
root=Path(__file__).resolve().parents[1]
p=root/'robocup-新建文件夹队/robocup-新建文件夹'
sys.path.insert(0,str(p/'test'))
from check_m10p import OUT,run
old=root/'tmp/round2_baseline/HARDWARE/CENTRE_LINE/path_track.c'
rows=list(csv.DictReader((root/'tmp/review_20261011/220019_joined.csv').open(encoding='utf-8-sig')))
items=[]
for r in rows:
    if not 2030<=int(r['control_seq'])<=2049:continue
    fields={k:float(r[v]) for k,v in dict(near_x='near_x',far_x='far_x',near_a='near_a',far_a='far_a',ref_y='ref_y',target_x='target_x',width='track_width',avoid_offset='avoid_offset',near_ref_y='near_ref_y',avoid_state='avoid_state',far_valid='far_valid',source='source').items()}
    fields['valid']=int(float(r['geometry_reason'])==0)
    fields['straight']=int(fields['far_valid'] and abs(fields['near_a'])<.1 and abs(fields['far_a'])<.1 and abs(fields['near_a']-fields['far_a'])<.1 and abs(fields['far_x']-fields['near_x'])<60)
    for side,label in enumerate(['left','right']):
        for suffix in ['count','rms']:
            fields[f'near_fit[{side}].{suffix}']=float(r[label+('_near_count' if suffix=='count' else '_rms')])
        fields[f'near_fit[{side}].valid']=int((int(float(r['fit_rejected']))>>(side*8))&255==0)
    items.append(dict(seq=int(r['control_seq']),now=int(float(r['control_end_ms'])*1000),actual=1445 if not items else int(float(rows[rows.index(r)-1]['servo_pwm'])),path=fields))
assert len(items)==20
def clit(v):return str(int(v)) if float(v).is_integer() else format(v,'.9g')+'f'
lines=['#include "'+old.as_posix()+'"','#include <stdio.h>','int main(void){PathObservation p;PathController c={0};PathCommand o;PathGains g={.035f,.035f,.040f,.022f,.0395f,.020f,-27,200,1170,1445,1720};']
for r in items:
    lines+=['memset(&p,0,sizeof p);']+[f'p.{k}={clit(v)};' for k,v in r['path'].items()]
    lines+=[f'Path_Control(&c,&p,&g,1,{r["now"]}u,{r["actual"]},&o);', 'printf("%u %u %u %u %u %u %d %.9g %.9g\\n",o.pwm,o.candidate_pwm,o.applied,o.reason,o.gate_reason,o.d_reset,o.bend,o.p,o.d);']
lines+=['return 0;}']
src=OUT/'baseline_preservation.c';src.write_text('\n'.join(lines),encoding='utf-8')
res=run('baseline_preservation',src)
for r,line in zip(items,filter(str.strip,res['output'].splitlines())):r['expected']=list(map(float,line.split()))
artifact=dict(baseline_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),source='v2_1010_220019_107147375000000.bin: control 2030..2049; recorded geometry, not simulated vehicle trajectory',cases=items)
(p/'test/path_control_baseline_20261011.json').write_text(json.dumps(artifact,ensure_ascii=False,indent=2),encoding='utf-8')
# 独立几何黄金结果：每帧从相同500mm初始宽度开始，不虚构快照之间的连续性。
cases=json.loads((p/'test/path_replay_20261011.json').read_text(encoding='utf-8'))
lines=['#include "'+old.as_posix()+'"','#include <stdio.h>','int main(void){PathGeometry g;PathObservation p;']
for r in cases:
    lines+=['{PathPoint a[]={'+','.join('{%d,%d}'%tuple(v) for v in r['points'])+'};memset(&g,0,sizeof g);',f'Path_Build(&g,a,{len(r["points"])},500,700,-27,&p);','printf("%u %.9g %.9g %.9g\\n",p.valid,p.target_x,p.ref_y,p.avoid_offset);','}']
lines+=['return 0;}'];src=OUT/'baseline_geometry.c';src.write_text('\n'.join(lines),encoding='utf-8')
res=run('baseline_geometry',src)
for r,line in zip(cases,filter(str.strip,res['output'].splitlines())):r['baseline']=list(map(float,line.split()))
(p/'test/path_replay_20261011.json').write_text(json.dumps(cases,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
