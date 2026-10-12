"""对同组连续快照运行旧/新生产C，跨采集缺口清状态；不是闭环行驶仿真。"""
from pathlib import Path
import json, sys
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
sys.path.insert(0,str(P/'test'))
from check_m10p import OUT,run
V=ROOT/'调试视频/10.12/成功过弯_AI可读_20261012/复核与控制方案'
cases=[]
for r in json.loads((P/'test/path_replay_s_20261012.json').read_text(encoding='utf-8')):
    cases.append(dict(kind='S',tag=r['tag'],seq=r['seq'],points=r['points'],width=r['width']))
for tag in ('160711','161100','161233','161426','161746','162029','164512','164616','164757','164942','165050','165134'):
    for r in json.loads((V/tag/'clouds.json').read_text(encoding='utf-8')):
        cases.append(dict(kind='acute',tag=tag,seq=r['meta']['control_seq'],points=r['points'],width=500))
def program(source):
    code=['#include "'+source.as_posix()+'"','#include <stdio.h>',
          'int main(void){PathGeometry g={0};PathController c={0};PathObservation p;PathCommand o;unsigned now=0,pwm=1445;',
          'PathGains gains={.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};']
    prev=None
    for r in cases:
        if not prev or prev['tag']!=r['tag'] or prev['seq']+1!=r['seq']:
            width=r['width'] if 400<=r['width']<=900 else 500
            code+=['memset(&g,0,sizeof g);memset(&c,0,sizeof c);pwm=1445;',
                   f'g.width={width:.6f}f;g.configured_width=500;g.measured=1;']
        code+=['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(p) for p in r['points'])+'};',
               f'Path_Build(&g,v,{len(r["points"])},500,900,10,&p);',
               'now+=80000;Path_Control(&c,&p,&gains,1,now,pwm,&o);pwm=o.pwm;',
               'printf("%u %u %.6f %.6f %.6f %u %u %u %.6f %u\\n",p.valid,p.model,p.target_x,p.ref_y,p.outer_offset,o.pwm,o.reason,o.applied,o.error,o.transition_limited);',
               '}']
        prev=r
    return '\n'.join(code+['return 0;}'])
base=ROOT/'tmp/acute_implementation_20261012/baseline_path_track.c'
for label,src in [('old',base),('new',P/'HARDWARE/CENTRE_LINE/path_track.c')]:
    f=OUT/f'acute_sequence_{label}.c';f.write_text(program(src),encoding='utf-8')
    run('acute_sequence_'+label,f)
    rows=(OUT/f'acute_sequence_{label}_run.log').read_text().splitlines()
    values=[list(map(float,r.split())) for r in rows if r.strip()]
    assert len(values)==len(cases)
    for r,v in zip(cases,values):r[label]=v
differences=[{k:r[k] for k in ('kind','tag','seq','old','new')} for r in cases if r['old']!=r['new']]
(OUT/'acute_sequence_differences.json').write_text(json.dumps(differences,indent=2),encoding='utf-8')
(P/'test/path_sequence_acute_20261012.json').write_text(json.dumps(cases,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
print('changed',len(differences),'S',[r['seq'] for r in differences if r['kind']=='S'])
