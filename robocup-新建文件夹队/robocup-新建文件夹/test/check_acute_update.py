"""回退验收：新批次56云及镜像全部恢复r2结果，缺帧/修订处断开状态。"""
import json
from check_m10p import P, OUT, run

cases=json.loads((P/'test/path_sequence_acute_20261013.json').read_text(encoding='utf-8'))['cases']
code=['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"', '#include <stdio.h>',
      'int main(void){PathGeometry g={0};PathObservation p;PathController c={0};PathCommand o;unsigned now=0,pwm=1445;',
      'PathGains gains={.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};']
fields=['p.valid','p.model','p.target_x','p.ref_y','p.near_a','p.acute_phase','o.pwm','o.reason','o.applied','o.error']
for mirror in (1,-1):
    previous=None
    code += [f'gains.center_x={mirror*10};']
    for case in cases:
        key=case['tag'],case['seq'],case['revision']
        if previous != (key[0],key[1]-1,key[2]):
            code += ['memset(&g,0,sizeof g);memset(&c,0,sizeof c);pwm=1445;',
                     f'g.width={case["width"]:.6f}f;g.configured_width=500;g.measured=1;']
        pts=case['points'] if mirror==1 else [[-x,y] for x,y in reversed(case['points'])]
        n=len(pts)
        preview=f'Path_Preview(700,{case["speed"]:.6f}f)'
        code += ['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(p) for p in pts)+'};',
                 f'Path_Build(&g,v,{n},500,{preview},gains.center_x,&p);',
                 'now+=83333;Path_Control(&c,&p,&gains,1,now,pwm,&o);pwm=o.pwm;']
        expected=case['r2_expected' if mirror==1 else 'r2_mirror_expected']
        tests=[f'fabsf({field}-({value:.6f}f))<.002f' for field,value in zip(fields,expected)]
        code += ['if(!('+'&&'.join(tests)+')){printf("FAIL '+str(key)+f' mirror={mirror} target=%.3f PWM=%u\\n",p.target_x,o.pwm);return 4;'+'}}']
        previous=key
code += ['puts("PASS 112 new/mirrored clouds: all geometry and PWM restored to r2 baseline");return 0;}']
src=OUT/'acute_update_replay.c'
src.write_text('\n'.join(code),encoding='utf-8')
result=run('acute_update_replay',src)
(OUT/'acute_update_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
