"""新批次56云及镜像：只允许T05反向目标修复改变旧结果，缺帧/修订处断开状态。"""
import json
from check_m10p import P, OUT, run

cases=json.loads((P/'test/path_sequence_acute_20261013.json').read_text(encoding='utf-8'))['cases']
code=['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"', '#include <stdio.h>',
      'int main(void){PathGeometry g={0},previous,other;PathObservation p,line,candidate;PathController c={0};PathCommand o;unsigned now=0,pwm=1445,i;',
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
        code += ['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(p) for p in pts)+'};', 'previous=g;',
                 f'Path_Build(&g,v,{n},500,{preview},gains.center_x,&p);',
                 'now+=83333;Path_Control(&c,&p,&gains,1,now,pwm,&o);pwm=o.pwm;']
        if key[0]=='223226' and key[1]==5825:
            code += [f'if(!p.valid || p.model!=2 || {mirror}*p.target_x<100 || !o.applied || {mirror}*(o.pwm-1445.0f)>=0)return 1;',
                f'printf("T05 mirror={mirror}: target=%.2f model=%u pwm=%u\\n",p.target_x,p.model,o.pwm);',
                f'other=previous;build_lines(&other,v,{n},500,{preview},gains.center_x,&line);',
                '/* 各反例独立起算：新远段、双侧、桶、失效历史均不能被旧弯方向覆盖。 */',
                'for(i=0;i<8;++i){other=previous;candidate=line;',
                'switch(i){case 0:other.history=0;break;case 1:other.history_misses=1;break;',
                'case 2:candidate.far_valid=1;candidate.far_a=candidate.near_a;break;',
                'case 3:candidate.source=PATH_DUAL;break;case 4:candidate.cone_candidates=1;break;',
                'case 5:candidate.avoid_state=AVOID_ACTIVE;break;case 6:other.history_x+=300;break;',
                'case 7:other.history_y+=300;break;}',
                f'if(curve_handoff(&other,v,{n},{preview},gains.center_x,&candidate))return 2;',
                'if(candidate.target_x!=line.target_x)return 3;}']
        else:
            expected=case['r2_expected' if mirror==1 else 'r2_mirror_expected']
            tests=[f'fabsf({field}-({value:.6f}f))<.002f' for field,value in zip(fields,expected)]
            code += ['if(!('+'&&'.join(tests)+')){printf("FAIL '+str(key)+f' mirror={mirror} target=%.3f PWM=%u\\n",p.target_x,o.pwm);return 4;'+'}}']
            previous=key
            continue
        code += ['}']
        previous=key
code += ['puts("PASS 112 new/mirrored clouds; only T05 handoff changes; 16 stale/priority/geometry counterexamples");return 0;}']
src=OUT/'acute_update_replay.c'
src.write_text('\n'.join(code),encoding='utf-8')
result=run('acute_update_replay',src)
(OUT/'acute_update_summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
