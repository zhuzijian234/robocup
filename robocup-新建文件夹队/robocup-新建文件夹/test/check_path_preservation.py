"""逐帧复核原版良好S弯控制；输入是记录的几何，不冒充闭环行驶仿真。"""
import json
from check_m10p import P,OUT,run

fixture=json.loads((P/'test/path_control_baseline_20261011.json').read_text(encoding='utf-8'))
source=['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"',
        '#include <stdio.h>',
        'int main(void){PathObservation p;PathController c={0};PathCommand o;',
        'PathGains g={.035f,.035f,.040f,.022f,.0395f,.020f,-27,200,1170,1445,1720,12};']
for r in fixture['cases']:
    source+=['memset(&p,0,sizeof p);']
    for k,v in r['path'].items():
        value=str(int(v)) if float(v).is_integer() else format(v,'.9g')+'f'
        source+=[f'p.{k}={value};']
    source+=[f'Path_Control(&c,&p,&g,1,{r["now"]}u,{r["actual"]},&o);']
    checks=[f'o.{k}=={int(v)}' for k,v in zip(['pwm','candidate_pwm','applied','reason','gate_reason','d_reset','bend'],r['expected'])]
    checks += [f'fabsf(o.{k}-({v:.9f}f))<.001f' for k,v in zip(['p','d'],r['expected'][7:])]
    source += ['if(!('+' && '.join(checks)+f')){{printf("FAIL S sequence {r["seq"]}\\n");return 1;}}']
source+=['puts("PASS original S-turn 2030..2049: PWM, gate, P/D and reset unchanged at speed 12");return 0;}']
src=OUT/'path_preservation.c';src.write_text('\n'.join(source),encoding='utf-8')
run('path_preservation',src)
