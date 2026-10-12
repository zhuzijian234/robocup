"""实车退步后核对32份S弯快照恢复到修改前几何基线，不冒充动态实车验证。"""
import json
from check_m10p import P, OUT, run

cases = json.loads((P/'test/path_replay_s_20261012.json').read_text(encoding='utf-8'))
code = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"',
        '#include <stdio.h>',
        'int main(void){PathGeometry g;PathObservation a,b;unsigned i;']
for case in cases:
    pts = ','.join('{%d,%d}' % tuple(v) for v in case['points'])
    init = (f'memset(&g,0,sizeof g);g.width={case["width"]:.9f}f;'
            'g.configured_width=500;g.measured=1;')
    call = f'Path_Build(&g,p,{len(case["points"])},500,900,10,&a);'
    base = case['baseline']
    code += ['{PathPoint p[]={'+pts+'};', init, call,
             f'if(a.valid!={base[0]})return 1;',
             'if(a.valid && (!isfinite(a.target_x)||!isfinite(a.ref_y)))return 2;']
    # 包含此前被当成“恢复成功”的四帧；实车未通过，因此全部核对原始基线。
    code += [f'if(a.model!={base[1]} || a.far_valid!={base[2]} || '
             f'fabsf(a.target_x-({base[3]:.6f}f))>.01f || '
             f'fabsf(a.ref_y-({base[4]:.6f}f))>.01f)'
             f'{{printf("FAIL baseline {case["tag"]}/{case["seq"]}\\n");return 4;}}']
    code += ['for(i=0;i<sizeof(p)/sizeof(p[0]);++i)p[i].x=-p[i].x;',
             'for(i=0;i<sizeof(p)/sizeof(p[0])/2;++i){unsigned k=sizeof(p)/sizeof(p[0])-1-i;PathPoint t=p[i];p[i]=p[k];p[k]=t;}',
             init, f'Path_Build(&g,p,{len(case["points"])},500,900,-10,&b);',
             'if(a.valid!=b.valid || a.model!=b.model || a.far_valid!=b.far_valid)return 8;',
             'if(a.valid && (fabsf(a.target_x+b.target_x)>1 || fabsf(a.ref_y-b.ref_y)>1))return 9;}',]
code += ['puts("PASS 32 S snapshots restored to pre-improvement baseline + 32 mirrors");return 0;}']
file = OUT/'s_continuity.c'
file.write_text('\n'.join(code), encoding='utf-8')
run('s_continuity', file)
