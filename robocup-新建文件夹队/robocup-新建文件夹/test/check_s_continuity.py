"""真实S弯快照与镜像：补充连接必须有现场支持，既有有效/拒绝结果不得退步。"""
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
    if case['seq'] in (3127, 3130, 4509, 6681):
        code += ['if(!a.valid||!a.far_valid||a.model!=0||a.ref_y!=700||a.target_x<100)return 3;']
    else:
        code += [f'if(a.model!={base[1]} || a.far_valid!={base[2]} || '
                 f'fabsf(a.target_x-({base[3]:.6f}f))>.01f || '
                 f'fabsf(a.ref_y-({base[4]:.6f}f))>.01f)return 4;']
    if case['seq'] in (4509, 6681):
        # 候选局部中心线上放入当前占用，补充连接必须撤销，不能只追求远段有效。
        code += ['{PathPoint obstacle[401],q;PathFit *n=&a.near_fit[0],*f=&a.far_fit[0];',
                 'q.y=550;q.x=.5f*(at(n,400)+a.width*.5f*sqrtf(1+n->a*n->a)+a.target_x);',
                 'memcpy(obstacle,p,sizeof p);obstacle[sizeof(p)/sizeof(p[0])]=q;',
                 'PathPoint samples[12];uint16_t count=boundary(p,sizeof(p)/sizeof(p[0]),-1,a.width,10,samples);',
                 'if(supported_turn_join(samples,count,obstacle,sizeof(p)/sizeof(p[0])+1,n,f,0,a.width,700))return 5;',
                 'f->rejected=FIT_RMS;f->valid=0;',
                 'if(supported_turn_join(samples,count,p,sizeof(p)/sizeof(p[0]),n,f,0,a.width,700))return 6;',
                 'f->rejected=FIT_SLOPE;n->a=-fabsf(n->a);',
                 'if(supported_turn_join(samples,count,p,sizeof(p)/sizeof(p[0]),n,f,0,a.width,700))return 7;}',
                 init, call]
    code += ['for(i=0;i<sizeof(p)/sizeof(p[0]);++i)p[i].x=-p[i].x;',
             'for(i=0;i<sizeof(p)/sizeof(p[0])/2;++i){unsigned k=sizeof(p)/sizeof(p[0])-1-i;PathPoint t=p[i];p[i]=p[k];p[k]=t;}',
             init, f'Path_Build(&g,p,{len(case["points"])},500,900,-10,&b);',
             'if(a.valid!=b.valid || a.model!=b.model || a.far_valid!=b.far_valid)return 8;',
             'if(a.valid && (fabsf(a.target_x+b.target_x)>1 || fabsf(a.ref_y-b.ref_y)>1))return 9;}',]
code += ['puts("PASS 32 S snapshots + 32 mirrors; 4 supported recoveries, 28 unchanged; blocked/RMS/reverse rejected");return 0;}']
file = OUT/'s_continuity.c'
file.write_text('\n'.join(code), encoding='utf-8')
run('s_continuity', file)
run('s_phase', P/'test/test_s_phase.c')
