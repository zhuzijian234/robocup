"""第二批实采点云与镜像；有新几何证据才恢复，无证据仍拒绝。"""
import json
from check_m10p import P, OUT, run

checks = [run('race_path', P/'test/test_race_path.c')]
cases = json.loads((P/'test/path_replay_20261011.json').read_text(encoding='utf-8'))
recover = {2623, 7350, 7351, 15580, 17680, 2053, 2054, 2055, 2056}
reject = {2716, 2719, 11815, 11816, 11818, 15645, 15646, 15647, 15648,
          17720, 17721, 17722, 17723}
source = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"',
          '#include <stdio.h>',
          'int main(void){PathGeometry g;PathObservation a,b,fast;unsigned i;']
for case in cases:
    seq = case['seq']
    pts = ','.join('{%d,%d}' % tuple(v) for v in case['points'])
    source += ['{PathPoint p[]={'+pts+'};', 'memset(&g,0,sizeof g);',
               f'Path_Build(&g,p,{len(case["points"])},500,700,-27,&a);',
               f'printf("{case["tag"]}:{seq} valid=%u model=%u\\n",a.valid,a.model);']
    if seq in recover:
        source += [f'if(!a.valid || !a.model) return {seq%200+1};']
    if seq in reject:
        source += ['if(a.valid) return 202;']
    old=case['baseline']
    if old[0]:
        source += ['if(!a.valid)return 206;']
        # 仅明确的弧形接管允许改变旧有效几何，其余有效路径仍逐毫米保持。
        source += [f'if(!a.model && (fabsf(a.target_x-({old[1]:.9f}f))>.01f || fabsf(a.ref_y-({old[2]:.9f}f))>.01f))return 207;']
    source += ['memset(&g,0,sizeof g);',
               f'Path_Build(&g,p,{len(case["points"])},500,900,-27,&fast);',
               'if(a.model==2 && (!fast.valid || fast.model!=2 || fabsf(a.target_x-fast.target_x)>.01f || fabsf(a.ref_y-fast.ref_y)>.01f))return 208;',
               'if(a.valid && !a.model && (fabsf(a.near_a)>.14f || (a.far_valid && fabsf(a.far_a)>.14f)) && (!fast.valid || fabsf(a.target_x-fast.target_x)>.01f || fabsf(a.ref_y-fast.ref_y)>.01f))return 209;']
    source += ['for(i=0;i<sizeof(p)/sizeof(p[0]);++i)p[i].x=-p[i].x;',
               'for(i=0;i<sizeof(p)/sizeof(p[0])/2;++i){unsigned k=sizeof(p)/sizeof(p[0])-1-i;PathPoint t=p[i];p[i]=p[k];p[k]=t;}',
               'memset(&g,0,sizeof g);',
               f'Path_Build(&g,p,{len(case["points"])},500,700,27,&b);',
               'if(a.valid!=b.valid || a.model!=b.model) return 203;',
               'if(a.valid && (fabsf(a.target_x+b.target_x)>1 || fabsf(a.ref_y-b.ref_y)>1)) return 204;',
               'if(a.valid && (!isfinite(a.target_x) || !isfinite(a.ref_y))) return 205;}',]
source += ['puts("PASS 63 recorded clouds and 63 mirrored clouds");return 0;}']
target=OUT/'race_replay.c';target.write_text('\n'.join(source),encoding='utf-8')
checks.append(run('race_replay',target))
(OUT/'race_summary.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
