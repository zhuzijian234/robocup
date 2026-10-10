"""回放四次实车早期快照及其镜像；数据独立保存，不依赖用户log目录。"""
import json
from check_m10p import P, OUT, run

fixtures = json.loads((P/'test/path_replay_20261010.json').read_text(encoding='utf-8'))
source = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"',
          '#include <stdio.h>',
          'int main(void) { PathGeometry g; PathObservation o; unsigned i;']
for j, case in enumerate(fixtures):
    # 23485远段RMS超限，不能为了消除拒绝帧而放宽生产拟合阈值。
    expected = 0 if case['seq'] == 23485 else 1
    pts = ','.join('{%d,%d}' % tuple(p) for p in case['points'])
    source += ['{ PathPoint points[] = {'+pts+'};',
               'memset(&g,0,sizeof g);',
               f'Path_Build(&g,points,{len(case["points"])},500,700,-27,&o);',
               f'printf("seq {case["seq"]}: valid=%u source=%u reason=%u\\n",o.valid,o.source,o.geometry_reason);',
               f'if (o.valid != {expected}) return 1;',
               'for(i=0;i<sizeof(points)/sizeof(points[0]);++i) points[i].x=-points[i].x;',
               # 镜像后反转射线排列，保持生产端的按角度递增约定。
               'for(i=0;i<sizeof(points)/sizeof(points[0])/2;++i) {',
               'PathPoint t=points[i]; unsigned k=sizeof(points)/sizeof(points[0])-1-i;',
               'points[i]=points[k]; points[k]=t;}',
               'memset(&g,0,sizeof g);',
               f'Path_Build(&g,points,{len(case["points"])},500,700,27,&o);',
               f'if (o.valid != {expected}) return 2; }}']
source += ['puts("PASS 16 recorded clouds and 16 mirrored clouds");return 0;}']
target = OUT/'path_replay.c'
target.write_text('\n'.join(source), encoding='utf-8')
result = run('path_replay', target)
(OUT/'path_replay_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
