"""Recheck seq5825 at recorded width/speed; helpers get independent geometry copies."""
from pathlib import Path
import json,sys
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(P/'test'))
from check_m10p import run
clouds=json.loads((OUT/'telemetry/223226/clouds.json').read_text(encoding='utf-8'))
c=next(c for c in clouds if c['meta']['control_seq']==5825)
v=c['points']; n=len(v)
code=['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"','#include <stdio.h>',
      'int main(void){PathPoint v[]={'+','.join('{%s,%s}'%tuple(p) for p in v)+'};',
      'PathGeometry base={0},a,b;PathObservation line,candidate,result;',
      'base.configured_width=500;base.width=522.075256f;base.measured=1;',
      'base.history=1;base.history_model=2;base.history_a=.39156923f;',
      'a=base;b=base;',
      f'build_lines(&a,v,{n},500,Path_Preview(700,15.2f),10,&line);candidate=line;',
      f'printf("curve_ok=%u ",recover_curve(v,{n},a.width,Path_Preview(700,15.2f),10,&candidate));',
      f'Path_Build(&b,v,{n},500,Path_Preview(700,15.2f),10,&result);',
      'printf("line=%.3f curve=%.3f final=%.3f model=%u near=%.4f rms=%.3f width=%.3f\\n",line.target_x,candidate.target_x,result.target_x,result.model,result.near_a,line.near_fit[0].rms,a.width);return 0;}']
(OUT/'verify_t05.c').write_text('\n'.join(code),encoding='utf-8')
r=run('new_t05_independent',OUT/'verify_t05.c')
(OUT/'verify_t05.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
print(r)
