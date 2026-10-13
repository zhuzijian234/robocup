from pathlib import Path
import json,sys
O=Path(__file__).resolve().parent;R=O.parents[2];P=R/'robocup-新建文件夹队/robocup-新建文件夹'
sys.path.insert(0,str(P/'test'))
from check_m10p import run
fs=json.loads((O/'frames.json').read_text())
code=['#include "instrumented.c"','#include <stdio.h>','static void pf(const char*s,PathFit f){printf("%s valid=%d rejected=%d n=%d a=%.4f b=%.1f y=%.1f..%.1f rms=%.2f gap=%.1f\\n",s,f.valid,f.rejected,f.count,f.a,f.b,f.min_y,f.max_y,f.rms,f.gap);}','int main(void){PathGeometry g={0};PathObservation p,before,q;PathPoint samples[2][STRIPS];uint16_t n[2];g.width=500;g.configured_width=500;g.measured=1;']
for f in fs:
 if f['tag']!='223226' or int(f['key'][3])>5825:continue
 if f['key'][3]=='5823':code+=['Path_ClearHistory(&g);']
 n=len(f['points']);seq=f['key'][3]
 code+=['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(x) for x in f['points'])+'};']
 code+=[f'printf("FRAME {seq} prior_a=%.3f prior_model=%d width=%.1f\\n",g.history_a,g.history_model,g.width);']
 code+=[f'for(int s=0;s<2;++s){{n[s]=boundary(v,{n},s?1:-1,g.width,10,samples[s]);printf("BOUNDARY s=%d n=%d ",s,n[s]);for(int j=0;j<n[s];++j)printf("(%.1f %.1f)",samples[s][j].x,samples[s][j].y);puts("");}}']
 code+=[f'build_lines(&g,v,{n},500,900,10,&before); printf("LINES valid=%d geom=%d source=%d duplicate=%d target=%.2f y=%.2f near=%.3f\\n",before.valid,before.geometry_reason,before.source,before.duplicate_samples,before.target_x,before.ref_y,before.near_a);pf("LN",before.near_fit[0]);pf("RN",before.near_fit[1]);pf("LF",before.far_fit[0]);pf("RF",before.far_fit[1]);']
 code+=[f'for(int bb=0,ee;bb<{n};bb=ee){{ee=bb+1;while(ee<{n} && point_distance(v[ee-1],v[ee])<100)++ee;for(int s=-1;s<=1;s+=2){{q=before;int ok=curve_candidate(v,{n},bb,ee,s,g.width,900,10,&q);printf("CURVE cluster=%d..%d side=%d ok=%d checks=%u target=%.1f support=%.1f\\n",bb,ee,s,ok,q.curve_checks,q.target_x,q.support_mm);}}}}']
 code+=[f'Path_Build(&g,v,{n},500,900,10,&p);printf("FINAL model=%d source=%d target=%.2f near=%.3f history=%.3f\\n",p.model,p.source,p.target_x,p.near_a,g.history_a);}}']
code+=['return 0;}']
(O/'trace_t05.c').write_text('\n'.join(code),encoding='utf-8')
res=run('acute_t05_trace',O/'trace_t05.c');(O/'trace_t05.txt').write_text(res['output'],encoding='utf-8')
