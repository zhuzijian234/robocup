from pathlib import Path
import csv,json,sys,shutil,collections
ROOT=Path(__file__).resolve().parents[3]
P=ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(P/'test'))
from check_m10p import run
tags=['222930','223105','223130','223154','223226','223303','224402']
src=(P/'HARDWARE/CENTRE_LINE/path_track.c').read_text(encoding='utf-8')
shutil.copy(P/'HARDWARE/CENTRE_LINE/path_track.h',OUT/'path_track.h')
src=src.replace('#define ACUTE_BINS 24', '#define ACUTE_BINS 24\ntypedef struct{int why, best,n;float first,last,rms,departure,peak;int valid;float distance,a,b,yaw;unsigned begin,end;} Trace;\nstatic Trace trace[2];\nstatic int prepare_gate;')
start=src.index('static AcuteWall acute_wall(');end=src.index('static void acute_prepare(',start)
a=src[start:end]
a=a.replace('AcuteWall out = {0};','AcuteWall out = {0}; Trace *t=&trace[side>0]; memset(t,0,sizeof(*t));')
a=a.replace('if (best < 8) return out;','t->best=best;t->begin=out.begin;t->end=out.end; if (best < 8) {t->why=1;return out;}')
a=a.replace('if (n < 5 || first > 180 || last < 325 || last - first < 200) return out;', 't->n=n;t->first=first;t->last=last; if (n < 5 || first > 180 || last < 325 || last - first < 200) {t->why=2;return out;}')
a=a.replace('if (yy < 1) return out;', 'if (yy < 1) {t->why=3;return out;}')
a=a.replace('if (sqrtf(residual / n) > 12) return out;', 't->rms=sqrtf(residual/n);t->a=out.a;t->b=out.b;t->yaw=atanf(out.a); if (sqrtf(residual / n) > 12) {t->why=4;return out;}')
a=a.replace('{ out.distance = 0; return out; }','{t->why=5;t->departure=out.distance;t->peak=peak_turn;t->valid=out.valid;out.distance = 0; return out;}')
a=a.replace('if (out.valid != 2) out.distance = 0;', 't->departure=out.distance;t->peak=peak_turn; if (out.valid != 2) out.distance = 0; t->why=out.valid==2?7:6;t->valid=out.valid;t->distance=out.distance;')
src=src[:start]+a+src[end:]
start=src.index('static void acute_prepare(');end=src.index('static void baseline_curve_offset(',start)
a=src[start:end]
a=a.replace('AcuteWall wall[2];','AcuteWall wall[2];prepare_gate=0;')
# Numeric gate is actual ordered return branch, not only public reason.
i=0
parts=a.split('return;')
for k in range(len(parts)-1):
 i+=1;parts[k]+=f'prepare_gate={i};return;'
a=''.join(parts)
a=a.replace('out->target_x = target.x;', 'prepare_gate=99;out->target_x = target.x;')
src=src[:start]+a+src[end:]
(OUT/'instrumented.c').write_text(src,encoding='utf-8')
frames=[]
for tag in tags:
 cloud=next((P/'上位机/log').glob('v2_1012_'+tag+'_*.cloud.csv'))
 by=collections.defaultdict(list)
 for row in csv.DictReader(cloud.open(encoding='utf-8-sig')):
  key=tuple(row[k] for k in ['session','snapshot_id','frame_index','control_seq','param_revision','input_seq'])
  by[key].append([int(row['point_index']),int(row['x_mm']),int(row['y_mm'])])
 for key,pts in by.items():
  frames.append(dict(tag=tag,key=key,points=[[x,y] for _,x,y in sorted(pts)]))
code=['#include "instrumented.c"','#include <stdio.h>','int main(void){PathGeometry g={0};PathObservation p;AcuteWall w[2];Trace l,r;']
last=None
for f in frames:
 tag=f['tag'];key=f['key'];seq=int(key[3]);rev=int(key[4]); n=len(f['points'])
 if last is None or last[0]!=tag or last[1]+1!=seq or last[2]!=rev:
  code+=['memset(&g,0,sizeof(g));g.width=500;g.configured_width=500;g.measured=1;']
 code+=['{PathPoint points[]={'+','.join('{%s,%s}'%tuple(pt) for pt in f['points'])+'};']
 code += [f'w[0]=acute_wall(points,{n},-1);w[1]=acute_wall(points,{n},1);l=trace[0];r=trace[1];prepare_gate=-1;Path_Build(&g,points,{n},500,900,10,&p);']
 code += [f'printf("{tag},{seq},{rev},{key[0]},{key[1]},{key[2]},%d,%d,%d,%d,%d,%d,%.4f,%.4f,%d,%d,%d,%d,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%.2f,%.3f,%.3f,%d,%d,%d,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%.2f,%.3f,%.3f\\n",p.valid,p.model,p.s_preview,p.avoid_state,p.acute_phase,prepare_gate,p.near_a,p.far_a,p.far_fit[0].valid,p.far_fit[1].valid,p.acute_direction,p.acute_reason,p.acute_distance,p.outer_offset,p.far_fit[0].a,p.far_fit[1].a,p.width,p.target_x,p.ref_y,l.why,l.best,l.n,l.first,l.last,l.rms,l.peak,l.departure,l.valid,l.distance,l.a,l.b,r.why,r.best,r.n,r.first,r.last,r.rms,r.peak,r.departure,r.valid,r.distance,r.a,r.b);}}']
 last=(tag,seq,rev)
code+=['return 0;}']
(OUT/'audit_gates.c').write_text('\n'.join(code),encoding='utf-8')
result=run('audit_acute_gates',OUT/'audit_gates.c')
fields='tag,seq,rev,session,snapshot,index,valid,model,s_preview,avoid_state,acute_phase,prepare_gate,near_a,far_a,lfvalid,rfvalid,dir,reason,distance,offset,lf_a,rf_a,width,target_x,ref_y,lwhy,lbest,ln,lfirst,llast,lrms,lpeak,ldeparture,lvalid,ldistance,la,lb,rwhy,rbest,rn,rfirst,rlast,rrms,rpeak,rdeparture,rvalid,rdistance,ra,rb'
(OUT/'gate_results.csv').write_text(fields+'\n'+result['output'],encoding='utf-8')
(OUT/'frames.json').write_text(json.dumps(frames),encoding='utf-8')
print('FRAMES',len(frames))
rows=list(csv.DictReader((OUT/'gate_results.csv').open()))
print('prepare',collections.Counter(r['prepare_gate'] for r in rows))
print('left',collections.Counter(r['lwhy'] for r in rows))
print('right',collections.Counter(r['rwhy'] for r in rows))
