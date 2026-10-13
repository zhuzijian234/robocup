"""Replay only complete real snapshots; reset every missing sequence/revision/run."""
from pathlib import Path
import json, subprocess, os
O=Path(__file__).resolve().parent; R=O.parents[2]
P=R/'robocup-新建文件夹队/robocup-新建文件夹'
T=O.parent/'telemetry'
env=dict(os.environ)
bat=O/'env.bat';bat.write_text('@echo off\ncall "D:\\VS\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul 2>&1\nset\n')
r=subprocess.run(['cmd','/d','/s','/c',str(bat)],capture_output=True,check=True)
for line in r.stdout.decode(errors='replace').splitlines():
    k,s,v=line.partition('=')
    if k and s:env[k]=v
frames=[]
for folder in sorted(T.iterdir()):
    if not folder.is_dir():continue
    rows=json.loads((folder/'joined.json').read_text(encoding='utf-8'))
    configs=json.loads((folder/'meta.json').read_text(encoding='utf-8'))['configs']
    for f in json.loads((folder/'clouds.json').read_text(encoding='utf-8')):
        m=f['meta'];key=(m['session'],m['control_seq'],m['param_revision'])
        row=next(x for x in rows if (x['session'],x['control_seq'],x['param_revision'])==key)
        f.update(tag=folder.name,row=row,cfg=configs[f"{key[0]}:{key[2]}"])
        frames.append(f)
(O/'frames.json').write_text(json.dumps(frames,ensure_ascii=False),encoding='utf-8')
code=['#include "'+(O.parent/'baseline/path_track.c').as_posix()+'"','#include <stdio.h>',
      'static void pf(const char*s,PathFit f){printf("FIT %s valid=%d rejected=%d n=%d a=%.4f b=%.1f y=%.1f..%.1f rms=%.2f gap=%.1f\\n",s,f.valid,f.rejected,f.count,f.a,f.b,f.min_y,f.max_y,f.rms,f.gap);}',
      'int main(void){PathGeometry g={0},temp;PathObservation p,raw,q;AcuteWall l,r;']
prev=None
for f in frames:
    m,row,cfg=f['meta'],f['row'],f['cfg'];tag=f['tag'];seq=m['control_seq'];n=len(f['points']);rev=m['param_revision']
    width=row['track_width'];speed=row['Speed_now']
    if prev!=(tag,seq-1,rev):
        code += [f'memset(&g,0,sizeof g);g.width={width:.6f}f;g.configured_width=500;g.measured=1;']
    code+=['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(x) for x in f['points'])+'};',f'printf("FRAME {tag} {seq} actual_valid={row["command_applied"]} actual_pwm={row["final_pwm"]} actual_x={row["target_x"]:.1f} actual_y={row["ref_y"]:.1f} actual_geom={row["geometry_reason"]}\\n");']
    code += [f'temp=g;build_lines(&temp,v,{n},500,Path_Preview(700,{speed:.6f}f),10,&raw);printf("RAW valid=%d geom=%d source=%d dup=%d target=%.1f ref_y=%.1f a=%.3f far_a=%.3f\\n",raw.valid,raw.geometry_reason,raw.source,raw.duplicate_samples,raw.target_x,raw.ref_y,raw.near_a,raw.far_a);pf("LN",raw.near_fit[0]);pf("RN",raw.near_fit[1]);pf("LF",raw.far_fit[0]);pf("RF",raw.far_fit[1]);']
    code += [f'Path_Build(&g,v,{n},500,Path_Preview(700,{speed:.6f}f),10,&p);l=acute_wall(v,{n},-1);r=acute_wall(v,{n},1);',
      'printf("FINAL valid=%d geom=%d model=%d source=%d target=%.2f ref_y=%.1f near_a=%.3f far_a=%.3f phase=%d checks=%d reason=%d dir=%d linked=%d/%d wallvalid=%d/%d yaw=%.3f/%.3f distance=%.1f/%.1f curves=%u\\n",p.valid,p.geometry_reason,p.model,p.source,p.target_x,p.ref_y,p.near_a,p.far_a,p.acute_phase,p.acute_checks,p.acute_reason,p.acute_direction,l.linked,r.linked,l.valid,r.valid,l.yaw,r.yaw,l.distance,r.distance,p.curve_checks);']
    code += [f'for(int bb=0,ee;bb<{n};bb=ee){{ee=bb+1;while(ee<{n} && point_distance(v[ee-1],v[ee])<100)++ee; if(ee-bb<10)continue;for(int s=-1;s<=1;s+=2){{q=raw;int ok=curve_candidate(v,{n},bb,ee,s,g.width,900,10,&q);printf("CURVE cluster=%d..%d side=%d ok=%d checks=%u target=%.1f support=%.1f\\n",bb,ee,s,ok,q.curve_checks,q.target_x,q.support_mm);}}}}','}']
    prev=(tag,seq,rev)
code+=['return 0;}']
(O/'probe.c').write_text('\n'.join(code),encoding='utf-8')
cmd=['D:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe','/nologo','/std:c11','/utf-8','/Od','/W4',str(O/'probe.c'),'/Fe:'+str(O/'probe.exe'),'/Fo:'+str(O/'probe.obj')]
r=subprocess.run(cmd,env=env,capture_output=True);(O/'compile.log').write_bytes(r.stdout+r.stderr);r.check_returncode()
r=subprocess.run([str(O/'probe.exe')],capture_output=True);(O/'probe.txt').write_bytes(r.stdout+r.stderr);r.check_returncode()
print('Probed',len(frames),'complete clouds:',O/'probe.txt')
