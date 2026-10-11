"""只在分析目录编译现有 C 几何代码；不修改工程，量化点云快照的边界重复关联。"""
from pathlib import Path
import json, subprocess, os

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
SRC = ROOT / 'robocup-新建文件夹队/robocup-新建文件夹/HARDWARE/CENTRE_LINE/path_track.c'
code = '#include <stdio.h>\n#include "' + SRC.as_posix() + '"\n' + r'''
int main(void) {
  int id, n, i, j, nl, nr, shared; float width; PathPoint pts[400], l[12], r[12];
  while (scanf("%d %d %f", &id, &n, &width)==3) {
    PathGeometry state={0}; PathObservation obs;
    for (i=0;i<n;i++) if(scanf("%f %f", &pts[i].x,&pts[i].y)!=2) return 2;
    nl=boundary(pts,n,-1,width,10,l); nr=boundary(pts,n,1,width,10,r); shared=0;
    for(i=0;i<nl;i++) for(j=0;j<nr;j++)
      if (point_distance(l[i],r[j])<1) ++shared;
    state.width=width; state.configured_width=500; state.measured=1;
    Path_Build(&state,pts,n,500,900,10,&obs);
    printf("%d %d %d %d %d %d %d %.3f %.3f\n",id,nl,nr,shared,
      obs.geometry_reason,obs.source,obs.model,obs.target_x,obs.ref_y);
  } return 0;
}
'''
(HERE/'probe.c').write_text(code,encoding='utf-8')
(HERE/'env.bat').write_text('@echo off\ncall "D:\\VS\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul 2>&1\nset\n',encoding='ascii')
result=subprocess.run(['cmd','/d','/c',str(HERE/'env.bat')],capture_output=True,check=True)
env=dict(os.environ)
for line in result.stdout.decode(errors='replace').splitlines():
    key,sep,value=line.partition('=')
    if key and sep:env[key]=value
cl='D:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe'
result=subprocess.run([cl,'/nologo','/std:c11','/utf-8','/Od','/W4',str(HERE/'probe.c'),'/Fe:'+str(HERE/'probe.exe'),'/Fo:'+str(HERE/'probe.obj')],env=env,capture_output=True)
(HERE/'probe_build.log').write_bytes(result.stdout+result.stderr)
result.check_returncode()
inputs=[]; cases=[]
for file in sorted(HERE.glob('*_clouds.json')):
    tag=file.stem.split('_')[0]
    rows={r['control_seq']:r for r in json.loads((HERE/f'{tag}_joined.json').read_text(encoding='utf-8'))}
    for cloud in json.loads(file.read_text(encoding='utf-8')):
        seq=cloud['meta']['control_seq'];row=rows.get(seq,{})
        width=row.get('track_width',row.get('paodao_distance',500))
        if not 400<=width<=900:width=500
        inputs.append(f'{len(cases)} {len(cloud["points"])} {width}\n'+'\n'.join(f'{x} {y}' for x,y in cloud['points']))
        cases.append(dict(tag=tag,seq=seq,width=width,recorded_geom=row.get('geometry_reason'),recorded_target=row.get('target_x')))
result=subprocess.run([str(HERE/'probe.exe')],input='\n'.join(inputs).encode(),capture_output=True,check=True)
for line in result.stdout.decode().splitlines():
    i,nl,nr,shared,geom,source,model,tx,ref=line.split()
    cases[int(i)].update(left=int(nl),right=int(nr),shared=int(shared),probe_geom=int(geom),probe_source=int(source),probe_model=int(model),probe_target=float(tx),probe_ref=float(ref))
(HERE/'probe_results.json').write_text(json.dumps(cases,indent=2,ensure_ascii=False),encoding='utf-8')
for c in cases:print(c)
