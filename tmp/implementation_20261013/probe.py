"""生产函数回放：只连接同组、同修订、连续序号，缺帧不补造。"""
import json, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
P = ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
sys.path.insert(0, str(P/'test'))
from check_m10p import run
frames = json.loads((ROOT/'tmp/new_turn_review_20261012/control/frames.json').read_text(encoding='utf-8'))
code = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"', '#include <stdio.h>',
        'int main(void){PathGeometry g={0};PathObservation p;AcuteWall l,r;']
previous = None
for f in frames:
    tag, seq, rev = f['tag'], int(f['key'][3]), int(f['key'][4])
    rows = json.loads((ROOT/f'tmp/new_turn_review_20261012/telemetry/{tag}/joined.json').read_text(encoding='utf-8'))
    row = next((r for r in rows if r['control_seq']==seq and r['param_revision']==rev), {})
    width = row.get('track_width', 500)
    speed = row.get('Speed_now', 15)
    if not previous or previous != (tag, seq-1, rev):
        code += [f'memset(&g,0,sizeof g);g.width={width:.6f}f;g.configured_width=500;g.measured=1;']
    code += ['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(v) for v in f['points'])+'};',
             f'Path_Build(&g,v,{len(f["points"])},500,Path_Preview(700,{speed:.6f}f),10,&p);',
             f'l=acute_wall(v,{len(f["points"])},-1);r=acute_wall(v,{len(f["points"])},1);',
             f'printf("{tag},{seq},{rev},%u,%u,%.2f,%.2f,%u,%u,%u,%u,%u,%.3f,%.3f,%.1f,%.1f\\n",p.valid,p.model,p.target_x,p.near_a,p.acute_phase,p.acute_checks,l.linked,r.linked,l.valid,l.yaw,r.yaw,l.distance,r.distance);}}']
    previous = tag,seq,rev
code += ['return 0;}']
src = Path(__file__).with_suffix('.c')
src.write_text('\n'.join(code), encoding='utf-8')
result = run('new_acute_probe', src)
src.with_suffix('.txt').write_text(result['output'], encoding='utf-8')
