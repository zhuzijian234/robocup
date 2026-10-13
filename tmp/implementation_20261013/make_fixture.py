"""从已校验BIN导出的56份快照建立固定回归样本；r2函数计算对照，非当前函数自证。"""
import hashlib, json, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
P = ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
sys.path.insert(0, str(P/'test'))
from check_m10p import run
frames = json.loads((ROOT/'tmp/new_turn_review_20261012/control/frames.json').read_text(encoding='utf-8'))
cases = []
for f in frames:
    tag, seq, rev = f['tag'], int(f['key'][3]), int(f['key'][4])
    rows = json.loads((ROOT/f'tmp/new_turn_review_20261012/telemetry/{tag}/joined.json').read_text(encoding='utf-8'))
    row = next((r for r in rows if r['control_seq']==seq and r['param_revision']==rev), {})
    raw = next((P/'上位机/log').glob(f'v2_1012_{tag}_*.bin'))
    cases.append(dict(tag=tag, seq=seq, revision=rev, points=f['points'],
        width=row.get('track_width',500), speed=row.get('Speed_now',15),
        source_bin=raw.name, sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
        width_recorded='track_width' in row, speed_recorded='Speed_now' in row))
code = ['#include "'+(Path(__file__).parent/'baseline/path_track.c').as_posix()+'"', '#include <stdio.h>',
        'int main(void){PathGeometry g={0};PathController c={0};PathObservation p;PathCommand o;unsigned now=0,pwm=1445;',
        'PathGains gains={.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};']
for mirror in (1,-1):
    previous = None
    code += [f'gains.center_x={mirror*10};']
    for f in cases:
        key = f['tag'],f['seq'],f['revision']
        if previous != (key[0],key[1]-1,key[2]):
            code += [f'memset(&g,0,sizeof g);memset(&c,0,sizeof c);pwm=1445;g.width={f["width"]:.6f}f;g.configured_width=500;g.measured=1;']
        pts = f['points'] if mirror==1 else [[-x,y] for x,y in reversed(f['points'])]
        code += ['{PathPoint v[]={'+','.join('{%s,%s}'%tuple(v) for v in pts)+'};',
            f'Path_Build(&g,v,{len(pts)},500,Path_Preview(700,{f["speed"]:.6f}f),gains.center_x,&p);',
            'now+=83333;Path_Control(&c,&p,&gains,1,now,pwm,&o);pwm=o.pwm;',
            'printf("%u %u %.6f %.6f %.6f %u %u %u %u %.6f\\n",p.valid,p.model,p.target_x,p.ref_y,p.near_a,p.acute_phase,o.pwm,o.reason,o.applied,o.error);}']
        previous=key
code += ['return 0;}']
source = Path(__file__).with_suffix('.c')
source.write_text('\n'.join(code),encoding='utf-8')
rows=run('new_acute_r2_baseline',source)['output'].splitlines()
rows=[list(map(float,r.split())) for r in rows if r.strip()]
assert len(rows)==112
for i, f in enumerate(cases):
    f['r2_expected']=rows[i];f['r2_mirror_expected']=rows[i+56]
dest=P/'test/path_sequence_acute_20261013.json'
dest.write_text(json.dumps(dict(baseline='3c1af22 r2',
    note='BIN毫米量化云。仅同组同修订连续快照连接；缺字段用width500/speed15并显式标识。控制间隔83.333ms仅用于同条件回归，非实车轨迹重建。镜像cx=-10用于对称检验。',cases=cases),ensure_ascii=False,separators=(',',':')),encoding='utf-8')
print(dest)
