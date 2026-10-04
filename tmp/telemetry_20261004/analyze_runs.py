from pathlib import Path
from collections import Counter
import csv, hashlib, json, math, struct
root=Path(__file__).resolve().parents[2]
p=root/'2-LXY传承/LXY传承-ld14p'; log=p/'上位机/log'
out={'legacy':[],'v2':[]}
def read_csv(f):
    with f.open(encoding='utf-8-sig',newline='') as stream:return list(csv.DictReader(stream))
for f in sorted(log.glob('run_*.bin')):
    raw=f.read_bytes(); assert len(raw)%36==0
    rows=[]
    for i in range(0,len(raw),36):
        assert raw[i+32:i+36]==bytes.fromhex('0000807f')
        rows.append(struct.unpack_from('<8f',raw,i))
    item={'file':f.name,'frames':len(rows),'modes':dict(Counter(int(r[4]) for r in rows))}
    for j,n in enumerate(['err','pwm','kp','kd','mode','speed','target','radar_dps']):
        v=[r[j] for r in rows if math.isfinite(r[j])]
        item[n]={'min':min(v),'max':max(v),'mean':sum(v)/len(v),'last':v[-1]} if v else None
    out['legacy'].append(item)
for f in sorted(log.glob('v2_*.bin')):
    meta=json.loads(Path(str(f)+'.meta.json').read_text(encoding='utf-8'))
    assert hashlib.sha256(f.read_bytes()).hexdigest()==meta['sha256']
    rows=read_csv(f.with_suffix('.csv'))
    item={'file':f.name,'frames_all':len(rows),'meta':meta}
    if rows:
        item['actions_all']=dict(Counter(r['action_reason'] for r in rows))
        item['forward_zero']=sum(float(r['Forward_cnt'])==0 for r in rows)
        item['sequence_gaps']=sum(int(b['control_seq'])-int(a['control_seq'])-1 for a,b in zip(rows,rows[1:]))
        item['timeline']=rows
        item['details']=read_csv(f.with_suffix('.detail.csv'))
        events=[json.loads(s) for s in f.with_suffix('.events.jsonl').read_text().splitlines()]
        item['events']=[{'ms':r['event_ms'],'body':bytes.fromhex(r['body_hex']).decode(errors='replace')} for r in events if r['type']==4]
        item['health_count']=sum(r['type']==2 for r in events)
    out['v2'].append(item)
manifest=json.loads((p/'诊断V2交付/source_manifest.json').read_text(encoding='utf-8'))
out['manifest_sha256']=manifest['sha256']
out['manifest_mismatches']=[f for f,h in manifest['files'].items() if not (p/f).exists() or hashlib.sha256((p/f).read_bytes().replace(b'\r\n',b'\n')).hexdigest()!=h]
Path(__file__).with_name('summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print('Legacy frame counts:',[r['frames'] for r in out['legacy']])
print('V2 frame counts:',[r['frames_all'] for r in out['v2']])
print('Manifest mismatches:',out['manifest_mismatches'])
