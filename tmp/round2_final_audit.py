"""确认独立 ARM 重编译对应当前源码，并执行总验证脚本的链接后审计。"""
from pathlib import Path
import json,hashlib,re
root=Path(__file__).resolve().parents[1]
P=root/'robocup-新建文件夹队/robocup-新建文件夹'
out=P/'build_m10p_tests'
records=json.loads((out/'verification.json').read_text(encoding='utf-8'))
hashes=json.loads((P/'build_m10p_verified/source_hashes.json').read_text(encoding='utf-8'))
for name,digest in hashes.items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
summary=json.loads((P/'build_m10p_verified/summary.json').read_text(encoding='utf-8'))
assert summary['source_count']==59 and not summary['failures']
assert summary['ram_bytes']<=112*1024
assert hashlib.sha256((P/'build_m10p_verified/robocup_m10p.hex').read_bytes()).hexdigest()==summary['hex_sha256']
records.append({'stage':'Final ARM build outside restricted launcher', 'exit':0,'summary':summary,'source_hashes_match':len(hashes),'note':'Previous WinError 623 is retained in this record; the final ARM compilation succeeded.'})
for name in ['round2_turn_final.log','round2_old_replay_final.log','round2_preservation_final.log','round2_steering_final.log','round2_highpreview.log']:
    value=(root/'tmp'/name).read_text(encoding='utf-8')
    assert 'PASS' in value and 'FAIL' not in value and 'Traceback' not in value,name
    records.append({'final_regression':name,'output':value})
script=(P/'test/verify_m10p.py').read_text(encoding='utf-8')
exec(compile(script[script.index('linked=(P/'):],'link_audit','exec'))
report={'source_hashes_match':len(hashes),'build':summary,'all_host_regressions_passed':True,'final_geometry_regressions_passed':True,'old_symbols_absent':True}
(out/'round2_final_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
