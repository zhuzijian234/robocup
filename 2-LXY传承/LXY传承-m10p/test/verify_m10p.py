"""One command: execute host regressions, verify 3 entry points, build ARM image."""
from pathlib import Path
import subprocess,sys,os,json,hashlib,re,xml.etree.ElementTree as ET
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P=Path(__file__).resolve().parents[1]
out=P/'build_m10p_tests';out.mkdir(exist_ok=True)
env=dict(os.environ, ROBOCUP_CONTROL_TEST_OUT=str(out/'control'), PYTHONIOENCODING='utf-8')
records=[]
def command(args,cwd=P):
 r=subprocess.run(args,cwd=cwd,env=env,capture_output=True)
 output=(r.stdout+r.stderr).decode(errors='replace')
 records.append({'command':args,'exit':r.returncode,'output':output})
 print(output,flush=True)
 (out/'verification.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
 if r.returncode:raise SystemExit(r.returncode)
for name in ['check_m10p.py','check_m10p_integration.py','check_m10p_geometry.py','check_turn_regression.py','check_m10p_telemetry.py']:
 command([sys.executable,str(P/'test'/name)])
command([sys.executable,'-m','unittest','discover','-s','tests','-v'],P/'上位机')
import yaml
tree=ET.parse(P/'USER/Template.uvprojx')
expected={(P/'USER'/f.findtext('FilePath').replace('\\','/')).resolve() for f in tree.findall('.//File') if f.findtext('FilePath','').endswith(('.c','.s'))}
assert tree.findtext('.//Device')=='STM32F407ZG'
for name,base in [('.eide/eide.yml',P),('USER/.eide/eide.yml',P/'USER')]:
 d=yaml.safe_load((P/name).read_text(encoding='utf-8'));found=set();stack=[d['virtualFolder']]
 while stack:
  node=stack.pop();stack.extend(node.get('folders',[]))
  found.update((base/f['path']).resolve() for f in node.get('files',[]) if f['path'].endswith(('.c','.s')))
 assert found==expected,(name,'source mismatch')
 cfg=d['targets']['Template']['toolchainConfigMap']['AC5']
 assert cfg['useCustomScatterFile'] and (base/cfg['scatterFilePath']).resolve()==P/'USER/m10p.sct'
 assert d['deviceName']=='STM32F407ZG' and d['outDir']=='build_m10p'
records.append({'project_sources':len(expected),'entrances':'Keil and both EIDE source/scatter/device configurations match'})
command([sys.executable,str(P/'test/build_m10p.py')])
linked=(P/'build_m10p_verified/robocup_m10p.map').read_text(errors='replace')
# 固件符号表确认已删除的旧入口/工作区没有被其他工程文件意外带回。
disabled=['LEIDA_DATA','LEIDA_ParserReset','LEIDA_DATA_HANDLE1','LEIDA_DATA_HANDLE3',
          'LEIDA_DATA_HANDLE3_2','LEIDA_ANGLE_jiuzheng','LEIDA_DATA_HANDLE12',
          'LEIDA_DATA_HANDLE13','LEIDA_PrintAll','LEIDA_PrintSample','LEIDA_PrintHead',
          'LEIDA_PWM_Init','TIM14_Int_Init']
for name in disabled:
 assert not re.search(r'^\s*'+re.escape(name)+r'\s+0x[0-9a-fA-F]+',linked,re.M),name
records.append({'disabled_legacy_symbols':disabled,'result':'absent from linked symbol table'})
(out/'verification.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS M10P host regressions, configuration audit and ARM build')
