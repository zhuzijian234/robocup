from pathlib import Path
import subprocess, os, json, tempfile
P=Path(__file__).resolve().parents[1]
OUT=P/'build_m10p_tests'; OUT.mkdir(exist_ok=True)
wrapper=OUT/'env.bat'
wrapper.write_bytes(b'@echo off\r\ncall "D:\\VS\\VC\\Auxiliary\\Build\\vcvars64.bat" >nul 2>&1\r\nset\r\n')
r=subprocess.run(['cmd','/d','/s','/c',str(wrapper)],capture_output=True,check=True)
env=dict(os.environ)
for line in r.stdout.decode(errors='replace').splitlines():
    k,s,v=line.partition('=')
    if k and s:env[k]=v
cl='D:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe'
def run(name,source):
    # Isolate compiler intermediates from stale/sync-locked files of a prior run.
    run_out=Path(tempfile.mkdtemp(prefix=name+'_',dir=OUT))
    exe=run_out/(name+'.exe')
    cmd=[cl,'/nologo','/std:c11','/utf-8','/Od','/W4',str(source),'/I'+str(P/'HARDWARE/LEIDA_DATA'),'/Fe:'+str(exe),'/Fo:'+str(run_out/(name+'.obj'))]
    b=subprocess.run(cmd,env=env,capture_output=True)
    (OUT/(name+'_compile.log')).write_bytes(b.stdout+b.stderr)
    if b.returncode:print((b.stdout+b.stderr).decode(errors='replace'));raise SystemExit(b.returncode)
    r=subprocess.run([str(exe)],capture_output=True)
    (OUT/(name+'_run.log')).write_bytes(r.stdout+r.stderr)
    print(r.stdout.decode(errors='replace'))
    if r.returncode:raise SystemExit(r.returncode)
    return {'build_exit':b.returncode,'run_exit':r.returncode,'output':r.stdout.decode(errors='replace')}
if __name__=='__main__':
    result=run('m10p',P/'test/test_m10p.c')
    (OUT/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
