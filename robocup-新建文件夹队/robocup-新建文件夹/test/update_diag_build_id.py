"""Run before a release build; digest includes dirty source contents."""
from pathlib import Path
import hashlib
import json

project=Path(__file__).resolve().parents[1]
target=project/'HARDWARE/hc-05/diag_build_id.h'
files={}
for directory in ['USER','CORE','SYSTEM','HARDWARE','FWLIB','test']:
    for p in sorted((project/directory).rglob('*')):
        if p.is_file() and p.suffix.lower() in ('.c','.h','.s','.py','.lib','.uvprojx','.yml','.sct') and p!=target:
            if any(part in ('build','Objects','Listings','__pycache__') for part in p.parts):continue
            # Newline normalization avoids hash drift caused solely by git autocrlf.
            data=p.read_bytes().replace(b'\r\n',b'\n')
            files[p.relative_to(project).as_posix()]=hashlib.sha256(data).hexdigest()
for relative in ['.eide/eide.yml','USER/.eide/eide.yml']:
    p=project/relative
    files[relative]=hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()
identity=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
target.write_bytes(('#define DIAG_BUILD_ID "sha256-'+identity[:16]+'"\r\n').encode())
out=project/'诊断V2交付'
out.mkdir(exist_ok=True)
(out/'source_manifest.json').write_text(json.dumps({'sha256':identity,'files':files},ensure_ascii=False,indent=2),encoding='utf-8')
print('DIAG_BUILD_ID=sha256-'+identity[:16])
