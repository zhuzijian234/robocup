from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess, json, hashlib, re, os

ROOT = Path(__file__).resolve().parents[3]
PROJECT = Path(__file__).resolve().parents[1] / 'USER/Template.uvprojx'
OUT = Path(os.environ.get('ROBOCUP_BUILD_OUT', str(PROJECT.parent.parent / 'build_m10p_verified')))
OUT.mkdir(parents=True, exist_ok=True)
BIN = Path('D:/Keil5/ARM/ARMCC/bin')
tree = ET.parse(PROJECT)
target = tree.find('.//Target')
controls = target.find('.//Cads/VariousControls')
includes = [(PROJECT.parent / p).resolve() for p in controls.findtext('IncludePath').split(';')]
defines = controls.findtext('Define').split(',')
records, objects, libs, hashes = [], [], [], {}
for f in target.findall('./Groups/Group/Files/File'):
    src = (PROJECT.parent / f.findtext('FilePath')).resolve()
    suffix = src.suffix.lower()
    if suffix == '.lib':
        libs.append(src)
        continue
    if suffix not in ('.c', '.s'):
        continue
    obj = OUT / (src.stem + '.o')
    assert obj not in objects, obj
    hashes[str(src.relative_to(ROOT))] = hashlib.sha256(src.read_bytes()).hexdigest()
    if suffix == '.c':
        cmd = [str(BIN/'armcc.exe'), '--cpu=Cortex-M4.fp.sp', '--c99', '-O0', '--library_type=microlib', '--split_sections', '-g', '-c']
        for inc in includes: cmd += ['-I', str(inc)]
        for define in defines: cmd += ['-D', define]
    else:
        cmd = [str(BIN/'armasm.exe'), '--cpu=Cortex-M4.fp.sp', '--pd', '__MICROLIB SETA 1', '-g']
    cmd += [str(src), '-o', str(obj)]
    proc = subprocess.run(cmd, capture_output=True)
    output = (proc.stdout+proc.stderr).decode('utf-8', errors='replace')
    records.append(dict(file=str(src), command=cmd, exit=proc.returncode, output=output))
    if proc.returncode: break
    objects.append(obj)
else:
    scatter = PROJECT.parent/'m10p.sct'
    cmd = [str(BIN/'armlink.exe'), '--cpu=Cortex-M4.fp.sp', '--library_type=microlib', '--scatter', str(scatter), '--map', '--symbols', '--info=sizes', '--list', str(OUT/'baseline.map'), '-o', str(OUT/'baseline.axf')] + [str(p) for p in objects+libs]
    proc = subprocess.run(cmd, capture_output=True)
    records.append(dict(file='LINK', command=cmd, exit=proc.returncode, output=(proc.stdout+proc.stderr).decode('utf-8', errors='replace')))
for p in PROJECT.parent.parent.rglob('*.h'):
    hashes[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'build_review.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'source_hashes.json').write_text(json.dumps(hashes,ensure_ascii=False,indent=2),encoding='utf-8')
summary=dict(source_count=len(objects), failures=[r['file'] for r in records if r['exit']], warnings=sum(r['output'].count('Warning:') for r in records), settings='AC5 Cortex-M4.fp.sp C99 O0 MicroLIB split_sections; current uvproj source/include/defines; explicit m10p.sct; ZG 1MiB Flash / 128KiB SRAM')
if not summary['failures']:
    txt=(OUT/'baseline.map').read_text(errors='replace')
    for name,pattern in [('ram_bytes',r'Total RW\s+Size.*?\s(\d+)\s+\('),('rom_bytes',r'Total ROM Size.*?\s(\d+)\s+\(')]:
        summary[name]=int(re.search(pattern,txt).group(1))
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
if not summary['failures']:
    assert summary['ram_bytes'] <= 112*1024, 'SRAM budget exceeded'
    subprocess.run([str(BIN/'fromelf.exe'), '--i32combined', '--output', str(OUT/'robocup_m10p.hex'), str(OUT/'baseline.axf')], check=True)
print(json.dumps(summary,ensure_ascii=False))

raise SystemExit(1 if summary["failures"] else 0)
