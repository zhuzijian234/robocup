from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess, json, hashlib, re, os, tempfile

ROOT = Path(__file__).resolve().parents[3]
PROJECT = Path(__file__).resolve().parents[1] / 'USER/Template.uvprojx'
OUT = Path(os.environ.get('ROBOCUP_BUILD_OUT', str(PROJECT.parent.parent / 'build_m10p_verified')))
OUT.mkdir(parents=True, exist_ok=True)
WORK = Path(tempfile.mkdtemp(prefix='run_', dir=OUT))
BIN = Path('D:/Keil5/ARM/ARMCC/bin')
import sys
subprocess.run([sys.executable, str(PROJECT.parent.parent/'test/update_diag_build_id.py')], check=True)
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
    obj = WORK / (src.stem + '.o')
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
    cmd = [str(BIN/'armlink.exe'), '--cpu=Cortex-M4.fp.sp', '--library_type=microlib', '--scatter', str(scatter), '--map', '--symbols', '--info=sizes', '--list', str(WORK/'robocup_m10p.map'), '-o', str(WORK/'robocup_m10p.axf')] + [str(p) for p in objects+libs]
    proc = subprocess.run(cmd, capture_output=True)
    records.append(dict(file='LINK', command=cmd, exit=proc.returncode, output=(proc.stdout+proc.stderr).decode('utf-8', errors='replace')))
for p in PROJECT.parent.parent.rglob('*.h'):
    hashes[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'build_review.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'source_hashes.json').write_text(json.dumps(hashes,ensure_ascii=False,indent=2),encoding='utf-8')
summary=dict(source_count=len(objects), failures=[r['file'] for r in records if r['exit']], warnings=sum(r['output'].count('Warning:') for r in records), settings='AC5 Cortex-M4.fp.sp C99 O0 MicroLIB split_sections; current uvproj source/include/defines; explicit m10p.sct; ZG 1MiB Flash / 128KiB SRAM')
if not summary['failures']:
    txt=(WORK/'robocup_m10p.map').read_text(errors='replace')
    for name,pattern in [('ram_bytes',r'Total RW\s+Size.*?\s(\d+)\s+\('),('rom_bytes',r'Total ROM Size.*?\s(\d+)\s+\(')]:
        summary[name]=int(re.search(pattern,txt).group(1))
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
if not summary['failures']:
    assert summary['ram_bytes'] <= 112*1024, 'SRAM budget exceeded'
    dma = re.search(r'^\s*DMA_USART2_RX_BUF\s+(0x[0-9a-fA-F]+)', txt, re.M)
    assert dma and 0x20000000 <= int(dma.group(1),16) <= 0x20020000-1024, 'DMA buffer outside ordinary SRAM'
    summary['dma_buffer_address'] = dma.group(1)
    subprocess.run([str(BIN/'fromelf.exe'), '--i32combined', '--output', str(WORK/'robocup_m10p.hex'), str(WORK/'robocup_m10p.axf')], check=True)
    # Publish through a sibling file created in OUT, inheriting OUT's ACL.
    # Moving straight out of tempfile.mkdtemp carries its owner-only Windows
    # ACL, making firmware unreadable to the user's IDE/Git account.
    for suffix in ('axf','map','hex'):
        destination = OUT/('robocup_m10p.'+suffix)
        published = OUT/(WORK.name+'_'+destination.name+'.publish')
        with published.open('xb') as stream:
            stream.write((WORK/destination.name).read_bytes())
        os.replace(published, destination)
    summary['hex_sha256'] = hashlib.sha256((OUT/'robocup_m10p.hex').read_bytes()).hexdigest()
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))

raise SystemExit(1 if summary["failures"] else 0)
