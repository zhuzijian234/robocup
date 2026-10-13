"""核对当前源码和HEX后归档；已有不同内容的同名文件不覆盖。"""
from pathlib import Path
import hashlib, json, re
ROOT=Path(__file__).resolve().parents[2]
P=ROOT/'robocup-新建文件夹队/robocup-新建文件夹'
OUT=P/'build_m10p_verified'
summary=json.loads((OUT/'summary.json').read_text(encoding='utf-8'))
hashes=json.loads((OUT/'source_hashes.json').read_text(encoding='utf-8'))
for name,digest in hashes.items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
manifest=json.loads((P/'诊断V2交付/source_manifest.json').read_text(encoding='utf-8'))
for name,digest in manifest['files'].items():
    assert hashlib.sha256((P/name).read_bytes().replace(b'\r\n',b'\n')).hexdigest()==digest,name
build_id=re.search(r'"([^"]+)"',(P/'HARDWARE/hc-05/diag_build_id.h').read_text()).group(1)
raw=(OUT/'robocup_m10p.hex').read_bytes()
assert hashlib.sha256(raw).hexdigest()==summary['hex_sha256']
image=bytearray();base=0
for line in raw.decode('ascii').splitlines():
    data=bytes.fromhex(line[1:]);assert line.startswith(':') and sum(data)%256==0
    n=data[0];kind=data[3];assert len(data)==n+5
    if kind==4:base=int.from_bytes(data[4:6],'big')<<16
    if kind==0:
        addr=base+int.from_bytes(data[1:3],'big')
        assert 0x08000000<=addr and addr+n<=0x08100000
        image.extend(data[4:4+n])
assert build_id.encode() in image
summary.update(build_id=build_id,known_stack_bytes=2272,configured_stack_bytes=4096,
               ordinary_sram_capacity=131072,ram_delta_from_r2=232,
               host_decoder_sha256=hashlib.sha256((P/'上位机/telemetry_v2.py').read_bytes()).hexdigest())
prefix='robocup_m10p_acute_entry_20261013'
def save(path,data):
    if path.exists():assert path.read_bytes()==data,path
    else:path.write_bytes(data)
for ext in ('hex','axf','map','htm'):
    save(OUT/f'{prefix}.{ext}',(OUT/f'robocup_m10p.{ext}').read_bytes())
save(OUT/f'{prefix}_summary.json',json.dumps(summary,ensure_ascii=False,indent=2).encode('utf-8'))
save(OUT/f'{prefix}_source_hashes.json',(OUT/'source_hashes.json').read_bytes())
save(OUT/f'{prefix}_source_manifest.json',(P/'诊断V2交付/source_manifest.json').read_bytes())
print(json.dumps(summary,ensure_ascii=False,indent=2))
print('PASS source hashes, manifest, HEX checksum/address/build ID and immutable archive')
