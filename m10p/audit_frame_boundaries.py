"""Independent raw-byte framing audit, then receive-chunk invariance check."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import struct

root=Path(__file__).resolve().parent
source=root/'reports/latest_capture.bin'
b=source.read_bytes()
def occurrences(marker):
    result=[]
    pos=b.find(marker)
    while pos>=0:
        result.append(pos)
        pos=b.find(marker,pos+1)
    return result

# A: all literal header positions, ignoring encoded length and old parser.
heads=occurrences(b'\xa5\x5a')
by_heads=list(zip(heads,heads[1:]))
assert by_heads
assert all(b[end-2:end]==b'\xfa\xfb' for start,end in by_heads)

# B: literal tail boundaries, ignoring both length fields and following headers.
tails=[t for t in occurrences(b'\xfa\xfb') if t>=heads[0]]
by_tails=[]
pos=heads[0]
for tail in tails:
    by_tails.append((pos,tail+2))
    pos=tail+2
assert by_tails==by_heads

# C: only step through the encoded length from the initial header.
# Never search/resynchronize after the initial position.
by_length=[]
pos=heads[0]
while pos+4<=len(b):
    n=int.from_bytes(b[pos+2:pos+4],'big')
    assert 22<=n<=512 and n%2==0
    if pos+n>len(b):break
    assert b[pos:pos+2]==b'\xa5\x5a'
    assert b[pos+n-2:pos+n]==b'\xfa\xfb'
    by_length.append((pos,pos+n))
    pos+=n
assert by_length==by_heads
lengths=Counter(end-start for start,end in by_heads)
deltas=Counter((int.from_bytes(b[end+4:end+6],'big')-int.from_bytes(b[start+4:start+6],'big'))%36000 for start,end in by_heads if end+6<=len(b))
fixed_starts=list(range(heads[0],len(b)-159,160))
fixed_heads=sum(b[s:s+2]==b'\xa5\x5a' for s in fixed_starts)
fixed_valid=sum(b[s:s+4]==b'\xa5\x5a\x00\xa0' and b[s+158:s+160]==b'\xfa\xfb' for s in fixed_starts)
examples=[]
for size in sorted(lengths):
    start,end=next((s,e) for s,e in by_heads if e-s==size)
    examples.append(dict(length=size,start=start,end=end,header=b[start:start+8].hex(' ').upper(),
                         boundary=b[end-2:end+8].hex(' ').upper()))
result=dict(source=str(source),bytes=len(b),sha256=hashlib.sha256(b).hexdigest(),
            literal_headers=len(heads),complete_frames=len(by_heads),frame_lengths=dict(sorted(lengths.items())),
            all_three_methods_identical=True,angle_deltas_cdeg=dict(deltas),
            prefix_bytes=heads[0],suffix_bytes=len(b)-by_heads[-1][1],
            accounted_bytes=heads[0]+sum(e-s for s,e in by_heads)+len(b)-by_heads[-1][1],
            fixed_160_trial=dict(blocks=len(fixed_starts),header_matches=fixed_heads,fully_valid_160_frames=fixed_valid),
            examples=examples)
print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)

# Only now import the production decoder and vary the simulated read boundaries.
from check_m10p import Parser
reference=None
trials=[]
for chunk in (1,7,160,4096,65536):
    p=Parser();h=hashlib.sha256();count=0
    for off in range(0,len(b),chunk):
        for angle,rpm,points,invalid in p.feed(b[off:off+chunk]):
            h.update(struct.pack('>ddHH',angle,rpm,len(points),invalid))
            for a,d,high in points:
                h.update(struct.pack('>dH?',a,d,high))
            count+=1
    fingerprint=h.hexdigest()
    if reference is None:reference=fingerprint
    assert fingerprint==reference
    assert dict(p.lengths)==dict(lengths)
    assert p.bad==0
    trials.append(dict(chunk_bytes=chunk,frames=count,decoded_sha256=fingerprint,bad_frames=p.bad))
result['read_chunk_trials']=trials
result['claim_scope']='Confirms varying frame sizes in bytes received by the PC; does not locate whether radar firmware or intermediate hardware caused them.'
(root/'reports/最新原始数据_帧边界独立审计.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('Read chunk trials: 1, 7, 160, 4096, 65536 bytes -> identical decoded results.',flush=True)
