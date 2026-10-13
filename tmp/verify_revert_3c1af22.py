# -*- coding: utf-8 -*-
"""决定性核对：现在的工作区源码，是否等于当年产出 r2 HEX 的那批源码。

依据 build_m10p_verified/acute_entry_20261012_r2_source_hashes.json
（3c1af22 交付时逐文件记录的 SHA256）。
只读，不修改任何文件。
"""
import hashlib, json, os, sys

ROOT = r'D:\OneDrive\Desktop\robocup'
MAN = os.path.join(ROOT, 'robocup-新建文件夹队', 'robocup-新建文件夹',
                   'build_m10p_verified', 'acute_entry_20261012_r2_source_hashes.json')

for cand in (MAN, MAN.replace('_r2_source_hashes', '_source_hashes')):
    if os.path.exists(cand):
        MAN = cand
        break
print('清单文件:', os.path.basename(MAN), os.path.getsize(MAN), 'B')
raw = open(MAN, 'rb').read()
try:
    man = json.loads(raw.decode('utf-8'))
    enc = 'utf-8'
except Exception as e:
    print('  utf-8 解析失败:', e)
    try:
        man = json.loads(raw.decode('gbk'))
        enc = 'gbk'
    except Exception as e2:
        print('  gbk 也失败:', e2); sys.exit(1)
print('  以 %s 解析成功，条目 %d 个' % (enc, len(man)))
k0 = next(iter(man))
print('  首个键示例:', k0)

ok = miss = bad = 0
badlist = []
for rel, want in man.items():
    p = os.path.join(ROOT, rel.replace('\\', os.sep).replace('/', os.sep))
    if not os.path.exists(p):
        miss += 1
        if miss <= 5:
            badlist.append(('MISSING', rel))
        continue
    got = hashlib.sha256(open(p, 'rb').read()).hexdigest()
    if got == want:
        ok += 1
    else:
        bad += 1
        if len(badlist) < 12:
            badlist.append(('HASH', rel, want[:12], got[:12]))

print()
print('  一致 %d / 不一致 %d / 文件缺失 %d   （共 %d）' % (ok, bad, miss, len(man)))
if badlist:
    print('  明细（最多12条）:')
    for x in badlist:
        print('   ', x)
print()
print('结论:', ('[OK] 工作区源码与产出 r2 HEX 的源码逐文件一致 (%d/%d)' % (ok, len(man)))
      if (bad == 0 and miss == 0) else
      ('[FAIL] 有 %d 个文件不一致、%d 个缺失，需查明' % (bad, miss)))
