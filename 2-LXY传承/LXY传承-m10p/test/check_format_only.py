"""证明一次"美化/加注释"没有改动任何逻辑。

判据一(强): 把两边的注释全部剥掉后切成 token 序列, 必须完全一致。
             空格/换行/缩进/运算符两侧空格的差异会被忽略, 但只要少一个分号、
             改一个数字、换一个标识符, 就一定会被抓出来。
判据二:     旧文件里的每一条注释都必须还能在新文件里找到(压缩空白后),
             防止格式化把别人写的注释吃掉。

用法:  python test/check_format_only.py <git rev> <文件> [文件...]
       旧版 = git show <rev>:./<文件>,  新版 = 磁盘上当前内容
退出码 0 = 只动了注释和空白; 1 = 有代码差异或注释丢失。
"""
import subprocess
import sys
from pathlib import Path

try:  # 中文注释要能打出来: 管道重定向时 stdout 会退化成 GBK 而报 UnicodeEncodeError
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except AttributeError:
    pass

PROJECT = Path(__file__).resolve().parents[1]

# 多字符运算符按长度降序贪心匹配, 保证 '<=' 不会被拆成 '<' '='
OPS = ['>>=', '<<=', '->*', '...', '##', '++', '--', '->', '<<', '>>', '<=', '>=', '==', '!=',
       '&&', '||', '+=', '-=', '*=', '/=', '%=', '&=', '|=', '^=', '::',
       '+', '-', '*', '/', '%', '<', '>', '=', '!', '&', '|', '^', '~', '?', ':', ';', ',',
       '.', '(', ')', '[', ']', '{', '}', '#', '\\']


def lex(text):
    """返回 (token 列表, 归一化后的注释列表)。注释不进 token。"""
    toks, comments = [], []
    i, n = 0, len(text)
    prev_kind = ''
    while i < n:
        c = text[i]
        if c.isspace():
            i += 1
            continue
        if text.startswith('//', i):
            j = i
            while j < n and text[j] not in '\r\n':
                j += 1
            while j > i and text[j - 1] == '\\' and j < n:  # 反斜杠续行的注释
                j += 1
                while j < n and text[j] not in '\r\n':
                    j += 1
            comments.append(text[i:j])
            i = j
            continue
        if text.startswith('/*', i):
            j = text.find('*/', i + 2)
            j = n if j < 0 else j + 2
            comments.append(text[i:j])
            i = j
            continue
        if c in '"\'':
            j = i + 1
            while j < n:
                if text[j] == '\\' and j + 1 < n:
                    j += 2
                    continue
                if text[j] == c:
                    j += 1
                    break
                j += 1
            toks.append(text[i:j])
            prev_kind = 'LIT'
            i = j
            continue
        if c.isdigit() or (c == '.' and i + 1 < n and text[i + 1].isdigit()
                           and prev_kind not in ('ID', 'NUM')):
            j = i
            while j < n and (text[j].isalnum() or text[j] == '_' or text[j] == '.'):
                if text[j] in 'eEpP' and j + 1 < n and text[j + 1] in '+-':
                    j += 2
                    continue
                j += 1
            toks.append(text[i:j])
            prev_kind = 'NUM'
            i = j
            continue
        if c.isalpha() or c == '_':
            j = i
            while j < n and (text[j].isalnum() or text[j] == '_'):
                j += 1
            toks.append(text[i:j])
            prev_kind = 'ID'
            i = j
            continue
        for op in OPS:
            if text.startswith(op, i):
                toks.append(op)
                i += len(op)
                break
        else:
            toks.append(c)
            i += 1
        prev_kind = 'OP'
    return toks, [' '.join(x.split()) for x in comments]


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    rev, files = sys.argv[1], sys.argv[2:]
    bad = 0
    for rel in files:
        old_bytes = subprocess.run(['git', 'show', f'{rev}:./{rel}'],
                                   cwd=str(PROJECT), capture_output=True).stdout
        if not old_bytes:
            print(f'!! 取不到 {rev}:{rel}')
            bad = 1
            continue
        old = old_bytes.decode('utf-8-sig')
        new = (PROJECT / rel).read_bytes().decode('utf-8-sig')
        ot, ocm = lex(old)
        nt, ncm = lex(new)
        print(f'--- {rel}')
        if ot == nt:
            print(f'    [OK] token 序列完全一致 ({len(nt)} 个 token)')
        else:
            bad = 1
            print(f'    [!!] token 有差异 (旧 {len(ot)} / 新 {len(nt)}):')
            for k in range(min(len(ot), len(nt))):
                if ot[k] != nt[k]:
                    print(f'         第 {k} 个 token: 旧 "{ot[k]}" -> 新 "{nt[k]}"')
                    print(f'         上下文 旧: {" ".join(ot[max(0, k - 12):k + 12])}')
                    print(f'         上下文 新: {" ".join(nt[max(0, k - 12):k + 12])}')
                    break
            else:
                tail = ot[len(nt):] if len(ot) > len(nt) else nt[len(ot):]
                print(f'         尾部多出/缺少: {" ".join(tail[:12])}')
        pool = '\n'.join(ncm)
        lost = [c for c in ocm if c and c not in pool]
        if lost:
            bad = 1
            print(f'    [!!] 丢了 {len(lost)} 条原注释:')
            for c in lost[:5]:
                print(f'         {c[:90]}')
        else:
            print(f'    [OK] 原有 {len(ocm)} 条注释全部保留 (现存 {len(ncm)} 条)')
    print('\n结论: 只涉及注释与排版, 逻辑零变化。' if not bad else '\n结论: 有实质改动, 需人工确认。')
    return bad


if __name__ == '__main__':
    sys.exit(main())
