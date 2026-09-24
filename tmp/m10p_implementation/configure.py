from pathlib import Path
import re
R=Path(__file__).resolve().parents[2]; P=R/'2-LXY传承/LXY传承-m10p'
def edit(p,f):
    p=P/p; p.write_text(f(p.read_text(encoding='utf-8-sig')),encoding='utf-8')
modules=['m10p','m10p_vehicle']
def uv(t):
    t=t.replace('STM32F407VETx','STM32F407ZGTx').replace('0x00080000','0x00100000').replace('0x80000','0x100000')
    t=t.replace('STM32F4xx_512','STM32F4xx_1024').replace('-FL080000','-FL0100000').replace('CLOCK(12000000)','CLOCK(8000000)')
    needle='          <GroupName>HARDWARE</GroupName>'
    # Actual group names may differ; add separate self-contained group.
    group='      <Group>\n        <GroupName>M10P</GroupName>\n        <Files>\n'
    for m in modules:
        group+=f'          <File><FileName>{m}.c</FileName><FileType>1</FileType><FilePath>..\\HARDWARE\\LEIDA_DATA\\{m}.c</FilePath></File>\n'
    group+='        </Files>\n      </Group>\n'
    t=t.replace('    </Groups>',group+'    </Groups>')
    return t
edit(Path('USER/Template.uvprojx'),uv)
for name,pre in [('.eide/eide.yml',''),('USER/.eide/eide.yml','../')]:
    def f(t):
        t=t.replace('deviceName: null','deviceName: STM32F407ZGTx')
        t=t.replace('size: "0x80000"','size: "0x100000"')
        t=re.sub(r'^outDir:.*$', 'outDir: build_m10p',t,flags=re.M)
        anchor=f'        - path: {pre}HARDWARE/LEIDA_DATA/LEIDA_DATA.c'
        assert anchor in t
        t=t.replace(anchor,anchor+''.join(f'\n        - path: {pre}HARDWARE/LEIDA_DATA/{m}.c' for m in modules))
        return t
    edit(Path(name),f)
scatter='''LR_IROM1 0x08000000 0x00100000 {
 ER_IROM1 0x08000000 0x00100000 {
  *.o (RESET, +First)
  *(InRoot$$Sections)
  .ANY (+RO)
  .ANY (+XO)
 }
 RW_IRAM1 0x20000000 0x00020000 {
  .ANY (+RW +ZI)
 }
}
'''
(P/'USER/m10p.sct').write_text(scatter)
edit(Path('USER/Template.uvprojx'),lambda t:t.replace('<ScatterFile></ScatterFile>','<ScatterFile>.\\m10p.sct</ScatterFile>').replace('<umfTarg>1</umfTarg>','<umfTarg>0</umfTarg>'))
# Raw HISR bits do not include SPL flag's 0x20000000 register-selector marker.
def dma(t):
    a=t.index('void DMA1_Stream5_IRQHandler')
    head,tail=t[:a],t[a:]
    for line in tail.splitlines():
        pass
    tail=re.sub(r'flags & ([^\n]+)',lambda m: 'flags & '+m.group(1).replace('DMA_FLAG_','DMA_HISR_'),tail)
    # raw macros are HTIF5 etc; strip selector from equality's RHS as well.
    tail=tail.replace('== (DMA_FLAG_HTIF5 | DMA_FLAG_TCIF5)','== (DMA_HISR_HTIF5 | DMA_HISR_TCIF5)')
    return head+tail
edit(Path('HARDWARE/DMA/DMA.c'),dma)
# Build runner independent of obsolete directory names and generated scatter.
t=(R/'tmp/radar_upgrade_v3/build_current.py').read_text(encoding='utf-8')
t=t.replace("ROOT = Path(__file__).resolve().parents[2]", "ROOT = Path(__file__).resolve().parents[3]")
t=t.replace("PROJECT = ROOT / '2-LXY传承/LXY传承-副本/USER/Template.uvprojx'", "PROJECT = Path(__file__).resolve().parents[1] / 'USER/Template.uvprojx'")
t=t.replace("str(Path(__file__).resolve().parent / 'current_build')", "str(PROJECT.parent.parent / 'build_m10p_verified')")
t=t.replace("PROJECT.parent/'build/Template/Template.sct'", "PROJECT.parent/'m10p.sct'")
t=t.replace('print(json.dumps(summary,ensure_ascii=False))', '''if not summary['failures']:
    assert summary['ram_bytes'] <= 112*1024, 'SRAM budget exceeded'
    subprocess.run([str(BIN/'fromelf.exe'), '--i32combined', '--output', str(OUT/'robocup_m10p.hex'), str(OUT/'baseline.axf')], check=True)
print(json.dumps(summary,ensure_ascii=False))''')
(P/'test/build_m10p.py').write_text(t,encoding='utf-8')
print('ZG + 3 project entrances configured')
