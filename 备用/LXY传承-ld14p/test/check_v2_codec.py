"""Generate golden vectors; compile actual C codec and run if a host cc exists."""
from pathlib import Path
import hashlib
import json
import shutil
import struct
import subprocess
import sys

project=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(project/'上位机'))
import robocup_telemetry as core
import telemetry_v2 as v2

out=project.parents[1]/'tmp/telemetry_v2_codec'
out.mkdir(parents=True,exist_ok=True)
src=(project/'HARDWARE/hc-05/ble_diag.c').read_text(encoding='utf-8')
codec=src[src.index('static const uint16_t scales'):src.index('void TIM6_DAC_IRQHandler')]
finalize=src[src.index('void Diag_Finalize'):src.index('void Diag_Submit')]
cases=[('err',500),('err',650),('err',-650),('Midline.k',.0345),('Midline.k',-.0345),
       ('Midline.k',40),('Midline.k',-.5),('servo_pwm',32767),('Speed_now',-3276.8),
       ('err',float('nan')),('err',float('inf')),('err',float('-inf')),('pid_select',10)]
vectors=[]
for name,val in cases:
    bits=struct.unpack('<I',struct.pack('<f',val))[0]
    encoded,valid,clipped=core.encode_field(name,val)
    vectors.append(dict(field=name,index=core.FIELD_NAMES.index(name),float32_bits=f'{bits:08x}',
                        stored=encoded,valid=int(valid),clipped=int(clipped)))
control=bytearray(92)
struct.pack_into('<26h',control,0,*([5000,1445,7,500]+[0]*22))
struct.pack_into('<7I',control,52,1,100,110,0,0x3fffffff,0x3fffffff,0)
struct.pack_into('<4BIHH',control,80,1,2,1,1,0,444,2160)
gold=v2.packet(1,control,0,1)
manifest={'layout':v2.LAYOUT,'crc_123456789':'29b1','encoding':vectors,'control_hex':gold.hex()}
delivery=project/'诊断V2交付';delivery.mkdir(exist_ok=True)
(delivery/'golden_vectors.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
checks=[]
for index,c in enumerate(vectors):
    checks.append('u.bits=0x%s; value=Diag_Encode(%d,u.v,&valid,&clipped); if(value!=%d || valid!=%d || clipped!=%d)return %d;'
                  %(c['float32_bits'],c['index'],c['stored'],c['valid'],c['clipped'],index+1))
gold_data=','.join(str(x) for x in gold)
program='''#include <stdint.h>
#include <float.h>
static uint32_t tx_seq;
'''+codec+finalize+'''
volatile int codec_test_result=-1;
static int run(void){
    union{uint32_t bits;float v;}u;uint8_t valid,clipped;int16_t value;unsigned i;
    uint8_t packet[108]={'''+gold_data+'''};
    const uint8_t expected[108]={'''+gold_data+'''};
'''+ '\n'.join(checks)+'''
    if(Diag_CRC((const uint8_t*)"123456789",9)!=0x29b1)return 50;
    packet[106]=packet[107]=0;Diag_Finalize(packet,108);
    for(i=0;i<108;i++)if(packet[i]!=expected[i])return 100+i;
    return 0;
}
int main(void){codec_test_result=run();return codec_test_result;}
'''
p=out/'codec_test.c';p.write_text(program,encoding='utf-8')
cc=shutil.which('gcc') or shutil.which('clang')
if cc:
    exe=out/'codec_test.exe'
    subprocess.run([cc,'-std=c99','-O0',str(p),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    print('C/Python golden vectors executed and matched')
else:
    bin=Path('D:/Keil5/ARM/ARMCC/bin')
    stack=out/'stack.s';stack.write_text('    AREA STACK, NOINIT, READWRITE, ALIGN=3\n    SPACE 4096\n    EXPORT __initial_sp\n__initial_sp\n    END\n')
    subprocess.run([str(bin/'armasm.exe'),'--cpu=Cortex-M4.fp.sp',str(stack),'-o',str(out/'stack.o')],check=True)
    subprocess.run([str(bin/'armcc.exe'),'--cpu=Cortex-M4.fp.sp','--c99','-O0','--library_type=microlib','-c',str(p),'-o',str(out/'codec.o')],check=True)
    subprocess.run([str(bin/'armlink.exe'),'--cpu=Cortex-M4.fp.sp','--library_type=microlib','--entry=__main','--ro_base=0x08000000','--rw_base=0x20000000',str(out/'codec.o'),str(out/'stack.o'),'-o',str(out/'codec_test.axf')],check=True)
    print('C codec golden test compiled/linked; NOT EXECUTED. Check codec_test_result in simulator.')
