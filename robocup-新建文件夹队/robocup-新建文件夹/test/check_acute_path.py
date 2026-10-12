"""直接编译生产路径与蓝牙发送层，检查急弯边界和STATE去抖。"""
import json
import re
from check_m10p import P, OUT, run

results = [run('acute_path', P/'test/test_acute_path.c')]
source = r'''
#include <stdint.h>
#include <string.h>
#include <stdio.h>
static uint32_t clock_ms, Diag_session=7;
static uint8_t link_up=1;
#define BSP_BLUETOOTH 0
#define USART_IT_TXE 0
#define ENABLE 1
#define DISABLE 0
void Bluetooth_Mode(void){}
uint8_t Get_Bluetooth_ConnectFlag(void){return link_up;}
uint32_t Diag_TimeMs(void){return clock_ms;}
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t x){(void)x;}
void __DMB(void){}
void USART_ITConfig(int a,int b,int c){(void)a;(void)b;(void)c;}
void USART_SendData(int a,uint8_t b){(void)a;(void)b;}
void Diag_Finalize(uint8_t *a,uint16_t b){(void)a;(void)b;}
'''
source += re.sub(r'^#include[^\n]*\n', '', (P/'HARDWARE/hc-05/ble_tx.c').read_text(encoding='utf-8'), flags=re.M)
source += r'''
int main(void){
 BLE_TxPoll();link_up=0;clock_ms=1;BLE_TxPoll();
 clock_ms=99;BLE_TxPoll();if(Diag_session!=7)return 1;
 link_up=1;BLE_TxPoll();link_up=0;clock_ms=100;BLE_TxPoll();
 clock_ms=199;BLE_TxPoll();if(Diag_session!=7)return 2;
 clock_ms=200;BLE_TxPoll();if(Diag_session!=0)return 3;
 link_up=1;Diag_session=9;clock_ms=0xfffffff0u;BLE_TxPoll();
 link_up=0;BLE_TxPoll();clock_ms+=100;BLE_TxPoll();
 if(Diag_session!=0)return 4;
 puts("PASS Bluetooth STATE glitch, persistent loss and timer wrap");return 0;
}
'''
file = OUT/'acute_ble.c'
file.write_text(source, encoding='utf-8')
results.append(run('acute_ble', file))

# 新现场与镜像。独立快照不能重建全部历史，但可验证去重、左右对称及正常出口保留。
cases = json.loads((P/'test/path_replay_acute_20261011.json').read_text(encoding='utf-8'))['cases']
source = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"',
          '#include <stdio.h>', 'int main(void){PathGeometry g;PathObservation a,b;unsigned i;']
for case in cases:
    pts = ','.join('{%d,%d}' % tuple(p) for p in case['points'])
    width = case['width']
    if not 400 <= width <= 900:
        width = 500
    init = f'memset(&g,0,sizeof g);g.width={width:.9f}f;g.configured_width=500;g.measured=1;'
    source += ['{PathPoint p[]={'+pts+'};', init,
               f'Path_Build(&g,p,{len(case["points"])},500,900,10,&a);',
               f'printf("{case["tag"]}:{case["seq"]} valid=%u model=%u duplicates=%u curve_checks=%lu\\n",a.valid,a.model,a.duplicate_samples,(unsigned long)a.curve_checks);',
               'if(a.valid && (!isfinite(a.target_x)||!isfinite(a.ref_y)||fabsf(a.outer_offset)>40))return 1;']
    if case['tag'] == '200137':
        source += [f'if(!a.valid || a.source!=PATH_DUAL || fabsf(a.target_x-({case["recorded_target"]:.9f}f))>2)return 2;']
    if case['tag'] == '195850' and case['seq'] == 17779:
        source += ['if(a.duplicate_samples<6 || (a.valid && a.source==PATH_DUAL))return 3;']
    source += ['for(i=0;i<sizeof(p)/sizeof(p[0]);++i)p[i].x=-p[i].x;',
               'for(i=0;i<sizeof(p)/sizeof(p[0])/2;++i){unsigned k=sizeof(p)/sizeof(p[0])-1-i;PathPoint t=p[i];p[i]=p[k];p[k]=t;}',
               init, f'Path_Build(&g,p,{len(case["points"])},500,900,-10,&b);',
               'if(a.valid!=b.valid || a.model!=b.model || a.duplicate_samples!=b.duplicate_samples)return 4;',
               'if(a.valid && (fabsf(a.target_x+b.target_x)>1 || fabsf(a.ref_y-b.ref_y)>1))return 5;}',]
source += ['puts("PASS 28 new acute snapshots and 28 mirrors; improved left exit preserved");return 0;}']
file = OUT/'acute_replay.c'
file.write_text('\n'.join(source), encoding='utf-8')
results.append(run('acute_replay', file))
(OUT/'acute_summary.json').write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
