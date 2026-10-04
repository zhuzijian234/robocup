"""验证真实蓝牙发送队列：整包优先级、ISR连续发送与预留槽。"""
from pathlib import Path
import os,runpy,subprocess
here=Path(__file__).resolve().parent
os.environ['ROBOCUP_CONTROL_TEST_OUT']=str(here.parents[2]/'tmp/ld14p_navigation/tx_tests')
b=runpy.run_path(str(here/'check_control_fixes.py'))
s=b['source']('HARDWARE/hc-05/ble_tx.c')
s='\n'.join(line for line in s.splitlines() if not line.startswith('#include'))
prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define __DMB() ((void)0)
#define BSP_BLUETOOTH 0
#define USART_IT_TXE 1
#define ENABLE 1
#define DISABLE 0
static uint32_t Diag_session=1,tx_seq;
static uint8_t connected=1,wire[1024];static unsigned used;static int tx_enabled;
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t m){}
static void Bluetooth_Mode(void){}
static uint8_t Get_Bluetooth_ConnectFlag(void){return connected;}
static void USART_ITConfig(int p,int flag,int en){tx_enabled=en;}
static void USART_SendData(int p,uint8_t byte){wire[used++]=byte;}
'''
for name in ['put16','put32','Diag_CRC','Diag_Finalize']:prefix+='\n'+b['function'](b['diag'],name)
program=prefix+s+r'''
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL %d: %s\n",__LINE__,#c);return 1;}}while(0)
int main(void){
    uint8_t low[16]={0xaa,0x55,2,5,16},high[16]={0xaa,0x55,2,2,16};
    int i;
    CHECK(BLE_Queue(low,16,0));CHECK(BLE_Queue(high,16,2));
    BLE_TxPoll();
    /* 不再调用main轮询，ISR仍需发送全部排队帧。 */
    for(i=0;i<32;i++)BLE_TxIRQ();
    CHECK(used==32 && wire[3]==2 && wire[19]==5 && !tx_enabled);
    CHECK(wire[6]==0 && wire[22]==1);
    CHECK((wire[14]|wire[15]<<8)==Diag_CRC(wire+2,12));
    CHECK((wire[30]|wire[31]<<8)==Diag_CRC(wire+18,12));
    CHECK(BLE_FreeCritical()==6);
    for(i=0;i<4;i++)CHECK(BLE_Queue(low,16,0));
    CHECK(!BLE_Queue(low,16,0));
    CHECK(BLE_Queue(high,16,1));CHECK(BLE_Queue(high,16,2));
    BLE_TxPoll();
    for(i=0;i<96;i++)BLE_TxIRQ();
    CHECK(BLE_FreeCritical()==6 && !tx_enabled);
    CHECK(Diag_tx_drop==1);
    printf("PASS %d TX scheduling/CRC checks (production C)\n",checks);
    return 0;
}
'''
out=b['OUT'];src=out/'tx_tests.c';src.write_text(program,encoding='utf-8');exe=out/'tx_tests.exe'
r=subprocess.run([str(b['compiler']),'/nologo','/std:c11','/utf-8',str(src),'/Fe:'+str(exe),'/Fo:'+str(out/'tx_tests.obj')],env=b['env'],capture_output=True)
(out/'tx_compile.log').write_bytes(r.stdout+r.stderr)
if r.returncode:print((r.stdout+r.stderr).decode(errors='replace'));raise SystemExit(r.returncode)
r=subprocess.run([str(exe)],capture_output=True);(out/'tx_run.log').write_bytes(r.stdout+r.stderr)
print(r.stdout.decode(errors='replace'));raise SystemExit(r.returncode)
