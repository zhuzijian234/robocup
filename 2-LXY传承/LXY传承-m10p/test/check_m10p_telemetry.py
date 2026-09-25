"""Generate a real C type-7 frame and decode it with the production PC decoder."""
import re,sys,json
from check_m10p import P,OUT,run
t=(P/'HARDWARE/hc-05/ble_diag.c').read_text(encoding='utf-8')
def fn(name):
 m=re.search(r'^(?:static )?\w+\s+'+name+r'\([^;]*?\)\s*\{',t,re.M);assert m,name
 pos=t.index('{',m.start())+1;d=1
 while d:d+=(t[pos]=='{')-(t[pos]=='}');pos+=1
 return t[m.start():pos]
code=r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "m10p.h"
M10P_Stats M10P_stats;
uint8_t Diag_mode=3,frame[256];uint32_t Diag_session=123,tx_seq;
uint32_t LidarRx_epoch=2,LidarRx_blocks=90,LidarRx_peak=1024,LidarRx_late=0,LidarRx_copy_max_cycles=1200;
uint32_t M10P_control_seq=12,M10P_control_front_us=950000;
uint16_t M10P_front_bins=81,M10P_left_bins=121,M10P_right_bins=121;
float M10P_clearance_mm=1200,Radar_effective_target=6.25f;
uint32_t Diag_TimeUs(void){return 1000000;}
uint8_t Radar_Permitted(void){return 1;}
uint8_t BLE_NormalSpace(void){return 1;}
void Diag_Finalize(uint8_t *,uint16_t);
static unsigned emitted;
uint8_t BLE_Queue(const uint8_t *data,uint16_t n,uint8_t priority){
 unsigned i;uint8_t copy[256];(void)priority;memcpy(copy,data,n);Diag_Finalize(copy,n);
 for(i=0;i<n;i++)printf("%02x",copy[i]);printf("\n");emitted++;return 1;
}
'''
code+='\n'.join(fn(n) for n in ['put16','put32','Diag_CRC','header','Diag_Finalize','m10p_health'])
code+='''
int main(void){M10P_stats.packets=288;M10P_stats.invalid_slots=17;M10P_stats.high_reflect=99;
m10p_health(499);if(emitted)return 1;m10p_health(500);m10p_health(501);return emitted==1?0:2;}
'''
p=OUT/'telemetry.c';p.write_text(code,encoding='utf-8')
r=run('telemetry',p)
sys.path.insert(0,str(P/'上位机'))
import telemetry_v2 as v2
raw=bytes.fromhex(r['output'].strip());assert len(raw)==120
d=v2.Decoder();d.accept(raw,[],[],0.0);m=d.messages[0]
assert m['type']==7 and m['schema']==1 and m['session']==123
assert m['packets']==288 and m['invalid_slots']==17 and m['high_reflect']==99
assert m['front_age_us']==50000 and m['motor_permitted']==1 and m['effective_target_milli']==6250
assert not m['raw_crc_available']
bad=bytearray(raw);bad[20]^=1
try:d.accept(bytes(bad),[],[],0.0)
except ValueError:pass
else:raise AssertionError('CRC corruption accepted')
bad=v2.packet(7,bytes(104),1,123)
try:d.accept(bad,[],[],0.0)
except ValueError:pass
else:raise AssertionError('schema zero accepted')
(OUT/'telemetry_summary.json').write_text(json.dumps({'production_c':r,'decoded':m,'crc_and_schema_rejected':True},indent=2))
print('PASS production C -> PC M10P telemetry, CRC and schema checks')
