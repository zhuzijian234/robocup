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

# Exercise the actual schema-2 producer, not just a Python-constructed payload.
path_code=code[:code.index('int main(void)')]+r'''
uint32_t Diag_detail_u[24],Diag_revision=7,control_seq=88,hold_count=2;
uint32_t Diag_input_drop,Diag_tx_drop,motor_dropped;
float Diag_detail_f[24];
'''+fn('detail_submit')+r'''
int main(void){
 Diag_detail_u[4]=5;Diag_detail_u[7]=1;Diag_detail_u[8]=2;
 Diag_detail_u[9]=0;Diag_detail_u[10]=1;Diag_detail_u[17]=0;
 Diag_detail_u[19]=1234;Diag_detail_u[20]=1445;
 Diag_detail_f[2]=-80;Diag_detail_f[10]=700;Diag_detail_f[13]=500;
 Diag_detail_f[16]=450;Diag_detail_f[22]=550;
 detail_submit(1800,2,0);return emitted==1?0:2;
}
'''
p=OUT/'path_telemetry.c';p.write_text(path_code,encoding='utf-8')
path_result=run('path_telemetry',p)
d=v2.Decoder();raw=bytes.fromhex(path_result['output'].strip());assert len(raw)==208
d.accept(raw,[],[],0.0);m=d.messages[0]
assert m['schema']==2 and m['source']==1 and m['command_applied']==0
assert m['candidate_pwm']==1234 and m['final_pwm']==1445
assert m['ref_y']==700 and m['left_span']==450 and m['left_far_span']==550
assert 'radar_crc_bad' not in m and m['pd_p']==-80
assert m['control_seq']==88 and m['process_us']==1800
(OUT/'path_telemetry_summary.json').write_text(json.dumps(m,indent=2))
print('PASS production C -> PC PATH schema-2 candidate/applied/geometry fields')

# 同时验证生产端扩展编码、快照冻结/限速/队列失败重试和PC完整性导出。
header=(P/'HARDWARE/CENTRE_LINE/path_track.h').read_text(encoding='utf-8')
cloud_start=t.index('typedef struct {\n    uint32_t control, revision')
cloud_end=t.index('static void motor_reset', cloud_start)
base=code[:code.index('int main(void)')]
base=base.replace('static unsigned emitted;', 'static unsigned emitted; static int queue_accept=1;')
base=base.replace('unsigned i;uint8_t copy[256];', 'unsigned i;uint8_t copy[256];if(!queue_accept)return 0;')
extended_code=base+header+r'''
uint32_t control_seq,Diag_revision=7,Diag_input_seq,Diag_input_us;
PathObservation Steering_path;
PathCommand Steering_command;
'''+t[cloud_start:cloud_end]+fn('path_extra_submit')+fn('cloud_poll')+r'''
int main(void){
 unsigned i;PathPoint points[400];
 Steering_path.valid=1;Steering_path.near_ref_y=425;
 Steering_path.near_fit[0].rejected=FIT_SPAN|FIT_POINTS;
 Steering_path.avoid_state=AVOID_RELEASE;Steering_path.avoid_offset=15.2f;
 Steering_command.road_error=15.9f;Steering_command.gate_reason=PATH_WAIT_EXIT;
 path_extra_submit();
 for(i=0;i<400;++i){points[i].x=(float)i-200.75f;points[i].y=100+(float)i;}
 cloud_arm();
 for(i=1;i<=6;++i){
   control_seq=i;Diag_input_seq=i+100;Diag_input_us=i*80000;
   if(i==5)Steering_path.valid=0;
   Diag_CloudCapture(points,400,2);
 }
 if(cloud_state!=2 || cloud_used!=4 || cloud_next!=2)return 3;
 control_seq=999;Diag_CloudCapture(points,400,99);
 if(cloud_ring[1].control!=6)return 4;
 cloud_freeze();
 if(cloud_count()!=8 || !recent_frozen)return 8;
 control_seq=1000;Diag_CloudCapture(points,400,100);
 if(cloud_recent[(recent_next+3)&3].control!=999)return 9;
 cloud_freeze(); /* 重复停止不能重写末段。 */
 cloud_sending=1;cloud_frame=0;cloud_offset=0;
 queue_accept=0;cloud_poll(200);
 if(cloud_offset || cloud_frame || emitted!=1)return 5;
 queue_accept=1;cloud_poll(199);if(emitted!=1)return 6;
 for(i=1;i<=80;++i)cloud_poll(i*200);
 if(cloud_sending || emitted!=81)return 7;
 return 0;
}
'''
p=OUT/'path_extra_cloud.c';p.write_text(extended_code,encoding='utf-8')
result=run('path_extra_cloud',p)
d=v2.Decoder()
for line in result['output'].splitlines():
    if line.strip():d.accept(bytes.fromhex(line.strip()),[],[],0.0)
extra=d.messages[0]
assert extra['type']==8 and extra['fit_rejected']==3 and extra['near_ref_y']==425
assert abs(extra['road_error']-15.9)<.001 and extra['gate_reason']==2
clouds=d.messages[1:]
assert len(clouds)==80 and sum(c['count'] for c in clouds)==3200
assert {c['control_seq'] for c in clouds[:40]}=={3,4,5,6}
assert {c['control_seq'] for c in clouds[40:]}=={4,5,6,999}
assert all(c['trigger_flags'] & 32 for c in clouds[40:])
assert clouds[-1]['trigger_flags'] & 8 and clouds[-1]['frame_index']==7
assert clouds[0]['points'][0]==(-200,100) and clouds[-1]['offset']==360
v2.write_clouds(d.messages,OUT/'production_snapshot')
import csv
with (OUT/'production_snapshot.cloud_frames.csv').open(encoding='utf-8-sig') as stream:
    assert all(r['complete']=='1' for r in csv.DictReader(stream))
assert v2.cloud_dump_complete(d.messages,clouds[0]['session'],clouds[0]['snapshot_id'],8)
print('PASS production C -> PC PATH extra and dual 8-frame/3200-point snapshot, freeze/throttle/retry')

# 第二组自动冻结必须等到连续恢复，再捕获连续失效；第一次孤立抖点不抢占缓存。
auto_code=extended_code[:extended_code.index('int main(void)')]+r'''
int main(void){
 PathPoint p[1]={{100,200}};unsigned i;
 cloud_arm();Steering_path.valid=1;
 for(i=1;i<=4;++i){control_seq=i;Diag_CloudCapture(p,1,2);}
 Steering_path.valid=0;
 for(i=5;i<=6;++i){control_seq=i;Diag_CloudCapture(p,1,2);}
 if(cloud_state!=2)return 1;
 Steering_path.valid=1;
 for(i=7;i<=9;++i){control_seq=i;Diag_CloudCapture(p,1,2);}
 Steering_path.valid=0;control_seq=10;Diag_CloudCapture(p,1,2);
 if(recent_good || recent_frozen)return 2;
 Steering_path.valid=1;
 for(i=11;i<=16;++i){control_seq=i;Diag_CloudCapture(p,1,2);}
 Steering_path.valid=0;control_seq=17;Diag_CloudCapture(p,1,2);
 if(recent_frozen)return 3;
 control_seq=18;Diag_CloudCapture(p,1,2);
 if(!recent_frozen || cloud_count()!=8)return 4;
 if(cloud_recent[(recent_next+3)&3].control!=18)return 5;
 control_seq=99;Diag_CloudCapture(p,1,2);cloud_freeze();
 if(cloud_recent[(recent_next+3)&3].control!=18 || (cloud_recent[(recent_next+3)&3].reason&8))return 6;
 puts("PASS second cloud auto-freeze: recovery, isolated failure, persistent failure, immutable event");return 0;
}
'''
p=OUT/'cloud_auto.c';p.write_text(auto_code,encoding='utf-8');run('cloud_auto',p)

race_code=base+header+r'''
uint32_t control_seq=4,Diag_revision=7,Diag_input_seq=100;
uint32_t Diag_uart_ore=2,Diag_uart_fe=3,Diag_uart_ne=4,Diag_uart_last_us=12345,Diag_uart_input_seq=99;
uint32_t Steering_speed_us=998000;
uint8_t Steering_speed_reason=1;
float Steering_speed_limit=10,Speed_mubiao=17,RACE_BEND_SPEED=10,RACE_ACCEL=2,RACE_DECEL=12;
PathObservation Steering_path;
PathCommand Steering_command;
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t p){(void)p;}
'''+fn('race_submit')+r'''
int main(void){
 Steering_path.model=2;Steering_path.support_mm=700;Steering_path.heading_rad=.3f;
 Steering_path.preview_heading_rad=-.4f;Steering_path.s_preview=1;Steering_path.turn_distance_mm=400;
 Steering_path.duplicate_samples=8;Steering_path.identity_corrected=1;
 Steering_path.outer_offset=-25;Steering_path.target_delta=-492;Steering_path.curve_checks=64;
 Steering_command.transition_limited=1;
 race_submit();control_seq=5;race_submit();return emitted==1?0:1;
}
'''
p=OUT/'race_codec.c';p.write_text(race_code,encoding='utf-8');r=run('race_codec',p)
raw=bytes.fromhex(r['output'].strip());assert len(raw)==176
d=v2.Decoder();d.accept(raw,[],[],0.0);m=d.messages[0]
assert m['model']==2 and m['s_preview']==1 and m['speed_ceiling']==17 and m['bend_speed']==10
assert m['uart_ore']==2 and m['uart_fe']==3 and m['uart_ne']==4 and m['uart_input_seq']==99
assert m['support_mm']==700 and m['speed_reason']==1 and m['speed_plan_us']==998000
assert m['schema']==3 and m['duplicate_samples']==8 and m['identity_corrected']==1
assert m['outer_offset']==-25 and m['target_delta']==-492 and m['curve_checks']==64 and m['transition_limited']==1
import struct
for payload in (raw[14:-2][:-4],struct.pack('<I',4)+raw[18:-2],raw[14:102]+struct.pack('<f',float('nan'))+raw[106:-2]):
    try:v2.Decoder().accept(v2.packet(10,payload,1,123),[],[],0.0)
    except ValueError:pass
    else:raise AssertionError('invalid race frame accepted')
from types import SimpleNamespace
v2.write_extensions(SimpleNamespace(v2=d),OUT/'production_race')
with (OUT/'production_race.race.csv').open(encoding='utf-8-sig') as stream:
    assert next(csv.DictReader(stream))['speed_ceiling']=='17.0'
print('PASS production C -> PC RACE schema/fields/rate/export; invalid length/schema/nonfinite rejected')
