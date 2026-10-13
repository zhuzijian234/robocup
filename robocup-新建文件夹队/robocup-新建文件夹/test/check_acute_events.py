"""执行生产事件队列/同值rate/冻结函数，并交由真实上位机解码。"""
import json
import sys
from check_m10p import P, OUT, run

source=(P/'HARDWARE/hc-05/ble_diag.c').read_text(encoding='utf-8')
def function(name):
    start=source.index(name+'(')
    start=source.rfind('\n',0,start)+1
    body=source.index('{',start)
    depth=1
    end=body+1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}')
        end+=1
    return source[start:end]+'\n'

queue=source[source.index('typedef struct {\n    uint32_t control, input, ms, revision;'):
             source.index('/* 首次事件4帧')]
cloud=source[source.index('typedef struct {\n    uint32_t control, revision, input, us, epoch, reason;'):
             source.index('static void motor_reset')]
rate=source[source.index('    if (!strncmp(line, "rate ", 5))'):source.index('static void m10p_health')]
code='#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.h').as_posix()+'"\n'+r'''
#include <stdio.h>
#include <string.h>
static uint8_t frame[256],Diag_mode=3,Diag_rate=1,sample=3,config_pending;
static uint32_t control_seq,Diag_input_seq,Diag_input_us,Diag_revision=100,Diag_session=7;
static PathObservation Steering_path;
static PathCommand Steering_command;
static unsigned queue_accept=1,space=1,wire,emitted,rejected;
static unsigned BLE_NormalSpace(void){return space;}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void put32(uint8_t *p,uint32_t v){put16(p,(uint16_t)v);put16(p+2,(uint16_t)(v>>16));}
static void header(uint8_t type,uint16_t n){memset(frame,0,n);frame[3]=type;}
static unsigned BLE_Queue(uint8_t *p,unsigned n,unsigned priority){
 unsigned i;(void)priority;if(!queue_accept)return 0;++emitted;
 if(wire){printf("WIRE ");for(i=14;i<n-2;++i)printf("%02x",p[i]);puts("");}return 1;
}
static void Diag_Reject(const char *s){(void)s;++rejected;}
static void Send_Bluetooth_Data(const char *s){(void)s;}
#define CHECK(x) do{if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''+queue+cloud+function('acute_event_poll')+function('number')+r'''
static uint8_t rate_command(char *line){uint32_t n;char ack[64];
'''+rate+r'''
int main(void){
 unsigned i;PathPoint point={0,100};
 CHECK(rate_command("rate 1") && Diag_revision==100 && sample==3);
 CHECK(rate_command("rate 2") && Diag_revision==101 && Diag_rate==2 && sample==0);
 CHECK(rate_command("rate 0") && rejected==1 && Diag_revision==101);
 config_pending=1;
 CHECK(rate_command("rate 3") && rejected==2 && Diag_rate==2);
 config_pending=0;
 acute_event_reset();
 for(i=1;i<=6;++i){
   control_seq=i;Diag_input_seq=i+10;
   Steering_path.acute_phase=i<3?ACUTE_OFF:(uint8_t)(i-2);
   Steering_path.acute_reason=i<3?ACUTE_NO_EVIDENCE:(uint8_t)(i-2);
   Steering_path.acute_direction=i<3?0:1;
   Steering_path.acute_checks=i==1?ACUTE_CHECK_PREFILTER:ACUTE_CHECK_NEAR;
   acute_event_capture(i*83);
 }
 CHECK(acute_head==5 && acute_tail==0); /* seq2仅拒绝位变化被抽样，其余阶段完整。 */
 space=0;acute_event_poll();CHECK(acute_tail==0);
 space=1;queue_accept=0;acute_event_poll();CHECK(acute_tail==0);
 queue_accept=1;wire=1;for(i=0;i<6;++i)acute_event_poll();wire=0;
 CHECK(acute_tail==5 && emitted==5);
 acute_event_reset();
 for(i=0;i<12;++i){control_seq=100+i;Steering_path.acute_phase=i&1u;acute_event_capture(i);}
 CHECK(acute_head-acute_tail==8 && acute_dropped==4);
 wire=1;acute_event_poll();wire=0;CHECK(acute_tail==1);
 acute_event_reset();CHECK(!acute_seen && !acute_dropped && acute_head==acute_tail);
 Diag_session=0;acute_event_capture(999);CHECK(acute_head==0);Diag_session=7;
 /* 急弯首次确认触发前两帧+当帧+后一帧，不扩大8帧总点云内存。 */
 cloud_arm();memset(&Steering_path,0,sizeof Steering_path);Steering_path.valid=1;
 for(i=1;i<=6;++i){control_seq=i;Steering_path.acute_phase=i<5?ACUTE_OFF:(uint8_t)(i-4);Diag_CloudCapture(&point,1,1);}
 CHECK(cloud_state==2 && cloud_used==4 && cloud_next==2);
 CHECK(cloud_ring[2].control==3 && cloud_ring[3].control==4);
 CHECK(cloud_ring[0].control==5 && (cloud_ring[0].reason&128));
 CHECK(cloud_ring[1].control==6 && !(cloud_ring[1].reason&128));
 puts("PASS same-value rate, event sampling/retry/overflow/session and early cloud freeze");return 0;
}
'''
src=OUT/'acute_events.c';src.write_text(code,encoding='utf-8')
result=run('acute_events',src)
sys.path.insert(0,str(P/'上位机'))
import telemetry_v2 as v2
decoder=v2.Decoder()
for line in result['output'].splitlines():
    if line.startswith('WIRE '):
        decoder.accept(v2.packet(4,bytes.fromhex(line[5:]),len(decoder.messages),7),[],[],0)
messages=decoder.messages
assert [m['control_seq'] for m in messages[:5]]==[1,3,4,5,6]
assert [m['acute_phase'] for m in messages[:5]]==[0,1,2,3,4]
assert [m['event_ms'] for m in messages[:5]]==[83,249,332,415,498]
assert messages[0]['acute_checks']==512 and messages[5]['acute_event_dropped']==4
assert all(m['param_revision']==101 for m in messages)
(OUT/'acute_events_summary.json').write_text(json.dumps(dict(result=result,decoded=messages),indent=2),encoding='utf-8')
print('PASS production EVENT code3 -> PC decoder; actual occurrence time and frame pairing')
