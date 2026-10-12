"""阶段序列、已有连续点云保留、生产RACE编码与上位机解码的一致性。"""
import json
import sys
from check_m10p import P, OUT, run

results = [run('acute_stages', P/'test/test_acute_stages.c')]
cases = json.loads((P/'test/path_sequence_acute_20261012.json').read_text(encoding='utf-8'))['cases']
code = ['#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.c').as_posix()+'"', '#include <stdio.h>',
        'int main(void){PathGeometry g={0};PathController c={0};PathObservation p;PathCommand o;unsigned now=0,pwm=1445;',
        'PathGains gains={.035f,.035f,.04f,.022f,.0395f,.02f,10,200,1170,1445,1720,17};']
previous = None
for case in cases:
    if not previous or previous['tag'] != case['tag'] or previous['seq'] + 1 != case['seq']:
        width = case['width'] if 400 <= case['width'] <= 900 else 500
        code += ['memset(&g,0,sizeof g);memset(&c,0,sizeof c);pwm=1445;',
                 f'g.width={width:.6f}f;g.configured_width=500;g.measured=1;']
    code += ['{PathPoint v[]={'+','.join('{%s,%s}' % tuple(p) for p in case['points'])+'};',
             f'Path_Build(&g,v,{len(case["points"])},500,900,10,&p);',
             'now+=80000;Path_Control(&c,&p,&gains,1,now,pwm,&o);pwm=o.pwm;']
    fields = ['p.valid','p.model','p.target_x','p.ref_y','p.outer_offset',
              'o.pwm','o.reason','o.applied','o.error','o.transition_limited']
    checks = [f'fabsf({field}-({value:.6f}f))<.002f' for field, value in zip(fields, case['expected'])]
    checks += ['p.acute_phase!=ACUTE_PREPARE']
    code += ['if(!('+' && '.join(checks)+')){printf("FAIL '+case['tag']+'/'+str(case['seq'])+'\\n");return 1;}}']
    previous = case
code += ['puts("PASS 128 recorded clouds in contiguous blocks: geometry and PWM baseline preserved");return 0;}']
source=OUT/'acute_sequence_preservation.c'
source.write_text('\n'.join(code),encoding='utf-8')
results.append(run('acute_sequence_preservation',source))

# 直接执行生产race_submit，再交给上位机解码，防止C/Python各自测试却互不兼容。
diag=(P/'HARDWARE/hc-05/ble_diag.c').read_text(encoding='utf-8')
submit=diag[diag.index('static void race_submit(void)'):diag.index('static void detail_submit(')]
extra=diag[diag.index('static void path_extra_submit(void)'):diag.index('/* 每200ms最多发送40点')]
source=OUT/'acute_race_wire.c'
source.write_text('#include "'+(P/'HARDWARE/CENTRE_LINE/path_track.h').as_posix()+'"\n'+r'''
#include <stdio.h>
#include <string.h>
static uint8_t frame[256], Diag_mode=3;
static uint32_t control_seq=8, Diag_session=7, Diag_revision=1, Diag_input_seq=8;
static uint32_t Diag_uart_ore,Diag_uart_fe,Diag_uart_ne,Diag_uart_last_us,Diag_uart_input_seq,Steering_speed_us;
static unsigned Steering_speed_reason;
static float Steering_speed_limit=15,Radar_effective_target=15,Speed_mubiao=17,RACE_BEND_SPEED=15,RACE_ACCEL=2,RACE_DECEL=12;
static PathObservation Steering_path;
static PathCommand Steering_command;
static unsigned BLE_NormalSpace(void){return 1;}
static unsigned __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(unsigned irq){(void)irq;}
static void header(unsigned type,unsigned n){(void)type;memset(frame,0,n);}
static void put32(uint8_t *p,uint32_t value){memcpy(p,&value,4);}
static void BLE_Queue(uint8_t *data,unsigned n,unsigned priority){
    unsigned i;(void)priority;
    for(i=14;i<n-2;++i)printf("%02x",data[i]);puts("");
}
'''+submit+extra+r'''
int main(void){
 Steering_path.acute_phase=ACUTE_PREPARE;Steering_path.acute_reason=ACUTE_POSITIONING;
 Steering_path.acute_direction=-1;Steering_path.acute_position=12.5f;
 Steering_path.acute_goal=40;Steering_path.acute_distance=925;Steering_path.acute_heading=-.1f;
 Steering_path.outer_offset=36;Steering_path.target_delta=99;
 Steering_path.road_target_x=-50;Steering_path.target_x=40;Steering_command.road_error=60;
 race_submit();path_extra_submit();return 0;
}
''',encoding='utf-8')
result=run('acute_race_wire',source)
sys.path.insert(0,str(P/'上位机'))
import telemetry_v2 as v2
lines=[line for line in result['output'].splitlines() if line.strip()]
payload=bytes.fromhex(lines[0])
decoder=v2.Decoder()
decoder.accept(v2.packet(10,payload,0,7),[],[],0)
msg=decoder.messages[0]
assert len(payload)+16==176 and msg['schema']==3
assert (msg['control_seq'],msg['param_revision'],msg['input_seq'])==(8,1,8)
assert (msg['acute_phase'],msg['acute_reason'],msg['acute_direction_code'])==(2,2,0)
assert (msg['acute_position_mm'],msg['acute_goal_mm'],msg['acute_feature_y_mm'])==(12.5,40,925)
assert msg['outer_offset']==36 and msg['target_delta']==99 and msg['speed_ceiling']==17
decoder.accept(v2.packet(8,bytes.fromhex(lines[1]),1,7),[],[],0)
msg=decoder.messages[1]
assert (msg['road_target'],msg['target_x'],msg['road_error'])==(-50,40,60)
results.append(dict(wire_bytes=176,decoded=True))
(OUT/'acute_stages_summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS production RACE schema3 encoder -> host decoder; size, offsets, enums and sequence pairing')
