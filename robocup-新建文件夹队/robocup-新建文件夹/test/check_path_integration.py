"""Run actual Steering_Update with peripheral shims; test the final write boundary."""
import json
import re
from check_m10p import P, OUT, run


def text(name):
    return (P/name).read_text(encoding='utf-8-sig')


def without_includes(source):
    return re.sub(r'^#include[^\n]*\n', '', source, flags=re.M)


vehicle=text('HARDWARE/LEIDA_DATA/m10p_vehicle.c')
usable=vehicle[vehicle.index('uint8_t M10P_ScanUsable'):]
program=r'''
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "m10p.h"
uint32_t LidarRx_epoch=1,Diag_revision,clock_us=100000;
uint32_t Diag_detail_u[24];float Diag_detail_f[24];
volatile float Diag_motor_integral,Diag_motor_prelimit;
uint8_t M10P_perception_why,M10P_steer_source;
#define M10P_WHY_CAPACITY 128u
#define M10P_SRC_HOLD 4u
static struct {uint32_t CCR1;} servo;
#define TIM3 (&servo)
static unsigned writes,calls,epoch_during_build;
uint32_t Diag_TimeUs(void) {
    if(++calls==2 && epoch_during_build) ++LidarRx_epoch;
    return clock_us;
}
void Servo_ChangePwm(uint16_t pwm){++writes;TIM3->CCR1=pwm;}
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
'''
program+=without_includes(text('HARDWARE/CENTRE_LINE/path_track.h'))
program+=without_includes(text('HARDWARE/CENTRE_LINE/path_track.c'))
program+=without_includes(text('HARDWARE/CENTRE_LINE/CENTRE_LINE.h'))
program+=usable
program+=without_includes(text('HARDWARE/CENTRE_LINE/CENTRE_LINE.c'))
program+=r'''
static M10P_Scan scan;
static PathPoint input[120];
int main(void){
 unsigned i,old_writes;float old_width;
 Steering_Init();TIM3->CCR1=1445;
 scan.epoch=1;scan.start_us=20000;scan.end_us=100000;scan.count=120;
 for(i=0;i<60;++i){
   input[2*i].x=-277;input[2*i+1].x=223;
   input[2*i].y=input[2*i+1].y=110+i*19.0f;
 }
 /* No front echo still permits a real dual/side path. */
 Steering_Update(&scan,input,120);
 CHECK(writes==1 && Servo_PD_valid && Steering_command.applied && M10P_steer_source==0);
 CHECK(Diag_detail_u[9]==1 && Diag_detail_u[7]==0 && TIM3->CCR1==1445);
 old_writes=writes;scan.epoch=2;Steering_Update(&scan,input,120);
 CHECK(writes==old_writes && !Servo_PD_valid && Steering_command.reason==PATH_BAD_SCAN);
 scan.epoch=1;scan.start_us=clock_us-M10P_MAX_AGE_US-1;
 Steering_Update(&scan,input,120);CHECK(writes==old_writes && !Servo_PD_valid);
 scan.start_us=20000;M10P_perception_why=M10P_WHY_CAPACITY;
 Steering_Update(&scan,input,120);CHECK(writes==old_writes);
 M10P_perception_why=0;
 Steering_Update(&scan,input,0);CHECK(writes==old_writes && Steering_command.reason==PATH_NO_GEOMETRY);
 old_width=geometry.width;
 /* Reception epoch changes while computing: final check forbids any write/width update. */
 for(i=0;i<60;++i){input[2*i].x-=50;input[2*i+1].x+=50;}
 epoch_during_build=1;calls=0;Steering_Update(&scan,input,120);
 CHECK(writes==old_writes && !Servo_PD_valid && geometry.width==old_width);
 CHECK(!Steering_path.valid && !Steering_path.width_measured && Steering_path.source==PATH_NONE);
 epoch_during_build=0;scan.epoch=LidarRx_epoch;
 Steering_Update(&scan,input,120);CHECK(writes==old_writes+1 && Steering_command.d==0);
 CHECK(Steering_path.width_measured && geometry.width>old_width);
 /* Parameter revision invalidates D history without resetting speed integral. */
 Speed_pid.err_sum=123;Diag_revision++;Steering_Update(&scan,input,120);
 CHECK(Servo_PD_valid && Steering_command.d_reset && Speed_pid.err_sum==123);
 printf("PASS %u production steering write/freshness/epoch checks\n",checks);return 0;
}
'''
source=OUT/'path_integration.c';source.write_text(program,encoding='utf-8')
result=run('path_integration',source)
(OUT/'path_integration_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
