"""Execute extracted production C on Windows (MSVC), with mocked hardware.

Tests M10P shared geometry, PD transitions/direction and
signed encoder wrap. This is not a vehicle or ARM peripheral simulation.
"""
from pathlib import Path
import json
import os
import re
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
OUT = Path(os.environ.get('ROBOCUP_CONTROL_TEST_OUT', str(PROJECT.parents[1] / 'tmp/control_fix_native')))
OUT.mkdir(parents=True, exist_ok=True)


def source(relative):
    return (PROJECT / relative).read_text(encoding='utf-8-sig')


def function(text, name):
    match = re.search(r'^(?:static )?\w+\s+' + name + r'\([^;]*?\)\s*\{', text, re.M)
    if not match:
        raise ValueError(name)
    start = text.index('{', match.start())
    pos, depth = start + 1, 1
    while depth:
        depth += (text[pos] == '{') - (text[pos] == '}')
        pos += 1
    return text[match.start():pos]


radar = source('HARDWARE/LEIDA_DATA/LEIDA_DATA.c')
steering = source('HARDWARE/CENTRE_LINE/CENTRE_LINE.c')
diag = source('HARDWARE/hc-05/ble_diag.c')
motor = source('HARDWARE/moto/moto.c')
prefix = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#define LEIDA_DATA_COUNTER 800
#define LEIDA_ANGLE_CENTER 90.0f
#define LEIDA_ANGLE_LEFT 180.0f
#define LEIDA_ANGLE_RIGHT 0.0f
#define LEIDA_ANGLE_yuliang 75.0f
#define LEIDA_ANGLE_piancha 0.5f
#define PI 3.14159265358979323846f
#define SERVO_PWM_MIN 1170
#define SERVO_PWM_MAX 1720
#define SERVO_PWM_MID 1445
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
typedef uint8_t u8;typedef uint16_t u16;
typedef struct{float angle,distance;} _LEIDA_DATA;
typedef struct{float _x,_y;} _LEIDA_DATA_plane;
typedef struct{float k,b;} Midline_type;
typedef struct{float kp,kp_2,kp_3,ki,kd,kd_2,kd_3,v_set,v_fb,err_ll,err_l,err,err_sum,erry,out,out_max,out_min;} pid_type;
static struct{uint16_t CCR1;} timer3={1445};
#define TIM3 (&timer3)
#define TIM4 4
static uint16_t encoder_counter;
static unsigned servo_writes;
uint16_t TIM_GetCounter(int ignored){return encoder_counter;}
void TIM_SetCounter(int ignored,uint16_t value){encoder_counter=value;}
uint32_t Diag_detail_u[24];float Diag_detail_f[16];
uint16_t LEIDA_speed_dps,LEIDA_raw_count;
float zhongxian_junzhi,zhongxian_chuizhi;
uint8_t LEIDA_vertical_valid,Servo_PD_valid;
static uint8_t pd_history_valid;static uint16_t pd_previous_mode;static uint32_t pd_previous_us;
float BLUE_Y_RIGHT=1200,BLUE_Y_LEFT=1350,BLUE_Y_STRA_SEL=0,BLUE_Y_STRA=750;
float BLUE_DIS_RIGHT=50,BLUE_DIS_LEFT=50,paodao_distance=800;
static uint32_t clock_us;
uint32_t Diag_TimeUs(void){return clock_us;}
void Diag_Fit(const void *line,uint8_t valid){}
void Servo_ChangePwm(uint16_t value){timer3.CCR1=value;servo_writes++;}
float Encoder_cnt,Speed_now;int16_t Encoder_cnt_arr[5];uint16_t Encoder_cnt_temp;
uint16_t LEIDA_DATA_HANDLE10(_LEIDA_DATA_plane *,u16);
'''
header = source('HARDWARE/CENTRE_LINE/CENTRE_LINE.h')
prefix += header[header.index('#define TURN_GUARD_US'):header.index('uint16_t TurnGuard_Apply')]
functions = ['static int16_t angle_heads[720], angle_next[LEIDA_DATA_COUNTER];',
             function(radar, 'angle_index_build'), function(radar, 'angle_nearest')]
functions += [function(radar, name) for name in ['LEIDA_DATA_HANDLE10','LEIDA_DATA_HANDLE4','LEIDA_DATA_HANDLE11']]
functions += [function(steering, name) for name in ['Midline_fit','Midline_PD_Reset','pd_reject',
    'turn_direction','turn_rank','Midline_PD_Calculate','Midline_PD','TurnGuard_Apply']]
functions += [function(motor, 'Get_Encoder')]

tests = r'''
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
static _LEIDA_DATA points[800];
static _LEIDA_DATA_plane plane[400];
static pid_type pid;
static int deferred;
static void begin(void){memset(Diag_detail_u,0,sizeof Diag_detail_u);memset(Diag_detail_f,0,sizeof Diag_detail_f);clock_us+=115000;}
static uint16_t drive(uint16_t mode,float error){
    Midline_type line={1,650+error};
    begin();plane[0]._y=plane[1]._y=700;
    plane[0]._x=plane[1]._x=mode==1?-error-400:mode==2?400-error:50-error;
    if(mode==3 || mode==4 || mode==8 || mode==9)line.k=fabs(error)>0?175.0f/fabs(error):INFINITY;
    LEIDA_vertical_valid=1;zhongxian_chuizhi=50-error;
    return deferred ? Midline_PD_Calculate(plane,&pid,&line,144.5f,0,2,mode) :
                      Midline_PD(plane,&pid,&line,144.5f,0,2,mode);
}
static int run(void){
    unsigned k;Midline_type line;
    uint16_t output;float before;
    TurnGuard guard={0};uint8_t held;unsigned writes;
    pid.kp=.035f;pid.kd=.035f;pid.kp_2=.040f;pid.kd_2=.022f;pid.kp_3=.0395f;pid.kd_3=.020f;
    CHECK(LEIDA_DATA_HANDLE10(plane,0)==0);
    for(k=0;k<8;k++){plane[k]._x=50;plane[k]._y=(float)k*100;}
    CHECK(LEIDA_DATA_HANDLE10(plane,8)==8);
    CHECK(LEIDA_DATA_HANDLE11(plane,0,8)==50 && LEIDA_vertical_valid);
    for(k=0;k<4;k++)plane[k]._x=(float)k*100;
    CHECK(LEIDA_DATA_HANDLE11(plane,0,4)==0 && !LEIDA_vertical_valid);
    CHECK(LEIDA_DATA_HANDLE11(plane,0,0)==0 && !LEIDA_vertical_valid);
    for(k=0;k<4;k++)plane[k]._x=0;
    CHECK(LEIDA_DATA_HANDLE11(plane,0,4)==0 && LEIDA_vertical_valid);
    points[0].angle=30;points[0].distance=400;points[1].angle=150;points[1].distance=400;
    CHECK(LEIDA_DATA_HANDLE4(plane,points,2)>0); /* index zero usable */
    points[0].angle=30.3f;points[1].angle=149.7f;
    CHECK(LEIDA_DATA_HANDLE4(plane,points,2)==1); /* overlapping windows, one pair */
    CHECK(LEIDA_DATA_HANDLE11(plane,0,1)==0 && !LEIDA_vertical_valid);
    points[0].angle=0;points[1].angle=179;
    CHECK(LEIDA_DATA_HANDLE4(plane,points,2)==0); /* no cross-angle stale pairing */
    CHECK(!Midline_fit(plane,0,0,&line));
    plane[0]._x=plane[1]._x=20;CHECK(!Midline_fit(plane,0,2,&line));
    before=timer3.CCR1;begin();Midline_PD(plane,&pid,&line,144.5f,0,0,2);
    CHECK(!Servo_PD_valid && timer3.CCR1==before && (Diag_detail_u[4]&512));
    Midline_PD_Reset();output=drive(0,-200);CHECK(output==1375);
    output=drive(5,247.7f);CHECK(output>=1531 && output<=1532 && Diag_detail_f[3]==0);
    output=drive(0,-102.4f);CHECK(output>=1409 && output<=1410 && Diag_detail_f[3]==0);
    Midline_PD_Reset();drive(5,421.156f);output=drive(5,71.5988f);
    CHECK(output>=1445 && fabs(Diag_detail_f[3])<=60); /* old PWM was 1202 */
    Midline_PD_Reset();drive(2,500);output=drive(2,-500);CHECK(output>=1445 && (Diag_detail_u[4]&1024));
    output=drive(1,100);CHECK(output<=1445);
    output=drive(4,500);CHECK(output>=1445);
    before=timer3.CCR1;output=drive(4,0);CHECK(!Servo_PD_valid && output==before);
    /* A reset/invalid observation does not leave a stale D kick. */
    Midline_PD_Reset();output=drive(7,-20);CHECK(Diag_detail_f[3]==0);
    /* Real telemetry: large-left e=500 -> side-wall e=82.391 is not a derivative. */
    drive(4,500);output=drive(2,82.391f);
    CHECK(output>=1477 && output<=1478 && Diag_detail_f[3]==0);
    drive(3,-500);output=drive(1,-82.391f);
    CHECK(output>=1412 && output<=1413 && Diag_detail_f[3]==0);
    output=drive(1,-182.391f);CHECK(Diag_detail_f[3]<-19.9f); /* same-mode D retained */
    output=drive(2,200);CHECK(Diag_detail_f[3]==0); /* opposite direction */
    line.k=1;line.b=650;before=timer3.CCR1;
    begin();Midline_PD_Calculate(plane,&pid,&line,144.5f,0,2,0);
    CHECK(Servo_PD_valid && timer3.CCR1==before); /* candidate must not write hardware */
    /* Replay observed left-turn errors using real PD + real guard, not mocked PD. */
    deferred=1;drive(0,0);writes=servo_writes;
    output=drive(2,284.289f);CHECK(output==1632 && Diag_detail_f[3]==0);
    CHECK(TurnGuard_Apply(&guard,2,1,0,output,clock_us,&held)==1632 && !held);
    output=drive(2,-49.897f);CHECK(output==1445);
    CHECK(TurnGuard_Apply(&guard,2,1,0,output,clock_us,&held)==1632 && held);
    CHECK(guard.pwm==1632); /* 14:45:03 used to overwrite the anchor with 1445 */
    output=drive(0,50);
    CHECK(TurnGuard_Apply(&guard,0,1,1,output,clock_us,&held)==1632 && held);
    output=drive(0,50);
    CHECK(TurnGuard_Apply(&guard,0,1,1,output,clock_us,&held)==1462 && !held);
    memset(&guard,0,sizeof guard);
    output=drive(4,500);CHECK(output==1720 && Diag_detail_f[3]==0);
    CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1720 && !held);
    for(k=0;k<3;k++){
        output=drive(4,500);CHECK(output==1645); /* entry compensation not repeated */
        CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1720 && held);
    }
    output=drive(4,500);
    CHECK(TurnGuard_Apply(&guard,4,1,0,output,clock_us,&held)==1645 && !held);
    drive(0,0);output=drive(2,83.583f);CHECK(output==1511); /* small entry bounded by |P| */
    drive(0,0);output=drive(2,-50);CHECK(output==1445); /* contradictory error never boosted */
    drive(0,0);output=drive(3,-500);CHECK(output==1170);
    CHECK(servo_writes==writes);deferred=0; /* whole candidate replay leaves hardware untouched */
    /* Encoder forward, reverse and modulo wrap: no huge unsigned speed. */
    for(k=0;k<5;k++){encoder_counter+=28;Get_Encoder();}
    CHECK(fabs(Speed_now-10.181818f)<.001f);
    for(k=0;k<5;k++){encoder_counter-=1;Get_Encoder();}
    CHECK(Encoder_cnt_temp==65535 && Speed_now<0 && Speed_now> -1);
    for(k=0;k<5;k++){encoder_counter=(uint16_t)(encoder_counter+30000);Get_Encoder();}
    CHECK(Encoder_cnt_temp==30000);
    printf("PASS %d checks (production C, mocked hardware)\n",checks);
    return 0;
}
int main(void){return run();}
'''
program = prefix + '\n'.join(functions) + tests
path = OUT / 'control_tests.c'
path.write_text(program, encoding='utf-8')
vcvars = Path('D:/VS/VC/Auxiliary/Build/vcvars64.bat')
# cmd does not accept forward slashes in a command path, and vcvars is a stub that
# must be invoked with `call`; go through a wrapper batch file for both reasons.
wrapper = OUT / 'env.bat'
wrapper.write_bytes(('@echo off\r\ncall "' + str(vcvars) + '" >nul 2>&1\r\nset\r\n').encode('ascii'))
result = subprocess.run(['cmd','/d','/s','/c', str(wrapper)],capture_output=True)
if result.returncode: raise RuntimeError(result.stderr.decode(errors='replace'))
env = dict(os.environ)
for line in result.stdout.decode(errors='replace').splitlines():
    key, sep, value = line.partition('=')
    if sep and key: env[key]=value
compiler = Path('D:/VS/VC/Tools/MSVC/14.44.35207/bin/Hostx64/x64/cl.exe')
exe = OUT / 'control_tests.exe'
cmd = [str(compiler),'/nologo','/std:c11','/utf-8','/Od',str(path),'/Fe:'+str(exe),'/Fo:'+str(OUT/'control_tests.obj')]
built = subprocess.run(cmd,env=env,capture_output=True)
(OUT/'compile.log').write_bytes(built.stdout+built.stderr)
if built.returncode:
    print((built.stdout+built.stderr).decode(errors='replace'));sys.exit(built.returncode)
ran = subprocess.run([str(exe)],capture_output=True)
(OUT/'run.log').write_bytes(ran.stdout+ran.stderr)
summary = dict(compiler='MSVC 14.44 /Od',build_exit=built.returncode,run_exit=ran.returncode,
               output=ran.stdout.decode(errors='replace'),note='Production function bodies; hardware mocked, not ARM execution.')
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(summary['output'])
if ran.returncode:
    sys.exit(ran.returncode)
