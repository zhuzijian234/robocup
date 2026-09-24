"""Run production turn-selection and forward-filter regressions with mocked hardware."""
from pathlib import Path
import os
import runpy
import subprocess

HERE = Path(__file__).resolve().parent
# Reuse the existing production-C harness and compiler setup, without overwriting
# its tracked output artifacts.
os.environ['ROBOCUP_CONTROL_TEST_OUT'] = str(HERE.parents[2] / 'tmp/turn_regression')
try:
    base = runpy.run_path(str(HERE / 'check_control_fixes.py'))
except SystemExit as exc:
    raise RuntimeError('Base harness must return its namespace') from exc

main = base['source']('USER/main.c')
start = main.index('            pid_select_last_last = pid_select_last;')
end = main.index('            } else {\n                CENTER_cnt =', start)
selection = main[start:end] + '\n}\n'
extra = r'''
#define BLE_MODE_HOLD 10
static uint16_t pid_select, pid_select_last, pid_select_last_last;
static uint16_t state_left_cnt, state_right_cnt, state_left_cnt_2, state_right_cnt_2;
static uint16_t RIGHT_duandian, LEFT_duandian, duandian_DIStance=550, duandian_distance=450;
static uint16_t LEFT_cnt=20, RIGHT_cnt=20, Forward_cnt=20, ref_start, ref_end, telemetry_mode;
static uint8_t forward_fit_ok, danbian_flag;
static float servo_midpwm=144.5f;
static pid_type Servo_pd;
static Midline_type Midline, Midline_forward, Midline_forward_2, Midline_forward_3;
static _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[400], LEIDA_DATA_RIGHT_Plane[400], LEIDA_DATA_Forward[400];
static _LEIDA_DATA LEIDA_DATA_LEFT[800], LEIDA_DATA_RIGHT[800];
static int calls, selected;
static void convert(_LEIDA_DATA_plane *p,_LEIDA_DATA *a,int n){}
static uint16_t keep(_LEIDA_DATA_plane *p,uint16_t n){return n;}
static uint16_t command(_LEIDA_DATA_plane *p,pid_type *pid,Midline_type *l,float mid,uint16_t s,uint16_t e,uint16_t mode){
    calls++;selected=mode;Servo_PD_valid=1;return 1445;
}
#define LEIDA_DATA_HANDLE2 convert
#define LEIDA_DATA_HANDLE10 keep
#define Midline_PD command
static void step(void){
    telemetry_mode=11;Servo_PD_valid=0;
'''
extra += selection + '\n}\n#undef Midline_PD\n#undef LEIDA_DATA_HANDLE10\n'
extra += r'''
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
int main(void){
    int direction,i;uint16_t n;
    _LEIDA_DATA scan[50];_LEIDA_DATA_plane wall[400];
    for(direction=0;direction<2;direction++){
        RIGHT_duandian=direction?0:300;LEFT_duandian=direction?300:0;
        pid_select=pid_select_last=direction?4:3;
        forward_fit_ok=0;calls=0;
        step();CHECK(telemetry_mode==10 && calls==0);
        for(i=0;i<4;i++){step();CHECK(calls==i+1 && selected==(direction?2:1));}
        /* Opposite turn must act immediately, not hold the previous direction. */
        pid_select=pid_select_last=direction?3:4;calls=0;
        step();CHECK(calls==1 && selected==(direction?2:1));
        /* A newly observed bend cancels a pending forced turn from the old bend. */
        state_left_cnt=state_right_cnt=2;forward_fit_ok=1;Midline_forward.k=.2f;
        step();CHECK(!state_left_cnt && !state_right_cnt);
        CHECK(selected==(direction?4:3));
    }
    memset(scan,0,sizeof scan);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,0)==0);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,1)==0);
    /* One accepted scan measurement creates a sparse window: keep its points. */
    scan[0].angle=80;scan[0].distance=600;
    n=LEIDA_DATA_HANDLE5(wall,scan,2);CHECK(n>0 && n<6);
    /* A wall perpendicular to the vehicle has nearly constant y. */
    for(i=0;i<41;i++){
        scan[i].angle=70.0f+i;
        scan[i].distance=700.0f/sinf(scan[i].angle*PI/180);
    }
    n=LEIDA_DATA_HANDLE5(wall,scan,42);CHECK(n>=30);
    CHECK(Midline_fit(wall,0,n,&Midline_forward));
    CHECK(fabs(Midline_forward.k)<.01f);
    printf("PASS %d turn/filter regression checks (production C, mocked hardware)\n",checks);
    return 0;
}
'''
radar = base['radar']
program = base['prefix'] + '\n'.join(base['functions'])
program += '\n' + base['function'](radar, 'reverse')
program += '\n' + base['function'](radar, 'LEIDA_DATA_HANDLE5') + extra
out = base['OUT']
src = out / 'turn_tests.c'
src.write_text(program, encoding='utf-8')
exe = out / 'turn_tests.exe'
built = subprocess.run([str(base['compiler']), '/nologo', '/std:c11', '/utf-8', '/Od',
                        str(src), '/Fe:' + str(exe), '/Fo:' + str(out / 'turn_tests.obj')],
                       env=base['env'], capture_output=True)
if built.returncode:
    print((built.stdout+built.stderr).decode(errors='replace'))
    raise SystemExit(built.returncode)
raise SystemExit(subprocess.run([str(exe)]).returncode)
