"""Run production turn-selection and forward-filter regressions with mocked hardware."""
from pathlib import Path
import os
import runpy
import subprocess

HERE = Path(__file__).resolve().parent
# Reuse the existing production-C harness and compiler setup, without overwriting
# its tracked output artifacts.
os.environ.setdefault('ROBOCUP_CONTROL_TEST_OUT', str(HERE.parents[2] / 'tmp/turn_guard_v1_tests'))
try:
    base = runpy.run_path(str(HERE / 'check_control_fixes.py'))
except SystemExit as exc:
    raise RuntimeError('Base harness must return its namespace') from exc

main = base['source']('USER/main.c')
start = main.index('            candidate_pwm = (uint16_t)TIM3->CCR1;')
end = main.index('            /* 本帧雷达处理完:', start)
selection = main[start:end]
extra = r"""
#define BLE_MODE_HOLD 10
#define BLE_MODE_INVALID 11
static uint16_t pid_select, candidate_pwm;
static uint8_t turn_held, straight_evidence;
static TurnGuard turn_guard;
static uint16_t RIGHT_duandian, LEFT_duandian, duandian_DIStance=550, duandian_distance=600;
static uint16_t LEFT_cnt=20, RIGHT_cnt=20, Forward_cnt=20, CENTER_cnt, valid_couter=100;
static uint16_t ref_start, ref_end, telemetry_mode;
static uint8_t forward_fit_ok, danbian_flag;
static float servo_midpwm=144.5f;
static pid_type Servo_pd;
static Midline_type Midline, Midline_forward, Midline_forward_2, Midline_forward_3;
static _LEIDA_DATA_plane LEIDA_DATA_LEFT_Plane[400], LEIDA_DATA_RIGHT_Plane[400];
static _LEIDA_DATA_plane LEIDA_DATA_Forward[400], LEIDA_DATA_CENTER[400];
static _LEIDA_DATA LEIDA_DATA_LEFT[800], LEIDA_DATA_RIGHT[800], LEIDA_DATA2[800];
static int calls, selected, mock_valid=1, mock_center_count=12;
static uint16_t mock_pwm;
static float mock_error;
static void convert(_LEIDA_DATA_plane *p,_LEIDA_DATA *a,int n){}
static uint16_t keep(_LEIDA_DATA_plane *p,uint16_t n){return n;}
static uint16_t center(_LEIDA_DATA_plane *p,_LEIDA_DATA *a,uint16_t n){return mock_center_count;}
static float vertical(_LEIDA_DATA_plane *p,uint16_t s,uint16_t e){LEIDA_vertical_valid=1;return 50-mock_error;}
static uint8_t fit(_LEIDA_DATA_plane *p,int s,int e,Midline_type *l){l->k=1;l->b=0;return 1;}
static void Diag_Field(int field,float value,int valid){}
static uint16_t command(_LEIDA_DATA_plane *p,pid_type *pid,Midline_type *l,float mid,uint16_t s,uint16_t e,uint16_t mode){
    calls++;selected=mode;Servo_PD_valid=mock_valid;pid->err=mock_error;
    if(mock_valid)Diag_detail_u[4]|=1;
    return mock_valid?mock_pwm:timer3.CCR1;
}
#define LEIDA_DATA_HANDLE2 convert
#define LEIDA_DATA_HANDLE10 keep
#define LEIDA_DATA_HANDLE4 center
#define LEIDA_DATA_HANDLE11 vertical
#define Midline_fit fit
#define Midline_PD_Calculate command
static void step(void){
    telemetry_mode=11;Servo_PD_valid=0;
    memset(Diag_detail_u,0,sizeof Diag_detail_u);
"""
extra += selection + r"""
}
#undef Midline_PD_Calculate
#undef Midline_fit
#undef LEIDA_DATA_HANDLE10
#undef LEIDA_DATA_HANDLE4
#undef LEIDA_DATA_HANDLE11
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
static int tick(uint16_t mode,int valid,uint16_t pwm,float error,uint32_t elapsed){
    unsigned before=servo_writes;
    clock_us+=elapsed;mock_valid=valid;mock_pwm=pwm;mock_error=error;
    RIGHT_duandian=(mode==1 || mode==3 || mode==8)?300:0;
    LEFT_duandian=(mode==2 || mode==4 || mode==9)?300:0;
    forward_fit_ok=mode==3 || mode==4 || mode==8 || mode==9;
    Midline_forward.k=(mode==3 || mode==4)?.2f:.5f;
    danbian_flag=1;
    step();
    return servo_writes-before==(valid?1u:0u);
}
int main(void){
    int direction,i;uint16_t n,large,small,strong,weak;
    _LEIDA_DATA scan[50];_LEIDA_DATA_plane wall[400];
    for(direction=0;direction<2;direction++){
        large=direction?4:3;small=direction?2:1;
        strong=direction?1645:1245;weak=direction?1477:1412;
        memset(&turn_guard,0,sizeof turn_guard);
        CHECK(tick(large,1,strong,500,115000));
        CHECK(!turn_held && timer3.CCR1==strong);
        CHECK(tick(small,1,weak,82,115000));
        CHECK(turn_held && timer3.CCR1==strong && telemetry_mode==10 && !(Diag_detail_u[4]&1));
        CHECK(tick(0,1,1462,50,115000));
        CHECK(turn_held && turn_guard.straight_frames==1 && timer3.CCR1==strong);
        CHECK(tick(0,1,1462,50,115000));
        CHECK(!turn_held && !turn_guard.active && timer3.CCR1==1462);

        /* Invalid geometry interrupts exit confirmation and never writes/renews. */
        CHECK(tick(large,1,strong,500,50000));
        CHECK(tick(0,1,1462,50,50000));CHECK(turn_guard.straight_frames==1);
        CHECK(tick(0,0,1462,50,50000));
        CHECK(turn_guard.active && !turn_guard.straight_frames && !Servo_PD_valid && telemetry_mode==11);
        CHECK(tick(0,1,1462,50,50000));CHECK(turn_held);
        CHECK(tick(0,1,1462,50,50000));CHECK(!turn_guard.active);

        /* Downgraded measurements cannot rearm the 350ms deadline. */
        CHECK(tick(large,1,strong,500,115000));
        for(i=0;i<3;i++){CHECK(tick(small,1,weak,82,115000));CHECK(turn_held);}
        CHECK(tick(small,1,weak,82,115000));CHECK(!turn_held && timer3.CCR1==weak);
        CHECK(tick(large,1,strong,500,115000));
        CHECK(tick(0,0,weak,0,350000));CHECK(!turn_guard.active && !Servo_PD_valid);

        /* Opposite bend and stronger same-direction commands act immediately. */
        CHECK(tick(large,1,strong,500,115000));
        CHECK(tick(direction?1:2,1,direction?1250:1640,300,115000));
        CHECK(!turn_held && selected==(direction?1:2));
        CHECK(tick(large,1,strong,500,115000));
        CHECK(tick(small,1,direction?1720:1170,450,115000));
        CHECK(!turn_held && timer3.CCR1==(direction?1720:1170));

        /* Large center offset is not evidence of a completed turn. */
        CHECK(tick(large,1,strong,500,115000));
        CHECK(tick(0,1,1515,200,115000));CHECK(turn_held && !turn_guard.straight_frames);
        CHECK(tick(0,1,1515,200,115000));CHECK(turn_held && !turn_guard.straight_frames);
        CHECK(tick(0,1,1515,200,120000));CHECK(!turn_held && !turn_guard.active);
        /* Too few center points cannot confirm straight exit either. */
        CHECK(tick(large,1,strong,500,115000));mock_center_count=4;
        CHECK(tick(0,1,1462,50,115000));CHECK(turn_held && !turn_guard.straight_frames);
        mock_center_count=12;
    }
    memset(&turn_guard,0,sizeof turn_guard);clock_us=0xffff0000u;
    CHECK(tick(4,1,1645,500,0));
    CHECK(tick(2,1,1477,82,115000));CHECK(turn_held); /* microsecond wrap */
    CHECK(tick(2,1,1477,82,235000));CHECK(!turn_held); /* exact timeout */

    memset(scan,0,sizeof scan);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,0)==0);
    CHECK(LEIDA_DATA_HANDLE5(wall,scan,1)==0);
    scan[0].angle=80;scan[0].distance=600;
    n=LEIDA_DATA_HANDLE5(wall,scan,2);CHECK(n>0 && n<6);
    for(i=0;i<41;i++){
        scan[i].angle=70.0f+i;
        scan[i].distance=700.0f/sinf(scan[i].angle*PI/180);
    }
    n=LEIDA_DATA_HANDLE5(wall,scan,42);CHECK(n>=30);
    CHECK(Midline_fit(wall,0,n,&Midline_forward));
    CHECK(fabs(Midline_forward.k)<.01f);
    printf("PASS %d turn/filter regression checks (production C, mocked geometry/hardware)\n",checks);
    return 0;
}
"""
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
