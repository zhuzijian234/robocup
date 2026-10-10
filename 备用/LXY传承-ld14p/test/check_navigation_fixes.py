"""生产C导航回归：缓存缺测/过期、锥桶背景、参考质量、换向和速度撤驱动。"""
from pathlib import Path
import os, runpy, subprocess
HERE=Path(__file__).resolve().parent
os.environ['ROBOCUP_CONTROL_TEST_OUT']=str(HERE.parents[2]/'tmp/ld14p_navigation/native')
base=runpy.run_path(str(HERE/'check_control_fixes.py'))
radar=base['radar']; steering=base['steering']
h=base['source']('HARDWARE/LEIDA_DATA/LEIDA_DATA.h')
header=h[h.index('#define NAV_CACHE_US'):h.index('/* ============ 雷达数据处理流水线')]
nav=radar[radar.index('/* ===== 带年龄的角度缓存'):]
timer=base['source']('HARDWARE/LEIDA_TIMER/timer.c')
timer_globals=timer[timer.index('volatile uint8_t Radar_drive_enabled'):timer.index('/**',timer.index('void Radar_GuardTick'))]
extra=r'''
#define RADAR_TIMEOUT_TICKS 50
static uint32_t __get_PRIMASK(void){return 0;}
static void __disable_irq(void){}
static void __set_PRIMASK(uint32_t x){}
static float Speed_mubiao=8;
static pid_type Speed_pid;
static float Diag_motor_integral,Diag_motor_prelimit;
static float moto_pwm;
static uint16_t daoche_flag;
#define TIM5 5
#define TIM2 2
#define TIM_IT_Update 1
#define SET 1
static int TIM_GetITStatus(int t,int f){return SET;}
static void TIM_ClearITPendingBit(int t,int f){}
static float motor_output;
static void Moto_Speed(float v){motor_output=v;}
static void TIM_SetCompare1(int t,uint16_t v){motor_output=v;}
static void Diag_MotorTick(uint16_t raw,uint8_t f,uint8_t p){}
'''
program=base['prefix']+'\n'+'\n'.join(base['functions'])+'\n'+header+nav+extra
program+='\n'+timer_globals
program+='\n'+base['function'](steering,'Speed_PID_Reset')+'\n'+base['function'](steering,'PID_realize')
program+='\n'+base['function'](timer,'TIM5_IRQHandler')
program+=r'''
static int checks;
#define CHECK(c) do{checks++;if(!(c)){printf("FAIL line %d: %s\n",__LINE__,#c);return 1;}}while(0)
static _LEIDA_DATA scan[800], snapshot[800];
static _LEIDA_DATA_plane wall[10];
static void full_scan(uint32_t now,float distance){
    int i;LEIDA_speed_dps=2160;
    for(i=0;i<360;i++){scan[i].angle=(float)(359-i);scan[i].distance=distance;}
    LEIDA_ScanUpdate(scan,360,now);
    LEIDA_InspectNavigation(now);
}
int main(void){
    int i;uint16_t count,pwm;float x;TurnGuard guard={0};uint8_t held;
    pid_type controller={0};Midline_type line={1,0};
    controller.kp=.035f;controller.kp_3=.0395f;
    LEIDA_ScanReset();LEIDA_InspectNavigation(1000000);
    CHECK(Navigation.action==NAV_UNKNOWN && !Navigation.front_bins);
    full_scan(1000000,2500);
    CHECK(LEIDA_ScanSnapshot(snapshot,1000000)==360);
    CHECK(Navigation.action==NAV_CLEAR && Navigation.front_bins==41);
    LEIDA_InspectNavigation(1200001);
    CHECK(Navigation.action==NAV_UNKNOWN && LEIDA_ScanSnapshot(snapshot,1200001)==0);
    /* 微秒时间回绕仍按实际年龄限龄。 */
    LEIDA_ScanReset();full_scan(0xfffffff0u,2500);
    CHECK(LEIDA_ScanSnapshot(snapshot,0x1000u)==360);
    CHECK(LEIDA_ScanSnapshot(snapshot,0x40000u)==0);
    /* 窄锥桶和远背景共存，不能误报空旷。 */
    LEIDA_ScanReset();full_scan(2000000,2500);
    for(i=0;i<360;i++)if(scan[i].angle>=89 && scan[i].angle<=92)scan[i].distance=700;
    LEIDA_ScanUpdate(scan,360,2110000);LEIDA_InspectNavigation(2110000);
    CHECK(Navigation.action>=NAV_OBSTACLE);
    CHECK(Navigation.obstacle_y>690 && Navigation.obstacle_y<=700);
    CHECK(Navigation.obstacle_width<100);
    /* 100～200mm近障碍仍响应，且单点足够停车。 */
    for(i=0;i<360;i++)if(scan[i].angle==90)scan[i].distance=150;
    LEIDA_ScanUpdate(scan,360,2220000);LEIDA_InspectNavigation(2220000);
    CHECK(Navigation.action==NAV_OBSTACLE && Navigation.obstacle_y==150);
    /* 零回波会使原有“空旷”失效，缺测不能沿用旧远点。 */
    for(i=0;i<360;i++)if(scan[i].angle>=70 && scan[i].angle<=110)scan[i].distance=0;
    LEIDA_ScanUpdate(scan,360,2330000);LEIDA_InspectNavigation(2330000);
    CHECK(Navigation.action==NAV_UNKNOWN && Navigation.front_bins==0);
    LEIDA_speed_dps=633;LEIDA_ScanUpdate(scan,360,2440000);
    CHECK(!LEIDA_ScanSnapshot(snapshot,2440000));
    /* 10月4日坏参考：只有17cm，不能冒充1.2m。 */
    for(i=0;i<8;i++){wall[i]._x=-350;wall[i]._y=172+i;}
    CHECK(!wall_reference(wall,0,8,1,1200,&x));
    clock_us=1000000;
    Midline_PD_Calculate(wall,&controller,&line,144.5f,0,8,1);
    CHECK(!Servo_PD_valid && Servo_reject_reason==3);
    for(i=0;i<8;i++){wall[i]._x=-350+.1f*i*100;wall[i]._y=500+i*100;}
    CHECK(wall_reference(wall,0,8,1,1200,&x) && fabs(x+280)<.01f);
    CHECK(!wall_reference(wall,0,8,1,1600,&x));
    wall[3]._x=-900;CHECK(!wall_reference(wall,0,8,1,1200,&x));
    /* 反向一次不执行，重复缓存也不能当成第二次。 */
    guard.evidence_us=100000;
    CHECK(TurnGuard_Apply(&guard,3,1,0,1170,100000,&held)==1170);
    guard.evidence_us=200000;
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,200000,&held)==1170 && held);
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,310000,&held)==1170 && held);
    guard.evidence_us=400000;
    CHECK(TurnGuard_Apply(&guard,2,1,0,1696,400000,&held)==1696 && !held);
    /* 实际TIM5撤驱动，不只是修改遥测目标；锁停无法被新感知解除。 */
    Speed_pid.kp=8.5f;Speed_pid.ki=.505f;
    Radar_ControlCompleted();Radar_SetSpeedLimit(8,0);
    TIM5_IRQHandler();CHECK(Speed_effective==8 && motor_output>0);
    Radar_drive_enabled=0;TIM5_IRQHandler();CHECK(motor_output==0);
    Radar_drive_enabled=1;
    Radar_SetSpeedLimit(0,2);Speed_pid.err_sum=150;
    TIM5_IRQHandler();CHECK(motor_output==0 && Speed_pid.err_sum==0);
    Radar_SetSpeedLimit(8,0);Radar_age_ticks=19;
    TIM5_IRQHandler();CHECK(Speed_effective==0 && motor_output==0);
    Radar_age_ticks=49;TIM5_IRQHandler();CHECK(Radar_stop_latched);
    Radar_ControlCompleted();CHECK(Radar_stop_latched);
    printf("PASS %d navigation/speed checks (production C)\n",checks);
    return 0;
}
'''
out=base['OUT'];src=out/'navigation_tests.c';src.write_text(program,encoding='utf-8')
exe=out/'navigation_tests.exe'
built=subprocess.run([str(base['compiler']),'/nologo','/std:c11','/utf-8','/Od',str(src),'/Fe:'+str(exe),'/Fo:'+str(out/'navigation_tests.obj')],env=base['env'],capture_output=True)
(out/'navigation_compile.log').write_bytes(built.stdout+built.stderr)
if built.returncode:
 print((built.stdout+built.stderr).decode(errors='replace'));raise SystemExit(built.returncode)
ran=subprocess.run([str(exe)],capture_output=True)
(out/'navigation_run.log').write_bytes(ran.stdout+ran.stderr)
print(ran.stdout.decode(errors='replace'));raise SystemExit(ran.returncode)
