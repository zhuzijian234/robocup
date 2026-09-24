
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "m10p.h"
#define PI 3.14159265358979323846f
#define arm_cos_f32 cosf
#define arm_sin_f32 sinf
typedef struct {float angle,distance;} _LEIDA_DATA;
static uint32_t clock_us;
uint32_t Diag_TimeUs(void){return clock_us;}
static unsigned checks;
#define CHECK(x) do{++checks;if(!(x)){printf("FAIL %d: %s\n",__LINE__,#x);return 1;}}while(0)
uint32_t __get_PRIMASK(void){return 0;}
void __disable_irq(void){}
void __set_PRIMASK(uint32_t x){(void)x;}
void __DMB(void){}

#include "m10p.c"

uint32_t LidarRx_epoch;
static uint16_t bins[720];
uint32_t M10P_control_seq,M10P_control_front_us,M10P_control_epoch;
uint16_t M10P_front_bins,M10P_left_bins,M10P_right_bins;
float M10P_clearance_mm,M10P_speed_scale;
uint8_t M10P_perception_ok;
#define RADAR_TIMEOUT_TICKS 50u
volatile uint16_t Radar_age_ticks;
volatile uint8_t Radar_started,Radar_stop_latched;
volatile uint32_t Radar_timeout_count;
typedef struct {float kp,ki,kd,err,err_l,err_sum;} pid_type;
volatile float Diag_motor_integral,Diag_motor_prelimit;
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity)
{
    uint16_t i, n = 0;
    M10P_front_bins = M10P_left_bins = M10P_right_bins = 0;
    M10P_clearance_mm = (float)M10P_MAX_MM;
    M10P_perception_ok = 0; M10P_speed_scale = 0;
    M10P_control_seq = scan->seq;
    M10P_control_front_us = scan->front_us;
    M10P_control_epoch = scan->epoch;
    M10P_Index(scan, bins);
    for (i = 0; i < M10P_BINS; ++i) if (bins[i] != 0xffffu) {
        const M10P_Point *p = &scan->points[bins[i]];
        float a = M10P_AlgorithmAngle(p->angle_cdeg) / 100.0f;
        if (n >= capacity) return 0;
        out[n].angle = a; out[n++].distance = p->range_mm;
        if (i >= 140 && i <= 220) M10P_front_bins++;
        if (i >= 220 && i <= 340) M10P_left_bins++;
        if (i >= 20 && i <= 140) M10P_right_bins++;
    }
    /* Independent near-obstacle path: use ALL raw returns, not wall filtering.
     * Include close (<100 mm) nonzero returns conservatively as obstacles. */
    for (i = 0; i < scan->count; ++i) {
        const M10P_Point *p = &scan->points[i];
        float angle, x, y;
        if (!p->range_mm || p->range_mm > M10P_MAX_MM) continue;
        angle = M10P_AlgorithmAngle(p->angle_cdeg) * (PI / 18000.0f);
        x = p->range_mm * arm_cos_f32(angle);
        y = p->range_mm * arm_sin_f32(angle);
        if (y > 0 && fabsf(x) < M10P_CORRIDOR_HALF_MM && y < M10P_clearance_mm)
            M10P_clearance_mm = y;
    }
    M10P_perception_ok = scan->front_seen && M10P_front_bins >= 40 &&
        M10P_left_bins >= 16 && M10P_right_bins >= 16 &&
        scan->epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs() - scan->front_us) <= M10P_MAX_AGE_US &&
        M10P_clearance_mm > M10P_STOP_Y_MM;
    if (M10P_perception_ok) {
        M10P_speed_scale = (M10P_clearance_mm - M10P_STOP_Y_MM) / (M10P_SLOW_Y_MM - M10P_STOP_Y_MM);
        if (M10P_speed_scale > 1) M10P_speed_scale = 1;
        if (M10P_speed_scale < 0.25f) M10P_speed_scale = 0.25f;
    }
    return n;
}static volatile uint32_t observation_us, observation_epoch, observation_seq, completed_us;
static volatile float observation_scale;
static volatile uint8_t observation_valid, warmup;
volatile float Radar_effective_target;
uint8_t Radar_Permitted(void)
{
    return Radar_started && !Radar_stop_latched && observation_valid &&
        observation_epoch == LidarRx_epoch &&
        (uint32_t)(Diag_TimeUs()-observation_us) <= M10P_MAX_AGE_US;
}
void Radar_Invalidate(void)
{
    uint32_t p = __get_PRIMASK(); __disable_irq();
    observation_valid = 0; warmup = 0;
    __set_PRIMASK(p);
}
void Radar_Observe(uint32_t seq, uint32_t front_us, uint32_t epoch, float scale)
{
    uint32_t p = __get_PRIMASK(), now = Diag_TimeUs();
    __disable_irq();
    if (Radar_stop_latched || epoch != LidarRx_epoch ||
        (uint32_t)(now-front_us) > M10P_MAX_AGE_US || seq == observation_seq) {
        observation_valid = 0; warmup = 0;
    } else {
        if (seq != observation_seq + 1u || now - completed_us > M10P_MAX_AGE_US) warmup = 0;
        observation_seq = seq; observation_us = front_us; observation_epoch = epoch;
        completed_us = now;
        observation_scale = scale < 0 ? 0 : scale > 1 ? 1 : scale;
        if (warmup < 3) warmup++;
        observation_valid = warmup >= 3;
        if (observation_valid) { Radar_age_ticks = 0; Radar_started = 1; }
    }
    __set_PRIMASK(p);
}
void Radar_GuardTick(void)
{
    if (Radar_started && !Radar_stop_latched) {
        if (Radar_age_ticks < RADAR_TIMEOUT_TICKS) Radar_age_ticks++;
        if (Radar_age_ticks >= RADAR_TIMEOUT_TICKS) {
            Radar_stop_latched = 1; Radar_timeout_count++;
        }
    }
    if (observation_epoch != LidarRx_epoch ||
        (uint32_t)(Diag_TimeUs()-observation_us) > M10P_MAX_AGE_US) {
        observation_valid = 0; warmup = 0;
    }
}

float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid)
{
    float moto_pwm = 0;


    /* 计算当前偏差 */
    speed_pid->err = speed_mubiao - speed_now;

    /* 累加积分 */
    speed_pid->err_sum += speed_pid->err;

    /* 积分抗饱和 */
    if (speed_pid->err_sum >= 200)
        speed_pid->err_sum = 200;
    if (speed_pid->err_sum <= -200)
        speed_pid->err_sum = -200;

    /* 位置式PI: pwm = kp*err + ki*积分 + kd*微分 */
    moto_pwm = speed_pid->kp * speed_pid->err + speed_pid->ki * speed_pid->err_sum + speed_pid->kd * (speed_pid->err - speed_pid->err_l);

    /* 记录上一次偏差 */
    speed_pid->err_l = speed_pid->err;

    Diag_motor_integral = speed_pid->err_sum;
    Diag_motor_prelimit = moto_pwm;
    /* 输出限幅 [0, 100] */
    if (moto_pwm >= 100)
        moto_pwm = 100;
    if (moto_pwm <= 0)
        moto_pwm = 0;

    return moto_pwm;
}
void Speed_PID_Reset(pid_type *pid)
{
    pid->err_sum = pid->err = pid->err_l = 0;
    Diag_motor_integral = Diag_motor_prelimit = 0;
}
static M10P_Scan s;static _LEIDA_DATA out[720];
int main(void){unsigned i;pid_type pid={8.5f,.505f,0};float pwm;
 s.seq=1;s.epoch=0;s.front_seen=1;s.front_us=50000;clock_us=100000;
 for(i=0;i<720;i++){s.points[i].angle_cdeg=(uint16_t)(i*50);s.points[i].range_mm=1000;}s.count=720;
 CHECK(M10P_Build(&s,out,720)==720);CHECK(M10P_perception_ok);CHECK(M10P_front_bins==81);
 CHECK(out[0].angle==0 && out[180].angle==90 && out[360].angle==180);
 CHECK(!M10P_Build(&s,out,719));CHECK(!M10P_perception_ok);
 s.points[0].range_mm=200;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);CHECK(M10P_clearance_mm<201);
 s.points[0].range_mm=80;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);CHECK(M10P_clearance_mm<81);
 s.points[0].range_mm=1000;clock_us=s.front_us+M10P_MAX_AGE_US+1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 clock_us=100000;s.count=0;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 s.count=720;s.epoch=1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);s.epoch=0;
 Radar_Observe(1,50000,0,1);CHECK(!observation_valid && !Radar_started);
 clock_us+=83000;Radar_Observe(2,clock_us-50000,0,.6f);CHECK(!observation_valid);
 clock_us+=83000;Radar_Observe(3,clock_us-50000,0,.6f);CHECK(observation_valid && Radar_started);CHECK(observation_scale==.6f);
 Radar_Observe(3,clock_us-50000,0,1);CHECK(!observation_valid);CHECK(warmup==0);
 for(i=4;i<=6;i++){clock_us+=83000;Radar_Observe(i,clock_us-50000,0,1);}CHECK(observation_valid);
 clock_us=observation_us+M10P_MAX_AGE_US+1;Radar_GuardTick();CHECK(!observation_valid);
 for(i=0;i<50;i++)Radar_GuardTick();CHECK(Radar_stop_latched && Radar_timeout_count==1);
 Radar_Observe(7,clock_us,0,1);CHECK(!observation_valid);CHECK(Radar_stop_latched);
 Radar_stop_latched=Radar_started=0;warmup=0;observation_seq=0;clock_us=100000;
 for(i=1;i<=3;i++){clock_us+=83000;Radar_Observe(i,clock_us-50000,0,1);}CHECK(observation_valid);
 LidarRx_epoch++;Radar_GuardTick();CHECK(!observation_valid);CHECK(warmup==0);
 Radar_Invalidate();CHECK(!observation_valid);
 for(i=0;i<20;i++)pwm=PID_realize(0,10,&pid);CHECK(pwm==100);CHECK(pid.err_sum==200);
 Speed_PID_Reset(&pid);CHECK(pid.err_sum==0 && pid.err_l==0);CHECK(Diag_motor_integral==0);
 CHECK(PID_realize(0,0,&pid)==0);
 printf("PASS %u adapter/guard/PI checks\n",checks);return 0;
}
