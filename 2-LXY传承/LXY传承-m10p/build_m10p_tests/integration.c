
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
static uint8_t rx[512];
static uint32_t seen_epoch;
typedef struct{uint32_t start_us,end_us,epoch;} LidarRxStamp;
void LidarRx_Service(void){}
uint16_t LidarRx_Read(uint8_t *p,uint16_t n,LidarRxStamp *s){(void)p;(void)n;(void)s;return 0;}
uint32_t M10P_control_seq,M10P_control_front_us,M10P_control_epoch;
uint16_t M10P_front_bins,M10P_left_bins,M10P_right_bins;
uint16_t M10P_front_gap_bins;
uint32_t M10P_build_age_us;
uint8_t M10P_front_seen,M10P_build_epoch_ok;
float M10P_clearance_mm;
uint8_t M10P_perception_ok;
/* 20261008 感知分级: 原因位枚举与分级结果。宏取自 m10p_vehicle.h,
 * 主机侧不能直接包含它(会拉进 LEIDA_DATA.h/arm_math.h), 故按同值重复定义。 */
#define M10P_WHY_FRONT_SEEN 1u
#define M10P_WHY_FRONT_BINS 2u
#define M10P_WHY_FRONT_GAP  4u
#define M10P_WHY_LEFT_BINS  8u
#define M10P_WHY_RIGHT_BINS 16u
#define M10P_WHY_EPOCH      32u
#define M10P_WHY_AGE        64u
#define M10P_WHY_CAPACITY   128u
uint8_t M10P_front_ok,M10P_left_ok,M10P_right_ok,M10P_perception_why,M10P_steer_source;
typedef struct {float kp,ki,kd,err,err_l,err_sum;} pid_type;
volatile float Diag_motor_integral,Diag_motor_prelimit;
uint16_t M10P_Build(const M10P_Scan *scan, _LEIDA_DATA *out, uint16_t capacity)
{
    uint16_t i, n = 0, missing = 0, max_missing = 0;
    M10P_front_bins = M10P_left_bins = M10P_right_bins = 0;
    M10P_clearance_mm = (float)M10P_MAX_MM; /* 先当"啥也没看见", 下面扫到更近的再改 */
    M10P_perception_ok = 0;
    M10P_front_gap_bins = 0;
    /* 提前 return(装不下)时留下确定的原因位, 不沿用上一帧的结果。 */
    M10P_front_ok = M10P_left_ok = M10P_right_ok = 0;
    M10P_perception_why = M10P_WHY_CAPACITY;
    M10P_front_seen = scan->front_seen;
    M10P_build_epoch_ok = scan->epoch == LidarRx_epoch;
    M10P_build_age_us = (uint32_t)(Diag_TimeUs() - scan->front_us);
    M10P_control_seq = scan->seq;
    M10P_control_front_us = scan->front_us;
    M10P_control_epoch = scan->epoch;
    M10P_Index(scan, bins);
    /* 总点数够多不代表正前方没瞎 —— 单独量一下最长的连续空桶。 */
    for (i = 140; i <= 220; ++i) {
        if (bins[i] == 0xffffu) {
            if (++missing > max_missing) max_missing = missing;
        } else missing = 0;
    }
    M10P_front_gap_bins = max_missing;
    for (i = 0; i < M10P_BINS; ++i) if (bins[i] != 0xffffu) {
        const M10P_Point *p = &scan->points[bins[i]];
        float a = M10P_AlgorithmAngle(p->angle_cdeg) / 100.0f; /* 0.01° -> 度 */
        if (n >= capacity) return 0; /* 装不下就整帧作废, 不喂半截数据 */
        out[n].angle = a; out[n++].distance = p->range_mm;
        if (i >= 140 && i <= 220) M10P_front_bins++;
        if (i >= 220 && i <= 340) M10P_left_bins++;
        if (i >= 20 && i <= 140) M10P_right_bins++;
    }
    /* 诊断净空：包含100mm以内的非零原始回波，不作为停车条件。 */
    for (i = 0; i < scan->count; ++i) {
        const M10P_Point *p = &scan->points[i];
        uint16_t theta;
        float angle, x, y;
        if (!p->range_mm || p->range_mm > M10P_MAX_MM) continue;
        theta = M10P_AlgorithmAngle(p->angle_cdeg);
        /* 后半圈 y <= 0, 不可能落进正前方走廊, 直接跳过。
         * 两侧保留完整判断, 让浮点边界行为偏保守。 */
        if (theta > 18000u) continue;
        angle = theta * (PI / 18000.0f); /* 0.01° -> 弧度 */
        x = p->range_mm * arm_cos_f32(angle);
        y = p->range_mm * arm_sin_f32(angle);
        if (y > 0 && fabsf(x) < M10P_CORRIDOR_HALF_MM && y < M10P_clearance_mm)
            M10P_clearance_mm = y;
    }
    /* 感知分级: 三个扇区分别判定, 再合成总判据。
     * 分级的意义: 锥桶赛道经常只有单侧可见, 旧的二值判据会整帧作废并冻结舵角;
     * 现在把"能不能用哪种来源"告诉 main.c, 由它决定双侧中线 / 单侧跟线 / 降级。 */
    M10P_build_age_us = (uint32_t)(Diag_TimeUs() - scan->front_us);
    M10P_build_epoch_ok = scan->epoch == LidarRx_epoch;
    M10P_front_ok = (M10P_front_bins >= M10P_FRONT_MIN_BINS) &&
                    (max_missing <= M10P_FRONT_MAX_MISSING_BINS);
    M10P_left_ok = M10P_left_bins >= M10P_SIDE_MIN_BINS;
    M10P_right_ok = M10P_right_bins >= M10P_SIDE_MIN_BINS;
    {
        uint8_t why = 0;
        if (!scan->front_seen) why |= M10P_WHY_FRONT_SEEN;
        if (M10P_front_bins < M10P_FRONT_MIN_BINS) why |= M10P_WHY_FRONT_BINS;
        if (max_missing > M10P_FRONT_MAX_MISSING_BINS) why |= M10P_WHY_FRONT_GAP;
        if (!M10P_left_ok) why |= M10P_WHY_LEFT_BINS;
        if (!M10P_right_ok) why |= M10P_WHY_RIGHT_BINS;
        if (scan->epoch != LidarRx_epoch) why |= M10P_WHY_EPOCH;
        if (M10P_build_age_us > M10P_MAX_AGE_US) why |= M10P_WHY_AGE;
        M10P_perception_why = why;
        M10P_perception_ok = (uint8_t)(why == 0u);
    }
    return n;
}void M10P_Poll(void)
{
    unsigned budget = 2;
    LidarRxStamp stamp;
    uint16_t n;
    if (seen_epoch != LidarRx_epoch) {
        seen_epoch = LidarRx_epoch;
        M10P_Lost(seen_epoch);
    }
    LidarRx_Service();
    while (budget-- && (n = LidarRx_Read(rx, sizeof rx, &stamp)) != 0) {
        if (stamp.epoch != seen_epoch) {
            seen_epoch = stamp.epoch; M10P_Lost(seen_epoch);
        }
        if ((uint32_t)(Diag_TimeUs() - stamp.start_us) > M10P_MAX_AGE_US) {
            M10P_Lost(seen_epoch); continue;
        }
        M10P_Feed(rx, n, stamp.start_us, stamp.end_us);
    }

}volatile uint32_t Radar_invalid_inputs = 0;
volatile float Radar_effective_target;
static volatile uint32_t control_end_us;
static volatile uint8_t have_control;

/* 仅记录最近一次有效转向的来源时间，供蓝牙诊断，不影响速度环。 */
void Radar_RecordControl(uint32_t end_us)
{
    uint32_t p = __get_PRIMASK(); __disable_irq();
    control_end_us = end_us;
    have_control = 1;
    __set_PRIMASK(p);
}
uint32_t Radar_ControlAgeUs(void)
{
    return have_control ? (uint32_t)(Diag_TimeUs()-control_end_us) : 0xffffffffu;
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
#define TIM5 5
#define TIM_IT_Update 1
#define SET 1
int TIM_GetITStatus(int t,int f){(void)t;(void)f;return SET;}
void TIM_ClearITPendingBit(int t,int f){(void)t;(void)f;}
uint16_t Encoder_cnt_temp,moto_pwm,hardware_pwm;
float Speed_now,Speed_mubiao=10;
pid_type Speed_pid={8.5f,.505f,0};
void Get_Encoder(void){}
void Moto_Speed(uint16_t pwm){hardware_pwm=pwm;}
void Diag_MotorTick(uint16_t raw,uint8_t enc,uint8_t pi){(void)raw;(void)enc;(void)pi;}
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) {
        extern uint16_t Encoder_cnt_temp;
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        Get_Encoder();
        Radar_effective_target = Speed_mubiao;
        moto_pwm = PID_realize(Speed_now, Radar_effective_target, &Speed_pid);
        Moto_Speed(moto_pwm);
        Diag_MotorTick(Encoder_cnt_temp, 1, 1);
    }
}
static M10P_Scan s;static _LEIDA_DATA out[720];
int main(void){unsigned i;pid_type pid={8.5f,.505f,0};float pwm;
 s.seq=1;s.epoch=0;s.front_seen=1;s.front_us=50000;clock_us=100000;
 for(i=0;i<720;i++){s.points[i].angle_cdeg=(uint16_t)(i*50);s.points[i].range_mm=1000;}s.count=720;
 CHECK(M10P_Build(&s,out,720)==720);CHECK(M10P_perception_ok);CHECK(M10P_front_bins==81);
 CHECK(out[0].angle==0 && out[180].angle==90 && out[360].angle==180);
 /* 40 valid front bins can still hide a 20-degree central blind sector. */
 for(i=0;i<720;i++)if(i<=20 || i>=700)s.points[i].range_mm=0;
 M10P_Build(&s,out,720);CHECK(M10P_front_bins==40);CHECK(!M10P_perception_ok);
 for(i=0;i<720;i++)s.points[i].range_mm=1000;
 M10P_Build(&s,out,720);CHECK(M10P_perception_ok);
 CHECK(!M10P_Build(&s,out,719));CHECK(!M10P_perception_ok);
 s.points[0].range_mm=200;M10P_Build(&s,out,720);CHECK(M10P_perception_ok);CHECK(M10P_clearance_mm<201);
 s.points[0].range_mm=80;M10P_Build(&s,out,720);CHECK(M10P_perception_ok);CHECK(M10P_clearance_mm<81);
 s.points[0].range_mm=1000;clock_us=s.front_us+M10P_MAX_AGE_US+1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 clock_us=100000;s.count=0;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
 s.count=720;s.epoch=1;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);s.epoch=0;
 /* Actual ISR runs immediately, even before the first radar frame. */
 CHECK(Radar_ControlAgeUs()==0xffffffffu);
 TIM5_IRQHandler();CHECK(Radar_effective_target==10 && hardware_pwm>0);
 {uint16_t previous=hardware_pwm;Speed_now=8;TIM5_IRQHandler();
  CHECK(Radar_effective_target==10 && hardware_pwm<previous);}
 Speed_now=0;
 /* Invalid geometry, epoch changes, rejected scans and complete radar loss
  * must not interrupt PI or change the fixed speed target. */
 for(i=0;i<1000;i++){
  clock_us+=10000;
  if(i==1){LidarRx_epoch++;M10P_Poll();}
  if(i==2){M10P_stats.rejected++;M10P_Poll();}
  if(i==3){M10P_stats.discontinuities++;M10P_Poll();}
  s.count=0;M10P_Build(&s,out,720);CHECK(!M10P_perception_ok);
  TIM5_IRQHandler();CHECK(Radar_effective_target==10 && hardware_pwm>0);
 }
 CHECK(Speed_pid.err_sum==200);
 Radar_RecordControl(clock_us-90000);CHECK(Radar_ControlAgeUs()==90000);
 clock_us=0xfffffff0u;Radar_RecordControl(clock_us);clock_us+=100;
 CHECK(Radar_ControlAgeUs()==100);
 /* Near-wall ranges from the reported corner failure are diagnostic only. */
 s.count=720;s.front_seen=1;s.epoch=LidarRx_epoch;
 for(i=0;i<720;i++){s.points[i].angle_cdeg=(uint16_t)(i*50);s.points[i].range_mm=1000;}
 {unsigned ranges[]={351,350,200,80,76};unsigned j;
  for(j=0;j<5;j++){
   s.points[0].range_mm=ranges[j];s.front_us=clock_us;
   M10P_Build(&s,out,720);CHECK(M10P_perception_ok);
   CHECK(M10P_clearance_mm<=ranges[j]+1);
   TIM5_IRQHandler();CHECK(Radar_effective_target==10 && hardware_pwm>0);
  }
 }
 /* Changed target still uses encoder feedback instead of fixed duty. */
 Speed_mubiao=12;TIM5_IRQHandler();CHECK(Radar_effective_target==12);
 for(i=0;i<20;i++)pwm=PID_realize(0,10,&pid);CHECK(pwm==100);CHECK(pid.err_sum==200);
 memset(&pid,0,sizeof pid);
 CHECK(PID_realize(0,0,&pid)==0);
 printf("PASS %u adapter/race/PI checks\n",checks);return 0;
}
