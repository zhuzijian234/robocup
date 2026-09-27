
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
uint32_t LEIDA_parse_calls,LEIDA_sync_failures,LEIDA_short_inputs,LEIDA_missing_packets;
uint16_t LEIDA_speed_dps,LEIDA_raw_count;
static uint8_t lidar_packet[47];static uint16_t lidar_pending;
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
#define TURN_GUARD_US 350000u
#define TURN_EXIT_FRAMES 2u
#define TURN_EXIT_ERROR_MM 100.0f
#define TURN_ENTRY_PWM 75.0f /* 入弯附加量同时不超过本帧|P|，不放大小误差噪声 */
#define TURN_MIN_OFFSET 20  /* 接近中位的候选不能成为弯道保持依据 */
#define TURN_RETRACT_PWM 60 /* 同模式单次明显收舵，需要出弯确认或期限到达 */
typedef struct {
    uint32_t observed_us;
    uint16_t mode, pwm;
    uint8_t active, straight_frames;
} TurnGuard;

void Midline_PD_Reset(void)
{
    pd_history_valid = 0;
    Servo_PD_valid = 0;
}
static uint16_t pd_reject(void)
{
    Diag_detail_u[4] &= ~1u;
    Diag_detail_u[4] |= 512u;
    Midline_PD_Reset();
    return (uint16_t)TIM3->CCR1;
}
static uint8_t turn_direction(uint16_t mode)
{
    return (mode == 1 || mode == 3 || mode == 8) ? 1 :
           (mode == 2 || mode == 4 || mode == 9) ? 2 : 0;
}
static uint8_t turn_rank(uint16_t mode)
{
    return (mode == 3 || mode == 4) ? 3 : (mode == 8 || mode == 9) ? 2 : 1;
}
uint16_t Midline_PD_Calculate(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t i;
    uint32_t now = Diag_TimeUs(), dt = now - pd_previous_us;
    float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, best = FLT_MAX, target, entry = 0;
    Servo_PD_valid = 0;
    Diag_detail_u[6] = pd_previous_mode;
    Diag_detail_u[7] = start;
    Diag_detail_u[8] = end;
    Diag_detail_u[9] = end > start ? end - start : 0;
    Diag_detail_f[13] = pid->err_l;
    Diag_detail_f[11] = line->k;
    Diag_detail_f[12] = line->b;
    if (mode > 9 || end <= start || end > LEIDA_DATA_COUNTER / 2) {
        if (mode == 1 || mode == 2)
            Diag_detail_u[4] |= 8;
        return pd_reject();
    }
    if (mode == 0) {
        if (!(fabs(line->k) > 0.1f && fabs(line->k) <= FLT_MAX && fabs(line->b) <= FLT_MAX))
            return pd_reject();
        target = BLUE_Y_STRA_SEL == 1 ? BLUE_Y_STRA : points[end - 1]._y;
        e = -((target - line->b) / line->k - 50);
        if (BLUE_Y_STRA_SEL != 1) {
            if (e > 200)
                e = 200;
            if (e < -200)
                e = -200;
        }
    } else if (mode == 1 || mode == 2) {
        target = mode == 1 ? BLUE_Y_RIGHT : BLUE_Y_LEFT;
        for (i = start; i < end; i++) {
            float dy = fabs(points[i]._y - target);
            if (dy < best && fabs(points[i]._x) <= FLT_MAX) {
                best = dy;
                x = points[i]._x;
                Diag_detail_u[4] |= 2;
                Diag_detail_f[8] = x;
                Diag_detail_f[9] = points[i]._y;
                Diag_detail_f[10] = dy;
            }
        }
        if (!(Diag_detail_u[4] & 2)) {
            Diag_detail_u[4] |= 8;
            return pd_reject();
        }
        /* 期望: 墙保持在车侧 paodao*BLUE_DIS/100 mm 处。x 是"离该墙的点"的横向坐标,
         * e<0 = 车离墙太远, 该往右打 (mode1); e>0 = 该往左打 (mode2)。 */
        e = mode == 1 ? -(x + paodao_distance * BLUE_DIS_RIGHT / 100) : -(x - paodao_distance * BLUE_DIS_LEFT / 100);
    } else if (mode == 3 || mode == 4 || mode == 8 || mode == 9) {
        if (!(fabs(line->k) <= FLT_MAX))
            return pd_reject();
        /* 转弯力度: 线越斜(弯越急)|k| 越小, 给的固定误差越大; |k|<0.35 直接顶到 500。
         * 方向由模式决定, 不看 k 的符号 —— 拟合退化时也不给反向指令。 */
        mag = fabs(line->k) < 0.35f ? 500.0f : 175.0f / fabs(line->k);
        e = (mode == 3 || mode == 8) ? -mag : mag;
    } else if (mode == 5) {
        if (!LEIDA_vertical_valid)
            return pd_reject();
        e = 50 - zhongxian_chuizhi;
    } else {
        if (end - start < 2)
            return pd_reject();
        for (i = start; i < end; i++) {
            if (!(fabs(points[i]._x) <= FLT_MAX))
                return pd_reject();
            x += points[i]._x;
        }
        x /= end - start;
        e = 50 - x;
        Diag_detail_f[14] = x;
        Diag_detail_u[4] |= 256;
    }
    if (!(fabs(e) <= FLT_MAX))
        return pd_reject();
    if (e > 500)
        e = 500;
    if (e < -500)
        e = -500;
    kp = (mode == 1 || mode == 2) ? pid->kp_3 : (mode == 0 || (mode >= 5 && mode <= 7)) ? pid->kp
                                                                                        : pid->kp_2;
    kd = (mode == 1 || mode == 2) ? pid->kd_3 : (mode == 0 || (mode >= 5 && mode <= 7)) ? pid->kd
                                                                                        : pid->kd_2;
    /* 不同模式的误差来自不同测量目标，不能直接相减当作运动变化。
     * 例如大左转500 -> 侧墙左转82，会产生假的负D并把舵机拉回中位。
     * 切模式首帧只用P；弯道延续由主循环的有界TurnGuard负责。 */
    if (!pd_history_valid || !dt || dt > 250000u || mode != pd_previous_mode) {
        pid->err_l = e;
        Diag_detail_u[4] |= 4;
    } else
        kd *= 115000.0f / dt;
    p = 10 * kp * e;
    d = 10 * kd * (e - pid->err_l);
    original_d = d;
    /* D 限幅 ±150: 舵机单边行程 275 (SERVO_PWM_MID 1445 → MIN 1170), 150 约半程,
     * 再小就会把入弯那记踢腿削掉。实测 mode 3 入弯 D ≈ -124, 原来的 ±60 砍掉一半。 */
    if (d > 150)
        d = 150;
    if (d < -150)
        d = -150;
    /* D 只能把 P 往中位拉, 不许把修正方向拽反 */
    if ((p >= 0 && p + d < 0) || (p <= 0 && p + d > 0))
        d = -p;
    if (d != original_d)
        Diag_detail_u[4] |= 2048;
    /* 入弯/换向/升级为更大弯时，单独给有界补偿，不使用跨测量目标的假D。
     * 只增强P已经指向目标弯向的指令；降级、同模式、无效后同模式恢复不重复加。
     * DETAIL中的P/D保持原义，补偿量=pwm_unclamped-pwm_mid-pd_p-pd_d。 */
    if (turn_direction(mode) &&
        (turn_direction(mode) != turn_direction(pd_previous_mode) || turn_rank(mode) > turn_rank(pd_previous_mode)) &&
        ((turn_direction(mode) == 1 && p < 0) || (turn_direction(mode) == 2 && p > 0))) {
        entry = fabs(p) < TURN_ENTRY_PWM ? fabs(p) : TURN_ENTRY_PWM;
        if (p < 0)
            entry = -entry;
    }
    output = 10 * mid + p + d + entry;
    /* 打方向的模式(1/3/8 往右, 2/4/9 往左)不许把舵机指到中位的另一边, 防反打 */
    if ((mode == 1 || mode == 3 || mode == 8) && output > 10 * mid) {
        output = 10 * mid;
        Diag_detail_u[4] |= 1024;
    }
    if ((mode == 2 || mode == 4 || mode == 9) && output < 10 * mid) {
        output = 10 * mid;
        Diag_detail_u[4] |= 1024;
    }
    if (!(fabs(output) <= FLT_MAX))
        return pd_reject();
    Diag_detail_u[4] |= 1;
    Diag_detail_f[0] = pid->err_l;
    Diag_detail_f[1] = e;
    Diag_detail_f[2] = p;
    Diag_detail_f[3] = d;
    Diag_detail_f[4] = kp;
    Diag_detail_f[5] = kd;
    Diag_detail_f[6] = output;
    Diag_detail_f[7] = 10 * mid;
    if (output < SERVO_PWM_MIN || output > SERVO_PWM_MAX)
        Diag_detail_u[4] |= 16;
    if (output < SERVO_PWM_MIN)
        output = SERVO_PWM_MIN;
    if (output > SERVO_PWM_MAX)
        output = SERVO_PWM_MAX;
    pid->err = pid->err_l = e;
    pd_previous_us = now;
    pd_previous_mode = mode;
    pd_history_valid = 1;
    Servo_PD_valid = 1;
    return (uint16_t)output;
}
uint16_t Midline_PD(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t pwm = Midline_PD_Calculate(points, pid, line, mid, start, end, mode);
    if (Servo_PD_valid)
        Servo_ChangePwm(pwm);
    return pwm;
}
#undef printf
int main(void){
 pid_type pid={0}; Midline_type line={.2f,0}; _LEIDA_DATA_plane pts[2]={{-350,600},{-350,775}};
 pid.kp=.035f;pid.kp_2=.040f;pid.kp_3=.0395f;pid.kd=.035f;pid.kd_2=.022f;pid.kd_3=.020f;
 paodao_distance=700;BLUE_DIS_RIGHT=50;
 clock_us=115000;LEIDA_vertical_valid=1;zhongxian_chuizhi=50;
 Midline_PD(pts,&pid,&line,144.5f,0,2,5);
 clock_us+=115000;unsigned first=Midline_PD(pts,&pid,&line,144.5f,0,2,3);
 clock_us+=115000;unsigned next=Midline_PD(pts,&pid,&line,144.5f,0,2,3);
 clock_us+=115000;Midline_PD(pts,&pid,&line,144.5f,0,2,5);
 memset(Diag_detail_u,0,sizeof Diag_detail_u);clock_us+=115000;
 unsigned small=Midline_PD(pts,&pid,&line,144.5f,0,2,1);
 printf("right large first=%u next=%u; small k=.2 x=-350 width=700: pwm=%u error=%.3f\n",first,next,small,pid.err);
 return 0;
}
