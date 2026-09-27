
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

float Speed_mubiao;
#define CONTROL_TRACE(...) ((void)0)
#define printf(...) ((void)0)
uint16_t Midline_PD(_LEIDA_DATA_plane centerline[], pid_type *midline_pid, Midline_type *midline,
                    uint16_t servo_midpwm, uint16_t CENTER_cnt_start, uint16_t CENTER_cnt_end, uint16_t flag)
{
    float servo_pwm    = 0;
    static int flag_r  = 0;  /* 上一帧的flag */
    static int y_r     = 0;
    float sum_y;

    /* 从小转弯模式切换到直道/垂线模式时，清零上次误差（避免D项跳变） */
    if (((flag_r == 1) || (flag_r == 2)) && ((flag == 0) || (flag == 5)))
        midline_pid->err_l = 0;

    /* ===== 根据控制模式计算偏差 ===== */

    /* 模式0: 普通中线循迹（直道/微弯）
     * 偏差 = 中线末点对应的x坐标偏离中心(50mm)的量 */
    if (flag == 0) {
        if (BLUE_Y_STRA_SEL == 1) { /*把y=kx+b反解为x,看中线往哪偏*/
            midline_pid->err = -((BLUE_Y_STRA - midline->b) / midline->k - 50);
        } else {
            midline_pid->err = -((centerline[CENTER_cnt_end - 1]._y - midline->b) / midline->k - 50);
            /* 限幅 [-200, 200] */
            if (midline_pid->err > 200)  midline_pid->err = 200;
            if (midline_pid->err < -200) midline_pid->err = -200;
        }
    }

    /* 模式1: 小角度右转 — 沿右边界走，加宽度补偿
     * 2026-09-06修复: 墙拟合斜率|k|较小时, (BLUE_Y-b)/k数值巨大且符号被
     * 雷达量化噪声随机翻转 -> err在±500间交替, 舵机乱摆、方向随机错。
     * 0.05阈值实测不够, 改为0.3: 墙近乎平行(|k|<0.3)时丢弃墙跟踪项,
     * 只用补偿项(负值)保证右转; 墙明显倾斜(真实弯道)才启用墙跟踪 */
    if (flag == 1) {
        if (fabs(midline->k) < 0.3f)
            midline_pid->err = -(paodao_distance / 100 * BLUE_DIS_RIGHT);
        else
            midline_pid->err = -((BLUE_Y_RIGHT - midline->b) / midline->k + paodao_distance / 100 * BLUE_DIS_RIGHT);
    }

    /* 模式2: 小角度左转 — 沿左边界走，减宽度补偿
     * 2026-09-06修复: 同模式1, |k|<0.3时只用补偿项(正值)保证左转 */
    if (flag == 2) {
        if (fabs(midline->k) < 0.3f)
            midline_pid->err = paodao_distance / 100 * BLUE_DIS_LEFT;
        else
            midline_pid->err = -((BLUE_Y_LEFT - midline->b) / midline->k - paodao_distance / 100 * BLUE_DIS_LEFT);
    }

    /* 模式3,4,8,9: 大/中等角度转弯 — 使用Δx= Δy/k 作为偏差
     * k的符号由main.c强制(右转正/左转负), 但竖直拟合段k=0时强制变号无效,
     * -fabs(Δy)/0=-inf -> 左弯(4/9)会打满右 (2026-09-06修复):
     * |k|<0.05按0.05算并保持符号; k==0时按flag定方向(3/8右, 4/9左) */
    if ((flag == 3) || (flag == 4) || (flag == 8) || (flag == 9)) {
        float k_safe = midline->k;
        if (fabs(k_safe) < 0.05f) {
            if (k_safe < 0)      k_safe = -0.05f;
            else if (k_safe > 0) k_safe = 0.05f;
            else                 k_safe = ((flag == 4) || (flag == 9)) ? -0.05f : 0.05f;
        }
        midline_pid->err = -fabs(centerline[CENTER_cnt_end - 1]._y - centerline[CENTER_cnt_start]._y) / k_safe;
    }

    /* 模式5: 中线垂直 — 让车保持在x=50mm（跑道中心） */
    if (flag == 5) midline_pid->err = -(zhongxian_chuizhi - 50);

    /* 模式6,7: 中线均值控制 */
    if (flag == 6) midline_pid->err = -(zhongxian_junzhi);
    if (flag == 7) midline_pid->err = -(zhongxian_junzhi);

    printf("ERROR:%f\r\n", midline_pid->err);

    /* 偏差限幅 [-500, 500] */
    if (midline_pid->err > 500)  midline_pid->err = 500;
    if (midline_pid->err < -500) midline_pid->err = -500;

    /* ===== 根据模式组选择PID参数，计算PD输出 ===== */

    /* 直道/垂线模式: kp, kd */
    if ((flag == 0) || (flag == 5) || (flag == 6) || (flag == 7))
        servo_pwm += servo_midpwm * 1.0f
                     + (midline_pid->kp * midline_pid->err)
                     + (midline_pid->kd * (midline_pid->err - midline_pid->err_l));

    /* 小转弯模式: kp_3, kd_3 */
    else if ((flag == 1) || (flag == 2))
        servo_pwm += servo_midpwm * 1.0f
                     + (midline_pid->kp_3 * midline_pid->err)
                     + (midline_pid->kd_3 * (midline_pid->err - midline_pid->err_l));

    /* 大/中等转弯模式: kp_2, kd_2 */
    else
        servo_pwm += servo_midpwm * 1.0f
                     + (midline_pid->kp_2 * midline_pid->err)
                     + (midline_pid->kd_2 * (midline_pid->err - midline_pid->err_l));

    /* 缩放到实际舵机PWM范围 (servo_midpwm已除10) */
    servo_pwm = servo_pwm * 10;

    /* 保存当前误差，供下一帧D项使用 */
    midline_pid->err_l = midline_pid->err;

    printf("flag:%d\r\n", flag);
    printf("speed_now:%f,speed_mubiao:%f\r\r\n", Speed_now, Speed_mubiao);
    printf("PWM_Before:%f\r\n", servo_pwm);

    /* 舵机PWM限幅 (行程参数见centre_line.h的SERVO_PWM_MIN/MAX) */
    if (servo_pwm > SERVO_PWM_MAX) servo_pwm = SERVO_PWM_MAX;
    if (servo_pwm < SERVO_PWM_MIN) servo_pwm = SERVO_PWM_MIN;

    /* 应用到舵机 */
    Servo_ChangePwm((uint16_t)servo_pwm);

    /* 保存flag供下一帧检测模式切换 */
    flag_r = flag;

    printf("PWM_After:%lf\r\n", (double)servo_pwm);
    printf("\r\n");
    printf("\r\n");
    return (uint16_t)servo_pwm;
}
#undef printf
int main(void){
 pid_type pid={0}; Midline_type line={.2f,0}; _LEIDA_DATA_plane pts[2]={{-350,600},{-350,775}};
 pid.kp=.035f;pid.kp_2=.040f;pid.kp_3=.0395f;pid.kd=.075f;pid.kd_2=.022f;pid.kd_3=.020f;
 paodao_distance=700;BLUE_DIS_RIGHT=27;
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
