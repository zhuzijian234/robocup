/**
 * @file    CENTRE_LINE.c
 * @brief   中线检测、PID控制与路径拟合算法实现
 *
 * 本模块是雷达小车循线算法的核心。
 *
 * 整体架构:
 *   1. 数据输入:  雷达笛卡尔点云 (LEIDA_DATA_plane数组)
 *   2. 中线拟合:  最小二乘法 -> 直线方程 y = k*x + b (Midline_type)
 *   3. 转向控制:  多模式PD控制器 -> 舵机PWM
 *   4. 速度控制:  位置式PI控制器 -> 电机PWM
 *
 * PID控制模式说明 (flag参数):
 *   0  = 普通中线循迹（直道/微弯）          误差: -(x_target - 50)               kp, kd
 *   1  = 小角度右转                         误差: -(x_right_ref + width_comp)     kp_3, kd_3
 *   2  = 小角度左转                         误差: -(x_left_ref - width_comp)      kp_3, kd_3
 *   3  = 大角度右转（右断点 + 前方数据）     误差: -Δy/k                           kp_2, kd_2
 *   4  = 大角度左转                         误差: -Δy/k                           kp_2, kd_2
 *   5  = 中线垂直直道模式                    误差: -(center_vertical - 50)         kp, kd
 *   6  = 中线均值控制A                       误差: -zhongxian_junzhi               kp, kd
 *   7  = 中线均值控制B                       误差: -zhongxian_junzhi               kp, kd
 *   8  = 中等角度右转                        误差: -Δy/k                           kp_2, kd_2
 *   9  = 中等角度左转                        误差: -Δy/k                           kp_2, kd_2
 */

#include "centre_line.h"
#include "moto.h"

Midline_type Midline;
Midline_type Midline2;
Midline_type Midline3;
Midline_type Midline_forward;
Midline_type Midline_forward_2;
Midline_type Midline_forward_3;

pid_type Servo_pd;
pid_type Speed_pid;

extern float Speed_mubiao;

/* ======================== 最小二乘法直线拟合 ======================== */

/**
 * @brief  使用最小二乘法将一组笛卡尔坐标点拟合为直线 y = k*x + b
 *
 * @param  centerline: 笛卡尔坐标点数组 (_x, _y)
 * @param  startline:  起始索引（含）
 * @param  endline:    结束索引（不含）
 * @param  midline:    输出结构体，接收斜率k和截距b
 *
 * 计算公式:
 *   k = Σ[(xi - x̄)(yi - ȳ)] / Σ[(xi - x̄)²]
 *   b = ȳ - k * x̄
 */
void Midline_fit(_LEIDA_DATA_plane *centerline, int startline, int endline, Midline_type *midline)
{
    int i = 0;
    float sumlines = endline - startline;  /* 参与拟合的点数 */

    float sumX     = 0;
    float sumY     = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp    = 0;  /* 分子: Σ(xi - x̄)(yi - ȳ) */
    float sumDown  = 0;  /* 分母: Σ(xi - x̄)² */

    /* 第一遍: 累加XY求均值 */
    for (i = startline; i < endline; i++) {
        sumX += centerline[i]._x;
        sumY += centerline[i]._y;
    }

    if (sumlines != 0) {
        averageX = sumX / sumlines;
        averageY = sumY / sumlines;
    } else {
        averageX = 0;
        averageY = 0;
    }

    /* 第二遍: 累加分子分母 */
    for (i = startline; i < endline; i++) {
        sumUp   += (centerline[i]._y - averageY) * (centerline[i]._x - averageX);
        sumDown += (centerline[i]._x - averageX) * (centerline[i]._x - averageX);
    }

    if (sumDown == 0)
        midline->k = 0;
    else
        midline->k = sumUp / sumDown;

    midline->b = averageY - midline->k * averageX;
}

/* ======================== 曲率计算 ======================== */

/**
 * @brief  三点法曲率计算（Menger曲率）
 *
 * K = 4 * S_ABC / (AB * BC * AC)
 * 其中 S_ABC 为三角形ABC的有向面积。
 * 返回有符号曲率: 正=逆时针, 负=顺时针。顺逆时针指u->v的旋转方向
 */
float curvity_cal1(float x1, float y1, float x2, float y2, float x3, float y3)
{
    float K;
    float S_of_ABC;
    float q1;
    float AB;
    float BC;
    float AC;

    /* 三角形有向面积 */
    S_of_ABC = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2;
    q1       = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    AB       = sqrt(q1);
    q1       = (x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2);
    BC       = sqrt(q1);
    q1       = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1);
    AC       = sqrt(q1);

    if (AB * BC * AC == 0) {
        K = 0;
    } else {
        K = 4 * S_of_ABC / (AB * BC * AC);
    }

    return K;
}

uint16_t forward = 10;

/**
 * @brief  对中线点集的平均曲率进行计算
 *
 * 步长 = forward (默认10)，即三点间隔10个采样点。
 * 对 counter/3 个三元组求曲率后取平均。
 */
float curvity_cal(_LEIDA_DATA_plane LEIDA_DATA_CENTER[], uint16_t counter)
{
    float K;
    float S_of_ABC;
    float q1;
    float AB;
    float BC;
    float AC;
    uint16_t i;
    float K_sum = 0;

    for (i = 0; i < (uint16_t)(counter / 3); i++) {
        S_of_ABC = ((LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x)
                    * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y)
                    - (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x)
                    * (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y)) / 2;

        q1 = (LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x)
             * (LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x)
             + (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y)
             * (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y);
        AB = sqrt(q1);

        q1 = (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i + forward]._x)
             * (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i + forward]._x)
             + (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i + forward]._y)
             * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i + forward]._y);
        BC = sqrt(q1);

        q1 = (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x)
             * (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x)
             + (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y)
             * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y);
        AC = sqrt(q1);

        if (AB * BC * AC == 0) {
            K = 0;
        } else {
            K = 4 * S_of_ABC / (AB * BC * AC);
        }

        K_sum += K;
    }

    return K_sum / i;  /* 平均曲率 */
}

/* ======================== PID初始化 ======================== */
/*只有 PD，没有 I（积分项）。因为舵机控制要的是快速响应，
积分项会让车在出弯后还"记得"之前的偏差，导致晃来晃去。*/

void Midline_PD_Init(pid_type *midline_pid, float kp, float kp_2, float kp_3,
                     float kd, float kd_2, float kd_3)
{
    midline_pid->err   = 0;
    midline_pid->err_l = 0;

    midline_pid->kp   = kp;
    midline_pid->kp_2 = kp_2;
    midline_pid->kp_3 = kp_3;
    midline_pid->kd   = kd;
    midline_pid->kd_2 = kd_2;
    midline_pid->kd_3 = kd_3;
}
/*速度控制则有 I 项——PI 控制。速度不需要像舵机那样快速响应，
积分项用来消除稳态误差（比如上坡时自动加力）。
*/
void Speed_PID_Init(pid_type *midline_pid, float kp, float ki, float kd)
{
    midline_pid->err   = 0;
    midline_pid->err_l = 0;

    midline_pid->kp = kp;
    midline_pid->ki = ki;
    midline_pid->kd = kd;
}

/* ======================== 蓝牙可调参数 ========================
 * 以下参数可通过蓝牙(HC-05)在运行时调整，
 * 用于微调控车行为。
 */

float BLUE_DIS_RIGHT = 62;   /* 右侧宽度补偿系数（小转弯模式） */
float BLUE_DIS_LEFT  = 60;   /* 左侧宽度补偿系数（小转弯模式） */

float BLUE_Y_RIGHT = 800;    /* 右转时的目标Y坐标 */
float BLUE_Y_LEFT  = 800;    /* 左转时的目标Y坐标 */

float BLUE_Y_STRA     = 750;  /* 直道模式下的目标Y坐标 */
float BLUE_Y_STRA_SEL = 0;    /* 直道模式选择: 0=用中线末点, 1=用BLUE_Y_STRA */
float err[5]          = {0};  /* 误差历史缓冲区（FIR滤波用，当前未使用） */

/* ======================== 多模式PD舵机转向控制器 ======================== */

/**
 * @brief  多模式PD舵机转向控制器
 *
 * @param  centerline:       用于转向参考的笛卡尔点数组
 * @param  midline_pid:      PID参数块
 * @param  midline:          拟合直线参数 (k, b)
 * @param  servo_midpwm:     舵机中位 (PWM值/10)
 * @param  CENTER_cnt_start: 拟合区域起始索引
 * @param  CENTER_cnt_end:   拟合区域结束索引
 * @param  flag:             控制模式选择 (0-9, 见文件头)
 *
 * @return 舵机PWM值 (SERVO_PWM_MIN~SERVO_PWM_MAX范围)
 *
 * 处理流程:
 *   1. 根据flag模式计算偏差
 *   2. 偏差限幅 [-500, 500]
 *   3. PD控制: pwm = 中位 + kp*err + kd*(err - err_last)
 *   4. 缩放到实际PWM范围 (*10)
 *   5. 限幅到 [SERVO_PWM_MIN, SERVO_PWM_MAX]
 *   6. 输出到舵机
 */
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

    /* 模式1: 小角度右转 — 沿右边界走，加宽度补偿 */
    if (flag == 1)
        midline_pid->err = -((BLUE_Y_RIGHT - midline->b) / midline->k + paodao_distance / 100 * BLUE_DIS_RIGHT);

    /* 模式2: 小角度左转 — 沿左边界走，减宽度补偿 */
    if (flag == 2)
        midline_pid->err = -((BLUE_Y_LEFT - midline->b) / midline->k - paodao_distance / 100 * BLUE_DIS_LEFT);

    /* 模式3,4,8,9: 大/中等角度转弯 — 使用Δx= Δy/k 作为偏差 */
    if (flag == 3) midline_pid->err = -fabs(centerline[CENTER_cnt_end - 1]._y - centerline[CENTER_cnt_start]._y) / midline->k;
    if (flag == 4) midline_pid->err = -fabs(centerline[CENTER_cnt_end - 1]._y - centerline[CENTER_cnt_start]._y) / midline->k;
    if (flag == 8) midline_pid->err = -fabs(centerline[CENTER_cnt_end - 1]._y - centerline[CENTER_cnt_start]._y) / midline->k;
    if (flag == 9) midline_pid->err = -fabs(centerline[CENTER_cnt_end - 1]._y - centerline[CENTER_cnt_start]._y) / midline->k;

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

/* ======================== 速度控制 ======================== */

float SPEED_ERR = 0;

/**
 * @brief  增量式PD速度控制器
 *
 * @param  speed_now:    当前速度
 * @param  speed_mubiao: 目标速度
 * @param  speed_pid:    PID参数块
 * @param  moto_pwm_now: 当前电机PWM值
 * @return 新的电机PWM值 (0-100)
 *
 * 公式: pwm += kp*err + ki*err_sum + kd*(err - err_last)
 */
uint16_t Speed_PID(float speed_now, float speed_mubiao, pid_type *speed_pid, uint16_t moto_pwm_now)
{
    float moto_pwm       = 0;
    static float err_sum = 0;

    speed_pid->err = speed_mubiao - speed_now;
    err_sum += speed_pid->err;

    /* 积分抗饱和 */
    if (err_sum >= 200)  err_sum = 200;
    if (err_sum <= -200) err_sum = -200;

    moto_pwm = moto_pwm_now
               + speed_pid->kp * speed_pid->err
               + speed_pid->ki * err_sum
               + speed_pid->kd * (speed_pid->err - speed_pid->err_l);

    speed_pid->err_l = speed_pid->err;

    /* 输出限幅 [0, 100] */
    if (moto_pwm >= 100) moto_pwm = 100;
    if (moto_pwm <= 0)   moto_pwm = 0;

    return (uint16_t)moto_pwm;
}

/**
 * @brief  位置式PI速度控制器（主速度控制回路，TIM5中断中调用）
 *
 * @return 电机PWM值 (0-100)
 *
 * 公式: pwm = kp*err + ki*err_sum + kd*(err - err_last)
 * 在TIM5 ISR中每10ms调用一次。
 */
float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid)
{
    float moto_pwm       = 0;
    static float err_sum = 0;

    /* 计算当前偏差 */
    speed_pid->err = speed_mubiao - speed_now;

    /* 累加积分 */
    err_sum += speed_pid->err;

    /* 积分抗饱和 */
    if (err_sum >= 200)  err_sum = 200;
    if (err_sum <= -200) err_sum = -200;

    /* 位置式PI: pwm = kp*err + ki*积分 + kd*微分 */
    moto_pwm = speed_pid->kp * speed_pid->err
               + speed_pid->ki * err_sum
               + speed_pid->kd * (speed_pid->err - speed_pid->err_l);

    /* 记录上一次偏差 */
    speed_pid->err_l = speed_pid->err;

    /* 输出限幅 [0, 100] */
    if (moto_pwm >= 100) moto_pwm = 100;
    if (moto_pwm <= 0)   moto_pwm = 0;

    return moto_pwm;
}
