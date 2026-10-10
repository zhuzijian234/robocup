#include "ble_diag.h"
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
#define CONTROL_TRACE(...) ((void)0)
#include "moto.h"
#include <float.h>

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
uint8_t Midline_fit(_LEIDA_DATA_plane *points, int start, int end, Midline_type *line)
{
    int i, n = end - start;
    float sx = 0, sy = 0, xx = 0, xy = 0, x, y;
    line->k = line->b = 0;
    if (start < 0 || n < 2 || end > LEIDA_DATA_COUNTER / 2) {
        Diag_Fit(line, 0);
        return 0;
    }
    for (i = start; i < end; i++) {
        x = points[i]._x;
        y = points[i]._y;
        if (!(x <= FLT_MAX && x >= -FLT_MAX && y <= FLT_MAX && y >= -FLT_MAX)) {
            Diag_Fit(line, 0);
            return 0;
        }
        sx += x;
        sy += y;
    }
    sx /= n;
    sy /= n;
    for (i = start; i < end; i++) {
        x = points[i]._x - sx;
        xx += x * x;
        xy += x * (points[i]._y - sy);
    }
    if (xx <= 1e-6f) {
        line->b = sy;
        Diag_Fit(line, 0);
        return 0;
    }
    line->k = xy / xx;
    line->b = sy - line->k * sx;
    if (!(line->k <= FLT_MAX && line->k >= -FLT_MAX && line->b <= FLT_MAX && line->b >= -FLT_MAX)) {
        line->k = line->b = 0;
        Diag_Fit(line, 0);
        return 0;
    }
    Diag_Fit(line, 1);
    return 1;
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
    q1 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    AB = sqrt(q1);
    q1 = (x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2);
    BC = sqrt(q1);
    q1 = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1);
    AC = sqrt(q1);

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
        S_of_ABC = ((LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x) * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y) - (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x) * (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y)) / 2;

        q1 = (LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x) * (LEIDA_DATA_CENTER[i + forward]._x - LEIDA_DATA_CENTER[i]._x) + (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y) * (LEIDA_DATA_CENTER[i + forward]._y - LEIDA_DATA_CENTER[i]._y);
        AB = sqrt(q1);

        q1 = (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i + forward]._x) * (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i + forward]._x) + (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i + forward]._y) * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i + forward]._y);
        BC = sqrt(q1);

        q1 = (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x) * (LEIDA_DATA_CENTER[i + 2 * forward]._x - LEIDA_DATA_CENTER[i]._x) + (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y) * (LEIDA_DATA_CENTER[i + 2 * forward]._y - LEIDA_DATA_CENTER[i]._y);
        AC = sqrt(q1);

        if (AB * BC * AC == 0) {
            K = 0;
        } else {
            K = 4 * S_of_ABC / (AB * BC * AC);
        }

        K_sum += K;
    }

    return K_sum / i; /* 平均曲率 */
}

/* ======================== PID初始化 ======================== */
/*只有 PD，没有 I（积分项）。因为舵机控制要的是快速响应，
积分项会让车在出弯后还"记得"之前的偏差，导致晃来晃去。*/

void Midline_PD_Init(pid_type *midline_pid, float kp, float kp_2, float kp_3,
                     float kd, float kd_2, float kd_3)
{
    Midline_PD_Reset();
    midline_pid->err = 0;
    midline_pid->err_l = 0;

    midline_pid->kp = kp;
    midline_pid->kp_2 = kp_2;
    midline_pid->kp_3 = kp_3;
    midline_pid->kd = kd;
    midline_pid->kd_2 = kd_2;
    midline_pid->kd_3 = kd_3;
}
/*速度控制则有 I 项——PI 控制。速度不需要像舵机那样快速响应，
积分项用来消除稳态误差（比如上坡时自动加力）。
*/
void Speed_PID_Init(pid_type *midline_pid, float kp, float ki, float kd)
{
    midline_pid->err = 0;
    midline_pid->err_l = 0;

    midline_pid->kp = kp;
    midline_pid->ki = ki;
    midline_pid->kd = kd;
}

/* ======================== 蓝牙可调参数 ========================
 * 以下参数可通过蓝牙(HC-05)在运行时调整，
 * 用于微调控车行为。
 */

float BLUE_DIS_RIGHT = 62; /* 右侧宽度补偿系数（小转弯模式） */
float BLUE_DIS_LEFT = 60;  /* 左侧宽度补偿系数（小转弯模式） */

float BLUE_Y_RIGHT = 800; /* 右转时的目标Y坐标 */
float BLUE_Y_LEFT = 800;  /* 左转时的目标Y坐标 */

float BLUE_Y_STRA = 750;   /* 直道模式下的目标Y坐标 */
float BLUE_Y_STRA_SEL = 0; /* 直道模式选择: 0=用中线末点, 1=用BLUE_Y_STRA */
float err[5] = {0};        /* 误差历史缓冲区（FIR滤波用，当前未使用） */

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
uint8_t Servo_PD_valid;
static uint8_t pd_history_valid;
static uint16_t pd_previous_mode;
static uint32_t pd_previous_us;
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
uint16_t Midline_PD(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t i;
    uint32_t now = Diag_TimeUs(), dt = now - pd_previous_us;
    float e = 0, kp, kd, p, d, original_d, output, mag, x = 0, best = FLT_MAX, target;
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
        e = mode == 1 ? -(x + paodao_distance * BLUE_DIS_RIGHT / 100) : -(x - paodao_distance * BLUE_DIS_LEFT / 100);
    } else if (mode == 3 || mode == 4 || mode == 8 || mode == 9) {
        if (!(fabs(line->k) <= FLT_MAX))
            return pd_reject();
        mag = fabs(line->k) < 0.35f ? 500.0f : 175.0f / fabs(line->k);
        /* Direction is the selected branch, never the sign of a degenerate fit. */
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
    /* Zero err_l on every change of measurement target, EXCEPT when entering a turn
     * branch: there the error step is real car motion and its D kick is wanted. */
    if (!pd_history_valid || !dt || dt > 250000u ||
        (mode != pd_previous_mode && !((mode == 1) || (mode == 2) || (mode == 3) || (mode == 4) || (mode == 8) || (mode == 9)))) {
        pid->err_l = e;
        Diag_detail_u[4] |= 4;
    } else
        kd *= 115000.0f / dt;
    p = 10 * kp * e;
    d = 10 * kd * (e - pid->err_l);
    original_d = d;
    /* 150 = about half of the one-sided servo travel (275), so the turn-entry
     * kick is not clipped away before it reaches the rudder. */
    if (d > 150)
        d = 150;
    if (d < -150)
        d = -150;
    /* D may damp P toward neutral, but cannot reverse the correction sign. */
    if ((p >= 0 && p + d < 0) || (p <= 0 && p + d > 0))
        d = -p;
    if (d != original_d)
        Diag_detail_u[4] |= 2048;
    output = 10 * mid + p + d;
    /* Explicit turn branches cannot command the opposite side of neutral. */
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
    Servo_ChangePwm((uint16_t)output);
    return (uint16_t)output;
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
    float moto_pwm = 0;
    static float err_sum = 0;

    speed_pid->err = speed_mubiao - speed_now;
    err_sum += speed_pid->err;

    /* 积分抗饱和 */
    if (err_sum >= 200)
        err_sum = 200;
    if (err_sum <= -200)
        err_sum = -200;

    moto_pwm = moto_pwm_now + speed_pid->kp * speed_pid->err + speed_pid->ki * err_sum + speed_pid->kd * (speed_pid->err - speed_pid->err_l);

    speed_pid->err_l = speed_pid->err;

    Diag_motor_integral = err_sum;
    Diag_motor_prelimit = moto_pwm;
    /* 输出限幅 [0, 100] */
    if (moto_pwm >= 100)
        moto_pwm = 100;
    if (moto_pwm <= 0)
        moto_pwm = 0;

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
    float moto_pwm = 0;
    static float err_sum = 0;

    /* 计算当前偏差 */
    speed_pid->err = speed_mubiao - speed_now;

    /* 累加积分 */
    err_sum += speed_pid->err;

    /* 积分抗饱和 */
    if (err_sum >= 200)
        err_sum = 200;
    if (err_sum <= -200)
        err_sum = -200;

    /* 位置式PI: pwm = kp*err + ki*积分 + kd*微分 */
    moto_pwm = speed_pid->kp * speed_pid->err + speed_pid->ki * err_sum + speed_pid->kd * (speed_pid->err - speed_pid->err_l);

    /* 记录上一次偏差 */
    speed_pid->err_l = speed_pid->err;

    /* 输出限幅 [0, 100] */
    if (moto_pwm >= 100)
        moto_pwm = 100;
    if (moto_pwm <= 0)
        moto_pwm = 0;

    return moto_pwm;
}
