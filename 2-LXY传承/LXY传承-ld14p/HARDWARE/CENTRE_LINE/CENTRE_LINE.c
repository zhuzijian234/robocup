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
 * PID控制模式说明 (Midline_PD 的 mode 参数, 只允许 0~9; 与 main.c 第8步的选法对应):
 *   0  = 普通中线循迹（直道/微弯）      误差: -((y_target - b)/k - 50)                    kp,   kd
 *        y_target = 拟合线末点 points[end-1]._y（BLUE_Y_STRA_SEL=1 时改用 BLUE_Y_STRA）
 *        |k| <= 0.1 视为"线太横"退化, 直接拒绝本帧
 *   1  = 小角度右转                    误差: -(x + paodao*BLUE_DIS_RIGHT/100)             kp_3, kd_3
 *        在窗口里找 y 最接近 BLUE_Y_RIGHT 的点, 用它的 x 算偏差, 即"把右墙保持在车右
 *        paodao*BLUE_DIS_RIGHT/100 mm 处"; 窗口里一个点都没有则拒绝本帧
 *   2  = 小角度左转                    误差: -(x - paodao*BLUE_DIS_LEFT/100)              kp_3, kd_3
 *        同上, 目标高度 BLUE_Y_LEFT
 *   3  = 大角度右转（右断点+前方拟合, |k|<0.35）    误差: -mag          kp_2, kd_2
 *   4  = 大角度左转                               误差: +mag          kp_2, kd_2
 *   8  = 中等角度右转（0.35<=|k|<0.7 且单边拟合）   误差: -mag          kp_2, kd_2
 *   9  = 中等角度左转                             误差: +mag          kp_2, kd_2
 *        mag = |k| < 0.35 ? 500 : 175/|k| —— 线越斜给的固定误差越大(上限 500);
 *        方向只由模式决定, 不看 k 的符号（k 退化时不给反向指令）
 *   5  = 中线垂直直道                  误差: 50 - zhongxian_chuizhi    kp, kd  (需 LEIDA_vertical_valid)
 *   7  = 中线均值兜底                  误差: 50 - mean(points[]._x)    kp, kd  (拟合失败时用)
 *   6  = 保留未使用                    公式同 7; 当前 main.c 不会选它
 *
 * 注意: 10=BLE_MODE_HOLD / 11=BLE_MODE_INVALID / 12=BLE_MODE_FORCED 是遥测伪模式,
 *       只出现在 main.c 的 telemetry_mode 里, 绝不能传进 Midline_PD (mode>9 会被直接拒绝)。
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
/* 本帧数据不可用时统一的出口: 记诊断位、清历史(下次重新学 err_l)、
 * 舵机保持原位(返回当前 CCR, 不写 PWM)。
 * 代价: 它会留下 Servo_PD_valid = 0 —— main.c 里只有 Servo_PD_valid=1 才会调
 * Radar_ControlCompleted(), 而 TIM5 里 500ms 等不到该调用就会永久锁电机,
 * 所以"拒绝"只能偶发, 不能变成常态。 */
static uint16_t pd_reject(void)
{
    Diag_detail_u[4] &= ~1u;
    Diag_detail_u[4] |= 512u;
    Midline_PD_Reset();
    return (uint16_t)TIM3->CCR1;
}
static uint8_t turn_direction(uint16_t mode);
static uint8_t turn_rank(uint16_t mode);
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

/* 保留直接驱动接口供测试/其他调用方使用。 */
uint16_t Midline_PD(_LEIDA_DATA_plane points[], pid_type *pid, Midline_type *line,
                    float mid, uint16_t start, uint16_t end, uint16_t mode)
{
    uint16_t pwm = Midline_PD_Calculate(points, pid, line, mid, start, end, mode);
    if (Servo_PD_valid)
        Servo_ChangePwm(pwm);
    return pwm;
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
uint16_t TurnGuard_Apply(TurnGuard *state, uint16_t mode, uint8_t valid,
                        uint8_t straight, uint16_t pwm, uint32_t now, uint8_t *held)
{
    uint8_t direction = turn_direction(mode);
    int offset, previous_offset;
    *held = 0;
    /* unsigned差值允许微秒时钟回绕；HOLD/INVALID绝不刷新这个时刻。 */
    if (state->active && (uint32_t)(now - state->observed_us) >= TURN_GUARD_US) {
        state->active = 0;
        state->straight_frames = 0;
    }
    if (!valid) {
        state->straight_frames = 0;
        return pwm;
    }
    if (direction) {
        state->straight_frames = 0;
        offset = direction == 1 ? SERVO_PWM_MID - (int)pwm : (int)pwm - SERVO_PWM_MID;
        previous_offset = direction == 1 ? SERVO_PWM_MID - (int)state->pwm : (int)state->pwm - SERVO_PWM_MID;
        /* 回中/方向矛盾不能覆盖有效锚点，更不能用来刷新保持期限。 */
        if (offset < TURN_MIN_OFFSET) {
            if (state->active && direction == turn_direction(state->mode)) {
                *held = 1;
                return state->pwm;
            }
            state->active = 0;
            return pwm;
        }
        if (state->active && direction == turn_direction(state->mode) &&
            ((turn_rank(mode) < turn_rank(state->mode) && offset < previous_offset) ||
             previous_offset - offset > TURN_RETRACT_PWM)) {
            *held = 1;
            return state->pwm;
        }
        /* 同向增强立即执行；反向观测立即替换旧状态，不锁死旧方向。 */
        state->active = 1;
        state->mode = mode;
        state->pwm = pwm;
        state->observed_us = now;
        return pwm;
    }
    if (!state->active)
        return pwm;
    state->straight_frames = straight ? state->straight_frames + 1 : 0;
    if (state->straight_frames >= TURN_EXIT_FRAMES) {
        state->active = 0;
        state->straight_frames = 0;
        return pwm;
    }
    *held = 1;
    return state->pwm;
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
