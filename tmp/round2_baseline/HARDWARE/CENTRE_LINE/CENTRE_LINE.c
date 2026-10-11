/** 转向编排与编码器 PI。路径计算不访问硬件；本文件负责时间、参数和唯一输出。 */
#include "centre_line.h"
#include "ble_diag.h"
#include "m10p_vehicle.h"
#include "Servo.h"
#include <string.h>

pid_type Servo_pd, Speed_pid;
uint8_t Servo_PD_valid;
float CENTER_X_TARGET_MM = -27.0f; /* 现有安装标定，换安装位置需重测 */
float MODE0_ERR_CLAMP_MM = 200.0f; /* 保留用户的直道限幅；弯道真实路径误差上限 500 */
float PATH_PREVIEW_MM = 700.0f;    /* 同一前向参考，实际支持不足时明确缩短 */
float PATH_WIDTH_MM = 500.0f;      /* 当前记录的 50 cm 通道；双侧可靠后按实测宽度更新 */
PathObservation Steering_path;
PathCommand Steering_command;
static PathGeometry geometry;
static PathController controller;
static uint32_t previous_revision;
static uint32_t previous_scan_us;

void Steering_Init(void)
{
    memset(&controller, 0, sizeof controller);
    memset(&geometry, 0, sizeof geometry);
    previous_scan_us = 0;
    memset(&Servo_pd, 0, sizeof Servo_pd);
    Servo_pd.kp = 0.035f;
    Servo_pd.kd = 0.035f;
    Servo_pd.kp_2 = 0.040f;
    Servo_pd.kd_2 = 0.022f;
    Servo_pd.kp_3 = 0.0395f;
    Servo_pd.kd_3 = 0.020f;
}

void Steering_Update(const M10P_Scan *scan, const PathPoint *points, uint16_t count)
{
    PathGains gains;
    uint8_t scan_valid;
    uint32_t now;
    PathGeometry previous_geometry = geometry;
    PathObservation *path = &Steering_path;
    PathCommand *command = &Steering_command;
    if (!scan || (uint32_t)(scan->end_us - previous_scan_us) > 250000u)
    {
        geometry.candidate_frames = geometry.avoid_hold = 0;
        geometry.avoid_offset = 0;
    }
    previous_scan_us = scan ? scan->end_us : 0;
    if (!M10P_ScanUsable(scan, Diag_TimeUs()))
        count = 0;
    Path_Build(&geometry, points, count, PATH_WIDTH_MM, PATH_PREVIEW_MM, CENTER_X_TARGET_MM, path);
    now = Diag_TimeUs();
    /* 接收代次/时效与几何质量分开：单侧可用不要求两侧桶数同时达标。
     * 计算结束再查一次代次和年龄，避免处理期间 DMA 异常使旧帧被执行。 */
    scan_valid = M10P_ScanUsable(scan, now) && !(M10P_perception_why & M10P_WHY_CAPACITY);
    if (!scan_valid)
    {
        geometry = previous_geometry;
        geometry.candidate_frames = geometry.avoid_hold = 0;
        geometry.avoid_offset = 0;
        path->valid = path->far_valid = path->width_measured = 0;
        path->avoid_offset = path->avoid_y = 0;
        path->avoid_state = AVOID_CLEAR;
        path->source = PATH_NONE;
        path->width = geometry.width;
    }
    if (!path->valid)
    {
        geometry.candidate_frames = geometry.avoid_hold = 0;
        geometry.avoid_offset = 0;
    }
    if (previous_revision != Diag_revision)
    {
        controller.history = 0; /* 在线改变预瞄/标定不制造假 D */
        previous_revision = Diag_revision;
    }
    gains.kp = Servo_pd.kp;
    gains.kd = Servo_pd.kd;
    gains.turn_kp = Servo_pd.kp_2;
    gains.turn_kd = Servo_pd.kd_2;
    gains.side_kp = Servo_pd.kp_3;
    gains.side_kd = Servo_pd.kd_3;
    gains.center_x = CENTER_X_TARGET_MM;
    gains.straight_limit = MODE0_ERR_CLAMP_MM;
    gains.pwm_min = SERVO_PWM_MIN;
    gains.pwm_mid = SERVO_PWM_MID;
    gains.pwm_max = SERVO_PWM_MAX;
    Path_Control(&controller, path, &gains, scan_valid, now, (uint16_t)TIM3->CCR1, command);
    Servo_PD_valid = command->applied;
    M10P_steer_source = command->applied ? path->source : M10P_SRC_HOLD;
    if (command->computed)
        Servo_pd.err = command->error;
    if (command->applied)
        Servo_ChangePwm(command->pwm);

    /* DETAIL schema 2：候选与实际动作分开，保舵不得标记为执行了本帧 PD。
     * 保留前八个浮点的 PD 语义，新几何字段由上位机按 schema 解析。 */
    Diag_detail_u[4] = (command->computed ? 1u : 0u) | (command->d_reset ? 4u : 0u);
    Diag_detail_u[6] = command->previous_source;
    Diag_detail_u[7] = path->source;
    Diag_detail_u[8] = command->reason;
    Diag_detail_u[9] = command->applied;
    Diag_detail_u[10] = path->far_valid;
    Diag_detail_u[12] = path->near_fit[0].count;
    Diag_detail_u[13] = path->near_fit[1].count;
    Diag_detail_u[14] = path->far_fit[0].count;
    Diag_detail_u[15] = path->far_fit[1].count;
    Diag_detail_u[16] = path->width_measured ? 2 : geometry.measured ? 1 : 0;
    Diag_detail_u[17] = (uint32_t)(command->bend + 1);
    Diag_detail_u[18] = count;
    Diag_detail_u[19] = command->candidate_pwm;
    Diag_detail_u[20] = command->pwm;
    Diag_detail_f[0] = Servo_pd.err_l;
    Diag_detail_f[1] = command->error;
    Diag_detail_f[2] = command->p;
    Diag_detail_f[3] = command->d;
    Diag_detail_f[4] = command->kp;
    Diag_detail_f[5] = command->kd;
    Diag_detail_f[6] = command->unclamped;
    Diag_detail_f[7] = SERVO_PWM_MID;
    Diag_detail_f[8] = path->near_x;
    Diag_detail_f[9] = path->far_x;
    Diag_detail_f[10] = path->ref_y;
    Diag_detail_f[11] = path->near_a;
    Diag_detail_f[12] = path->far_a;
    Diag_detail_f[13] = path->width;
    Diag_detail_f[14] = path->near_fit[0].rms;
    Diag_detail_f[15] = path->near_fit[1].rms;
    Diag_detail_f[16] = path->near_fit[0].max_y - path->near_fit[0].min_y;
    Diag_detail_f[17] = path->near_fit[1].max_y - path->near_fit[1].min_y;
    Diag_detail_f[18] = path->near_fit[0].gap;
    Diag_detail_f[19] = path->near_fit[1].gap;
    Diag_detail_f[20] = path->far_fit[0].rms;
    Diag_detail_f[21] = path->far_fit[1].rms;
    Diag_detail_f[22] = path->far_fit[0].max_y - path->far_fit[0].min_y;
    Diag_detail_f[23] = path->far_fit[1].max_y - path->far_fit[1].min_y;
    if (command->applied)
        Servo_pd.err_l = command->error;
}

void Speed_PID_Init(pid_type *pid, float kp, float ki, float kd)
{
    memset(pid, 0, sizeof *pid);
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* TIM5 每 10 ms 调用；目标固定，雷达/路径异常不参与速度环。 */
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
    moto_pwm = speed_pid->kp * speed_pid->err + speed_pid->ki * speed_pid->err_sum +
               speed_pid->kd * (speed_pid->err - speed_pid->err_l);

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
