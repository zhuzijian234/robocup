/**
 * @file    CENTRE_LINE.h
 * @brief   中线检测、PID控制与路径拟合模块
 *
 * 本模块实现雷达小车的核心循线算法：
 * - 最小二乘法直线拟合雷达点云，提取道路边界
 * - 多模式PD舵机转向控制（9种模式应对不同路况）
 * - 速度PID控制（增量式和位置式两种）
 * - 曲率计算，用于弯道检测
 *
 * 整体架构：
 *   雷达数据 -> 最小二乘拟合 -> 中线参数(k,b) -> 多模式PD -> 舵机PWM
 *   编码器   -> 速度PID / 位置式PI        -> 电机PWM
 */

#ifndef _CENTRE_LINE_H
#define _CENTRE_LINE_H

#include "math.h"
#include "stm32f4xx.h"
#include "Servo.h"
#include "LEIDA_DATA.h"

extern uint16_t forward;
extern float paodao_distance;

/* 拟合直线参数：y = k*x + b */
typedef struct
{
    float k;  /* 斜率 */
    float b;  /* 截距 */
} Midline_type;

/* PID控制器参数块
 * 支持三组kp/kd对应不同转向模式：
 *   kp/kd     -> 直道/垂线模式 (flag 0,5,6,7)
 *   kp_2/kd_2 -> 大转弯模式 (flag 3,4,8,9)
 *   kp_3/kd_3 -> 小转弯模式 (flag 1,2)
 */
typedef struct
{
    float kp;
    float kp_2;
    float kp_3;
    float ki;
    float kd;
    float kd_2;
    float kd_3;

    float v_set;     /* 速度设定 */
    float v_fb;      /* 速度反馈 */

    float err_ll;    /* 上上次误差 */
    float err_l;     /* 上次误差 */
    float err;       /* 当前误差 */
    float err_sum;   /* 误差积分 */
    float erry;

    float out;       /* PID输出 */
    float out_max;   /* 输出上限 */
    float out_min;   /* 输出下限 */
} pid_type;

extern pid_type Servo_pd;
extern pid_type Speed_pid;
extern Midline_type Midline;
extern Midline_type Midline2;
extern Midline_type Midline3;
extern Midline_type Midline_forward;
extern Midline_type Midline_forward_2;
extern Midline_type Midline_forward_3;
extern float SPEED_ERR;

extern float BLUE_DIS_LEFT;
extern float BLUE_DIS_RIGHT;
extern float BLUE_Y_LEFT;
extern float BLUE_Y_RIGHT;
extern float BLUE_Y_STRA;
extern float BLUE_Y_STRA_SEL;

/* ===== 舵机行程参数 (循线模式: PD输出限幅 + 中位 + 强制打满) =====
 * LXY车默认:  MIN=1360, MAX=1800, MID=1565
 * 换谢露车只改这三行: MIN=1170, MAX=1720, MID=1445
 * 单位: 真实PWM值 (main.c里MID除以10转成servo_midpwm的/10格式) */
#define SERVO_PWM_MIN 1360   /* 右打满 */
#define SERVO_PWM_MAX 1800   /* 左打满 */
#define SERVO_PWM_MID 1565   /* 中位 */

uint16_t Midline_PD(_LEIDA_DATA_plane centerline[], pid_type *midline_pid, Midline_type *midline,
                    uint16_t servo_midpwm, uint16_t CENTER_cnt_start, uint16_t CENTER_cnt_end, uint16_t flag);
uint16_t Speed_PID(float speed_now, float speed_mubiao, pid_type *speed_pid, uint16_t moto_pwm_now);

void Midline_PD_Init(pid_type *midline_pid, float kp, float kp_2, float kp_3, float kd, float kd_2, float kd_3);
void Speed_PID_Init(pid_type *midline_pid, float kp, float ki, float kd);
float PID_realize(float speed_now, float speed_mubiao, pid_type *speed_pid);

/* 最小二乘法直线拟合：对 centerline[startline..endline] 拟合 y = k*x + b */
void Midline_fit(_LEIDA_DATA_plane *centerline, int startline, int endline, Midline_type *midline);

/* 三点法（Menger）曲率计算 */
float curvity_cal(_LEIDA_DATA_plane LEIDA_DATA_CENTER[], uint16_t counter);
float curvity_cal1(float x1, float y1, float x2, float y2, float x3, float y3);

#endif
