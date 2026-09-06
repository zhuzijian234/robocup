/**
 * @file    xielu.h
 * @brief   谢露舵机控制算法移植模块(雷达点流水线 + 信号构造 + qianfang30_PD)
 *
 * 移植自 谢露/robocup/HARDWARE/LEIDA_DATA/LEIDA_DATA.c 与 谢露/robocup/Test/Test.c,
 * 保留 LXY 骨架(main 结构/电机闭环/雷达接收/蓝牙), 只替换舵机控制算法。
 *
 * 错误构造: err = (a×归一化拟合角度 + d×归一化跑道偏差 [+ZW×S弯特征]) × 舵机半行程275,
 * 限幅 ±275, 位置式 PD: servo_pwm = 1445 + kp*err + kd*(err-err_1) + ki*err_sum。
 * 模式: R_CL 右曲线 / L_CL 左曲线 / YZ_j 右直角 / ZZ_j 左直角;
 * 拟合角度 <65° 或 >115° 直接打满(绕过 PD)。
 *
 * 对外接口(主循环/蓝牙调):
 *   XIELU_ProcessFrame(cnt)   一帧雷达数据的完整处理
 *   XIELU_GetErr / XIELU_GetServoPwm / XIELU_GetPidSlt   遥测
 *   XIELU_ActiveGains(kp,kd)  按当前模式取活动增益(遥测)
 */

#ifndef __XIELU_H
#define __XIELU_H

#include "LEIDA_DATA.h"   /* 复用 _LEIDA_DATA / _LEIDA_DATA_plane / LEIDA_DATA_COUNTER */

/* ============ 模式 ============ */
#define R_CL 1   /* 右曲线 */
#define L_CL 2   /* 左曲线 */
#define YZ_j 3   /* 右直角 */
#define ZZ_j 4   /* 左直角 */

/* 跑道偏差模式标志(paodao_G1 用) */
#define ZW 0     /* 直角(单侧断点) */
#define CL 1     /* 曲线(双侧有墙) */

/* 平面滤波方向(Jizuobiao_lvbo 用) */
#define zuo 1    /* 只留左半平面(角度>90°) */
#define you 2    /* 只留右半平面(角度<90°) */
#define wu  0    /* 全部保留 */

/* ============ 谢露车实测常量(谢露/robocup 原值, 换赛道要重标定) ============ */
#define CL_jixianjiao   65.0    /* 曲线极限角: 拟合角度<65°直接打满右 */
#define Start_dis       0.0f    /* 断点检测起点 y 下限 */
#define End_dis         2500.0f /* 前方有效区 y 上限 */
#define LR_bianjiejiao  60.0f   /* 左右边界扫描角: 90±60 */
#define Start_ang       60.0f   /* 前向墙点收集起始角 */
#define zhuan_a1_L      120.0   /* 左直角归一化基准角 */
#define zhuan_a1_R      60.0    /* 右直角归一化基准角 */
#define chekuan         140.0f  /* 车宽(mm), 跑道偏差归一化用 */

/* ============ 舵机(谢露车: 中位1445 左满1720 右满1170) ============ */
#define XIELU_SERVO_MID    1445
#define XIELU_SERVO_LEFT   1720
#define XIELU_SERVO_RIGHT  1170

/* ============ PD 参数(谢露原值, 蓝牙 kp/kp2/kd/kd2 现场可调) ============ */
extern float R_CL_a,    R_CL_d,    R_CL_kp,  R_CL_ki,  R_CL_kd;
extern float L_CL_a,    L_CL_d,    L_CL_kp,  L_CL_ki,  L_CL_kd;
extern float YZ_j_a,    YZ_j_d,    YZ_j_ZW,  YZ_j_kp,  YZ_j_ki,  YZ_j_kd;
extern float ZZ_j_a,    ZZ_j_d,    ZZ_j_ZW,  ZZ_j_kp,  ZZ_j_ki,  ZZ_j_kd;

/* 断点阈值 / 墙点 y 带通上限(蓝牙 zd/dd 可调) */
extern float Duandian_d;
extern float zhuandian;

/* ============ 对外接口 ============ */
void     XIELU_ProcessFrame(uint16_t cnt);      /* 一帧雷达数据的完整处理(已含三分支+qianfang30_PD) */
float    XIELU_GetErr(void);                    /* 最近一帧 err(遥测) */
float    XIELU_GetServoPwm(void);               /* 最近一帧舵机输出(遥测) */
uint16_t XIELU_GetPidSlt(void);                 /* 当前模式 1-4, 0=未决策 */
void     XIELU_ActiveGains(float *kp, float *kd); /* 按当前模式取活动 kp/kd(遥测) */

#endif
