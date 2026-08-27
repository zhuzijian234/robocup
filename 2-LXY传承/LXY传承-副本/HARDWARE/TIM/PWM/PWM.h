/**
 * @file    PWM.h
 * @brief   通用定时器PWM输出驱动
 *
 * 本项目定时器分配（对应谢露版引脚复用）：
 *   TIM1  CH1(P/N)  PA8/PA7: 高级互补PWM（预留SPWM用）
 *   TIM2  CH4       PB11:    电机PWM
 *   TIM3  CH1       PA6:     舵机PWM
 *   TIM4  PD12/PD13:         正交编码器接口
 *   TIM9  CH1       PA2:     雷达电机转速PWM (⚠ PA2也被USART2_TX使用, 初始化顺序保证TIM9后初始化)
 *   TIM10 CH1       PF6:     通用PWM
 *   TIM11 CH1       PB9:     通用PWM
 */

#ifndef _PWM_H
#define _PWM_H

#include "sys.h"

extern uint16_t b;

void TIM10_PWM_Init(u32 psc, u32 arr, u32 pulse);    /* PF6 */
void TIM11_PWM_Init(u32 psc, u32 arr, u32 pulse);    /* PB9 */
void TIM2_PWM_Init(u32 psc, u32 arr, u32 pulse);     /* PB11 — 电机, TIM2 CH4 */
void TIM3_PWM_Init(u32 psc, u32 arr, u32 pulse);     /* PA6 — 舵机, TIM3 CH1 */
void TIM3_Int_Init(u16 arr, u16 psc);                 /* TIM3 中断定时器（与舵机PWM共用） */
void TIM4_PWM_Init(void);                             /* PD12/PD13 — 编码器模式 */

void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);  /* 通用CCR1更新函数 */

/* 波形表生成函数（基于DSP库，已注释保留） */
void SineWave_Data(u16 cycle, double* D, float Um);
void CosWave_Data(u16 cycle, double* D, float Um);
void Sine_N_Wave_Data(u16 cycle, double* D, float Um);

/* 雷达电机PWM (TIM9 CH1, PA2) */
void PWM_Init_leida(void);
void PWM_SetCompare_leida(uint16_t Compare);

#endif
