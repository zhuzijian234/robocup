/**
 * @file    PWM_pn_out.h
 * @brief   高级定时器TIM1互补PWM输出（含死区）
 *
 * TIM1 CH1:  P通道 -> PA8
 * TIM1 CH1N: N通道 -> PA7
 * 死区时间: ~3us (TIM_DeadTime = 11)
 */

#ifndef _PWM_PN_OUT_H
#define _PWM_PN_OUT_H

void TIM_PWM_PN_Init(u16 psc, u16 arr);

#endif
