/**
 * @file    TIM.h
 * @brief   TIM14中断定时器驱动（预留SPWM用）
 *
 * 内含预计算的800点正弦波查找表，用于SPWM生成。
 * 用法：TIM14中断以载波频率触发，每次更新TIM1 CCR2为查找表下一值。
 */

#ifndef __TIM_H
#define __TIM_H

void TIM14_Init(u16 psc, u16 arr);
void TIM8_TRG_COM_TIM14_IRQHandler(void);

#endif
