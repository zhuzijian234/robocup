/**
 * @file    timer.h
 * @brief   系统定时器模块 — 定时测速与速度PID计算
 *
 * TIM5: 10ms定时中断，读取编码器 -> 计算速度 -> 位置式PI -> 更新电机PWM
 * TIM14: 辅助定时器（预留）
 */

#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "DMA.h"

extern uint8_t ENCODER_TIM;
extern uint8_t TIM_IRQ_COUNTER;
extern uint16_t moto_pwm;
extern uint16_t daoche_flag;   /* 倒车标志 */

void TIM5_Int_Init(u16 arr, u16 psc);
void TIM14_Int_Init(u16 arr, u16 psc);

#endif
