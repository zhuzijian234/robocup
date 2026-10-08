/**
 * @file PWM.h
 * @brief M10P小车使用的PWM与编码器初始化接口。
 */
#ifndef _PWM_H
#define _PWM_H
#include "sys.h"

void TIM2_PWM_Init(u32 psc, u32 arr, u32 pulse); /* PA1：电机，TIM2_CH2 */
void TIM3_PWM_Init(u32 psc, u32 arr, u32 pulse); /* PA6：舵机，TIM3_CH1 */
void TIM4_PWM_Init(void);                     /* PD12/PD13：正交编码器 */

#endif
