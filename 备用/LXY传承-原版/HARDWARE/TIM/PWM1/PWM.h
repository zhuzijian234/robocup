/**
 * @file    PWM1.h
 * @brief   简化版PWM初始化（TIM2 CH1, TIM3 CH1），不含pulse参数
 *
 * PWM驱动变体，使用固定的初始脉宽1050，用于部分配置下的电机和舵机初始化。
 */

#ifndef _PWM_H
#define _PWM_H

#include "sys.h"

void TIM2_PWM_Init(u32 psc, u32 arr);  /* TIM2, 32位, PA2 */
void TIM3_PWM_Init(u32 psc, u32 arr);  /* TIM3, 16位, PA6 */

void TIM_SetCompare1(TIM_TypeDef* TIMx, uint32_t Compare1);

#endif
