/**
 * @file    leida_pwm.h
 * @brief   雷达电机PWM转速控制
 *
 * TIM1 CH4 (PA11): 雷达电机转速PWM
 * TIM9 CH1 (PE5):  通用PWM（预留）
 */

#ifndef __LEIDA_PWM_H
#define __LEIDA_PWM_H

#include "sys.h"

void LEIDA_PWM_Init(u32 arr, u32 psc);
void TIM9_PWM_Init(u32 psc, u32 arr, u32 pulse);

#endif
