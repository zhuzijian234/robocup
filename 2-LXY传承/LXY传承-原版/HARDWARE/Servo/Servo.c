/**
 * @file    Servo.c
 * @brief   舵机PWM驱动（TIM3 CH1, PA6）
 *
 * PWM周期: 20000 (50Hz舵机标准)
 * 有效脉宽: 1360~1800（约1.0~2.0ms）
 * 中位: ~1560（约1.5ms）
 */

#include "stm32f4xx.h"
#include "PWM.h"

void Servo_Init(uint16_t psc, uint16_t arr, uint16_t puse)
{
    TIM3_PWM_Init(psc - 1, arr - 1, puse);
}

void Servo_ChangePwm(uint16_t servo_pwm)
{
    TIM_SetCompare1(TIM3, servo_pwm);
}
