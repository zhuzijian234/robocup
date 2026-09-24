/**
 * @file    Servo.h
 * @brief   舵机PWM控制（TIM3 CH1, PA6）
 *
 * PWM周期: 20000 (50Hz舵机标准)
 * 有效范围: 1360~1800（对应约1.0~2.0ms脉宽）
 * 中位: ~1560（约1.5ms）
 */

#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(uint16_t psc, uint16_t arr, uint16_t puse);
void Servo_ChangePwm(uint16_t servo_pwm);

#endif
