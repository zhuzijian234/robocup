/**
 * @file    moto.h
 * @brief   直流电机驱动与编码器测速
 *
 * 硬件映射：
 *   电机PWM:  TIM2 CH3 -> PA2
 *   电机方向:  PE5 (高=正转), PE4 (低=使能)
 *   编码器:    TIM4 正交编码器 -> PD12, PD13
 *
 * 速度计算：
 *   减速比: 6.25
 *   编码器PPR: 11
 *   速度 = (EncoderCNT * 100) / (4 * 11 * 6.25)
 */

#ifndef __MOTO_H
#define __MOTO_H

void Moto_Init(uint16_t psc, uint16_t arr, uint16_t puse);
void Moto_Speed(uint16_t Compare);
void Encoder_Init(void);
void Get_Encoder(void);

extern float Encoder_cnt;
extern float Speed_now;
extern float Speed_mubiao;

#endif
