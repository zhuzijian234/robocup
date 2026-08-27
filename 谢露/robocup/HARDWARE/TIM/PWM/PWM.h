#ifndef _PWM_H
#define _PWM_H

#include "sys.h"

void TIM2_PWM_Init(u32 psc,u32 arr,u32 pulse);		//电机初始化 		PB11 	    TIM2_CH4
void TIM3_PWM_Init(u32 psc,u32 arr,u32 pulse);		//舵机初始化			PA6     	TIM3_CH1

void TIM4_PWM_Init(void);								//编码器初始化		PD12 PD13   TIM4_CH3 TIM4_CH4


//void TIM11_PWM_Init(u32 psc,u32 arr,u32 pulse);		//TIM11 PWM部分初始化(备用）   PB9 TIM11_CH1  /TIM4_CH4
//void SineWave_Data(u16 cycle, double* D, float Um);
//void CosWave_Data(u16 cycle, double* D, float Um);
//void Sine_N_Wave_Data(u16 cycle, double* D, float Um);

#endif  

